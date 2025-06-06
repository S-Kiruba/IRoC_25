#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <numeric>
#include <deque>
#include <iomanip>
#include <sstream>
#include <cmath>

using std::placeholders::_1;

struct LandingSpot {
  double x, y, z, variance;
};

class LandingSpotFinder : public rclcpp::Node {
public:
  LandingSpotFinder()
  : Node("landing_spot_finder"),
    smoothing_window_size_(5),
    max_spots_(4) // Home + 3 safe spots max
  {
    declare_parameter("grid_resolution", 0.1);
    declare_parameter("drone_footprint", 0.8);
    declare_parameter("max_height_variance", 0.02);

    grid_res_ = get_parameter("grid_resolution").as_double();
    drone_footprint_ = get_parameter("drone_footprint").as_double();
    max_height_var_ = get_parameter("max_height_variance").as_double();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/segmented_plane", rclcpp::SensorDataQoS(),
      std::bind(&LandingSpotFinder::pointCloudCallback, this, _1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/landing_spots", 1);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  double grid_res_, drone_footprint_, max_height_var_;
  std::string current_frame_;

  const size_t smoothing_window_size_;
  const size_t max_spots_;

  // History keyed by spot ID for smoothing
  std::unordered_map<int, std::deque<LandingSpot>> spot_history_;

  // Last frame smoothed spots
  std::vector<LandingSpot> last_smoothed_spots_;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    current_frame_ = msg->header.frame_id;

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      min_x = std::min(min_x, static_cast<double>(pt.x));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_y = std::max(max_y, static_cast<double>(pt.y));
    }

    int grid_width = std::ceil((max_x - min_x) / grid_res_);
    int grid_height = std::ceil((max_y - min_y) / grid_res_);
    std::vector<std::vector<double>> height_map(grid_width, std::vector<double>(grid_height, std::numeric_limits<double>::max()));

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      int i = static_cast<int>((pt.x - min_x) / grid_res_);
      int j = static_cast<int>((pt.y - min_y) / grid_res_);
      if (i >= 0 && i < grid_width && j >= 0 && j < grid_height) {
        height_map[i][j] = pt.z;
      }
    }

    int fp_cells = static_cast<int>(drone_footprint_ / grid_res_);
    std::vector<std::vector<bool>> occupied(grid_width, std::vector<bool>(grid_height, false));
    std::vector<LandingSpot> safe_spots;

    for (int i = 0; i <= grid_width - fp_cells; ++i) {
      for (int j = 0; j <= grid_height - fp_cells; ++j) {
        bool skip = false;
        for (int wx = 0; wx < fp_cells && !skip; ++wx)
          for (int wy = 0; wy < fp_cells; ++wy)
            if (occupied[i + wx][j + wy]) {
              skip = true;
              break;
            }
        if (skip) continue;

        bool missing = false;
        std::vector<double> heights;
        for (int wx = 0; wx < fp_cells && !missing; ++wx) {
          for (int wy = 0; wy < fp_cells; ++wy) {
            double h = height_map[i + wx][j + wy];
            if (h > 1e5) {
              missing = true;
              break;
            }
            heights.push_back(h);
          }
        }
        if (missing) continue;

        double mean = std::accumulate(heights.begin(), heights.end(), 0.0) / heights.size();
        double var = 0.0;
        for (double h : heights) var += (h - mean) * (h - mean);
        var /= heights.size();
        if (var > max_height_var_) continue;

        for (int wx = 0; wx < fp_cells; ++wx)
          for (int wy = 0; wy < fp_cells; ++wy)
            occupied[i + wx][j + wy] = true;

        double cx = min_x + (i + fp_cells / 2.0) * grid_res_;
        double cy = min_y + (j + fp_cells / 2.0) * grid_res_;

        safe_spots.push_back({cx, cy, mean, var});
      }
    }

    // Limit max spots to max_spots_ (home + 3 spots)
    if (safe_spots.size() > max_spots_) {
      // Sort by variance ascending to pick best flat spots
      std::sort(safe_spots.begin(), safe_spots.end(),
                [](const LandingSpot &a, const LandingSpot &b){ return a.variance < b.variance; });
      safe_spots.resize(max_spots_);
    }

    // Match new spots to previous smoothed spots by proximity for stable IDs
    std::vector<int> matched_indices(safe_spots.size(), -1); // index in last_smoothed_spots_ for each new spot

    const double max_match_distance = drone_footprint_; // max allowed distance to consider same spot

    std::vector<bool> last_spots_used(last_smoothed_spots_.size(), false);

    for (size_t i = 0; i < safe_spots.size(); ++i) {
      double best_dist = max_match_distance;
      int best_idx = -1;
      for (size_t j = 0; j < last_smoothed_spots_.size(); ++j) {
        if (last_spots_used[j]) continue;
        double dx = safe_spots[i].x - last_smoothed_spots_[j].x;
        double dy = safe_spots[i].y - last_smoothed_spots_[j].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < best_dist) {
          best_dist = dist;
          best_idx = static_cast<int>(j);
        }
      }
      if (best_idx >= 0) {
        matched_indices[i] = best_idx;
        last_spots_used[best_idx] = true;
      }
    }

    // Build smoothed spots with consistent IDs
    std::unordered_map<int, LandingSpot> new_spot_map;
    for (size_t i = 0; i < safe_spots.size(); ++i) {
      int spot_id = (matched_indices[i] >= 0) ? matched_indices[i] : static_cast<int>(last_smoothed_spots_.size()) + static_cast<int>(i);
      auto& history = spot_history_[spot_id];
      history.push_back(safe_spots[i]);
      if (history.size() > smoothing_window_size_) {
        history.pop_front();
      }

      // Average history
      double avg_x = 0, avg_y = 0, avg_z = 0, avg_var = 0;
      for (const auto& spot : history) {
        avg_x += spot.x;
        avg_y += spot.y;
        avg_z += spot.z;
        avg_var += spot.variance;
      }
      size_t count = history.size();
      new_spot_map[spot_id] = {avg_x / count, avg_y / count, avg_z / count, avg_var / count};
    }

    // Convert map back to vector sorted by spot ID (stable IDs)
    std::vector<LandingSpot> smoothed_spots;
    for (int i = 0; i < static_cast<int>(max_spots_); ++i) {
      if (new_spot_map.find(i) != new_spot_map.end()) {
        smoothed_spots.push_back(new_spot_map[i]);
      }
    }
    last_smoothed_spots_ = smoothed_spots;

    // Find best spot by lowest variance
    LandingSpot best_spot;
    bool found_best = false;
    double best_score = std::numeric_limits<double>::max();
    for (const auto& s : smoothed_spots) {
      if (s.variance < best_score) {
        best_score = s.variance;
        best_spot = s;
        found_best = true;
      }
    }

    publishMarkers(smoothed_spots, best_spot, found_best);
    RCLCPP_INFO(get_logger(), "Found %zu stable safe landing spots", smoothed_spots.size());
  }

  std::string formatVariance(double var) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << var;
    return oss.str();
  }

  void publishMarkers(const std::vector<LandingSpot> &spots,
                      const LandingSpot &best_spot,
                      bool found_best) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (size_t idx = 0; idx < spots.size(); ++idx) {
      const auto& s = spots[idx];

      visualization_msgs::msg::Marker m;
      m.header.frame_id = current_frame_;
      m.header.stamp = this->now();
      m.ns = "landing_spots";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = s.x;
      m.pose.position.y = s.y;
      m.pose.position.z = s.z + 0.05;
      m.scale.x = 0.2;
      m.scale.y = 0.2;
      m.scale.z = 0.1;

      if (found_best && std::abs(s.x - best_spot.x) < 1e-6 && std::abs(s.y - best_spot.y) < 1e-6) {
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
      } else {
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
      }
      m.color.a = 1.0;
      marker_array.markers.push_back(m);

      // Draw boundary square
      visualization_msgs::msg::Marker box;
      box.header.frame_id = current_frame_;
      box.header.stamp = this->now();
      box.ns = "boundaries";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::LINE_LIST;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.scale.x = 0.01;
      box.color.r = 1.0;
      box.color.g = 1.0;
      box.color.b = 0.0;
      box.color.a = 1.0;

      double half = drone_footprint_ / 2.0;
      geometry_msgs::msg::Point p1, p2, p3, p4;
      p1.x = s.x - half; p1.y = s.y - half; p1.z = s.z;
      p2.x = s.x + half; p2.y = s.y - half; p2.z = s.z;
      p3.x = s.x + half; p3.y = s.y + half; p3.z = s.z;
      p4.x = s.x - half; p4.y = s.y + half; p4.z = s.z;

      // Draw lines between corners
      box.points.push_back(p1); box.points.push_back(p2);
      box.points.push_back(p2); box.points.push_back(p3);
      box.points.push_back(p3); box.points.push_back(p4);
      box.points.push_back(p4); box.points.push_back(p1);

      marker_array.markers.push_back(box);

      // Text label with ID and variance
      visualization_msgs::msg::Marker text;
      text.header.frame_id = current_frame_;
      text.header.stamp = this->now();
      text.ns = "labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = s.x;
      text.pose.position.y = s.y;
      text.pose.position.z = s.z + 0.3;
      text.scale.z = 0.15;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;

      std::ostringstream oss;
      oss << "ID:" << idx << "\nVar:" << formatVariance(s.variance);
      text.text = oss.str();

      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingSpotFinder>());
  rclcpp::shutdown();
  return 0;
}
