#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

class PlaneSegmentationNode : public rclcpp::Node {
public:
    PlaneSegmentationNode() : Node("plane_segmentation_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&PlaneSegmentationNode::pointCloudCallback, this, std::placeholders::_1) ///camera/camera/depth/color/points 
        );
        plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_plane", 10);
        coeff_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/plane_coefficients", 10);
        center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/plane_center", 10);
        RCLCPP_INFO(this->get_logger(), "Optimized Plane Segmentation Node Initialized");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty input cloud.");
            return;
        }

        // Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.05f, 0.05f, 0.05f);
        voxel.filter(*downsampled);

        if (downsampled->size() < 50) {
            RCLCPP_WARN(this->get_logger(), "Downsampled cloud too small.");
            return;
        }

        // Plane segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.03);
        seg.setInputCloud(downsampled);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No plane found.");
            return;
        }

        // Extract plane cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(downsampled);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane);

        // Tilt check
        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        float tilt_angle = std::acos(normal.dot(Eigen::Vector3f::UnitZ())) * (180.0 / M_PI);
        if (tilt_angle > 15.0) {
            RCLCPP_WARN(this->get_logger(), "Rejected: Tilt = %.2f°", tilt_angle);
            return;
        }

        // Compute centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane, centroid);

        // Compute bounding box area
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*plane, min_pt, max_pt);
        float area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);

        // Publish segmented plane
        sensor_msgs::msg::PointCloud2 plane_msg;
        pcl::toROSMsg(*plane, plane_msg);
        plane_msg.header = msg->header;
        plane_pub_->publish(plane_msg);

        // Publish coefficients (normal + distance)
        geometry_msgs::msg::Vector3Stamped coeff_msg;
        coeff_msg.header = msg->header;
        coeff_msg.vector.x = coefficients->values[0];
        coeff_msg.vector.y = coefficients->values[1];
        coeff_msg.vector.z = coefficients->values[2];
        coeff_pub_->publish(coeff_msg);

        // Publish center
        geometry_msgs::msg::PointStamped center_msg;
        center_msg.header = msg->header;
        center_msg.point.x = centroid[0];
        center_msg.point.y = centroid[1];
        center_msg.point.z = centroid[2];
        center_pub_->publish(center_msg);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        RCLCPP_INFO(this->get_logger(), "✅ Plane OK | Tilt: %.2f° | Area: %.2f m² | Time: %ld ms", tilt_angle, area, duration);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr coeff_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <geometry_msgs/msg/vector3_stamped.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/common/common.h>
// #include <pcl/common/centroid.h>
// #include <Eigen/Dense>

// class PlaneSegmentationNode : public rclcpp::Node {
// public:
//     PlaneSegmentationNode() : Node("plane_segmentation_node") {
//         subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/zed/zed_node/point_cloud/cloud_registered", 10,
//             std::bind(&PlaneSegmentationNode::pointCloudCallback, this, std::placeholders::_1)
//         );
//         plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_plane", 10);
//         coeff_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/plane_coefficients", 10);
//         center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/plane_center", 10);

//         RCLCPP_INFO(this->get_logger(), "🔧 ZED2i Plane Segmentation Node Initialized");
//     }

// private:
//     void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//         auto start_time = std::chrono::high_resolution_clock::now();

//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(*msg, *cloud);

//         if (cloud->empty()) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Empty input cloud.");
//             return;
//         }

//         // Filter out invalid or distant points
//         cloud->erase(
//             std::remove_if(cloud->begin(), cloud->end(), [](const pcl::PointXYZ& pt) {
//                 return !pcl::isFinite(pt) || pt.z > 5.0 || pt.z < 0.3;
//             }),
//             cloud->end()
//         );

//         if (cloud->size() < 100) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Too few valid points after range filtering.");
//             return;
//         }

//         // Remove statistical outliers
//         pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//         sor.setInputCloud(cloud);
//         sor.setMeanK(50);
//         sor.setStddevMulThresh(1.0);
//         sor.filter(*cloud);

//         if (cloud->size() < 100) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Too few points after outlier removal.");
//             return;
//         }

//         // Downsample using voxel grid
//         pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::VoxelGrid<pcl::PointXYZ> voxel;
//         voxel.setInputCloud(cloud);
//         voxel.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm resolution
//         voxel.filter(*downsampled);

//         if (downsampled->size() < 50) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Downsampled cloud too small.");
//             return;
//         }

//         // Plane segmentation using RANSAC
//         pcl::SACSegmentation<pcl::PointXYZ> seg;
//         seg.setOptimizeCoefficients(true);
//         seg.setModelType(pcl::SACMODEL_PLANE);
//         seg.setMethodType(pcl::SAC_RANSAC);
//         seg.setMaxIterations(100);
//         seg.setDistanceThreshold(0.03);
//         seg.setInputCloud(downsampled);

//         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//         pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//         seg.segment(*inliers, *coefficients);

//         if (inliers->indices.empty()) {
//             RCLCPP_WARN(this->get_logger(), "❌ No plane found.");
//             return;
//         }

//         // Extract inlier points (plane)
//         pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud(downsampled);
//         extract.setIndices(inliers);
//         extract.setNegative(false);
//         extract.filter(*plane);

//         // Compute plane normal tilt angle
//         Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
//         float tilt_angle = std::acos(normal.dot(Eigen::Vector3f::UnitZ())) * (180.0 / M_PI);

//         if (tilt_angle > 15.0) {
//             RCLCPP_WARN(this->get_logger(), "⛔ Rejected plane: Tilt = %.2f°", tilt_angle);
//             return;
//         }

//         // Compute centroid of plane
//         Eigen::Vector4f centroid;
//         pcl::compute3DCentroid(*plane, centroid);

//         // Compute bounding area of plane
//         pcl::PointXYZ min_pt, max_pt;
//         pcl::getMinMax3D(*plane, min_pt, max_pt);
//         float area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);

//         // Publish segmented plane
//         sensor_msgs::msg::PointCloud2 plane_msg;
//         pcl::toROSMsg(*plane, plane_msg);
//         plane_msg.header = msg->header;
//         plane_pub_->publish(plane_msg);

//         // Publish plane coefficients
//         geometry_msgs::msg::Vector3Stamped coeff_msg;
//         coeff_msg.header = msg->header;
//         coeff_msg.vector.x = coefficients->values[0];
//         coeff_msg.vector.y = coefficients->values[1];
//         coeff_msg.vector.z = coefficients->values[2];
//         coeff_pub_->publish(coeff_msg);

//         // Publish centroid
//         geometry_msgs::msg::PointStamped center_msg;
//         center_msg.header = msg->header;
//         center_msg.point.x = centroid[0];
//         center_msg.point.y = centroid[1];
//         center_msg.point.z = centroid[2];
//         center_pub_->publish(center_msg);

//         auto end_time = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

//         RCLCPP_INFO(this->get_logger(), "✅ Plane OK | Tilt: %.2f° | Area: %.2f m² | Time: %ld ms", tilt_angle, area, duration);
//     }

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr coeff_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PlaneSegmentationNode>());
//     rclcpp::shutdown();
//     return 0;
// }
