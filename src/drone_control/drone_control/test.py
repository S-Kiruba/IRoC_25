import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from dronekit import connect, VehicleMode
from tf_transformations import quaternion_from_euler
import logging
import time

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DroneController")

class DroneController:
    def __init__(self, connection_port: str, baudrate: int):
        self.drone = None
        self.is_connected = False

        try:
            logger.info("Connecting to vehicle on port %s with baudrate %d...", connection_port, baudrate)
            self.drone = connect(connection_port, baud=baudrate, wait_ready=True, timeout=60)
            logger.info("Connected to vehicle successfully.")
            self.is_connected = True
        except Exception as e:
            logger.error("Failed to connect to vehicle: %s", str(e))
            return

        logger.info("Switching to FLOWHOLD mode...")
        self.drone.mode = VehicleMode("FLOWHOLD")

        # Wait until mode switch is successful
        while self.drone.mode.name != "FLOWHOLD":
            logger.warning("Waiting for FLOWHOLD mode... Current mode: %s", self.drone.mode.name)
            time.sleep(1)

        logger.info("FLOWHOLD mode set successfully.")

    def arm(self, timeout: int = 10):
        if self.drone.armed:
            logger.info("Drone is already armed.")
            return

        logger.info("Arming drone...")
        self.drone.armed = True
        start_time = time.time()

        while not self.drone.armed:
            if time.time() - start_time > timeout:
                logger.error("Arming timeout reached. Failed to arm the drone.")
                return
            logger.info("Waiting for arming...")
            time.sleep(1)

        logger.info("Drone armed successfully.")


    def disarm(self):
        if not self.drone.armed:
            logger.info("Drone is already disarmed.")
            return

        current_alt = self.drone.rangefinder.distance
        if current_alt is None:
            logger.warning("Disarm aborted: Altitude reading unavailable.")
            return

        if current_alt > 0.2:
            logger.warning(f"Disarm aborted: Drone is not on the ground (altitude = {current_alt:.2f} m).")
            return

        logger.info("Disarming drone...")
        self.drone.armed = False

        while self.drone.armed:
            logger.info("Waiting to disarm...")
            time.sleep(1)

        logger.info("Drone disarmed successfully.")


    def lift(self, throttle: int = 1700):
        throttle = max(1000, min(2000, throttle))
        print(f"Throttle with power: {throttle}")
        self.drone.channels.overrides = {'3': throttle}
        time.sleep(1)

    def lift_to_distance(self, altitude: float, timeout: int = 10, throttle: int = 1700):
        self.lift(throttle=throttle)
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            current_alt = self.drone.rangefinder.distance
            print(f"Current altitude: {current_alt:.2f} m")
            if current_alt >= altitude:
                print(f"Reached target altitude {current_alt}")
                break
            time.sleep(0.5)

    def forward(self, throttle=1700, forward_tilt=1600):
        forward_tilt = min(2000, max(1500, forward_tilt))
        self.drone.channels.overrides = {'2': forward_tilt, '3': throttle}
        time.sleep(1)

    def backward(self, throttle=1700, backward_tilt=1000):
        backward_tilt = max(1000, min(1500, backward_tilt))
        self.drone.channels.overrides = {'2': backward_tilt, '3': throttle}
        time.sleep(1)

    def left(self, throttle=1700, left_yaw=1000):
        left_yaw = max(1000, min(1500, left_yaw))
        self.drone.channels.overrides = {'4': left_yaw, '3': throttle}
        time.sleep(1)

    def right(self, throttle=1700, right_yaw=1700):
        right_yaw = max(1500, min(2000, right_yaw))
        self.drone.channels.overrides = {'4': right_yaw, '3': throttle}
        time.sleep(1)

    def hover(self):
        current_throttle = self.drone.channels.overrides.get('3', 1500)
        self.drone.channels.overrides = {'3': current_throttle}
        time.sleep(1)
        current_alt = self.drone.rangefinder.distance
        if current_alt < 0.2:
            self.get_logger().info('the drone on the land')
        else:
            self.get_logger().info('Hovering..')

    def land(self):
        self.neutralize()
        current_throttle_pwm = self.drone.channels.overrides.get('3', 1700)
        if current_throttle_pwm <= 1000:
            print("Already landed")
            return
        for pwm in range(current_throttle_pwm, 900, -25):
            print(f"Throttle: {pwm}")
            self.drone.channels.overrides = {'3': pwm}
            time.sleep(0.5)
        print("Landing complete")


class FlowHoldTelemetryNode(Node):
    def __init__(self):
        super().__init__('flowhold_telemetry_node')

        # Connect to drone and set up controller
        self.get_logger().info("Initializing drone controller...")
        self.drone_controller = DroneController('/dev/ttyACM0', 57600)
        if not self.drone_controller.is_connected:
            self.get_logger().error("Failed to connect to drone.")
            exit(1)

        # Publishers under /drone/telemetry namespace
        ns = '/drone/telemetry/'
        self.status_pub = self.create_publisher(String, ns + 'system_status', 10)
        self.battery_pub = self.create_publisher(Float32, ns + 'battery', 10)
        self.arm_pub = self.create_publisher(Bool, ns + 'armed', 10)
        self.mode_pub = self.create_publisher(String, ns + 'mode', 10)
        self.imu_pub = self.create_publisher(Imu, ns + 'imu', 10)
        self.attitude_pub = self.create_publisher(Quaternion, ns + 'attitude_quat', 10)
        self.flow_pub = self.create_publisher(Vector3, ns + 'optical_flow', 10)
        self.range_pub = self.create_publisher(Float32, ns + 'rangefinder', 10)
        self.velocity_pub = self.create_publisher(Vector3, ns + 'local_velocity', 10)
        self.warning_pub = self.create_publisher(String, ns + 'warnings', 10)
        self.odom_pub = self.create_publisher(Odometry, ns + 'odom', 10)
        self.rc_channels_pub = self.create_publisher(String, ns + 'rc_channels', 10)
        self.ekf_ok_pub = self.create_publisher(Bool, ns + 'ekf_ok', 10)
        self.heading_pub = self.create_publisher(Float32, ns + 'heading', 10)

        # TF broadcaster for odom->base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        self.yaw_filtered = 0.0
        self.alpha = 0.5  # smoothing factor for yaw

        # Timer to read and publish data at 10Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)

        # Subscriber for control commands
        self.control_sub = self.create_subscription(String,'/drone/control_cmd',self.control_cmd_callback,10)

    def publish_telemetry(self):
        vehicle = self.drone_controller.drone

        try:
            self.arm_pub.publish(Bool(data=vehicle.armed))
            self.mode_pub.publish(String(data=str(vehicle.mode.name)))
            if vehicle.battery.voltage is not None:
                self.battery_pub.publish(Float32(data=vehicle.battery.voltage))
            self.status_pub.publish(String(data=str(vehicle.system_status.state)))

            range_distance = 0.0
            if vehicle.rangefinder and vehicle.rangefinder.distance is not None:
                if vehicle.rangefinder.distance > 0.05:
                    range_distance = vehicle.rangefinder.distance
            self.range_pub.publish(Float32(data=range_distance))

            if vehicle.attitude:
                roll = vehicle.attitude.roll or 0.0
                pitch = vehicle.attitude.pitch or 0.0
                yaw = vehicle.attitude.yaw or 0.0
                self.yaw_filtered = self.alpha * yaw + (1 - self.alpha) * self.yaw_filtered
                q = quaternion_from_euler(roll, -pitch, -self.yaw_filtered)
                quat_msg = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
                self.attitude_pub.publish(quat_msg)

                imu_msg = Imu()
                imu_msg.orientation = quat_msg
                imu_msg.linear_acceleration.x = 0.0
                imu_msg.linear_acceleration.y = 0.0
                imu_msg.linear_acceleration.z = 0.0
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0
                self.imu_pub.publish(imu_msg)

            if vehicle.velocity:
                vx_enu = vehicle.velocity[1]
                vy_enu = vehicle.velocity[0]
                vz_enu = -vehicle.velocity[2]
                self.velocity_pub.publish(Vector3(x=vx_enu, y=vy_enu, z=vz_enu))

            if vehicle.channels:
                rc_str = ','.join(f"{ch}:{val}" for ch, val in vehicle.channels.items())
                self.rc_channels_pub.publish(String(data=rc_str))
            else:
                self.rc_channels_pub.publish(String(data="No RC channels"))

            if hasattr(vehicle, 'ekf_ok'):
                self.ekf_ok_pub.publish(Bool(data=vehicle.ekf_ok))
            else:
                self.ekf_ok_pub.publish(Bool(data=False))

            if vehicle.heading is not None:
                self.heading_pub.publish(Float32(data=float(vehicle.heading)))

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.orientation = quat_msg
            odom_msg.twist.twist.linear.x = vx_enu
            odom_msg.twist.twist.linear.y = vy_enu
            odom_msg.twist.twist.linear.z = vz_enu
            self.odom_pub.publish(odom_msg)

            # Broadcast TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation = quat_msg
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error publishing telemetry: {e}")

    def control_cmd_callback(self, msg: String):
        cmd = msg.data.lower()
        self.get_logger().info(f"Received control command: {cmd}")

        if cmd == "arm":
            self.drone_controller.arm()

        elif cmd == "disarm":
            self.drone_controller.disarm()

        elif cmd.startswith("lift_to"):
            parts = cmd.split()
            altitude = 3.0
            if len(parts) > 1:
                try:
                    altitude = float(parts[1])
                except ValueError:
                    pass
            self.drone_controller.lift_to_distance(altitude=altitude)

        elif cmd.startswith("lift"):
            parts = cmd.split()
            throttle = 1700
            if len(parts) > 1 and parts[1].isdigit():
                throttle = int(parts[1])
            self.drone_controller.lift(throttle=throttle)

        elif cmd == "land":
            self.drone_controller.land()

        elif cmd == "neutralize":
            self.drone_controller.neutralize()

        elif cmd == "forward":
            self.drone_controller.forward()

        elif cmd == "backward":
            self.drone_controller.backward()

        elif cmd == "left":
            self.drone_controller.left()

        elif cmd == "right":
            self.drone_controller.right()

        else:
            self.get_logger().warn(f"Unknown control command: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = FlowHoldTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
