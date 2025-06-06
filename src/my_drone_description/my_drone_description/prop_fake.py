import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.position = [0.0, 0.0, 0.0, 0.0]  # Initial position for each propeller
        self.velocity = [20.0, 20.0, 20.0, 20.0]  # Velocity for continuous rotation

    def publish_joint_states(self):
        # Increment the position value for continuous rotation
        for i in range(4):  # 4 propellers
            self.position[i] += self.velocity[i] * 0.1  # Small increment for smooth rotation
            if self.position[i] > 3.14:  # Wrap around at +π
                self.position[i] -= 6.28  # 2 * π (full rotation)

            if self.position[i] < -3.14:  # Wrap around at -π
                self.position[i] += 6.28  # 2 * π (full rotation)

        # Prepare JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['prop1_joint', 'prop2_joint', 'prop3_joint', 'prop4_joint']
        msg.position = self.position  # Updated positions for continuous rotation
        msg.velocity = self.velocity  # Rotation speed
        msg.effort = [0.0, 0.0, 0.0, 0.0]  # Optional, can be left as zero

        # Publish the joint state message
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing continuous joint states with updated position.")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

