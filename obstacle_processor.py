import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Quaternion, TwistWithCovariance
from std_msgs.msg import Header
from my_msgs.msg import Obstacle  # Custom message type
import math
import time

class ObstacleProcessor(Node):
    def __init__(self):
        super().__init__('obstacle_processor')

        # Subscriber to PoseArray
        self.pose_subscription = self.create_subscription(
            PoseArray,
            '/world/weather/pose/info',
            self.pose_callback,
            10
        )

        # Publisher for custom Obstacle message
        self.obstacle_publisher = self.create_publisher(Obstacle, '/obstacle_info', 10)

        # Store previous positions to calculate velocity
        self.previous_poses = {}
        self.previous_time = time.time()

    def pose_callback(self, msg: PoseArray):
        # Check if PoseArray has the required 16 elements
        if len(msg.poses) < 16:
            self.get_logger().warn('PoseArray does not contain 16 elements. Skipping...')
            return

        # Filter elements 3, 5, and 6
        indices = [3, 5, 6]
        filtered_poses = {idx: msg.poses[idx] for idx in indices}

        # Calculate velocity and publish Obstacle messages
        current_time = time.time()
        delta_time = current_time - self.previous_time

        for idx, pose in filtered_poses.items():
            # Calculate velocity (if previous positions are available)
            if idx in self.previous_poses:
                prev_pose = self.previous_poses[idx]
                velocity = TwistWithCovariance()
                velocity.twist.linear.x = (pose.position.x - prev_pose.position.x) / delta_time
                velocity.twist.linear.y = (pose.position.y - prev_pose.position.y) / delta_time
                velocity.twist.linear.z = (pose.position.z - prev_pose.position.z) / delta_time
            else:
                velocity = TwistWithCovariance()

            # Create and publish Obstacle message
            obstacle_msg = Obstacle()
            obstacle_msg.header = Header()
            obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            obstacle_msg.radius = self.calculate_radius(pose)
            obstacle_msg.id = f'actor{idx}'  # Assign actor name based on index
            obstacle_msg.orientation = pose.orientation
            obstacle_msg.value = velocity

            self.obstacle_publisher.publish(obstacle_msg)

        # Update previous poses and time
        self.previous_poses = filtered_poses
        self.previous_time = current_time

    def calculate_radius(self, pose):
        """Dummy radius calculation based on position."""
        return math.sqrt(pose.position.x**2 + pose.position.y**2)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
