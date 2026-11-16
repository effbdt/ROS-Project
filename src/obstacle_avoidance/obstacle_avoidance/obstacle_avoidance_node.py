import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

import math


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscribe to LiDAR data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publish movement commands
        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Controller parameters
        self.safe_distance = 0.6
        self.emergency_distance = 0.25
        self.forward_speed = 0.15
        self.turn_speed = 0.6

        self.get_logger().info('Obstacle avoidance node started')

    def scan_callback(self, msg):

        ranges = list(msg.ranges)

        # Divide the LiDAR scan into three sectors
        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, distance in enumerate(ranges):

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if -math.pi / 6 <= angle <= math.pi / 6:
                front_ranges.append(distance)

            elif math.pi / 6 < angle <= math.pi / 2:
                left_ranges.append(distance)

            elif -math.pi / 2 <= angle < -math.pi / 6:
                right_ranges.append(distance)

        def valid_ranges(values):
            return [
                value for value in values
                if math.isfinite(value)
                and msg.range_min <= value <= msg.range_max
            ]

        front_ranges = valid_ranges(front_ranges)
        left_ranges = valid_ranges(left_ranges)
        right_ranges = valid_ranges(right_ranges)

        front_distance = min(front_ranges, default=msg.range_max)
        left_distance = min(left_ranges, default=msg.range_max)
        right_distance = min(right_ranges, default=msg.range_max)

        # Create movement command
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        if front_distance < self.emergency_distance:

            # Emergency stop
            command.twist.linear.x = 0.0
            command.twist.angular.z = 0.0

        elif front_distance < self.safe_distance:

            # Turn towards the side with more space
            command.twist.linear.x = 0.0

            if left_distance > right_distance:
                command.twist.angular.z = self.turn_speed
            else:
                command.twist.angular.z = -self.turn_speed

        else:

            # Move forward
            command.twist.linear.x = self.forward_speed
            command.twist.angular.z = 0.0

        self.cmd_publisher.publish(command)

    def stop_robot(self):

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        self.cmd_publisher.publish(command)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()