import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Declare configurable ROS parameters
        self.declare_parameter('safe_distance', 0.6)
        self.declare_parameter('emergency_distance', 0.25)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.6)

        self.safe_distance = self.get_parameter(
            'safe_distance'
        ).value

        self.emergency_distance = self.get_parameter(
            'emergency_distance'
        ).value

        self.forward_speed = self.get_parameter(
            'forward_speed'
        ).value

        self.turn_speed = self.get_parameter(
            'turn_speed'
        ).value

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

        self.get_logger().info('Obstacle avoidance node started')

        self.get_logger().info(
            f'Safe distance: {self.safe_distance} m'
        )

    def get_sector_distance(self, ranges, msg, min_angle, max_angle):

        sector_ranges = []

        for i, distance in enumerate(ranges):

            angle = msg.angle_min + i * msg.angle_increment

            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            if min_angle <= angle <= max_angle:

                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    sector_ranges.append(distance)

        return min(sector_ranges, default=msg.range_max)

    def scan_callback(self, msg):

        ranges = msg.ranges

        # Divide LiDAR measurements into three sectors
        front_distance = self.get_sector_distance(
            ranges, msg, -math.pi / 6, math.pi / 6
        )

        left_distance = self.get_sector_distance(
            ranges, msg, math.pi / 6, math.pi / 2
        )

        right_distance = self.get_sector_distance(
            ranges, msg, -math.pi / 2, -math.pi / 6
        )

        # Create movement command
        command = TwistStamped()

        command.header.stamp = self.get_clock().now().to_msg()

        if front_distance < self.emergency_distance:

            # Emergency stop
            command.twist.linear.x = 0.0
            command.twist.angular.z = 0.0

        elif front_distance < self.safe_distance:

            # Obstacle detected: turn towards the clearer side
            command.twist.linear.x = self.forward_speed * 0.3

            if left_distance > right_distance:
                command.twist.angular.z = self.turn_speed
            else:
                command.twist.angular.z = -self.turn_speed

        else:

            # Path is clear
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