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

        #Robot states
        self.FORWARD = 0
        self.TURNING = 1
        self.EMERGENCY_STOP = 2

        self.current_state = self.FORWARD
        self.turn_direction = 0.0

        #Distance required before resuming forward movement after turning
        self.clear_distance = 0.8

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

    def choose_best_direction(self, far_left, left, right, far_right):
        directions = {
            'far_left': far_left,
            'left': left,
            'right': right,
            'far_right': far_right
        }

        best_direction = max(
            directions, key=directions.get
        )

        return best_direction

    def get_turn_direction(self, direction):

        direction_mapping = {
            'far_left': 1.0,
            'left': 1.0,
            'straight': 0.0,
            'right': -1.0,
            'far_right': -1.0
        }

        return direction_mapping[direction]


    def scan_callback(self, msg):

        ranges = msg.ranges

        # Divide LiDAR measurements into five sectors
        far_left_distance = self.get_sector_distance(
            ranges, msg, math.pi / 3, math.pi / 2
        )

        left_distance = self.get_sector_distance(
            ranges, msg, math.pi / 6, math.pi / 3
        )

        front_distance = self.get_sector_distance(
            ranges, msg, -math.pi / 6, math.pi / 6
        )

        right_distance = self.get_sector_distance(
            ranges, msg, -math.pi / 3, -math.pi / 6
        )

        far_right_distance = self.get_sector_distance(
            ranges, msg, -math.pi / 2, -math.pi / 3
        )

        left_space = min(left_distance, far_left_distance)
        right_space = min(right_distance, far_right_distance)

        self.get_logger().info(
            f'Far Left: {far_left_distance:.2f} m | '
            f'Left: {left_distance:.2f} m | '
            f'Front: {front_distance:.2f} m | '
            f'Right: {right_distance:.2f} m | '
            f'Far Right: {far_right_distance:.2f} m'
        )

        # Create movement command
        command = TwistStamped()

        command.header.stamp = self.get_clock().now().to_msg()

        if front_distance < self.emergency_distance:

            self.current_state = self.EMERGENCY_STOP

        elif self.current_state == self.EMERGENCY_STOP:

            if (front_distance >= self.emergency_distance + 0.1):
                self.current_state = self.TURNING
        
        elif (self.current_state == self.FORWARD):

            if(front_distance < self.safe_distance):

                best_direction = self.choose_best_direction(
                    far_left_distance,
                    left_distance,
                    right_distance,
                    far_right_distance
                )

                self.turn_direction = self.get_turn_direction(best_direction)
                
                self.current_state = self.TURNING

        elif(self.current_state == self.TURNING):

            if(front_distance >= self.clear_distance):

                self.current_state = self.FORWARD
                self.turn_direction = 0.0
        
        if(self.current_state == self.EMERGENCY_STOP):

            command.twist.linear.x = 0.0
            command.twist.angular.z = 0.0

        elif(self.current_state == self.TURNING):

            command.twist.linear.x = self.forward_speed *0.3
            command.twist.angular.z = self.turn_direction *self.turn_speed

        else:

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