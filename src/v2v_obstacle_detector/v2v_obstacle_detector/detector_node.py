import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from v2v_interfaces.msg import ObstacleInfo


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('v2v_obstacle_detector')

        # Declare parameters
        self.declare_parameter('max_detection_range', 1.0)  # from 3.0 meters
        self.max_range = self.get_parameter('max_detection_range').value

        # Relative topic names automatically inherit the robot namespace (e.g. /robot1/scan)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.obstacle_pub = self.create_publisher(
            ObstacleInfo,
            'obstacle_info',
            10
        )

        self.get_logger().info(f'{self.get_namespace()}/v2v_obstacle_detector node initialized.')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        min_dist = float('inf')
        min_index = -1

        # Find closest valid reading within limits
        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if msg.range_min <= r <= self.max_range:
                if r < min_dist:
                    min_dist = r
                    min_index = i

        # If no obstacle within max range, do nothing
        if min_index == -1:
            obs_msg = ObstacleInfo()
            obs_msg.distance = float('inf')
            obs_msg.angle = 0.0
            obs_msg.direction = 'FRONT'
            obs_msg.confidence = 0.0
            self.obstacle_pub.publish(obs_msg)
            return

        # Calculate angle (in radians and degrees)
        angle_rad = msg.angle_min + (min_index * msg.angle_increment)
        # Normalize to [-pi, pi]
        angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
        angle_deg = math.degrees(angle_rad)

        # Determine direction
        if -45.0 <= angle_deg <= 45.0:
            direction = 'FRONT'
        elif 45.0 < angle_deg <= 135.0:
            direction = 'LEFT'
        elif -135.0 <= angle_deg < -45.0:
            direction = 'RIGHT'
        else:
            direction = 'BACK'

        # Calculate confidence (higher confidence for closer detections)
        confidence = float(max(0.0, min(1.0, 1.0 - (min_dist / self.max_range))))

        # Populate and publish ObstacleInfo
        obs_msg = ObstacleInfo()
        obs_msg.distance = float(min_dist)
        obs_msg.angle = float(angle_deg)
        obs_msg.direction = direction
        obs_msg.confidence = confidence

        self.obstacle_pub.publish(obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()