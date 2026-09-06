import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from v2v_interfaces.msg import ObstacleInfo, VehicleStatus


class V2VVisualization(Node):

    def __init__(self):
        super().__init__('v2v_visualization')

        self.robots = ['robot1', 'robot2', 'robot3']

        # Store latest robot positions
        self.robot_positions = {
            'robot1': None,
            'robot2': None,
            'robot3': None
        }

        # Store paths
        self.paths = {
            'robot1': Path(),
            'robot2': Path(),
            'robot3': Path()
        }

        # Publishers


        self.path_publishers = {}

        for robot in self.robots:
            self.path_publishers[robot] = self.create_publisher(
                Path,
                f'/{robot}/path',
                10
            )

        self.marker_publishers = {}

        for robot in self.robots:
            self.marker_publishers[robot] = self.create_publisher(
                Marker,
                f'/{robot}/viz_markers',
                10
            )

        self.communication_pub = self.create_publisher(
            Marker,
            '/communication_marker',
            10
        )

        self.robot_id_pub = self.create_publisher(
            Marker,
            '/robot_ids',
            10
        )

    
        # Odometry subscribers

        for robot in self.robots:
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )

        # Obstacle subscriber
       

        self.create_subscription(
            ObstacleInfo,
            '/robot1/obstacle_info',
            self.obstacle_callback,
            10
        )

        # V2V status subscriber

        self.create_subscription(
            VehicleStatus,
            '/vehicle_status',
            self.vehicle_status_callback,
            10
        )



        # Timer for robot IDs

        self.create_timer(
            0.5,
            self.publish_robot_ids
        )

        self.get_logger().info(
            'V2V Visualization Node started'
        )

  


    def odom_callback(self, msg, robot):

        self.robot_positions[robot] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        path = self.paths[robot]

        pose = PoseStamped()

        pose.header = msg.header
        pose.header.frame_id = f'{robot}/odom'

        pose.pose = msg.pose.pose

        path.header = msg.header
        path.header.frame_id = f'{robot}/odom'

        path.poses.append(pose)

        # Limit path length
        if len(path.poses) > 1000:
            path.poses.pop(0)

        self.path_publishers[robot].publish(path)

    # OBSTACLE MARKER

    def obstacle_callback(self, msg):

        distance = msg.distance
        angle = math.radians(msg.angle)

        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        marker = Marker()

        marker.header.frame_id = 'robot1/base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'obstacle'
        marker.id = 1

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 0.9

        marker.lifetime.sec = 1

        self.marker_publishers['robot1'].publish(marker)

    # V2V COMMUNICATION MARKER


    def vehicle_status_callback(self, msg):

        sender = msg.robot_id

        if sender not in self.robot_positions:
            return

        sender_position = self.robot_positions[sender]

        if sender_position is None:
            return

        # Find another robot to represent communication
        receiver = None

        for robot in self.robots:
            if robot != sender and self.robot_positions[robot] is not None:
                receiver = robot
                break

        if receiver is None:
            return

        receiver_position = self.robot_positions[receiver]

        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'v2v_communication'
        marker.id = 10

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = []

        from geometry_msgs.msg import Point

        p1 = Point()
        p1.x = sender_position[0]
        p1.y = sender_position[1]
        p1.z = 0.4

        p2 = Point()
        p2.x = receiver_position[0]
        p2.y = receiver_position[1]
        p2.z = 0.4

        marker.points.append(p1)
        marker.points.append(p2)

        marker.scale.x = 0.05
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        self.communication_pub.publish(marker)


    # ROBOT ID MARKERS
    

    def publish_robot_ids(self):

        marker_id = 100

        for robot in self.robots:

            position = self.robot_positions[robot]

            if position is None:
                continue

            marker = Marker()

            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'robot_ids'
            marker.id = marker_id

            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.8

            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.35

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.text = robot.upper()

            self.robot_id_pub.publish(marker)

            marker_id += 1


def main(args=None):

    rclpy.init(args=args)

    node = V2VVisualization()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()