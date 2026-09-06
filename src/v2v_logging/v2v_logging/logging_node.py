import os
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from v2v_interfaces.msg import ObstacleInfo, VehicleStatus

class LoggingNode(Node):
    def __init__(self):
        super().__init__('v2v_logging')
        
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'unknown_robot'
            
        # 1. Logging Setup
        self.log_dir = os.path.join(os.path.expanduser('~'), 'keshava_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, f'{self.robot_id}_full_log.csv')
        

        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w') as f:
                f.write('Timestamp,Pos_X,Pos_Y,Vel_Linear,Vel_Angular,Obstacle_Dist(m),Latency(ms),DDS_Msg_Rate(Hz),Battery(%)\n')
                
        # 2. State Variables for Task 8
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        self.obs_dist = float('inf')
        self.latency_ms = 0.0
        self.battery = 100.0
        
        # DDS Stats variables
        self.msg_count = 0
        self.dds_rate = 0.0
        
        # 3. Subscriptions
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(ObstacleInfo, 'obstacle_info', self.obs_callback, 10)
        self.create_subscription(VehicleStatus, '/vehicle_status', self.v2v_callback, 10)
        
        # 4. Timers
        self.log_timer = self.create_timer(1.0, self.write_log)              # Write to CSV at 1 Hz
        self.battery_timer = self.create_timer(1.0, self.update_battery)     # Drain battery at 1 Hz
        self.dds_timer = self.create_timer(1.0, self.calculate_dds_stats)    # Calculate DDS throughput at 1 Hz
        
        self.get_logger().info(f'{self.robot_id} Full Task 8 Logging Node Initialized.')

    def odom_callback(self, msg: Odometry):
        # Update Position and Velocity
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.vel_linear = msg.twist.twist.linear.x
        self.vel_angular = msg.twist.twist.angular.z

    def obs_callback(self, msg: ObstacleInfo):
        # Update Obstacle Distance
        self.obs_dist = msg.distance

    def v2v_callback(self, msg: VehicleStatus):
        if msg.robot_id != self.robot_id:
            # Communication Latency: Difference between now and when the peer sent the message
            latency_sec = time.time() - msg.timestamp
            self.latency_ms = max(0.0, latency_sec * 1000.0) # Convert to milliseconds
            
            # DDS Stats: Count incoming messages
            self.msg_count += 1

    def calculate_dds_stats(self):
        # DDS Stats: Messages received per second (Hz)
        self.dds_rate = float(self.msg_count)
        self.msg_count = 0

    def update_battery(self):
        # Simulated Battery: Drains faster if the robot is actively moving
        drain = 0.01
        if abs(self.vel_linear) > 0.01 or abs(self.vel_angular) > 0.01:
            drain = 0.05
        self.battery = max(0.0, self.battery - drain)

    def write_log(self):
        current_time = time.strftime('%H:%M:%S')
        
        # Append the complete snapshot row to the CSV file
        with open(self.log_file, 'a') as f:
            f.write(f"{current_time},{self.pos_x:.2f},{self.pos_y:.2f},{self.vel_linear:.2f},{self.vel_angular:.2f},"
                    f"{self.obs_dist:.2f},{self.latency_ms:.2f},{self.dds_rate:.1f},{self.battery:.2f}\n")


def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()