import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from v2v_interfaces.msg import ObstacleInfo, VehicleStatus
from enum import Enum

# Define the explicit states required for Task 7
class State(Enum):
    SEARCH = 1
    MOVE = 2
    OBSTACLE = 3
    BROADCAST = 4
    RECEIVE = 5
    AVOID = 6
    CONTINUE = 7

class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__('v2v_decision_making')

        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'unknown_robot'

        # Subscriptions
        self.obs_sub = self.create_subscription(ObstacleInfo, 'obstacle_info', self.obs_callback, 10)
        self.v2v_sub = self.create_subscription(VehicleStatus, '/vehicle_status', self.v2v_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Variables
        self.local_dist = float('inf')
        self.local_dir = 'FRONT'
        self.v2v_emergency = False
        self.v2v_caution = False
        self.last_v2v_time = 0.0

        # Initial FSM State
        self.state = State.SEARCH
        self.get_logger().info(f'{self.robot_id} FSM Initialized in state: {self.state.name}')

        # 10 Hz Control Loop
        self.timer = self.create_timer(0.1, self.fsm_loop)

    def change_state(self, new_state):
        if self.state != new_state:
            # Deliverable: State Transition Logs
            self.get_logger().info(f'[FSM LOG] Transition: {self.state.name} -> {new_state.name}')
            self.state = new_state

    def obs_callback(self, msg: ObstacleInfo):
        self.local_dist = msg.distance
        self.local_dir = msg.direction

    def v2v_callback(self, msg: VehicleStatus):
        if msg.robot_id != self.robot_id:
            self.last_v2v_time = time.time()
            if msg.priority == 3:
                self.v2v_emergency = True
                self.v2v_caution = False
            elif msg.priority == 2:
                self.v2v_caution = True
                self.v2v_emergency = False

    def fsm_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = f'{self.robot_id}/base_footprint'

        # Clear stale V2V
        if time.time() - self.last_v2v_time > 3.0:
            self.v2v_emergency = False
            self.v2v_caution = False

        # fsm_logic
        
        
        if self.state == State.SEARCH:
            # Looking for initial data. If clear, start moving.
            if self.local_dist > 0.4:
                self.change_state(State.MOVE)
                
        elif self.state == State.MOVE:
            cmd.twist.linear.x = 0.2
            cmd.twist.angular.z = 0.0
            
            # Transition Triggers
            if self.local_dist < 0.25 or (self.local_dir == 'FRONT' and self.local_dist < 0.4):
                self.change_state(State.OBSTACLE)
            elif self.v2v_emergency or self.v2v_caution:
                self.change_state(State.RECEIVE)
                
        elif self.state == State.OBSTACLE:
            # Stop immediately
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            # Instantly transition to Broadcast warning
            self.change_state(State.BROADCAST)
            
        elif self.state == State.BROADCAST:
            # Proceed to avoidance maneuver
            self.change_state(State.AVOID)
            
        elif self.state == State.RECEIVE:
            # Process incoming network warning
            self.change_state(State.AVOID)
            
        elif self.state == State.AVOID:
            # Evasive action logic
            if self.v2v_emergency:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
            elif self.local_dist < 0.25:
                if self.local_dir == 'FRONT':
                    cmd.twist.linear.x = -0.05
                elif self.local_dir == 'RIGHT':
                    cmd.twist.angular.z = 0.5
                elif self.local_dir == 'LEFT':
                    cmd.twist.angular.z = -0.5
            elif self.local_dir == 'FRONT' and self.local_dist < 0.4:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.75
            elif self.v2v_caution:
                cmd.twist.linear.x = 0.08
                
            # If safe again, transition to recovery
            if self.local_dist >= 0.4 and not self.v2v_emergency and not self.v2v_caution:
                self.change_state(State.CONTINUE)
                
        elif self.state == State.CONTINUE:
            # Path recovered, transition back to standard movement
            self.change_state(State.MOVE)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()