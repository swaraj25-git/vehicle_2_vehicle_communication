import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from v2v_interfaces.msg import VehicleStatus


class VehicleCommNode(Node):
    def __init__(self):
        super().__init__('v2v_vehicle_comm')
        
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'unknown_robot'
            
        # Declare QoS Mode Parameter (normal, obstacle, emergency)
        self.declare_parameter('qos_mode', 'normal')
        self.current_mode = self.get_parameter('qos_mode').value
        
        # Add parameter callback to change QoS dynamically
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.status_pub = None
        self.status_sub = None
        
        # Apply the initial QoS Profile
        self.setup_qos_profiles(self.current_mode)
        
        # Timer to broadcast status at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'{self.robot_id} V2V node initialized in {self.current_mode.upper()} mode.')

    def setup_qos_profiles(self, mode):
        # Destroy existing pub/sub if they exist to re-create them with new QoS
        if self.status_pub is not None:
            self.destroy_publisher(self.status_pub)
        if self.status_sub is not None:
            self.destroy_subscription(self.status_sub)
            
        # Define the QoS Profile based on the mode
        if mode == 'emergency':
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        elif mode == 'obstacle':
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else: # normal
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            
        self.status_pub = self.create_publisher(VehicleStatus, '/vehicle_status', qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus, 
            '/vehicle_status', 
            self.status_callback, 
            qos_profile
        )
        self.get_logger().info(f'QoS Profile switched to: {mode.upper()}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'qos_mode':
                new_mode = param.value
                if new_mode in ['normal', 'obstacle', 'emergency']:
                    self.current_mode = new_mode
                    self.setup_qos_profiles(self.current_mode)
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().warn('Invalid QoS mode. Use normal, obstacle, or emergency.')
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def publish_status(self):
        msg = VehicleStatus()
        msg.robot_id = self.robot_id
        msg.obstacle_id = 0
        msg.distance = 0.0
        msg.angle = 0.0
        msg.velocity = 0.0
        msg.confidence = 0.0
        msg.timestamp = time.time()
        
        # Adjust priority based on mode
        if self.current_mode == 'emergency':
            msg.priority = 3
        elif self.current_mode == 'obstacle':
            msg.priority = 2
        else:
            msg.priority = 1
            
        self.status_pub.publish(msg)

    def status_callback(self, msg: VehicleStatus):
        if msg.robot_id != self.robot_id:
            self.get_logger().info(f'Received Prio {msg.priority} update from {msg.robot_id}.')


def main(args=None):
    rclpy.init(args=args)
    node = VehicleCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()