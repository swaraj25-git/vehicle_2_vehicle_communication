import time
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from v2v_interfaces.msg import ObstacleInfo, VehicleStatus


class NavigationNode(Node):

  def __init__(self):
    super().__init__('v2v_navigation')

    self.robot_id = self.get_namespace().strip('/')
    if not self.robot_id:
      self.robot_id = 'unknown_robot'

    # Subscriptions
    self.obs_sub = self.create_subscription(
        ObstacleInfo, 'obstacle_info', self.local_obstacle_callback, 10
    )
    self.v2v_sub = self.create_subscription(
        VehicleStatus, '/vehicle_status', self.v2v_callback, 10
    )

    # Publisher for driving commands
    self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

    # State Variables
    self.local_dist = float('inf')
    self.local_dir = 'FRONT'
    self.v2v_emergency = False
    self.v2v_caution = False
    self.last_v2v_time = 0.0

    # Adjusted Thresholds for TurtleBot3 World (meters)
    self.critical_dist = 0.28  # Local Emergency Stop
    self.avoid_dist = 0.50  # Obstacle Avoidance Trigger

    # Control Loop at 10 Hz
    self.timer = self.create_timer(0.1, self.control_loop)
    self.get_logger().info(
        f'{self.robot_id} Cooperative Navigation initialized.'
    )

  def local_obstacle_callback(self, msg: ObstacleInfo):
    self.local_dist = msg.distance
    self.local_dir = msg.direction

  def v2v_callback(self, msg: VehicleStatus):
    if msg.robot_id != self.robot_id:
      current_time = time.time()
      self.last_v2v_time = current_time

      if msg.priority == 3:
        self.v2v_emergency = True
        self.v2v_caution = False
      elif msg.priority == 2:
        self.v2v_caution = True
        self.v2v_emergency = False
      else:
        self.v2v_emergency = False
        self.v2v_caution = False

  def control_loop(self):
    cmd = TwistStamped()
    cmd.header.stamp = self.get_clock().now().to_msg()
    cmd.header.frame_id = f'{self.robot_id}/base_footprint'
    current_time = time.time()

    if current_time - self.last_v2v_time > 3.0:
      self.v2v_emergency = False
      self.v2v_caution = False

    # 1. EMERGENCY STOP & PATH RECOVERY (< 0.25m)
    if self.local_dist < 0.25 or self.v2v_emergency:
      if self.v2v_emergency:
        # A network emergency means freeze completely
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.get_logger().warn('V2V EMERGENCY STOP!', throttle_duration_sec=2.0)
      else:
        # Local emergency: Path Recovery Maneuver
        self.get_logger().warn(f'CRITICAL DISTANCE ({self.local_dist:.2f}m) on {self.local_dir}! Recovering...', throttle_duration_sec=1.5)
        
        # Wiggle out of danger based on where the obstacle is
        if self.local_dir == 'FRONT':
          cmd.twist.linear.x = -0.05  # Back up slowly
          cmd.twist.angular.z = 0.0
        elif self.local_dir == 'BACK':
          cmd.twist.linear.x = 0.05   # Creep forward
          cmd.twist.angular.z = 0.0
        elif self.local_dir == 'RIGHT':
          cmd.twist.linear.x = 0.0
          cmd.twist.angular.z = 0.5   # Spin left away from wall
        elif self.local_dir == 'LEFT':
          cmd.twist.linear.x = 0.0
          cmd.twist.angular.z = -0.5  # Spin right away from wall

    # 2. OBSTACLE AVOIDANCE (Only pivot if FRONT is blocked < 0.4m)
    elif self.local_dir == 'FRONT' and self.local_dist < 0.4:
      cmd.twist.linear.x = 0.0      
      cmd.twist.angular.z = 0.75    
      self.get_logger().info(f'Front blocked ({self.local_dist:.2f}m). Pivoting.', throttle_duration_sec=1.5)
      


    # 3. NUDGE (If walls on sides get too close < 0.3m, gently steer away)
    elif self.local_dir == 'RIGHT' and self.local_dist < 0.3:
      cmd.twist.linear.x = 0.1
      cmd.twist.angular.z = 0.5     
    elif self.local_dir == 'LEFT' and self.local_dist < 0.3:
      cmd.twist.linear.x = 0.1
      cmd.twist.angular.z = -0.5    

    # 4. NORMAL CRUISING
    elif self.v2v_caution:
      cmd.twist.linear.x = 0.08
    else:
      cmd.twist.linear.x = 0.2

    self.cmd_pub.publish(cmd)

  # def control_loop(self):
  #   cmd = TwistStamped()
  #   cmd.header.stamp = self.get_clock().now().to_msg()
  #   cmd.header.frame_id = f'{self.robot_id}/base_footprint'

  #   current_time = time.time()

  #   # Clear stale V2V states after 3 seconds
  #   if current_time - self.last_v2v_time > 3.0:
  #     self.v2v_emergency = False
  #     self.v2v_caution = False

  #   # 1. EMERGENCY STOP (Local collision danger or V2V Priority 3)
  #   if self.local_dist < self.critical_dist or self.v2v_emergency:
  #     cmd.twist.linear.x = 0.0
  #     cmd.twist.angular.z = 0.0
  #     if self.v2v_emergency:
  #       self.get_logger().info(
  #           'V2V EMERGENCY STOP!', throttle_duration_sec=2.0
  #       )
  #     else:
  #       self.get_logger().warn(
  #           'LOCAL EMERGENCY STOP!', throttle_duration_sec=2.0
  #       )

  #   # 2. OBSTACLE AVOIDANCE (Only pivot when obstacles block the path ahead)
  #   elif self.local_dist < self.avoid_dist and self.local_dir in [
  #       'FRONT',
  #       'RIGHT',
  #       'LEFT',
  #   ]:
  #     cmd.twist.linear.x = 0.0  # Stop forward motion while pivoting

  #     if self.local_dir in ['FRONT', 'RIGHT']:
  #       cmd.twist.angular.z = 0.7  # Pivot left
  #     elif self.local_dir == 'LEFT':
  #       cmd.twist.angular.z = -0.7  # Pivot right

  #     self.get_logger().info(
  #         f'Pivoting away from {self.local_dir} obstacle ({self.local_dist:.2f}m)',
  #         throttle_duration_sec=1.5,
  #     )

  #   # 3. SHARED DECISION MAKING (V2V Caution)
  #   elif self.v2v_caution:
  #     cmd.twist.linear.x = 0.08
  #     cmd.twist.angular.z = 0.0
  #     self.get_logger().info(
  #         'V2V CAUTION: Slow cruising.', throttle_duration_sec=2.0
  #     )

  #   # 4. NORMAL CRUISING (Path forward is clear)
  #   else:
  #     cmd.twist.linear.x = 0.18
  #     cmd.twist.angular.z = 0.0

  #   self.cmd_pub.publish(cmd)


def main(args=None):
  rclpy.init(args=args)
  node = NavigationNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()