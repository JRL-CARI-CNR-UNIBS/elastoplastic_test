import geometry_msgs.msg
import rclpy
import rclpy.duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation as R
import geometry_msgs
from math import pi as PI
from math import sin

class ExampleTrj(rclpy.node.Node):
  def __init__(self):
    super().__init__('example_trj_node')

    self.buffer = Buffer()
    self.listener = TransformListener(self.buffer, self, spin_thread=True)
    self.base_tf = 'base_footprint'
    self.cmd_vel_pub = self.create_publisher(geometry_msgs.msg.Twist, '/target_cmd_vel', 10)
    self.t = 0.0
    self.dt = 0.01

  def compute_and_send_cmd(self):
    # while not self.buffer.can_transform(self.base_tf, 'map', rclpy.time.Time()):
    #   start = self.buffer.lookup_transform(self.base_tf)
    #   self.get_clock().sleep_for(rclpy.duration.Duration(nanoseconds=1e8))
    v_amp = 0.2
    twist_msg = geometry_msgs.msg.Twist()
    twist_msg.linear.x = v_amp * sin(2 * PI * 0.5 * self.t) # Lateral
    self.t = self.t + self.dt
    #twist_msg.linear.y = v_amp # Forward
    #twist_msg.angular.z = v_amp
    self.cmd_vel_pub.publish(twist_msg)

  def loop_cmd(self):
    while True:
      self.compute_and_send_cmd()
      self.get_clock().sleep_for(rclpy.duration.Duration(nanoseconds=1e7))


if __name__ == '__main__':
  rclpy.init()
  node = ExampleTrj()
  node.loop_cmd()
  rclpy.shutdown()
  exit(0)
