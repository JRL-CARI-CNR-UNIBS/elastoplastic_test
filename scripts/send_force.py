# This Python file uses the following encoding: utf-8

import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.logging
from rclpy.node import Node
import rclpy.timer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import geometry_msgs

from scipy.spatial.transform import Rotation as R
import numpy as np

import argparse
import time

class SendForce(Node):
  def __init__(self):
    super().__init__('send_force')
    self.pub = self.create_publisher(geometry_msgs.msg.WrenchStamped, '/io_and_status_controller/fake_wrench', 1)
    self.buffer = Buffer()
    self.tflistener = TransformListener(self.buffer, self, spin_thread=True)
    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

  def send_force(self, ref_frame: str, wrench_in_ref: list[float]):
    self.get_logger().info("PRE")
    tf = self.buffer.lookup_transform(ref_frame, 'az_robotiq_ft_frame_id', rclpy.time.Time(), rclpy.duration.Duration(seconds=5))
    self.get_logger().info("POST")
    R_sensor_ref = R.from_quat([tf.transform.rotation.x,
                                tf.transform.rotation.y,
                                tf.transform.rotation.z,
                                tf.transform.rotation.w])
    wrench_in_sensor: list[float] = R_sensor_ref.apply(wrench_in_ref[0:3])
    wrench_in_sensor = np.append(wrench_in_sensor, R_sensor_ref.apply(wrench_in_ref[3:6]))
    msg = geometry_msgs.msg.WrenchStamped()
    msg.wrench.force.x = wrench_in_sensor[0]
    msg.wrench.force.y = wrench_in_sensor[1]
    msg.wrench.force.z = wrench_in_sensor[2]
    msg.wrench.torque.x = wrench_in_sensor[3]
    msg.wrench.torque.y = wrench_in_sensor[4]
    msg.wrench.torque.z = wrench_in_sensor[5]
    def repeat_send():
      self.get_logger().info("sending")
      self.pub.publish(msg)
    self.timer = self.create_timer(0.1, repeat_send)

if __name__ == '__main__':
  rclpy.init()
  node = SendForce()
  parser = argparse.ArgumentParser()
  parser.add_argument('ref_frame')
  parser.add_argument('-w', '--wrench_in_ref', nargs=6, metavar=('fx', 'fy', 'fz', 'tx','ty','tz'), required=True, type=float, default=None)
  args = parser.parse_args()
  print(args)
  node.send_force(args.ref_frame, args.wrench_in_ref)
  rclpy.spin(node)
  rclpy.shutdown()
  exit(0)

