# This Python file uses the following encoding: utf-8

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import WrenchStamped

from scipy.spatial.transform import Rotation as R
import numpy as np

class SendForce(Node):
  def __init__(self):
    super().__init__('send_force')
    self.__timer_dt = 0.01

    self.__ref_frame: str = 'map'
    self.__sensor_frame: str = 'demo/robotiq_ft_frame_id'

    self.__timed_force_in_world = np.array([
      [2.0, 5.0, 2.0, 5.0   , 1.0], # Time
      [0.0, 15.0, 0.0, -15.0, 0.0], # Fx
      [0.0, 0.0, 0.0, 0.0   , 0.0], # Fy
      [0.0, 0.0, 0.0, 0.0   , 0.0], # Fz
    ])

    self.__time_cum = np.cumsum(self.__timed_force_in_world[0,:])
    self.pub = self.create_publisher(WrenchStamped, '/io_and_status_controller/fake_wrench', 1)
    self.buffer = Buffer()
    self.tflistener = TransformListener(self.buffer, self, spin_thread=True)
    self.timer = self.create_timer(timer_period_sec=self.__timer_dt, callback=self.send_force)
    self.time = 0.0
    self.get_clock().sleep_for(Duration(seconds=1, nanoseconds=0))

  def send_force(self):
    if self.__time_cum.size != 0 and self.time > self.__time_cum[0]:
      self.__time_cum = self.__time_cum[1:]
      self.__timed_force_in_world = self.__timed_force_in_world[:, 1:]
    if self.__timed_force_in_world.size == 0:
      return

    tf = self.buffer.lookup_transform(self.__sensor_frame, self.__ref_frame, Time(), Duration(seconds=5))
    R_sensor_ref = R.from_quat([tf.transform.rotation.x,
                                tf.transform.rotation.y,
                                tf.transform.rotation.z,
                                tf.transform.rotation.w])

    wrench_in_sensor = R_sensor_ref.apply(self.__timed_force_in_world[1:, 0])
    wrench_in_sensor = np.append(wrench_in_sensor, [0.0,0.0,0.0])
    msg = WrenchStamped()
    msg.header.frame_id = self.__sensor_frame
    msg.wrench.force.x =  float(wrench_in_sensor[0])
    msg.wrench.force.y =  float(wrench_in_sensor[1])
    msg.wrench.force.z =  float(wrench_in_sensor[2])
    msg.wrench.torque.x = float(wrench_in_sensor[3])
    msg.wrench.torque.y = float(wrench_in_sensor[4])
    msg.wrench.torque.z = float(wrench_in_sensor[5])
    self.pub.publish(msg)
    self.time = self.time + self.__timer_dt

if __name__ == '__main__':
  rclpy.init()
  node = SendForce()
  rclpy.spin(node)
  rclpy.shutdown()
  exit(0)
