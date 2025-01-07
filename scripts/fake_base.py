# This Python file uses the following encoding: utf-8

import rclpy
from rclpy.node import Node

import rclpy.qos
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

import math

class FakeBaseNode(Node):
  def __init__(self):
    self.__TIMER_RATE = 0.001
    self.__CMD_VEL_TOPIC = "/elastoplastic_controller/cmd_vel"
    self.__ODOM_TOPIC = "/odom"

    super().__init__('fake_base_node')
    self.tf_pose_bcast = TransformBroadcaster(self, rclpy.qos.QoSProfile(
      durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
      history=rclpy.qos.HistoryPolicy.KEEP_LAST,
      depth=10
    ))

    self.cmd_vel_sub = self.create_subscription(Twist, self.__CMD_VEL_TOPIC, self.update_state, 10)
    self.odom_pub = self.create_publisher(Odometry, self.__ODOM_TOPIC, 10)
    self.timer_odom = self.create_timer(self.__TIMER_RATE, self.timer_callback)
    self.x = 0.0
    self.y = 0.0
    self.rz = 0.0
    self.twist_last_msg = Twist()

    transform_stamped: TransformStamped = TransformStamped()
    transform_stamped.header.frame_id = 'map'
    transform_stamped.child_frame_id = 'odom'
    transform_stamped.transform.translation.x = 0.0
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0
    transform_stamped.transform.rotation.x = 0.0
    transform_stamped.transform.rotation.y = 0.0
    transform_stamped.transform.rotation.z = 0.0
    transform_stamped.transform.rotation.w = 1.0
    self.tf_pose_bcast.sendTransform(transform_stamped)

    self.time = 0
    self.start = False

  def update_state(self, msg: Twist):
    self.twist_last_msg = msg

  def timer_callback(self):

    self.x += self.twist_last_msg.linear.x *   self.__TIMER_RATE
    self.y += self.twist_last_msg.linear.y *   self.__TIMER_RATE
    self.rz += self.twist_last_msg.angular.z * self.__TIMER_RATE

    self.time = self.get_clock().now().nanoseconds

    odom = Odometry()
    odom.twist.twist = self.twist_last_msg
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.orientation.w = math.cos(self.rz/2.0)
    odom.pose.pose.orientation.z = math.sin(self.rz/2.0)
    header = Header()
    header.frame_id = 'odom'
    header.stamp = self.get_clock().now().to_msg()
    odom.header = header

    transform_stamped: TransformStamped = TransformStamped()
    transform_stamped.header.stamp = self.get_clock().now().to_msg()
    transform_stamped.header.frame_id = 'map'
    transform_stamped.child_frame_id = 'odom'
    transform_stamped.transform.translation.x = 0.0
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0
    transform_stamped.transform.rotation.x = 0.0
    transform_stamped.transform.rotation.y = 0.0
    transform_stamped.transform.rotation.z = 0.0
    transform_stamped.transform.rotation.w = 1.0
    self.tf_pose_bcast.sendTransform(transform_stamped)

    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = self.get_clock().now().to_msg()
    transform_stamped.header.frame_id = 'odom'
    transform_stamped.child_frame_id = 'demo/base_footprint'
    transform_stamped.transform.translation.x = self.x
    transform_stamped.transform.translation.y = self.y
    transform_stamped.transform.translation.z = 0.0
    transform_stamped.transform.rotation.x = 0.0
    transform_stamped.transform.rotation.y = 0.0
    transform_stamped.transform.rotation.z = odom.pose.pose.orientation.z
    transform_stamped.transform.rotation.w = odom.pose.pose.orientation.w

    self.odom_pub.publish(odom)
    self.tf_pose_bcast.sendTransform(transform_stamped)


def main():
  rclpy.init()
  fake_base_node = FakeBaseNode()
  fake_base_node.get_logger().info(f"Starting fake base");
  rclpy.spin(fake_base_node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
