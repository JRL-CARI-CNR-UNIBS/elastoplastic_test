from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  rviz_config = PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"config","config.rviz"])

  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="screen",
      arguments=["-d", rviz_config],
  )

  return LaunchDescription(rviz_node)