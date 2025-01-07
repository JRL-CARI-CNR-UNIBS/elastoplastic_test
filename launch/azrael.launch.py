from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction, TimerAction

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  return LaunchDescription([OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  demo_moveit = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"launch","demo_moveit.launch.py"])
    ]),
    launch_arguments={
      "robot_ip" : "192.168.254.31"
    }.items()
  )

  demo_ur_controllers = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"launch","demo_ur_controllers.launch.py"])
    ])
  )

  demo_mobile_base = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"launch","demo_mobile_base.launch.py"])
    ])
  )

  return [
    demo_moveit,
    # demo_mobile_base,
    TimerAction(
      period=3.0,
      actions=[demo_ur_controllers],
    )
  ]
