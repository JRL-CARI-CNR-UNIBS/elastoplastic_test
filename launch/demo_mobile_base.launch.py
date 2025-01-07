#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

# IMPORTANT: Unusable

def generate_launch_description():

    azrael_driver_udp = Node(
        package="azrael_driver_udp",
        namespace='azrael',
        executable="azrael_driver_udp_node",
        output="log")


    sick = Node(
            package='sick_scan',
            executable='sick_generic_caller',
            namespace='azrael',
            output='screen',
            remappings= [('/azrael/sick_lms_1xx/scan', '/azrael/scan')],
            parameters=
            [{"intensity"                           : False},
            {"intensity_resolution_16bit"           : False},
            {"min_ang"                              : -2.35619},
            {"max_ang"                              : 2.35619},
            {"frame_id"                             :"azrael/laser"},
            {"use_binary_protocol"                  : True},
            {"scanner_type"                         :"sick_lms_1xx"},
            {"hostname"                             :"192.170.1.1"},
            {"cloud_topic"                          :"cloud"},
            {"port"                                 :"2112"},
            {"timelimit"                            : 5},
            {"min_intensity"                        : 0.0},
            {"use_generation_timestamp"             : True},
            {"range_min"                            : 0.05},
            {"range_max"                            : 25.0},
            {"scan_freq"                            : 50.0},
            {"ang_res"                              : 0.5},
            {"range_filter_handling"                : 0},
            {"add_transform_xyz_rpy"                : "0,0,0,0,0,0"},
            {"add_transform_check_dynamic_updates"  : False},
            {"start_services"                       : True},
            {"message_monitoring_enabled"           : True},
            {"read_timeout_millisec_default"        : 5000},
            {"read_timeout_millisec_startup"        : 120000},
            {"read_timeout_millisec_kill_node"      : 15000000},
            {"client_authorization_pw"              :"F4724744"},
            {"imu_enable"                           : False},
            {"ros_qos"                              : 4}])


    laser_throttle = Node(
            package='topic_tools',
            executable='throttle',
            namespace='azrael',
            parameters=[{
                'input_topic': 'scan',
                'throttle_type': 'messages',
                'msgs_per_sec': 2.0,
                'output_topic': 'scan_rviz'
            }],
            arguments=['messages scan 2 scan_rviz'],
            output='screen')

    nodes_to_start = [
        sick,
        laser_throttle,
        azrael_driver_udp,
    ]

    return LaunchDescription(nodes_to_start)
