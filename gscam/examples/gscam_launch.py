from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device = LaunchConfiguration('device')
    v4l2_ctl_path = '/usr/bin/v4l2-ctl'  # Specify the full path to v4l2-ctl

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video60',
            description='Video device'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='10/1',  # Try lower frame rate
            description='Frame rate'
        ),
        DeclareLaunchArgument(
            'width',
            default_value='1280',
            description='Frame width'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='720',
            description='Frame height'
        ),
        DeclareLaunchArgument(
            'publish_frame',
            default_value='false',
            description='Whether to publish the frame'
        ),
        # Set auto focus to off
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--set-ctrl=focus_auto=0'],
            shell=True
        ),
        # Set auto brightness to off
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--set-ctrl=brightness_auto=0'],
            shell=True
        ),
        # Set absolute focus to 20
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--set-ctrl=focus_absolute=0'],
            shell=True
        ),
        # Set brightness to 128
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--set-ctrl=brightness=128'],
            shell=True
        ),
        # Verify the settings
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--get-ctrl=focus_auto'],
            shell=True
        ),
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--get-ctrl=brightness_auto'],
            shell=True
        ),
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--get-ctrl=focus_absolute'],
            shell=True
        ),
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device'), '--get-ctrl=brightness'],
            shell=True
        ),
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_driver_v4l',
            namespace='v4l',
            output='screen',
            parameters=[{
                'camera_name': 'default',
                'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                'gscam_config': [
                    'v4l2src device=', device,
                    ' ! image/jpeg,width=', LaunchConfiguration('width'),
                    ',height=', LaunchConfiguration('height'),
                    ',framerate=', LaunchConfiguration('fps'),
                    ' ! jpegdec ! videoconvert ! video/x-raw,format=RGB ! queue'
                ],
                'frame_id': '/v4l_frame',
                'sync_sink': True
            }],
            arguments=['--gst-debug=2']
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('publish_frame')),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='v4l_transform',
            arguments=['1', '2', '3', '0', '-3.141', '0', '/world', '/v4l_frame']
        )
    ])

