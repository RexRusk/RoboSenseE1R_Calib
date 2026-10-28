import os
import pwd
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _setup(context):
    bringup_share = get_package_share_directory('e1r_calib_bringup')
    config_path = os.path.join(bringup_share, 'config', 'e1r_online.yaml')
    frame = LaunchConfiguration('frame').perform(context).strip().lower()
    use_world = frame == 'world'
    rviz_name = 'e1r_world.rviz' if use_world else 'e1r.rviz'
    rviz_config = os.path.join(bringup_share, 'rviz', rviz_name)

    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_camera = LaunchConfiguration('use_camera').perform(context)
    record = LaunchConfiguration('record').perform(context)
    video_device = LaunchConfiguration('video_device').perform(context)
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context).strip()
    camera_transport = LaunchConfiguration('camera_transport').perform(context).strip()
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    capture_width = int(LaunchConfiguration('capture_width').perform(context))
    capture_height = int(LaunchConfiguration('capture_height').perform(context))
    pixel_format = LaunchConfiguration('pixel_format').perform(context)
    jpeg_path = LaunchConfiguration('jpeg_path').perform(context)
    win_python = LaunchConfiguration('win_python').perform(context)
    win_device = LaunchConfiguration('win_device').perform(context)

    actions = [
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            output='screen',
            parameters=[{'config_path': config_path}],
        ),
        # Gives the rslidar frame a TF parent so RViz Global Status can leave Warn.
        # The cloud is already in this frame; the transform is identity.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'rslidar',
                '--child-frame-id', 'rslidar_link',
            ],
        ),
    ]

    if use_rviz.lower() in ('1', 'true', 'True'):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ))

    home = pwd.getpwuid(os.getuid()).pw_dir
    bringup_scripts = os.path.join(bringup_share, 'scripts')
    repo_root = LaunchConfiguration('repo_root').perform(context).strip()
    actions.append(ExecuteProcess(
        cmd=[
            os.path.join(home, 'e1r_calib', 'venv', 'bin', 'python'),
            os.path.join(bringup_scripts, 'ground_height.py'),
        ],
        output='screen',
    ))
    if use_world:
        actions.append(ExecuteProcess(
            cmd=[
                os.path.join(home, 'e1r_calib', 'venv', 'bin', 'python'),
                os.path.join(bringup_scripts, 'axis_frame.py'),
            ],
            output='screen',
        ))

    if use_camera.lower() in ('1', 'true', 'True'):
        packaged_yaml = os.path.join(
            bringup_share, 'config', 'usb_camera_1024x576.yaml')
        info_url = camera_info_url or ('file://' + packaged_yaml)
        if camera_transport == 'v4l2':
            # A later camera that has a clean native mode in WSL. image_width
            # and image_height must be a mode that camera actually enumerates,
            # and camera_info_url must be solved at that same size.
            camera_params = {
                'video_device': video_device,
                'image_size': [image_width, image_height],
                'pixel_format': pixel_format,
            }
            if info_url:
                camera_params['camera_info_url'] = info_url
            actions.append(Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                namespace='camera',
                output='screen',
                parameters=[camera_params],
            ))
        else:
            # This Etron sensor has no 1024x576 mode. 1280x720 is the 16:9
            # mode the Zhang photos came from (scaled by 0.8). USB/IP tears
            # that JPEG, so Windows writes a complete scaled frame and this
            # process publishes it with the matching CameraInfo.
            home = pwd.getpwuid(os.getuid()).pw_dir
            if not repo_root:
                raise RuntimeError(
                    'camera_transport:=win_scale needs repo_root:=/path/to/this/repo')
            ps1 = _windows_path(repo_root.rstrip('/\\') + '/scripts/start_win_cam.ps1')
            actions.append(ExecuteProcess(
                cmd=[
                    'powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                    '-File', ps1,
                    '-CaptureWidth', str(capture_width),
                    '-CaptureHeight', str(capture_height),
                    '-Width', str(image_width),
                    '-Height', str(image_height),
                    '-Out', _windows_path(jpeg_path),
                    '-Device', win_device,
                ] + (['-Python', win_python] if win_python else []),
                output='screen',
            ))
            actions.append(ExecuteProcess(
                cmd=[
                    os.path.join(home, 'e1r_calib', 'venv', 'bin', 'python'),
                    os.path.join(bringup_scripts, 'cam_file_pub.py'),
                    '--ros-args',
                    '-p', 'jpeg_path:=' + jpeg_path,
                    '-p', 'camera_info_url:=' + info_url,
                ],
                output='screen',
            ))
        home = pwd.getpwuid(os.getuid()).pw_dir
        extrinsic_yaml = os.path.join(bringup_share, 'config', 'lidar_to_camera.yaml')
        actions.append(ExecuteProcess(
            cmd=[
                os.path.join(home, 'e1r_calib', 'venv', 'bin', 'python'),
                os.path.join(bringup_scripts, 'colorize_cloud.py'),
                '--ros-args',
                '-p', 'extrinsic_yaml:=' + extrinsic_yaml,
                '-p', 'camera_yaml:=' + packaged_yaml,
            ],
            output='screen',
        ))

    if record.lower() in ('1', 'true', 'True'):
        home = pwd.getpwuid(os.getuid()).pw_dir
        bag_root = os.path.join(home, 'e1r_calib', 'bags')
        os.makedirs(bag_root, exist_ok=True)
        bag_path = os.path.join(
            bag_root, datetime.now().strftime('calib_%Y%m%d_%H%M%S'))
        topics = ['/rslidar_points']
        if use_camera.lower() in ('1', 'true', 'True'):
            topics.extend(['/camera/image_raw', '/camera/camera_info'])
        actions.append(TimerAction(
            period=3.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_path, *topics],
                output='screen',
            )],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # WSL mirrored networking drops DDS multicast. Keep discovery on localhost
        # so rviz and the driver in this launch can see /rslidar_points.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'frame',
            default_value='lidar',
            description='lidar draws /rslidar_points in the lidar frame. world draws /rslidar_points_world on the ground plane.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camera_transport',
            default_value='v4l2',
            description='v4l2 opens video_device on Linux. win_scale reads the camera on Windows and needs repo_root.'),
        DeclareLaunchArgument('image_width', default_value='1024'),
        DeclareLaunchArgument('image_height', default_value='576'),
        DeclareLaunchArgument('capture_width', default_value='1280'),
        DeclareLaunchArgument('capture_height', default_value='720'),
        DeclareLaunchArgument('pixel_format', default_value='MJPG'),
        DeclareLaunchArgument(
            'repo_root',
            default_value='',
            description='Checkout path. Required only for camera_transport:=win_scale.'),
        DeclareLaunchArgument(
            'jpeg_path',
            default_value='/tmp/e1r_live.jpg'),
        DeclareLaunchArgument(
            'win_python',
            default_value='',
            description='Windows Python for win_scale. Empty uses the default in start_win_cam.ps1.'),
        DeclareLaunchArgument(
            'win_device',
            default_value='0',
            description='Windows camera index for win_scale.'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='',
            description='camera_info yaml URL. Empty uses the packaged 1024x576 intrinsics.'),
        OpaqueFunction(function=_setup),
    ])


def _windows_path(path):
    if path.startswith('/mnt/') and len(path) > 6 and path[6] == '/':
        return path[5].upper() + ':' + path[6:].replace('/', '\\')
    return path
