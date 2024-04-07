from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'width', default_value='1280',
            description='libcamera image width'),
        DeclareLaunchArgument(
            'height', default_value='400',
            description='libcamera image height'),
        DeclareLaunchArgument(
            'pixelformat', default_value='XBGR8888',
            description='libcamera pixel format'),
        DeclareLaunchArgument(
            'contrast', default_value='1.0',
            description='libcamera image contrast'),
        DeclareLaunchArgument(
            'exposure_time', default_value='0',
            description='libcamera image ExposureTime between 0...13'),
        DeclareLaunchArgument(
            'enable_exposure', default_value='True',
            description='enable/disable auto expousre'),
        # Add more arguments as needed
        
    ]

    container = ComposableNodeContainer(
        name='stereo_image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_splitter_ros',
                plugin='StereoImageSplitter',
                name='stereo_image_splitter',
                parameters=[
                    {'is_grey': True},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('stitched_images', '/camera/image_raw'),
                    ('left_image', '/left_image'),
                    ('right_image', '/right_image'),
                ]
            ),
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{'format': LaunchConfiguration('pixelformat')},
                            {'width': LaunchConfiguration('width')},
                            {'height': LaunchConfiguration('height')},
                            {'AeEnable': LaunchConfiguration('enable_exposure')},
                            {'Contrast': LaunchConfiguration('contrast') },
                            {'ExposureValue': 0.0},
                            {'ExposureTime': LaunchConfiguration('exposure_time')},
                            {'FrameDurationLimits': [16667,16667]}, #60fps
                ]
            )
        ],
    )

    return LaunchDescription(launch_args + [container])
