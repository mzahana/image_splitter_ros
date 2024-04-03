from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
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
                parameters=[{'format': 'YUYV'},
                            {'width': 1280},
                            {'height': 400},
                            {'AeEnable': False},
                            {'Contrast': 1.5 },
                            {'ExposureValue': 0.0},
                            {'is_grey': True}]
            )
        ],
    )

    return LaunchDescription([container])
