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
                    ('image_raw', '//camera/image_raw'),
                    ('left_image', '/left_image'),
                    ('right_image', '/right_image'),
                ]
            ),
            # ComposableNode(
            #     package='stereo_image_processor',
            #     plugin='StitchedImagePublisher',
            #     name='stitched_image_publisher',
            #     extra_arguments=[{'use_intra_process_comms': True}],
            #     parameters=[{'image_topic': '/stitched_images'}],
            # )
        ],
    )

    return LaunchDescription([container])
