from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # launchの構成を示すLaunchDescription型の変数の定義
    ld = LaunchDescription()

    namespace = "drone"

    nodes = [
        Node(
            package='pressure',
            executable='pressure_subscriber',
            name='pressure',
            namespace=namespace,
            remappings=[('image_topic', 'pressure_image')]
        ),

        Node(
            package='scan_qr',
            executable='listener',
            name='listener',
            namespace=namespace,
            remappings=[
                ('image_raw', 'qr_image'),
                ('qr_image', 'qr_result_image'),
                ("qr_value", "qr_result_value")
            ]
        ),

        Node(
            package='manual_package',
            executable='manual_publisher',
            name='qr_manual',
            namespace=namespace,
            remappings=[('manual_input_qr', 'qr_image_manual'),('text_topic','qr_result_value')]
        ),

        Node(
            package='drone_distribute_image',
            executable='drone_distribute_image',
            name='drone_distribute_image',
            namespace=namespace,
            parameters=[{"target_dir":"/home/ros/ros2_ws/drone_data"}], # 参照するディレクトリ
        ),

        Node(
            package='crack',
            executable='crack_subscriber',
            name='crack',
            namespace=namespace,
            remappings=[('image_raw', 'crack_image')]
        ),

        Node(
            package='digital_twin_client',
            executable='digital_twin_client',
            namespace=namespace,
            remappings=[('send_image', 'send_topic')],
            parameters=[{"host": "", "robot_id": ""}] # サーバでの値を設定する
        ),
        Node(
            package='photo',
            executable='photo_sub_pub',
            name='photo',
            namespace=namespace,
            remappings=[('input_photo_image', 'situation_image')]
        ),

        Node(
            package='bulb',
            executable='bulb_param',
            name='bulb',
            namespace=namespace,
            remappings=[('input_bulb_image', 'bulb_image')]
        )

    ]

    # LaunchDescriptionに、起動したいノードを追加する
    for node in nodes:
        ld.add_action(node)

    # launch構成を返すようにする
    return ld
