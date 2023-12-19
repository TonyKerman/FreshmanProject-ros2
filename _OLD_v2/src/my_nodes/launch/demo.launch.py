# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'my_nodes'
    urdf_name = "mechanical-arm-v5.urdf"
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    """launch内容描述函数，由ros2 launch 扫描调用"""
    # action_01 = Node(
    #     package=package_name,
    #     executable="serial_node"
    # )
    # action_02 = Node(
    #     package=package_name,
    #     executable="tf2_node"
    # )
    joint_state_publisher_node = Node(
        package=package_name,
        executable='control_node',
        name='control_node',
        arguments=[urdf_model_path]
        )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        output='screen',
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    #ld.add_action(action_01)
    #ld.add_action(action_02)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    return ld

