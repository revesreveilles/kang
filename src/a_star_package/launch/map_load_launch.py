import os
import yaml
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    ld =LaunchDescription()
    package_share_dir =get_package_share_directory("search_base_ag")
    #map file
    map_file_path = os.path.join(
        get_package_share_directory("search_base_ag"),
        "maps",
        "maze.yaml"
    )
    #rviz2
    rviz_config_path =os.path.join(package_share_dir,'rviz','default.rviz')
    rviz2_cmd =Node(
        package="rviz2",
        executable="rviz2",
        output ="screen",
        arguments=['-d',rviz_config_path]
    )
    ld.add_action(rviz2_cmd)
    #map server
    map_server_cmd =Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file_path}]
    )

    lifecycle_nodes =["map_server"]
    use_sim_time =True
    autostart = True

    start_lifecycle_manager_cmd =launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes}
        ]
    )
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    return ld