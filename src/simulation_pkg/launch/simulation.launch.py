from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bot_sim = Node(
        package="simulation_pkg",
        executable="bot_simulation",
        name="simulation_node",
        output="screen"
    )

    target_gui = Node(
        package="simulation_pkg",
        executable="target_gui",
        name="target_gui_node",
        output="screen"
    )

    controller = Node(
        package="o1",
        executable="controller1",
        name="controller_cpp_node",
        output="screen"
    )

    motion_planner = Node(
        package="o1",
        executable="main1",
        name="motion_planner_cpp_node",
        output="screen"
    )

    return LaunchDescription([
        bot_sim,
        target_gui,
        controller,
        motion_planner,
    ])
