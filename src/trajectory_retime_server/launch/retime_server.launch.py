#!/usr/bin/env python3

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    robot_description = LaunchConfiguration("robot_description").perform(context).strip()
    robot_description_semantic = LaunchConfiguration(
        "robot_description_semantic"
    ).perform(context).strip()
    kinematics_yaml = LaunchConfiguration(
        "robot_description_kinematics"
    ).perform(context).strip()
    service_name = LaunchConfiguration("service_name").perform(context).strip()

    if not robot_description:
        raise RuntimeError("robot_description must be injected as URDF XML")
    if not robot_description_semantic:
        raise RuntimeError("robot_description_semantic must be injected as SRDF XML")
    if not kinematics_yaml:
        raise RuntimeError("robot_description_kinematics must be injected as YAML")

    robot_description_kinematics = yaml.safe_load(kinematics_yaml)
    if not isinstance(robot_description_kinematics, dict) or not robot_description_kinematics:
        raise RuntimeError("robot_description_kinematics must decode to a non-empty mapping")
    if not service_name:
        raise RuntimeError("service_name must not be empty")

    return [
        Node(
            package="trajectory_retime_server",
            executable="retime_server",
            name="trajectory_retime_server",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": robot_description_semantic},
                {"robot_description_kinematics": robot_description_kinematics},
                {"service_name": service_name},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_description",
                default_value="",
                description="URDF XML injected by the robot-specific parent launch",
            ),
            DeclareLaunchArgument(
                "robot_description_semantic",
                default_value="",
                description="SRDF XML injected by the robot-specific parent launch",
            ),
            DeclareLaunchArgument(
                "robot_description_kinematics",
                default_value="",
                description="MoveIt kinematics YAML injected by the parent launch",
            ),
            DeclareLaunchArgument(
                "service_name",
                default_value="/retime_trajectory",
                description="ROS service name exposed by the retime server",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
