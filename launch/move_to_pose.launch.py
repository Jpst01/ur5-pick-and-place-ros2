from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():

    ur_type = "ur5"

    robot_description_content = Command(
        [
            PathJoinSubstitution(
                [FindExecutable(name="xacro")],
            ),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur_description"),
                 "urdf", "ur.urdf.xacro"
            ]),
            " ", "ur_type:=" + ur_type,
            " ", "name:=ur",
            " ", "prefix:=",
            " ", "use_fake_hardware:=false",
            " ", "sim_gazebo:=true",
        ]
    )

    robot_description_semantic_content = Command([
        PathJoinSubstitution(
            [FindExecutable(name="xacro")],
        ),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"),
             "srdf", "ur.srdf.xacro"
        ]),
        " ", "name:=ur",
        " ", "prefix:=",
    ])

    kinematics_yaml_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "kinematics.yaml"
    )
    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)
    
    ompl_yaml_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "ompl_planning.yaml"
    )
    with open(ompl_yaml_path, "r") as f:
        ompl_yaml = yaml.safe_load(f)

    joint_limits_yaml_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "joint_limits.yaml"
    )
    with open(joint_limits_yaml_path, "r") as f:
        joint_limits_yaml = yaml.safe_load(f)

    controllers_yaml_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "moveit_controllers.yaml"
    )
    with open(controllers_yaml_path, "r") as f:
        controllers_yaml = yaml.safe_load(f)

    move_node = Node(
        package='ur5_controller',
        executable='move_to_pose',
        output='screen',
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            {"robot_description_kinematics": kinematics_yaml},
            {"planning_pipelines": {"pipeline_names": ["ompl"]}},
            {"ompl": ompl_yaml},
            {"robot_description_planning": joint_limits_yaml},  
            controllers_yaml,
            {"plan_request_params": {
                "planning_pipeline": "ompl",
                "planning_id": "RRTConnectkConfigDefault",
                "planning_time": 5.0,
                "planning_attempts": 10,
                "max_velocity_scaling_factor": 1.0,
                "max_acceleration_scaling_factor": 1.0,
            }},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_node,
    ])