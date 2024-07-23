from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch



from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)




def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("stretch", package_name="my_stretch_moveit").to_moveit_configs()
    # TODO replace these with my trivel file and lines!

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             str(moveit_config.package_path / "launch/rsp.launch.py")
    #         ),
    #     )
    # )

    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)



    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             str(moveit_config.package_path / "launch/move_group.launch.py")
    #         ),
    #     )
    # )
    # a = {                'robot_description_semantic': moveit_config.robot_description_semantic,}

    # raise Exception(f"Content: \n {moveit_config.to_dict()} ")

    # ld.add_action(Node(package="robot_state_publisher", executable=f))

    robot_params = {}
    robot_params.update(moveit_config.robot_description_semantic)
    robot_params.update(moveit_config.trajectory_execution)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # {
            #     'robot_description_semantic': moveit_config.robot_description_semantic,

            # },
            # robot_params,
            # Each filed of moveit_config is a dict, with correct param name in the key
            # moveit_config.robot_description_semantic,
            # moveit_config.trajectory_execution,
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": False,
                "publish_transforms_updates": False,
            }
        ],
    )
    ld.add_action(move_group_node)




    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        # We don't provide this, expects it to get it from topics.
        # moveit_config.robot_description_kinematics,
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    ld.add_action(rviz_node)


    # # Fake joint driver
    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="ros2_control_node",
    #         parameters=[
    #             moveit_config.robot_description,
    #             str(moveit_config.package_path / "config/ros2_controllers.yaml"),
    #         ],
    #     )
    # )


    # # The moveit_config.trajectory_execution is a dict that loads in the content of `moveit_controller.yaml`
    # controller_names = moveit_config.trajectory_execution.get(
    #     "moveit_simple_controller_manager", {}
    # ).get("controller_names", [])
    # for controller in controller_names + ["joint_state_broadcaster"]:
    #     ld.add_action(
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=[controller],
    #             output="screen",
    #         )
    #     )


    return ld
