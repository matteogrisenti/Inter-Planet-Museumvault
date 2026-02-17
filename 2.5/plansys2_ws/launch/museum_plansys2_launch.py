import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'museum_plansys2'
    pkg_share = get_package_share_directory(pkg_name)

    # Launch PlanSys2 infrastructure
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl')
        }.items()
    )

    # Action nodes - one for each durative action in domain.pddl
    # Movement actions
    move_pressurized_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='move_to_pressurized_room_node',
        arguments=['move-to-pressurized-room', '10.0'],
        output='screen'
    )

    move_unpressurized_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='move_to_unpressurized_room_node',
        arguments=['move-to-unpressurized-room', '10.0'],
        output='screen'
    )

    # Sealing action
    activate_seal_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='activate_seal_node',
        arguments=['activate-seal', '2.0'],
        output='screen'
    )

    # Pod management actions
    pick_up_empty_pod_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='pick_up_empty_pod_slot_1_node',
        arguments=['pick-up-empty-pod-slot-1', '2.0'],
        output='screen'
    )

    pick_up_full_pod_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='pick_up_full_pod_slot_1_node',
        arguments=['pick-up-full-pod-slot-1', '4.0'],
        output='screen'
    )

    drop_pod_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='drop_pod_slot_1_node',
        arguments=['drop-pod-slot-1', '4.0'],
        output='screen'
    )

    # Artifact pickup actions
    pick_up_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='pick_up_slot_1_node',
        arguments=['pick-up-slot-1', '1.0'],
        output='screen'
    )

    put_in_pod_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='put_in_pod_slot_1_node',
        arguments=['put-in-pod-slot-1', '4.0'],
        output='screen'
    )

    # Artifact release actions
    release_artifact_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='release_artifact_slot_1_node',
        arguments=['release-artifact-slot-1', '1.0'],
        output='screen'
    )

    release_artifact_from_pod_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='release_artifact_from_pod_slot_1_node',
        arguments=['release-artifact-from-pod-slot-1', '4.0'],
        output='screen'
    )

    # Cooling actions
    cool_artifact_carrying_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='cool_artifact_while_carrying_slot_1_node',
        arguments=['cool-artifact-while-carrying-slot-1', '12.0'],
        output='screen'
    )

    cool_artifact_carrying_in_pod_slot1_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='cool_artifact_while_carrying_in_pod_slot_1_node',
        arguments=['cool-artifact-while-carrying-in-pod-slot-1', '14.0'],
        output='screen'
    )

    # Second slot actions (for technician)
    pick_up_slot2_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='pick_up_slot_2_node',
        arguments=['pick-up-slot-2', '2.0'],
        output='screen'
    )

    release_artifact_slot2_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='release_artifact_slot_2_node',
        arguments=['release-artifact-slot-2', '2.0'],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()
    
    # Add PlanSys2 infrastructure
    ld.add_action(plansys2_cmd)
    
    # Add all action nodes
    ld.add_action(move_pressurized_cmd)
    ld.add_action(move_unpressurized_cmd)
    ld.add_action(activate_seal_cmd)
    ld.add_action(pick_up_empty_pod_slot1_cmd)
    ld.add_action(pick_up_full_pod_slot1_cmd)
    ld.add_action(drop_pod_slot1_cmd)
    ld.add_action(pick_up_slot1_cmd)
    ld.add_action(put_in_pod_slot1_cmd)
    ld.add_action(release_artifact_slot1_cmd)
    ld.add_action(release_artifact_from_pod_slot1_cmd)
    ld.add_action(cool_artifact_carrying_slot1_cmd)
    ld.add_action(cool_artifact_carrying_in_pod_slot1_cmd)
    ld.add_action(pick_up_slot2_cmd)
    ld.add_action(release_artifact_slot2_cmd)

    return ld
