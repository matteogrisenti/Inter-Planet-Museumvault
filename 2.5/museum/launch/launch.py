# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('museum')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl', # <----
          'namespace': namespace
          }.items())

    # Specify the actions
    fly_into_seismic_room = Node(
        package='museum',
        executable='fly_into_seismic_room',
        name='fly_into_seismic_room',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_to_pressurized_room = Node(
        package='museum',
        executable='move_to_pressurized_room',
        name='move_to_pressurized_room',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_to_pressurized_room2 = Node(
        package='museum',
        executable='move_to_pressurized_room',
        name='move_to_pressurized_room2',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_to_pressurized_room3 = Node(
        package='museum',
        executable='move_to_pressurized_room',
        name='move_to_pressurized_room3',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_to_unpressurized_room = Node(
        package='museum',
        executable='move_to_unpressurized_room',
        name='move_to_unpressurized_room',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_to_unpressurized_room2 = Node(
        package='museum',
        executable='move_to_unpressurized_room',
        name='move_to_unpressurized_room2',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_to_unpressurized_room3 = Node(
        package='museum',
        executable='move_to_unpressurized_room',
        name='move_to_unpressurized_room3',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    activate_seal = Node(
        package='museum',
        executable='activate_seal',
        name='activate_seal',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    activate_seal2 = Node(
        package='museum',
        executable='activate_seal',
        name='activate_seal2',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    activate_seal3 = Node(
        package='museum',
        executable='activate_seal',
        name='activate_seal3',
        namespace=namespace,
        output='screen',
        parameters=[])

    pick_up_empty_pod_slot_1 = Node(
        package='museum',
        executable='pick_up_empty_pod_slot_1',
        name='pick_up_empty_pod_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_empty_pod_slot_12 = Node(
        package='museum',
        executable='pick_up_empty_pod_slot_1',
        name='pick_up_empty_pod_slot_12',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_empty_pod_slot_13 = Node(
        package='museum',
        executable='pick_up_empty_pod_slot_1',
        name='pick_up_empty_pod_slot_13',
        namespace=namespace,
        output='screen',
        parameters=[])

    pick_up_full_pod_slot_1 = Node(
        package='museum',
        executable='pick_up_full_pod_slot_1',
        name='pick_up_full_pod_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_full_pod_slot_12 = Node(
        package='museum',
        executable='pick_up_full_pod_slot_1',
        name='pick_up_full_pod_slot_12',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_full_pod_slot_13 = Node(
        package='museum',
        executable='pick_up_full_pod_slot_1',
        name='pick_up_full_pod_slot_13',
        namespace=namespace,
        output='screen',
        parameters=[])

    drop_pod_slot_1 = Node(
        package='museum',
        executable='drop_pod_slot_1',
        name='drop_pod_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    pick_up_slot_1 = Node(
        package='museum',
        executable='pick_up_slot_1',
        name='pick_up_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    put_in_pod_slot_1 = Node(
        package='museum',
        executable='put_in_pod_slot_1',
        name='put_in_pod_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    release_artifact_slot_1 = Node(
        package='museum',
        executable='release_artifact_slot_1',
        name='release_artifact_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    release_artifact_from_pod_slot_1 = Node(
        package='museum',
        executable='release_artifact_from_pod_slot_1',
        name='release_artifact_from_pod_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    cool_artifact_while_carrying_slot_1 = Node(
        package='museum',
        executable='cool_artifact_while_carrying_slot_1',
        name='cool_artifact_while_carrying_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    cool_artifact_while_carrying_in_pod_slot_1 = Node(
        package='museum',
        executable='cool_artifact_while_carrying_in_pod_slot_1',
        name='cool_artifact_while_carrying_in_pod_slot_1',
        namespace=namespace,
        output='screen',
        parameters=[])

    pick_up_slot_2 = Node(
        package='museum',
        executable='pick_up_slot_2',
        name='pick_up_slot_2',
        namespace=namespace,
        output='screen',
        parameters=[])

    release_artifact_slot_2 = Node(
        package='museum',
        executable='release_artifact_slot_2',
        name='release_artifact_slot_2',
        namespace=namespace,
        output='screen',
        parameters=[])

    
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    # Actions
    ld.add_action(fly_into_seismic_room)
    ld.add_action(move_to_pressurized_room)
    ld.add_action(move_to_pressurized_room2)
    ld.add_action(move_to_pressurized_room3)
    ld.add_action(move_to_unpressurized_room)
    ld.add_action(move_to_unpressurized_room2)
    ld.add_action(move_to_unpressurized_room3)
    ld.add_action(activate_seal)
    ld.add_action(activate_seal2)
    ld.add_action(activate_seal3)
    ld.add_action(pick_up_empty_pod_slot_1)
    ld.add_action(pick_up_empty_pod_slot_12)
    ld.add_action(pick_up_empty_pod_slot_13)
    ld.add_action(pick_up_full_pod_slot_1)
    ld.add_action(pick_up_full_pod_slot_12)
    ld.add_action(drop_pod_slot_1)
    ld.add_action(pick_up_slot_1)
    ld.add_action(put_in_pod_slot_1)
    ld.add_action(release_artifact_slot_1)
    ld.add_action(release_artifact_from_pod_slot_1)
    ld.add_action(cool_artifact_while_carrying_slot_1)
    ld.add_action(cool_artifact_while_carrying_in_pod_slot_1)
    ld.add_action(pick_up_slot_2)
    ld.add_action(release_artifact_slot_2)

    return ld
