# Copyright 2026 WheelHub Intelligent
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')

    # Declare arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    
    # Path to the config file
    config_file = PathJoinSubstitution([
        FindPackageShare('whi_modbus_server'),
        'config',
        'config.yaml'
    ])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Node
    start_modbus_io_node = Node(
        package='whi_modbus_server',
        executable='whi_modbus_server_node',
        name='whi_modbus_server',
        namespace=namespace,
        parameters=[configured_params],
        output='screen',
    )
    
    return LaunchDescription([
        declare_namespace_arg,
        start_modbus_io_node
    ])