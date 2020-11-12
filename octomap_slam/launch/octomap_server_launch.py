# Copyright 2020 Intelligent Robotics Lab
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
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Load params
    pkg_dir = get_package_share_directory('octomap_slam')
    config_file_path = pkg_dir + '/config/octomap_server.yaml'

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
              get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')),
            launch_arguments={
              'params_file': pkg_dir + '/config/nav2_params.yaml'
            }.items())

    yolact = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
              get_package_share_directory('yolact_ros2'), 'launch', 'yolact_ros2.launch.py')),
            launch_arguments={})

    yolact3d = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
              get_package_share_directory('yolact_ros2_3d'), 'launch', 'yolact_ros2_3d.launch.py')),
            launch_arguments={})

    octomap_server = Node(
        package='octomap_slam',
        node_executable='octomap_server',
        output='screen',
        parameters=[{
            "mapping" : False,
            "octomap_file" : pkg_dir + "/maps/octo_save.ot"
          },
          config_file_path
          ],
        remappings=[
                ('/octomap_server/input_octomaps', '/yolact_ros2_3d_node/output_octomaps'),
            ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(nav2)
    ld.add_action(yolact)
    ld.add_action(yolact3d)
    ld.add_action(octomap_server)

    return ld
