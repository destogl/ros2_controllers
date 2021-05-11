# Copyright (c) 2021, PickNik, Inc.
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
#
# \author: Denis Stogl

import os
import unittest

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts


def generate_test_description():
    test_exe_arg = launch.actions.DeclareLaunchArgument(
        'test_exe',
        description='Path to executable test',
    )

    process_under_test = launch.actions.ExecuteProcess(
        cmd=[launch.substitutions.LaunchConfiguration('test_exe')],
        output='screen',
    )

    urdf_file = os.path.join(os.path.dirname(__file__), '6d_robot_description.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        test_exe_arg,
        process_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class TestAdmittanceControllerExe(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=10)


@launch_testing.post_shutdown_test()
class AdmittanceControllerExeTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )
