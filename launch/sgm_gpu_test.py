# Copyright 2020 Hironori Fujimoto
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sgm_gpu',
            node_executable='sgm_gpu_node',
            node_name='sgm_gpu',
            remappings=[
                ('left_image', '/test_stereo_publisher/left/image_raw'),
                ('right_image', '/test_stereo_publisher/right/image_raw'),
                ('left_camera_info', '/test_stereo_publisher/left/camera_info'),
                ('right_camera_info', '/test_stereo_publisher/right/camera_info')
            ]
        ),
        Node(
            package='sgm_gpu',
            node_executable='test_stereo_publisher',
            node_name='test_stereo_publisher'
        ),
        Node(
            package='image_view',
            node_executable='disparity_view',
            node_name='disparity_view',
            remappings=[('image', '/sgm_gpu/disparity')],
            parameters=[{'window_name': 'Disparity'}]
        ),
        Node(
            package='image_view',
            node_executable='image_view',
            node_name='left_view',
            remappings=[('image', '/test_stereo_publisher/left/image_raw')],
            parameters=[{'window_name': 'Left image'}]
        ),
        Node(
            package='image_view',
            node_executable='image_view',
            node_name='left_view',
            remappings=[('image', '/test_stereo_publisher/right/image_raw')],
            parameters=[{'window_name': 'Right image'}]
        )
    ])

