import multiprocessing
import launch_ros
from launch import LaunchService
from launch import LaunchDescription
from launch import LaunchIntrospector
import time
import nest_asyncio
import os
import signal

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from robot_interfaces import StartStop, StartStopRequest

nest_asyncio.apply()

# cartographer
cartographer_prefix = get_package_share_directory('open_source_slam_launch')
cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                cartographer_prefix, 'config'))
cartographer_configuration_basename = LaunchConfiguration('configuration_basename',
                                                default='cartographer_mapping.lua')
DeclareLaunchArgument(
    'cartographer_config_dir',
    default_value=cartographer_config_dir,
    description='Full path to config file to load')
DeclareLaunchArgument(
    'configuration_basename',
    default_value=cartographer_configuration_basename,
    description='Name of lua file for cartographer')
cartographer_ld = [LaunchDescription([
    launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters = [{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename],
        # remappings=[('chatter', 'my_chatter'+str(i % num_parallel))]
        ),
    launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        # remappings=[('chatter', 'my_chatter'+str((i+1) % num_parallel))]
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05},
            {'-publish_period_sec': 1.0}]
        ),
    launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_odom_preproc',
        output='screen',
        parameters = [
            {'use_sim_time': True}],
        ),
    ])
]

class NodeMgmt(Node):

    def __init__(self):
        super().__init__('node_mgmt')
        self.srv = self.create_service(StartStop, 'node_mgmt', self.node_mgmt_callback)

    def node_mgmt_callback(self, request, response):
        self.get_logger().info('Incoming request\n node_name: %d action: %d' % (request.node, request.action))
        node_name = request.node
        pPool = self.get_process_pool(node_name)

        if response.action == StartStopRequest.START:
            self.process_start(pPool)

        elif response.action == StartStopRequest.STOP:
            self.process_stop(pPool)

        elif response.action == StartStopRequest.RESTART:

            self.process_restart(pPool)
        else:
            pass

        response.success = True;
        return response
    
    def get_process_pool(ld_name):
        #todo
        if ld_name == "cartographer":
            ld = cartographer_ld
        elif ld_name == "navigation2":
            pass


        # Construct launch service pool. Each launch service can handle one launch description
        # add noninteractive=True to handle sigint
        lsPool = [LaunchService(noninteractive=True) for i in ld]
        i = 0
        for ls in lsPool:
            ls.include_launch_description(ld[i])
            i += 1

        # Construct process pool. Each process starts one launch service
        pPool = [multiprocessing.Process(target=ls.run) for ls in lsPool]
        return pPool

    def process_start(pPool):
        for p in pPool:
            p.start()

    def process_stop(pPool):
        for p in pPool:
            # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
            os.kill(p.pid, signal.SIGINT)
        for p in pPool:
            p.join()

    def process_restart(pPool):
        for p in pPool:
        # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
            os.kill(p.pid, signal.SIGINT)
        for p in pPool:
            p.join()
        
        time.sleep(1)

        for p in pPool:
            p.start()


        


def main():
    rclpy.init()

    node_mgmt = NodeMgmt()

    rclpy.spin(node_mgmt)

    rclpy.shutdown()


if __name__ == '__main__':
    main()