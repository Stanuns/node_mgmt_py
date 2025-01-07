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
from robot_interfaces.srv import StartStop

nest_asyncio.apply()

class NodeMgmt(Node):

    def __init__(self):
        super().__init__('node_mgmt')
        self.srv = self.create_service(StartStop, 'node_start_stop', self.node_mgmt_callback)

        #---------------cartographer-start---------------
        self.cartographer_running = False
        self.cartographer_pre_pPool = None

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
        self.cartographer_ld = [LaunchDescription([
            launch_ros.actions.Node(
                package='cartographer_ros',
                executable='cartographer_node',
                # output='screen',
                parameters = [{'use_sim_time': True}],
                arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', cartographer_configuration_basename],
                # remappings=[('chatter', 'my_chatter'+str(i % num_parallel))]
                ),
            launch_ros.actions.Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                # output='screen',
                # remappings=[('chatter', 'my_chatter'+str((i+1) % num_parallel))]
                parameters = [
                    {'use_sim_time': True},
                    {'resolution': 0.05},
                    {'-publish_period_sec': 1.0}]
                ),
            launch_ros.actions.Node(
                package='cartographer_ros',
                executable='cartographer_odom_preproc',
                # output='screen',
                parameters = [
                    {'use_sim_time': True}],
                ),
            ])
        ]
        #---------------cartographer-end---------------

    def node_mgmt_callback(self, request, response):
        self.get_logger().info('Incoming request\n node_name: %s action: %s' % (request.node, request.action))
        node_name = request.node
        pPool = self.get_process_pool(node_name)

        if request.action == StartStop.Request().START:
            self.process_start(pPool, node_name)

        elif request.action == StartStop.Request().STOP:
            self.process_stop(pPool, node_name)

        elif request.action == StartStop.Request().RESTART:

            self.process_restart(pPool, node_name)
        else:
            pass

        response.success = True;
        return response
    
    def get_process_pool(self, ld_name):
        if_runnning = None
        #todo
        if ld_name == "cartographer":
            ld = self.cartographer_ld
            if_runnning = self.cartographer_running
        elif ld_name == "navigation2":
            ld = None
            pass
        
        if not if_runnning:
            # Construct launch service pool. Each launch service can handle one launch description
            # add noninteractive=True to handle sigint
            lsPool = [LaunchService(noninteractive=True) for i in ld]
            i = 0
            for ls in lsPool:
                ls.include_launch_description(ld[i])
                i += 1
            # Construct process pool. Each process starts one launch service
            pPool = [multiprocessing.Process(target=ls.run) for ls in lsPool]
            if ld_name == "cartographer":
                self.cartographer_pre_pPool = pPool
            elif ld_name == "navigation2":
                pass

            return pPool
        elif if_runnning:
            if ld_name == "cartographer":
                return self.cartographer_pre_pPool
            elif ld_name == "navigation2":
                pass


    def process_start(self, pPool, node_name):
        self.get_logger().info('node start---------------->')
        if node_name == "cartographer":
            self.cartographer_running = True
        elif node_name == "navigation2":
            pass

        for p in pPool:
            print("----->before node starting: "+str(p.pid))
            p.start()
            print("----->after node starting: "+str(p.pid))

    def process_stop(self, pPool, node_name):
        if node_name == "cartographer":
            self.cartographer_running = False
        elif node_name == "navigation2":
            pass

        self.get_logger().info('node stop---------------->')
        for p in pPool:
            print("----->before node stopping: "+str(p.pid))
            # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
            os.kill(p.pid, signal.SIGINT)
        for p in pPool:
            p.join()

    def process_restart(self, pPool, node_name):
        self.get_logger().info('node restart---------------->')
        for p in pPool:
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