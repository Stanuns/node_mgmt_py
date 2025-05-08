import multiprocessing
import launch_ros
from launch import LaunchService
from launch import LaunchDescription
from launch import LaunchIntrospector
import time
import nest_asyncio
import os
import signal

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import yaml 

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import StartStop

nest_asyncio.apply()

class NodeMgmt(Node):

    def __init__(self):
        super().__init__('node_mgmt')
        self.srv = self.create_service(StartStop, 'node_start_stop', self.node_mgmt_callback)
        self.action_ma = {
                StartStop.Request().START: "START",
                StartStop.Request().STOP: "STOP",
                StartStop.Request().RESTART: "RESTART"
        }

        #---------------cartographer-start---------------
        self.cartographer_running = False
        self.cartographer_pre_pPool = None

        cartographer_prefix = get_package_share_directory('open_source_slam_launch')
        cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                        cartographer_prefix, 'config'))
        cartographer_configuration_basename = LaunchConfiguration('configuration_basename',
                                                        default='luxsharerobot_cartographer_mapping.lua')
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
                parameters = [{'use_sim_time': False}],
                arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', cartographer_configuration_basename],
                remappings=[('/imu', '/imu/data_raw')]
                ),
            launch_ros.actions.Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                # output='screen',
                # remappings=[('chatter', 'my_chatter'+str((i+1) % num_parallel))]
                parameters = [
                    {'use_sim_time': False},
                    {'resolution': 0.05},
                    {'-publish_period_sec': 1.0}]
                ),
            # launch_ros.actions.Node(
            #     package='cartographer_ros',
            #     executable='cartographer_odom_preproc',
            #     # output='screen',
            #     parameters = [
            #         {'use_sim_time': True}],
            #     ),
            ])
        ]
        #---------------cartographer-end---------------

        #---------------navigation2-start--------------
        self.navigation2_running = False
        self.navigation2_pre_pPool = None
        # map_dir = LaunchConfiguration(
        #     'map',
        #     default=os.path.join(
        #         get_package_share_directory('map_server_extension'),
        #         'maps',
        #         'luxsharenanjinghall01.yaml')),
        nav2_param_file_name = 'nav2_params_luxsharerobot.yaml'
        nav2_param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('open_source_slam_launch'),
                'launch',
                nav2_param_file_name))
        nav2_launch_file_dir = os.path.join(get_package_share_directory('open_source_slam_launch'), 'launch')
        # DeclareLaunchArgument(
        #     'map',
        #     default_value=map_dir,
        #     description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_dir,
            description='Full path to param file to load'),
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),
        def load_map_config(context):
            map_mgmt_path = os.path.join(
                get_package_share_directory('map_server_extension'),
                'params',
                'map_mgmt.yaml'
            )
            with open(map_mgmt_path, 'r') as f:
                map_name = yaml.safe_load(f)['map_mgmt_server']['ros__parameters']['current_map_name']
            
            return [
                DeclareLaunchArgument(
                    'map',
                    default_value=os.path.join(
                        get_package_share_directory('map_server_extension'),
                        'maps',
                        f'{map_name}.yaml'
                    )
                )
            ]
        self.navigation2_ld = [LaunchDescription([
                OpaqueFunction(function=load_map_config),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_launch_file_dir, '/luxsharerobot_bringup_launch.py']),
                    launch_arguments={
                        'map': LaunchConfiguration('map'),
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_param_dir
                        }.items(),
                )
            ])
        ]
        #---------------navigation2-end---------------

    def node_mgmt_callback(self, request, response):
        try:
            self.get_logger().info('Incoming request\n node_name: %s action: %s' % (request.node, request.action))
            node_name = request.node
            # pPool = self.get_process_pool(node_name)

            if request.action == StartStop.Request().START:
                self.process_start(node_name)
            elif request.action == StartStop.Request().STOP:
                self.process_stop(node_name)
            elif request.action == StartStop.Request().RESTART:
                self.process_restart(node_name)
            else:
                raise ValueError(f"Invalid action: {request.action}")
            
            response.success = True
            action_str = self.action_ma.get(request.action, "UNKNOWN")
            response.message = f'{action_str} {node_name} successfully'
        except Exception as e:  
            self.get_logger().error(f'Error handling request: {str(e)}')
            response.success = False   
            response.message = f'Error handling request: {str(e)}'

        return response
    
    def get_process_pool(self, ld_name):
        if_runnning = None
        #todo
        if ld_name == "cartographer":
            ld = self.cartographer_ld
            if_runnning = self.cartographer_running
        elif ld_name == "navigation2":
            ld = self.navigation2_ld
            if_runnning = self.navigation2_running
        else:
            raise ValueError(f"Unknown node name: {ld_name}")
        
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
                self.navigation2_pre_pPool = pPool
            else:
                pass
            return pPool, if_runnning
        elif if_runnning:
            if ld_name == "cartographer":
                return self.cartographer_pre_pPool, if_runnning 
            elif ld_name == "navigation2":
                return self.navigation2_pre_pPool, if_runnning 
            else:
                pass


    def process_start(self, node_name):
        try:
            self.get_logger().info(f'Starting {node_name}----->')
            pPool, if_runnning = self.get_process_pool(node_name)
            #debug
            print("----->process_start: "+str(pPool[0].pid))

            if (pPool[0].pid is None) and (not if_runnning): 
                for p in pPool:
                    print("----->before node starting: "+str(p.pid))
                    p.start()
                    print("----->after node starting: "+str(p.pid))

                if node_name == "cartographer":
                    self.cartographer_running = True
                elif node_name == "navigation2":
                    self.navigation2_running = True
                else:
                    pass
            else:
                pass
        except Exception as e:
            self.get_logger().error(f'Failed to start {node_name}: {str(e)}')
            raise

    def process_stop(self, node_name):
        try:
            self.get_logger().info(f'Stopping {node_name}----->')
            pPool, if_runnning = self.get_process_pool(node_name)
            #debug
            print("----->process_stop: "+str(pPool[0].pid))

            if (pPool[0].pid is not None) and (if_runnning): 
                for p in pPool:
                    print("----->before node stopping: "+str(p.pid))
                    # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
                    os.kill(p.pid, signal.SIGINT)
                    print("----->after node stopping: "+str(p.pid))
                for p in pPool:
                    p.join()

                if node_name == "cartographer":
                    self.cartographer_running = False
                elif node_name == "navigation2":
                    self.navigation2_running = False
                else:
                    pass
            else:
                pass
        except Exception as e:
            self.get_logger().error(f'Failed to stop {node_name}: {str(e)}')
            raise

    def process_restart(self, node_name):
        try:
            self.get_logger().info(f'Restarting {node_name}----->')
            self.process_stop(node_name)
            time.sleep(1)
            self.process_start(node_name)
        except Exception as e:
            self.get_logger().error(f'Failed to restart {node_name}: {str(e)}')
            raise

        


def main():
    rclpy.init()

    node_mgmt = NodeMgmt()

    rclpy.spin(node_mgmt)

    rclpy.shutdown()


if __name__ == '__main__':
    main()