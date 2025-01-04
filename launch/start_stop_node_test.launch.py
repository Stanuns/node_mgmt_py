# ROS2 launch python api test
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

cartographer_prefix = get_package_share_directory('open_source_slam_launch')
cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                cartographer_prefix, 'config'))
configuration_basename = LaunchConfiguration('configuration_basename',
                                                default='cartographer_mapping.lua')
DeclareLaunchArgument(
    'cartographer_config_dir',
    default_value=cartographer_config_dir,
    description='Full path to config file to load')
DeclareLaunchArgument(
    'configuration_basename',
    default_value=configuration_basename,
    description='Name of lua file for cartographer')

# apply nest_asyncio because we are nesting parallel processes
nest_asyncio.apply()

# Here is an example of three cyclic talkers/listeners
num_parallel = 3
# whatever launch description for each node is put in here.
ld = [LaunchDescription([
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
    # for i in range(num_parallel)
    ]

for item in ld:
    print(LaunchIntrospector().format_launch_description(item))

# Construct launch service pool. Each launch service can handle one launch description
# add noninteractive=True to handle sigint
lsPool = [LaunchService(noninteractive=True) for i in ld]
i = 0
for ls in lsPool:
    ls.include_launch_description(ld[i])
    i += 1

# Construct process pool. Each process starts one launch service
pPool = [multiprocessing.Process(target=ls.run) for ls in lsPool]

# starting the stack
print("Starting everything")
for p in pPool:
    p.start()
time.sleep(30)
# shutting down the stack
print("Stopping everything")
for p in pPool:
    print("stopping: "+str(p.pid))
    # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
    os.kill(p.pid, signal.SIGINT)
    # DON'T DO THIS as it will lead to a dirty shutdown with zombie nodes
    # p.terminate()
for p in pPool:
    p.join()

time.sleep(30)
# re-starting the stack
# Launch service pool and process pool are non-reusable
lsPool = [LaunchService(noninteractive=True) for i in ld]
i = 0
for ls in lsPool:
    ls.include_launch_description(ld[i])
    i += 1
pPool = [multiprocessing.Process(target=ls.run) for ls in lsPool]

print("Re-Starting everything")
for p in pPool:
    p.start()
time.sleep(30)
print("Stopping everything")
for p in pPool:
    print("stopping: "+str(p.pid))
    # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
    os.kill(p.pid, signal.SIGINT)
for p in pPool:
    p.join()