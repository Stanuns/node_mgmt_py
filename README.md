# node 管理

## 1. 自主探索建图节点管理

### 1.1 自动探索建图模块当前状态反馈

- topic name: /auto_explore_mapping/state
- topic type: app_msgs/msg/AutoExploreMappingState
- 状态解释：

```bashrc
   -1:未初始化; 
   0:已初始化未开始; 
   1:开始自动建图；
   2:建图中; 
   3:停止自动建图;
```

### 1.2 对自动探索建图模块进行控制

- 其他模块控制该模块的接口，比如语音模块控制该模块
- topic name: /auto_explore_mapping/trigger
- topic type: app_msgs/msg/AutoExploreMappingTrigger
- 状态解释：

```bashrc
  -1:未有动作; ### 该值是默认的，其他模块无需发此状态
  1:开始自动建图; ### 发此值之前/auto_explore_mapping/state是0，
                ### 发完之后/auto_explore_mapping/state变成1，随后变成2.
  3:停止自动建图; ### 发此值之前/auto_explore_mapping/state是2， 
                ### 发完之后/auto_explore_mapping/state变成3，随后变成0，待机下次自动建图
  5：保存当前地图; ###发此值之前/auto_explore_mapping/state是2.
                 ###发此值之后/auto_explore_mapping/state是2.
```

## 2. 利用cartographer_ros 提供的接口进行建图功能的启停

[reference](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html?highlight=start_trajectory)

[注意：该方法没有解决关闭重启建图之后，map数据重叠的问题](https://github.com/ros2/cartographer_ros/issues/69)

### 2.1 查询trajectory_id, 判断哪一个trajectory_id是Active, 需要关闭的是Active的trajectory

查看service msg代表的意义

```
ros2 interface show cartographer_ros_msgs/srv/GetTrajectoryStates
```

查看各个trajectory的状态

```bashrc
ros2 service call /get_trajectory_states cartographer_ros_msgs/srv/GetTrajectoryStates
```

### 2.2 关闭建图

```bashrc
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
```

### 2.3 打开建图

```bashrc
ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory "{configuration_directory: '/home/sunwei/robot_ws/install/open_source_slam_launch/share/open_source_slam_launch/config/', configuration_basename: 'cartographer_mapping.lua', use_initial_pose: true, initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, relative_to_trajectory_id: 0}"
```

## 3. 采用管理进程pid方式实现node管理
[reference](https://haoguangyang.github.io/robotics%20notes/gists/programmatically-start-stop-nodes/)
