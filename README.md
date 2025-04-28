# node 管理

## 3. 采用管理进程pid方式实现node管理
启动节点管理service server节点：
```bashrc
ros2 run node_mgmt_py node_mgmt_service
```

调用service名称: /node_start_stop
### 3.1 建图启停
- 开启建图传入参数：
node:'cartographer'
action:1

- 停止建图传入参数：
node:'cartographer'
action:2

- 重启建图传入参数：
node:'cartographer'
action:3

### 3.2 导航启停
- 开启导航传入参数：
node:'navigation2'
action:1

- 停止导航传入参数：
node:'navigation2'
action:2

- 重启导航传入参数：
node:'navigation2'
action:3

服务调用成功返回参数：
success: True
服务调用失败返回参数：
success: False