# controller_pkg
继电器控制包---喷洒装置

## 编译

```bash
colcon build --packages-select wvcsc_interfaces controller_pkg
source install/setup.bash
```

## 运行
启动服务：
```bash
ros2 launch controller_pkg controller.launch.py
```
节点默认读取 `config/fault.ini`，通过 Modbus RTU 控制继电器。
服务请求中可以动态指定需要操作的通道。

### 测试
调用服务：

```bash
ros2 service call <service_name> <service_type> <arguments>
```
各部分含义：

- `<service_name>`：服务名称。本功能包使用 `/relay/set`。
- `<service_type>`：服务接口类型。本功能包使用
  `wvcsc_interfaces/srv/SetRelay`。
- `<arguments>`：发送给服务端的请求参数，使用 YAML 格式。本服务只有
  `channel`、`enabled` 和 `duration` 三个字段。

因此，本功能包对应的完整命令格式是：

```bash
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: <通道号>, enabled: <true_or_false>, duration: <持续秒数>}"
```

参数含义：
- `channel`：1（广域）或2（虫害喷洒）。
- `enabled: true`：请求指定通道吸合。
- `enabled: false`：请求指定通道断开。
- `duration > 0`：吸合指定秒数后，由服务端自动断开。
- `duration: 0.0`：不自动断开，保持状态直到收到新请求。
- 断开请求中的 `duration` 会被忽略，推荐填写 `0.0`。

实际调用示例：

```bash
# 第 1 路继电器吸合
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 1, enabled: true, duration: 0.0}"

# 第 1 路继电器断开
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 1, enabled: false, duration: 0.0}"

# 第 2 路继电器吸合 3 秒，然后自动断开
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: true, duration: 3.0}"
```

### 查看服务

```bash
# 列出当前 ROS 2 网络中的服务
ros2 service list

# 查看指定服务的接口类型
ros2 service type /relay/set

# 查看自定义服务的请求和响应字段
ros2 interface show wvcsc_interfaces/srv/SetRelay
```

## Python客户端测试 
先启动继电器服务节点，再在另一个终端调用客户端：
```bash
source install/setup.bash
# 请求第 1 路继电器吸合
ros2 run controller_pkg relay_client_demo 1 on
# 请求第 1 路继电器断开
ros2 run controller_pkg relay_client_demo 1 off
# 请求第 2 路继电器吸合
ros2 run controller_pkg relay_client_demo 2 on
# 请求第 2 路吸合 3 秒，然后由服务端自动断开
ros2 run controller_pkg relay_client_demo 2 on 3.0
```
