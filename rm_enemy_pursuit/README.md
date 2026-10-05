# rm_enemy_pursuit

ROS 2 Humble 教学仿真功能包。导航过程中收到模拟敌方位置后，取消普通目标并交给 Nav2 追踪；检测消失后恢复最新的普通导航目标。保留 Nav2 的路径规划、避障和底盘速度转换，不直接发布速度。

## 已实现

- 任意水平角度的模拟检测；0° 前、90° 左、180° 后、-90° 右。
- 检测生成时转换到 map 坐标，单轮目标固定在地图中，不随机器人移动。
- 同时只保留一个本包发起的 Nav2 action。必须收到旧目标终态后才发送新目标。
- 默认保留 1 m 跟随距离，0.3 m 释放迟滞，2 s 检测超时，30 s 单轮追踪上限。
- 检测过期、错误坐标系、非有限数值、TF 缺失、目标拒绝和导航失败处理。
- 追踪期间收到新的普通目标时更新暂存目标；停止追踪后恢复最新目标。

## 依赖与编译

需要 ROS 2 Humble、Nav2、tf2、rclpy。完整演示还需要已经编译的 pb_rm_simulation 工作空间。

```bash
cd /home/user/pb_rm_simulation
source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"
colcon build --symlink-install --packages-select rm_enemy_pursuit rm_nav_bringup pb_rm_simulation ros2_livox_simulation
source install/setup.bash
```

## 使用已有仿真

先启动原仿真，但设置 `nav_rviz:=False`。在另一个终端启动本包：

```bash
source /opt/ros/humble/setup.bash
source /home/user/pb_rm_simulation/install/setup.bash
ros2 launch rm_enemy_pursuit pursuit.launch.py bearing_deg:=180.0
```

在本包打开的 RViz 中使用 **2D Goal Pose**。它向 `/mission_goal` 发目标。不要同时使用原版 Nav2 Goal、Navigate Through Poses 或其他程序直接发送导航 action，这些目标绕过本包，不能被本包自动暂存和恢复。

模拟检测默认延迟 10 秒，每次出现 8 秒、消失 6 秒。红球表示模拟目标，不是 Gazebo 实体，不参与传感器碰撞。默认目标可能落在墙内；演示前选择开阔区域，必要时减小 `distance`。Nav2 仍会拒绝无法到达的目标，本包不会绕过障碍物强行移动。

```bash
ros2 topic echo /pursuit_status
ros2 param set /enemy_simulator bearing_deg 90.0
ros2 param set /enemy_simulator distance 2.0
ros2 service call /enemy_simulator/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /enemy_simulator/enable std_srvs/srv/SetBool '{data: true}'
```

角度和距离修改在下一轮目标生成时生效；重新启用会重置轮次并等待 start_delay。关闭模拟检测约 2 秒后恢复普通目标。停止整个演示时关闭全部 launch，避免只退出仲裁节点后遗留导航任务。

## 一条命令启动整个工程

`demo.launch.py` 依赖本次交付中 rm_nav_bringup 和 pb_rm_simulation 的启动补丁与实验配置。若只将本功能包加入未经修改的上游工程，使用上面的 `pursuit.launch.py`。

```bash
ros2 launch rm_enemy_pursuit demo.launch.py mode:=nav profile:=baseline bearing_deg:=180.0
```

边建图边导航：`mode:=mapping`。单独做定位实验：`mode:=nav simulate:=false profile:=buffer5`。可选配置：baseline、buffer5、window08、resolution0025、combined。mapping 模式使用原建图参数，profile 只作用于 nav 模式的 slam_toolbox 定位。

## 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/mission_goal` | geometry_msgs/PoseStamped | 普通导航目标，map 坐标，单位四元数 |
| `/enemy_pose` | geometry_msgs/PoseStamped | 带有效当前时间戳的检测位置，map 坐标 |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | 唯一导航输出接口 |
| `/pursuit_status` | std_msgs/String | IDLE、MISSION、PURSUIT、HOLD、WAIT_TF |
| `/enemy_marker` | visualization_msgs/Marker | 模拟检测可视化 |
| `/clear_mission` | std_srvs/Trigger | 清空暂存普通目标，不关闭当前追踪 |
| `/enemy_simulator/enable` | std_srvs/SetBool | 开关模拟检测 |

参数见 `config/pursuit.yaml`。管理器在启动时读取参数；修改该文件后需重启。模拟器通过 ROS 参数动态读取角度、距离和周期。

## 验证

```bash
source /opt/ros/humble/setup.bash
source /home/user/pb_rm_simulation/install/setup.bash
cd /home/user/pb_rm_simulation/src/rm_enemy_pursuit
ROS_DOMAIN_ID=87 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -s test
```

12 项测试通过。测试含真实 DDS 和 rclpy action 通信，Nav2 服务端由测试程序模拟：四方位抢占、旧消息拒绝、近距停止、任务恢复、更新暂存目标、失败与拒绝重试、TF 丢失、追踪时限，以及模拟目标固定于 map。八方向几何测试验证 1 m 跟随点。

另完成 60 秒 Gazebo + 真实 Nav2 联调，收到 184 条检测，记录到四次追踪目标成功和普通目标恢复。无界面雷达不出点云问题已通过 mid360.xacro 的 always_on 修复；启动仍可能出现 spawn_entity 超时提示，但随后传感器及 Nav2 正常运行。详见交付报告；暂未录制屏幕视频。当前接口按单目标检测设计，不含真实视觉识别、多目标关联、武器控制或动态目标轨迹预测。

## 文件

- `manager.py`：优先级仲裁、action 生命周期、检测时效和恢复逻辑。
- `simulator.py`：模拟全向检测与 RViz 红球。
- `geometry.py`：跟随点计算和输入校验。
- `launch/`：独立启动与工程一体启动入口。
- `config/`：参数与避免绕过仲裁的 RViz 配置。
- `test/`：几何、通信和模拟器测试。

## 发布

这是可独立上传 GitHub 的源码包，不包含 build、install 或第三方子模块。上传前将 package.xml 和 setup.py 中的维护者占位信息改为自己的信息。适配的 RViz 配置来源于 pb_rm_simulation，原作者版权声明保留在 LICENSE 中。当前未创建或推送任何 GitHub 仓库。
