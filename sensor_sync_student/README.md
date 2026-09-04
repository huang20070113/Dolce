# 第五次作业：并发传感器数据处理与时间戳同步

## 背景

RoboMaster 视觉程序通常需要同时处理相机图像、IMU 姿态和下位机数据。这些数据由不同设备产生，频率不同、到达时间也不一致。本题用文本文件模拟相机和 IMU 通信数据，重点考察并发队列、线程退出协议以及时间戳匹配算法。

本题不需要真实相机、串口、ROS、OpenCV、Eigen 或通信板。

## 任务

使用 C++ 完成一个可执行程序 `sensor_sync`：

1. 启动两个生产者线程，分别读取相机文件和 IMU 文件。
2. 将两路数据写入同一个有界线程安全队列。
3. 启动一个消费者线程，持续取出数据并按传感器类型保存。
4. 所有生产者结束后关闭队列；消费者必须处理完队列中的剩余数据再退出。
5. 对每个相机包匹配时间上最近的 IMU 包，并按指定格式输出结果。

必须使用 `std::thread`、`std::mutex` 和 `std::condition_variable`。等待队列时不得忙等或定时轮询。

## 队列要求

实现 `ThreadSafeQueue<T>`，至少提供以下行为：

- 容量固定且大于 0。
- 队列满时 `push` 阻塞，直到有空间或队列被关闭。
- 队列空时 `pop` 阻塞，直到有数据或队列被关闭。
- 关闭后不再接受新数据。
- 关闭时唤醒所有等待线程。
- 关闭后仍允许取出队列中已有的数据；队列为空后 `pop` 返回 `false`。
- 多线程下不得丢数据、重复消费或死锁。

## 时间戳匹配规则

对每个相机包，在全部 IMU 包中选择 `abs(camera.timestamp_us - imu.timestamp_us)` 最小的一条：

1. 差值大于 `max_delta_us` 时输出 `UNMATCHED`。
2. 差值相同时，选择时间戳更早的 IMU 包。
3. 时间戳也相同时，选择 `sequence` 更小的 IMU 包。
4. 同一个 IMU 包允许匹配多个相机包。
5. 输出按相机时间戳升序排列；相机时间戳相同则按 `sequence` 升序排列。

匹配部分不得对每个相机包完整扫描全部 IMU 包。允许排序，总复杂度上限为 `O(C log C + I log I + C + I)`。

## 输入与配置

运行方式：

```bash
./build/sensor_sync config/sensor_sync.conf
```

配置文件包含：

- `queue_capacity`：队列容量，必须大于 0。
- `max_delta_us`：最大允许时间差，单位为微秒，必须大于等于 0。
- `camera_file`：相机数据文件路径。
- `imu_file`：IMU 数据文件路径。
- `output_file`：结果文件路径。

相对路径以配置文件所在目录为基准。数据包格式见 `docs/packet_format.md`。

## 输出格式

匹配成功：

```text
CAMERA <camera_sequence> IMU <imu_sequence> DELTA <absolute_delta_us>
```

未匹配：

```text
CAMERA <camera_sequence> UNMATCHED
```

配置缺失、数值非法、文件打不开或数据包格式错误时，应向标准错误输出清晰原因并以非零状态退出。


## 构建与公开样例

```bash
bash verify_sample.sh
```

脚本会构建程序、运行公开样例并与 `data/expected_result.txt` 比较。初始骨架只保证可以编译，补全实现后样例才会通过。
