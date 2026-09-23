# SLAM 作业一：RMUC2025 模型转二维可通行地图

本提交版本使用 ROS2 Jazzy，最终分辨率为 **0.025 m/格**，地图 **680×1460**。已完成 Linux 编译与测试、ROS `/map` 发布、RViz 显示以及三条离线路径验证。详细过程和实际截图见 [工作流程报告](工作流程报告.md)，浏览器可打开 `工作流程报告.html`。

模型由既有 `RMUC2025.stp` 转换记录追溯而来；当前 STL 的 SHA256 与记录一致。**尚未取得题目链接中的原始 STL，不能声称已核实它与题目文件相同。** 详见 `模型来源说明.md` 和 `evidence/model_provenance.json`。

## 下载完整模型

本仓库保留源码、参数、地图、报告和验证证据。约 372 MiB 的 `RMUC2025.STL` 不放入普通 Git 仓库。

复现前，请在本仓库右侧的 **Releases** 中下载 `SLAM_HW1_submission_20260923.zip`，解压后将其中 `SLAM_HW1_submission/data/RMUC2025.STL` 复制到本仓库的 `data/` 目录。也可直接使用完整包中的工程。若尚未看到 Release，请联系仓库维护者补充发布。

模型 SHA256 应为 `d6c9da11f09604db0c2bbe8e368b7f6176de36b1039edac401aee69264f14f31`；可用 `sha256sum data/RMUC2025.STL` 核对。

## 文件说明

| 路径 | 内容 |
|---|---|
| `ros2_ws/src/stl_to_grid_map/` | C++17 转换器、ROS2 节点、启动文件、参数、测试和 C++ A* 验证器 |
| `data/RMUC2025.STL` | 本次实际输入模型，约 372 MiB，需从 Release 完整包取得 |
| `data/model_metadata.json` | 原有 STEP→STL 转换记录 |
| `output/RMUC2025.pgm`、`.yaml` | 最终膨胀地图 |
| `output/RMUC2025_raw.pgm`、`.yaml` | 同参数、零几何膨胀的对照地图 |
| `output/path_*.csv` | 三条 C++ A* 路径，列为 PGM 图像坐标 column,row |
| `figures/` | 原模型预览、实际 RViz 截图、最终地图、路径验证图 |
| `evidence/` | 编译测试、ROS 实测、路径审计、模型校验记录 |
| `MANIFEST.sha256` | 本次仓库上传文件校验和（不包含自身，也不包含单独发布的 STL） |

## 环境与编译

目标环境：Ubuntu 24.04、ROS2 Jazzy，桌面环境用于 RViz。已有 Jazzy 软件源时，缺失依赖可安装：

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions \
  ros-jazzy-ros-base ros-jazzy-rviz2 \
  python3-numpy python3-pil python3-yaml python3-scipy python3-matplotlib
```

取得模型后，在仓库根目录打开终端（后续命令均从该目录运行）：

```bash
bash build.sh
bash start.sh
```

`build.sh` 编译并运行包内测试。`start.sh` 启动地图和 RViz，并自动核对配置、PGM 与实时 `/map`。无需另启 map_server。本包不包含与原机器路径绑定的 build/install 缓存，首次使用必须先编译。

## 参数调整与重启

编辑 `ros2_ws/src/stl_to_grid_map/config/rmuc2025.yaml` 并保存。只改参数无须重新编译，但节点不支持在线重新转换。由 `start.sh` 启动时可运行：

```bash
bash stop.sh map
bash start.sh
```

完整关闭本作业窗口和节点：`bash stop.sh`。停止脚本会核对 PID 对应的实际命令，不会仅凭过期 PID 结束其他程序。若使用手动 `ros2 launch` 启动，请在原终端按 Ctrl+C 停止，避免同时启动多个 `/map` 发布者。

运行中复核：

```bash
bash verify.sh
```

该命令根据当前参数重新生成零膨胀地图、运行 C++ A*，再独立检查路径合法性与障碍距离；若参数令测试端点阻塞或通道消失，会报失败，应审查参数与端点。它会更新验证结果和路径图片，修改参数后需同步更新报告。

## 验证摘要与使用范围

提交版跨场地路径为 45.840233 m，上、下横向路径各为 13.572971 m；三条路径最小障碍格中心距离均为 0.304138 m，均无斜向穿墙。几何膨胀半径为 0.30 m。这些结果仅证明所设假设下的离散几何通行性，不是机器人动力学或实车通过认证。

接入 Nav2 时，可考虑使用零膨胀 `RMUC2025_raw.yaml` 并根据实车 footprint 配置代价地图，避免对最终地图重复几何膨胀。raw 图仍保留地形筛选与最大连通域限制，不是原始三维模型的无筛选投影。

## 尚待外部确认

题目原始 STL 的下载链接未提供。取得文件后应与本包模型做来源和几何一致性核对；不同导出产生不同哈希，并不自动意味着几何不同。模型若需更换，应重新转换、验证并更新报告。姓名、学号及课程指定提交位置由提交者按实际要求填写；本任务未向任何外部平台上传。
