# G1 MuJoCo ROS2 导航联合仿真

这个工作空间把 Unitree G1 MuJoCo 仿真、MJLab 策略部署、ROS2 Navigation、2D SLAM，以及 Mid360 / FAST-LIO 建图流程整合到一起。目标是在 MuJoCo 里验证导航、感知和行走策略，再逐步迁移到真实 G1 机器人。

强烈建议直接 clone 本仓库提供的完整 workspace 使用，不建议重新从各个上游仓库分别 clone 后再手动拼装。原因是本项目对 Unitree、MJLab、FAST-LIO、Livox 等开源包做过少量适配修改，直接使用完整 workspace 可以避免版本和接口兼容问题。

## 实现功能

- 使用 `unitree_mujoco` 启动 Unitree G1 MuJoCo 仿真。
- 使用 `unitree_rl_mjlab/deploy/robots/g1/g1_ctrl` 部署并运行 G1 行走策略。
- 支持 Nav2 输出 `/cmd_vel` 控制 G1 行走。
- MuJoCo 到 ROS2 的桥接节点发布：
  - `/scan`
  - `/livox/lidar`
  - `/imu/data`
  - `/odom`
  - `/tf`
- 在 G1 头部模拟 Livox Mid360 三维点云。
- 支持 `slam_toolbox` 进行 2D 建图。
- 支持 `FAST_LIO_ROS2` 进行 3D PCD 建图和定位实验。
- 支持 Nav2 读取保存好的 2D `.yaml/.pgm` 地图进行导航。
- 支持切换多个 MuJoCo 仿真场景，包括 VLN 公寓和乒乓球房间（`tt_room_29dof.xml`）。

## 总体数据流

```text
Nav2 目标点
  -> Nav2 planner/controller
  -> /cmd_vel
  -> g1_ctrl --cmd_vel
  -> MJLab / Unitree 行走策略
  -> Unitree MuJoCo 机器人运动
  -> shared memory
  -> mujuco_sim bridge
  -> /scan /livox/lidar /imu/data /tf
  -> SLAM / FAST-LIO / Nav2 / RViz
```

## 目录结构

新建工作空间并 clone 本仓库后，`src` 目录结构应类似：

```text
src/
  FAST_LIO_ROS2
  Livox-SDK2
  lidar_localization_ros2
  livox_ros_driver2
  maps
  mujuco_sim
  unitree_mujoco
  unitree_rl_mjlab
  unitree_ros2
  unitree_sdk2
```

主要目录说明：

```text
src/mujuco_sim/              # ROS2 launch 文件和 MuJoCo-ROS 桥接节点
src/unitree_mujoco/          # Unitree MuJoCo 仿真器，已加入导航共享内存和 Mid360 仿真
src/unitree_rl_mjlab/        # MJLab G1 策略部署和 g1_ctrl
src/FAST_LIO_ROS2/           # FAST-LIO2 ROS2 包
src/Livox-SDK2/              # Livox SDK2 依赖
src/livox_ros_driver2/       # Livox ROS2 驱动，主要用于实机
src/unitree_sdk2/            # Unitree SDK2
src/unitree_ros2/            # Unitree ROS2 示例和接口
src/maps/                    # 2D 地图和保存的 PCD 地图
```

## 本项目修改过的上游文件

如果你没有直接 clone 本 workspace，而是自己从上游仓库重新下载，需要至少同步这些改动，否则功能可能不完整：

- `src/unitree_mujoco/simulate/src/main.cc`
  - 接入导航共享内存 bridge。
- `src/unitree_mujoco/simulate/src/shm_nav_bridge.cc`
  - 从 MuJoCo 中生成 `/scan`、Mid360 点云、IMU、位姿等共享内存数据。
- `src/unitree_mujoco/simulate/src/shm_nav_bridge.h`
  - 定义共享内存数据结构。
- `src/unitree_mujoco/simulate/config/mid360_pattern.csv`
  - Mid360 仿真扫描 pattern。
- `src/unitree_mujoco/simulate/config.yaml`
  - 配置当前仿真场景。
- `src/unitree_mujoco/unitree_robots/g1/*.xml`
  - G1 导航场景、VLN 房间、乒乓球房间、雷达 site 等 MuJoCo XML。
- `src/unitree_mujoco/example/COLCON_IGNORE`
  - 避免 `stand_go2` 重名导致 colcon 编译失败。
- `src/unitree_rl_mjlab/deploy/robots/g1/main.cpp`
  - 支持键盘、默认控制、`/cmd_vel` 控制模式。
- `src/unitree_rl_mjlab/deploy/include/input/velocity_command_source.h`
  - 统一速度命令来源。
- `src/unitree_rl_mjlab/deploy/include/param.h`
  - 与部署参数相关的适配。
- `src/FAST_LIO_ROS2/config/mid360.yaml`
  - 仿真 Mid360 + MuJoCo IMU 的 FAST-LIO 配置。
- `src/FAST_LIO_ROS2/config/mid360_real.yaml`
  - 实机 Mid360 配置。
- `src/FAST_LIO_ROS2/src/laserMapping.cpp`
  - 增加 `/map_save` 服务，用于手动保存 PCD。
  - Foxy 分支中对 service callback 做了 Foxy 兼容适配。
- `src/mujuco_sim/`
  - 本项目新增的 ROS2 包，包含 launch、Nav2 参数、RViz 配置、MuJoCo 共享内存 bridge、FAST-LIO TF bridge。

因此，推荐做法是直接使用本仓库完整 workspace；如果你只 git 上游开源包，则需要把以上文件替换为本仓库版本。

## 环境要求

主分支主要在 Ubuntu 22.04 + ROS2 Humble 下测试。Ubuntu 20.04 + ROS2 Foxy 请优先使用本仓库的 `foxy` 分支，因为 Foxy 对部分 ROS2 C++ API、Livox 驱动和 FAST-LIO 编译方式有额外适配。

安装 ROS2 导航相关依赖。Humble 示例：

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rmw-cyclonedds-cpp
```

Foxy 示例：

```bash
sudo apt install \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox \
  ros-foxy-rviz2 \
  ros-foxy-tf2-ros \
  ros-foxy-pcl-ros \
  ros-foxy-pcl-conversions \
  ros-foxy-rmw-cyclonedds-cpp \
  libeigen3-dev \
  libpcl-dev \
  libapr1-dev \
  libpcap-dev \
  libusb-1.0-0-dev
```

`rmw_cyclonedds_cpp` 很重要。本项目的 launch 默认使用 CycloneDDS，如果缺少对应 ROS 版本的 RMW 包，会出现类似：

```text
failed to find shared library 'rmw_cyclonedds_cpp'
```

Unitree MuJoCo、Unitree SDK2、Unitree RL MJLab 的基础环境配置请参考 Unitree 官方文档。本项目默认你已经能够单独运行 Unitree MuJoCo 和 G1 `g1_ctrl` 控制程序。完整编译和实机雷达相关功能还需要配置 Livox-SDK2、`livox_ros_driver2` 和 FAST-LIO。

## 编译并安装 Unitree SDK2

`unitree_mujoco/simulate` 和 `g1_ctrl` 都依赖 Unitree SDK2，并且默认它安装在 `/opt/unitree_robotics`。请从本工作空间编译安装，不要使用 Unitree 官方仓库 —— 本工作空间的副本包含 `g1_ctrl` 必需的 `dds_wrapper` 头文件（官方仓库没有）：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF
make -j4
sudo make install
```

验证安装：

```bash
ls /opt/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake
ls /opt/unitree_robotics/include/unitree/dds_wrapper/robots/g1/g1.h
```

如果跳过这一步，后续编译 `unitree_mujoco/simulate` 会报：

```text
Could not find a package configuration file provided by "unitree_sdk2"
```

编译 `g1_ctrl` 会报：

```text
Could not find unitree_sdk2 include directory
```

如果编译 SDK 时报：

```text
fatal error: unitree/common/log/log.hpp: No such file or directory
```

说明你的克隆缺少 `src/unitree_sdk2/include/unitree/common/log/` 目录。原因是早期 `.gitignore` 中的 `log/` 规则（本意是忽略 colcon 的输出目录）意外把这些头文件排除出了仓库，现已改为锚定写法 `/log/`。可以从官方 SDK 恢复这些头文件（各上游版本中该目录内容完全一致）：

```bash
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2 /tmp/unitree_sdk2_official
cp -r /tmp/unitree_sdk2_official/include/unitree/common/log \
      ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/unitree_sdk2/include/unitree/common/log
```

该修复已在 Ubuntu 22.04 + GCC 11.4 上验证有效：恢复头文件后，SDK 可正常安装，`unitree_mujoco` 和 `g1_ctrl` 均可编译并运行。

## 编译 Unitree MuJoCo

如果编译 `unitree_mujoco/simulate` 时提示找不到 MuJoCo、`glfw_adapter.h` 或 `-lmujoco`，说明 MuJoCo C/C++ SDK 没有安装或没有正确链接。

安装 MuJoCo 3.2.7 示例：

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
find mujoco-3.2.7 -name "glfw_adapter.h"
find mujoco-3.2.7 -name "libmujoco.so"
```

在 Ubuntu 20.04 / ROS2 Foxy 上，如果链接时报：

```text
undefined reference to `shm_open'
```

需要修改 `src/unitree_mujoco/simulate/CMakeLists.txt`，在 `link_libraries(...)` 中加入 `rt`。例如：

```cmake
link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
  rt
)
```

然后重新编译：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/unitree_mujoco/simulate
rm -rf build
mkdir build
cd build
cmake .. \
  -DMUJOCO_DIR=$HOME/.mujoco/mujoco-3.2.7 \
  -DCMAKE_PREFIX_PATH=$HOME/.mujoco/mujoco-3.2.7
make -j4
```

编译成功后运行：

```bash
./unitree_mujoco
```

能看到 MuJoCo 中的 G1 仿真环境说明成功。

## 编译 MJLab / G1 控制器

先安装编译依赖：

```bash
sudo apt install libboost-program-options-dev libspdlog-dev libfmt-dev libeigen3-dev libyaml-cpp-dev zlib1g-dev
```

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/unitree_rl_mjlab/deploy/robots/g1
mkdir -p build
cd build
cmake .. -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
make -j4
```

`-DCMAKE_PREFIX_PATH=/opt/unitree_robotics` 是必需的。直接执行 `cmake ..` 会报：

```text
Could not find unitree_sdk2 include directory
```

因为 `/opt/unitree_robotics` 不在 CMake 的默认搜索路径中。参见上文「编译并安装 Unitree SDK2」一节。

运行键盘控制：

```bash
./g1_ctrl --network=lo --domain=1 --keyboard
```

如果 `g1_ctrl` 在打印 CycloneDDS 网卡信息后立刻崩溃：

```text
free(): invalid pointer
Aborted (core dumped)
```

说明进程加载了错误的 CycloneDDS 库。二进制的 RUNPATH 中记录了 `/opt/unitree_robotics/lib`，但 `LD_LIBRARY_PATH` 的优先级高于 RUNPATH —— 在 source 过 ROS 2 环境（或激活了 conda 环境）的终端里，会加载 ROS 自带的 `libddsc.so.0` 而不是 Unitree 的版本，导致 DDS 初始化时堆损坏。用以下命令检查：

```bash
ldd ./g1_ctrl | grep ddsc
```

`libddsc.so.0` 和 `libddscxx.so.0` 都必须解析到 `/opt/unitree_robotics/lib`。解决方法：在没有 source ROS 2 的终端里运行 `g1_ctrl`（它不依赖 ROS，直接通过 DDS 通信），或者显式指定库路径：

```bash
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
./g1_ctrl --network=lo --domain=1 --keyboard
```

该问题已在 Ubuntu 22.04 上验证：使用干净的库路径后，同一个二进制可正常启动并进入 `Waiting for connection to robot...`。`g1_nav_sim.launch.py` 已为 `unitree_mujoco` 和 `g1_ctrl` 进程自动前置 `/opt/unitree_robotics/lib`，因此在 source 过 ROS 2 的终端里使用 `ros2 launch` 是安全的。

等待控制器检测到机器人后，在 MuJoCo 界面点击 reset，机器人进入 FixStand / Velocity 流程。在运行 `g1_ctrl` 的终端里使用键盘控制机器人走动：

```text
w/s: x 方向
 a/d: y 方向
q/e: yaw 方向
```

能控制机器人在仿真环境中走动即说明基础控制成功。

## 编译 Livox-SDK2 和 livox_ros_driver2

如果需要实机 Mid360，或者需要编译包含 Livox 的完整工作空间，需要先配置 Livox-SDK2。MuJoCo 仿真本身不依赖真实 Livox 驱动，因为仿真的 `/livox/lidar` 由 `mujuco_sim` bridge 发布；但完整 workspace 或实机流程建议把这一步配置好。

以 ROS2 Foxy 为例，先编译并安装 Livox-SDK2：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

检查是否安装成功：

```bash
find /usr/local -name "livox_lidar_api.h"
ldconfig -p | grep livox
```

再编译 `livox_ros_driver2`。Livox 上游把清单文件命名为 `package_ROS1.xml` / `package_ROS2.xml`，由其 `build.sh` 生成 `package.xml`；而 `src/livox_ros_driver2/` 里旧的 `.gitignore` 规则把生成的 `package.xml` 排除在了仓库之外。缺少该文件时 colcon 会报：

```text
CMake Error: File .../livox_ros_driver2/package.xml does not exist.
...
Packages installing interfaces must include
'<member_of_group>rosidl_interface_packages</member_of_group>' in their package.xml
```

本仓库现在已直接提供 `package.xml`（内容即 `package_ROS2.xml`），并删除了对应的 `.gitignore` 规则。如果你的克隆早于该修复，请手动创建：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/livox_ros_driver2
cp package_ROS2.xml package.xml
```

注意：不要用 Livox 的 `build.sh` 来解决这个问题 —— 它会对整个工作空间的 `build/` 和 `install/` 目录执行 `rm -rf`。

ROS2 Humble 编译：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 \
  --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=/usr/local/include \
  -DLIVOX_LIDAR_SDK_LIBRARY=/usr/local/lib/liblivox_lidar_sdk_shared.so
```

`-DDISTRO_ROS=humble` 很重要：在 Humble/Jazzy 上 `CMakeLists.txt` 必须走 `rosidl_get_typesupport_target` 分支；不加该参数会退回到只适用于 Foxy 时代发行版的旧 typesupport 链接方式。

ROS2 Foxy 编译：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/foxy/setup.bash
colcon build --packages-select livox_ros_driver2 \
  --cmake-args \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=/usr/local/include \
  -DLIVOX_LIDAR_SDK_LIBRARY=/usr/local/lib/liblivox_lidar_sdk_shared.so
```

本仓库推荐直接使用完整 workspace，避免 Livox ROS2 驱动版本和 ROS 发行版不匹配。

## FAST-LIO Foxy 兼容说明

本仓库的 `foxy` 分支已经对 `src/FAST_LIO_ROS2/src/laserMapping.cpp` 做过 Foxy 兼容修改。如果你自己从 FAST-LIO 上游重新下载代码，需要特别注意 `/map_save` 服务的 callback 签名。Foxy 下应使用三参数 callback。

`create_service` 建议写法：

```cpp
map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "map_save",
    std::bind(
        &LaserMappingNode::map_save_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
```

对应回调函数建议写法：

```cpp
void map_save_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)request_header;
    (void)req;

    RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
    if (pcd_save_en)
    {
        save_to_pcd();
        res->success = true;
        res->message = "Map saved.";
    }
    else
    {
        res->success = false;
        res->message = "Map save disabled.";
    }
}
```

单独编译 FAST-LIO：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/{your_ros_distro}/setup.bash
colcon build --packages-select fast_lio
source install/setup.bash
ros2 pkg prefix fast_lio
```

`ros2 pkg prefix fast_lio` 有输出即说明安装成功。

## 编译整个工作空间

新电脑第一次拿到这个 workspace，或者拷贝整个工作空间后，建议先清空旧的编译产物再编译：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
rm -rf build install log
source /opt/ros/{your_ros_distro}/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

在 ROS2 Humble 上，请加上 Livox 相关参数，使 `livox_ros_driver2` 走正确的 typesupport 分支：

```bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

工作空间里有一些包与仿真流程无关，并且在普通 ROS2 环境下无法编译。这些目录现在都带有 `COLCON_IGNORE` 标记，因此上面这一条 `colcon build` 命令即可一键完成编译：

- `src/unitree_ros2/`（`unitree_api`、`unitree_go`、`unitree_hg`、`unitree_ros2_example`）：仅用于实机 ROS2 通信。报错 `Could not find ... "rosidl_generator_dds_idl"`，因为该生成器来自 Unitree 的 `cyclonedds_ws` 配置，标准 ROS2 中没有。
- `src/lidar_localization_ros2/`：实验性 NDT 定位，所有 launch 文件都没有用到。报错 `Could not find ... "ndt_omp_ros2"`，这是一个只能从源码克隆到 `src/` 的包。
- `src/unitree_sdk2/` 和 `src/Livox-SDK2/`：前面步骤中已手动安装到系统，在 colcon 里重复编译没有意义。
- `src/unitree_rl_mjlab/`：RL 训练仓库；部署用的 `g1_ctrl` 是单独用 CMake 编译的。
- `src/unitree_mujoco/`：仿真器需要手动用 CMake 编译（见上文「编译 Unitree MuJoCo」），launch 使用的是 `simulate/build/` 下的产物。colcon 编译它时缺少 MuJoCo SDK 参数，会报 `glfw_adapter.h: No such file` / `cannot find -lmujoco`。

以后如果需要重新启用某个包（例如配置实机时先编译 Unitree 的 `cyclonedds_ws` 再启用 `src/unitree_ros2`，或克隆 `ndt_omp_ros2` 后启用 `lidar_localization_ros2`），删除对应目录下的 `COLCON_IGNORE` 文件即可。如果你的克隆早于这些标记文件，请手动创建：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
touch src/unitree_ros2/COLCON_IGNORE src/unitree_sdk2/COLCON_IGNORE \
      src/Livox-SDK2/COLCON_IGNORE src/unitree_rl_mjlab/COLCON_IGNORE \
      src/lidar_localization_ros2/COLCON_IGNORE src/unitree_mujoco/COLCON_IGNORE
```

其余包的系统（apt）依赖可以用 rosdep 自动安装，不必逐个排查：

```bash
sudo rosdep init   # 仅第一次需要
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

注意 rosdep 的能力边界：它只安装 `package.xml` 中声明的二进制 apt 依赖（PCL、Nav2、tf2 等），无法提供厂商 SDK（Unitree SDK2、Livox-SDK2）、纯源码包（`ndt_omp_ros2`、`rosidl_generator_dds_idl`），也无法修复仓库本身的问题 —— 这些正是上文各节所解决的内容。`-r` 参数让它跳过无法解析的依赖继续执行。

对本工作空间来说 rosdep 是可选的：「环境要求」一节的 apt 安装列表已经覆盖了仿真流程所需的依赖。如果 `rosdep init` / `update` 超时（它从 `raw.githubusercontent.com` 下载数据，国内网络经常无法访问），可以直接跳过 rosdep，或者改用清华 TUNA 镜像：

```bash
sudo mkdir -p /etc/ros/rosdep/sources.list.d
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list \
  https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
rosdep update
```

建议把 `export ROSDISTRO_INDEX_URL=...` 写进 `~/.bashrc`，以后执行 `rosdep update` 也会走镜像。

launch 文件已不再硬编码工作空间路径：`WS_SRC` 会根据 `mujuco_sim` 的安装前缀自动推导（`<ws>/install/mujuco_sim` → `<ws>/src`），因此工作空间可以放在任意目录、使用任意名称。只有在非标准布局下（例如 `--merge-install`）才需要显式指定：

```bash
export G1_WS_SRC=~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src
```

如果修改了 launch 文件，需要重新编译并 source：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
colcon build --packages-select mujuco_sim
source install/setup.bash
```

如果遇到 `stand_go2` duplicate package 错误，请确认下面这个文件存在：

```text
src/unitree_mujoco/example/COLCON_IGNORE
```

如果 launch 启动后仍然搜索旧工作空间（例如日志中出现某个过期的 `.../old_ws/install` 路径），说明 shell 环境被旧 workspace 污染。建议新开终端并执行：

```bash
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/{your_ros_distro}/setup.bash
source ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/install/setup.bash
```

同时检查 `~/.bashrc`，不要自动 source 旧工作空间。

## 启动 MuJoCo + G1 控制

键盘控制模式：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=keyboard
```

Nav2 `/cmd_vel` 控制模式：

```bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=cmd_vel
```

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## 仿真场景

G1 的导航场景位于 `src/unitree_mujoco/unitree_robots/g1/`：

- `navigation_room_29dof.xml`：简单导航房间。
- `vln_apartment_29dof.xml`：VLN 公寓（之前的默认场景）。
- `tt_room_29dof.xml`：乒乓球房间（当前默认场景）。11 m x 8 m 房间，外墙与 VLN 公寓一致，内含一张 ITTF 标准乒乓球桌（2.74 m x 1.525 m，桌面高 0.76 m）和球网。G1 出生在原点，位于近端底线后方 0.63 m 处。

当前场景需要在两处保持一致：

1. 主仿真器：`src/unitree_mujoco/simulate/config.yaml` 中的 `robot_scene`，启动时读取，无需重新编译。
2. 相机桥接：`g1_nav_sim.launch.py` 的 `camera_model_path` 参数。可以在启动时覆盖：

```bash
ros2 launch mujuco_sim g1_nav_sim.launch.py \
  camera_model_path:=$HOME/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/unitree_mujoco/unitree_robots/g1/vln_apartment_29dof.xml
```

也可以直接修改 `src/mujuco_sim/launch/g1_nav_sim.launch.py` 中的默认值，修改后需要重新编译使 install 目录下的副本生效（使用 `--symlink-install` 编译的工作空间可跳过）：

```bash
colcon build --packages-select mujuco_sim
source install/setup.bash
```

切换场景时，Nav2 的 2D 地图（`nav.launch.py` 的 `map:=` 参数）也要换成在该场景中建立的地图，否则 AMCL 无法定位。

## 使用 slam_toolbox 进行 2D 建图

修改 `src/mujuco_sim/launch/map.launch.py` 顶部参数：

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = True
```

修改该参数后需要重新编译 `mujuco_sim`，安装目录里的 launch 文件副本才会更新（如果编译时用了 `--symlink-install` 则不需要）：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
colcon build --packages-select mujuco_sim
source install/setup.bash
```

启动建图：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

用键盘控制机器人走动建图，等 RViz 中的地图覆盖整个环境后再保存 2D 地图。

**注意：**`map.launch.py` 会把整个仿真放到一个独立的 DDS 网络上（`ROS_DOMAIN_ID=1`，CycloneDDS 且仅使用 loopback 回环网卡）。新开的终端默认在另一个网络上（domain 0，Fast DDS），看不到 `/map` 话题，此时 `map_saver_cli` 会报错 `Failed to spin map subscription`。所以保存前必须先在该终端里设置相同的环境变量：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/mujuco_sim/config/cyclonedds_lo.xml
```

先确认能看到地图话题，再执行保存：

```bash
ros2 topic list | grep map   # 应该能看到 /map
ros2 run nav2_map_server map_saver_cli -f ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/maps/new_2d_map
```

保存后会生成 `new_2d_map.pgm` 和 `new_2d_map.yaml`；同名旧文件会被覆盖。

## 使用 FAST-LIO 保存 PCD 地图

修改 `src/mujuco_sim/launch/map.launch.py` 顶部参数：

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = False
```

修改该参数后同样需要重新编译 `mujuco_sim`，方法见上面 2D 建图小节。

启动 FAST-LIO 建图：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

控制机器人在环境里走动。注意：不要依赖 `Ctrl+C` 保存 PCD，必须显式调用保存服务：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

和 2D 建图小节里的 `map_saver_cli` 一样，这个服务调用也必须在加入了仿真 DDS 网络的终端里执行——先设置相同的三个环境变量（`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI`），否则看不到该服务。

仿真 FAST-LIO 的 PCD 输出路径在下面文件里配置：

```text
src/FAST_LIO_ROS2/config/mid360.yaml
```

默认保存到：

```text
./src/maps/vln_fastlio_map.pcd
```

FAST-LIO 3D PCD 建图效果示例：

![FAST-LIO 3D PCD 建图效果](images/pcd.png)

## 启动导航

确认 `src/mujuco_sim/launch/nav.launch.py` 中使用的是需要的 2D 地图。默认地图为：

```text
src/maps/vln_navigation_room.yaml
```

启动导航：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim nav.launch.py
```

如果想使用其他地图，不需要改 launch 文件，直接传参数即可：

```bash
ros2 launch mujuco_sim nav.launch.py map:=$HOME/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/maps/new_2d_map.yaml
```

在 RViz 中发送 `2D Goal Pose`。Nav2 会发布 `/cmd_vel`，`g1_ctrl --cmd_vel` 接收后驱动 MuJoCo 中的 G1 行走。

定位工作原理：

- FAST-LIO 将 Mid360 点云与 IMU（`/imu/data`）紧耦合，提供高频局部里程计（`odom -> base_link`）。
- AMCL 用 `/scan` 与静态地图做匹配，持续修正 `map -> odom`，因此 FAST-LIO 的漂移不会在 map 坐标系中累积。
- RViz（`nav.rviz`）会显示全局路径（绿色）、DWB 局部路径（橙色）、全局/局部代价地图和 AMCL 粒子云。

避障同时使用两种传感器。2D `/scan` 从头部雷达（离地约 1.2 m）水平发射，会直接越过低矮家具——乒乓球桌（桌面高 0.76 m）根本不会出现在 SLAM 地图里。因此两个代价地图还额外订阅 Mid360 三维点云（`/livox/lidar`），并用 0.15–1.8 m 的高度过滤：低矮障碍物在运行时被实时标记，即使静态地图中是空白，规划器也会绕开它们。

如果机器人在地图上的位置明显不对，可以用 RViz 的 `2D Pose Estimate` 工具重新给 AMCL 设定初始位姿。

Nav2 联合仿真效果示例：

![MuJoCo Mid360 点云与 Nav2 地图显示](images/mujoco_3Dcloud.png)

![FAST-LIO 定位接入 Nav2 导航效果](images/nav2_fastlio.png)

## 实机部署说明

实机 G1 上的大致流程：

1. 按 Unitree 官方文档配置网络、SDK2 和 ROS2。
2. 启动真实 Mid360 驱动，确认存在 `/livox/lidar`。
3. 如果要用 `slam_toolbox` 建 2D 图，需要用 `pointcloud_to_laserscan` 把 Mid360 的 `PointCloud2` 转成 `/scan`。
4. 保存 `.yaml/.pgm` 地图后给 Nav2 使用。
5. 在真实机器人网络接口上运行 `g1_ctrl`，例如：

```bash
cd ~/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=enp5s0
```

建议先验证键盘控制和策略部署没问题，再切换到 `/cmd_vel` 模式接入 Nav2。

## Launch 文件说明

- `g1_nav_sim.launch.py`：启动 Unitree MuJoCo、`g1_ctrl` 和共享内存 ROS2 bridge。
- `map.launch.py`：启动建图流程；在文件顶部切换 2D `slam_toolbox` 或 FAST-LIO PCD 建图。
- `nav.launch.py`：启动仿真、FAST-LIO 定位桥接、Nav2 和 RViz。

## DDS 说明

仿真默认使用 `ROS_DOMAIN_ID=1`，并通过下面配置让 CycloneDDS 使用本机 loopback：

```text
src/mujuco_sim/config/cyclonedds_lo.xml
```

本仓库的 launch 文件会自动设置这些环境变量，但只对它启动的节点生效。任何手动打开、需要和仿真交互的终端（`ros2 topic`、`ros2 service`、`map_saver_cli` 等）都必须先设置相同的环境变量，否则该终端处于默认 DDS 网络（domain 0，Fast DDS），看不到任何仿真话题：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/Python_project/G1_ROS/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src/mujuco_sim/config/cyclonedds_lo.xml
```

建议把这三行放进一个脚本（例如 `~/g1_sim_env.sh`），每开一个新终端 `source` 一下即可。

如果提示找不到 `rmw_cyclonedds_cpp`，请安装对应 ROS 版本的包，例如：

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
# 或
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
