# 实机局部避障项目

当前运行入口是 `algorithm/onboard/src/onboard_main.cpp`。独立仿真工程、主程序仿真模式、合成雷达和虚拟 RTK 台架已移除。离线测试保留在 `test/`，用于验证实机代码，默认不编译。

```text
上电 WAIT_KML → 文件稳定/校验成功 → 自动启动任务 → 地面站解锁起飞
                                      ↓
飞控串口 MAVLink → RTK/姿态/HOME/TIMESYNC → 位姿历史
                                              ↓
Mid360S → 以太网 SDK2 → PTP/组帧 → 时间对齐/去畸变 → EGO 地图/规划
                                                        ↓
                                       串口位置+yaw目标 → 飞控
                                       UDP JSON状态 → 地面站（可选）
```

## 编译与依赖

Ubuntu 需安装 C++ 编译器、CMake、Eigen3、yaml-cpp、libxml2；硬件观察脚本还需 Python3 和 linuxptp。从项目根目录执行：

```bash
# 初始化固定版本的 SDK，并应用项目所需的数据长度/CRC 校验补丁。
# 已有补丁时直接返回，不覆盖本地改动。
bash lidar/setup_sdk.sh
cmake -S algorithm/onboard -B .build/onboard -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build .build/onboard -j4
```

SDK 的现有本地补丁已保存到 `lidar/patches/mid360s-data-validation.patch`。保留完整 SDK 源码和第三方依赖，主程序只构建所需静态库。历史抓包、协议文档和部署记录继续保留；其中的仿真、虚拟 RTK 和旧构建路径属于历史步骤，当前操作以本文件为准。生成的构建目录、Python 缓存和后续抓包目录已加入忽略规则。

## 实机观察

部署配置为 `algorithm/onboard/config/onboard.yaml`，雷达网络配置为 `algorithm/onboard/deploy/mid360s.json`。当前雷达 IP 为 `192.168.1.173`，主机地址为 `192.168.1.50/24`，串口默认 `921600` 波特率，目标飞控为 `1/1`。

避障按含桨叶机体包络半径 1.24 m、桨尖设计余量至少 1.5 m 配置：`vehicle.min_obstacle_gap=1.5`，`map.obstacles_inflation=2.75`（0.25 m 栅格下为 11 格），名义桨尖余量 1.51 m。配置加载会拒绝膨胀小于机体半径与余量之和的参数。虚拟地板偏移和地图高度余量同步改为 3.25 m，以保持原有巡航高度处的可飞空间。实际定位、外参和跟踪误差仍需实机验收。

先为雷达物理网卡配置地址并启动 PTP：

```bash
sudo bash algorithm/onboard/scripts/setup_lidar_network.sh <雷达物理网卡>
sudo ptp4l -i <雷达物理网卡> -f algorithm/onboard/deploy/ptp4l-master.conf -m
```

在另一个终端运行，替换串口设备；记录目录每次使用新名称：

```bash
.build/onboard/onboard_main \
  --serial /dev/serial/by-id/<飞控设备> --baud 921600 \
  --config algorithm/onboard/config/onboard.yaml \
  --livox-config algorithm/onboard/deploy/mid360s.json --lidar-ip 192.168.1.173 \
  --observe-only --kml 26-9-21-1.kml --duration 120 --record-dir /tmp/uav-observe-001
python3 algorithm/onboard/scripts/check_link_capture.py /tmp/uav-observe-001
```

观察模式只发心跳、遥测请求、HOME 查询和校时，不上传任务或发送运动目标。观察程序正常退出只代表采集结束；`check_link_capture.py` 进一步检查最后约 5 秒内的 RTK、位姿、飞控校时、PTP、点云对齐、地图持续更新和 CRC 增长，条件不满足时返回非零。它验证输入链路，不代表飞控接受/执行控制目标或外部地面站收到 UDP。

现有一键观察脚本 `verify_bench.sh` 使用固定的 ACFLy USB by-id 设备名并管理 PTP；它会编译和自检、采集并校验输入链路。`verify_lidar.sh` 仅检查真实雷达输入，不打开飞控串口。WSL 的串口直通/TCP 桥接脚本及独立协议探针仍保留。

UDP 状态默认关闭；需要时追加 `--uplink <地面站IPv4>:14560`，以约 2 Hz 上报 JSON。接收端和防火墙需在实际地面站验证。

## 控制入口

程序默认等待 `missions/mission.kml`。文件不存在时保持 `WAIT_KML`，不打开飞控串口或雷达 SDK；文件稳定至少 0.5 秒并通过校验后，自动继续启动。文件无效时记录原因并继续等待修改，不退出。同一次运行只接受一条任务，任务就绪后替换文件不会改写当前航线。路径和等待行为由 `mission_input.file`、`wait_for_file`、`file_settle_s` 配置；`--kml` 可覆盖路径。

云端接收程序只需把下载完成的文件原子交付到这个路径，不需要再发送“开始”指令。推荐同目录临时文件写完后重命名：

```bash
cp <收到的KML文件> missions/mission.kml.tmp
mv missions/mission.kml.tmp missions/mission.kml
```

该项目提供接收路径的自动检测，云端通信/下载仍由外部接收程序完成。已有有效文件在启动时也会被加载；完成任务后应归档或移走旧文件，再重启服务等待下一条。`--duration` 从文件校验成功、设备启动后计时，等待 KML 不超时；等待期间可以用 SIGTERM/Ctrl-C 正常停止。

先离线校验航线：

```bash
.build/onboard/onboard_main --prepare-only --config algorithm/onboard/config/onboard.yaml --kml 26-9-21-1.kml
```

`--prepare-only` 始终立即校验并返回，缺文件或坏文件会报错，不进入等待。纯雷达检查 `--lidar-only` 不读取 KML；需要无任务的飞控观察时使用 `--observe-only --no-wait-kml`。

手动实机控制在观察命令基础上使用 `--enable-control`；开机启动脚本已包含该开关。有效 KML 到达后仍会检查雷达配置/IP及 `extrinsic.confirmed=true`。当前保留 `false`，没有用默认单位阵冒充已标定外参。RTK 固定解、校时、位姿覆盖、地图健康及接管位置/高度条件继续生效。地面站负责解锁、起飞和落地；串口断开时主程序停止并返回失败。

## 机载 Linux 开机启动

在实际 RK3588 的 Linux（systemd）上完成一次部署即可；之后每次上电由操作系统自动启动，无需登录、SSH、打开终端或手工执行程序。首次部署需要已安装 CMake、g++、Eigen3、yaml-cpp、libxml2、Python3 和 linuxptp；替换运行用户、物理雷达网卡和实际串口设备，执行一次：

```bash
sudo bash algorithm/onboard/scripts/install_boot_service.sh <运行用户> <雷达物理网卡> /dev/serial/by-id/<飞控设备>
```

安装程序会在 RK3588 本机自动编译 ARM 程序、检查可执行性、验证 systemd 配置，并启用下次开机启动，不立即启动硬件进程。它会生成 `/etc/uav-onboard.env` 和两个 systemd 服务：`uav-onboard` 在文件系统挂载、基本系统服务就绪后自动进入 KML 等待，`uav-lidar-ptp` 并行等待网络并为指定物理网卡添加雷达地址/专用路由、启动 PTP。主程序不等待登录或雷达上线才进入 WAIT_KML。现有 Wi-Fi 默认路由不作更改。安装脚本拒绝在 WSL 上部署；开发电脑不作为飞机开机服务运行。不要把开发机的 x86 构建目录拷到 RK3588 使用。

开机顺序为：`系统启动 → 自动运行主程序 → WAIT_KML → 文件稳定/校验通过 → 自动启动任务流程`。主程序及 PTP 日志由系统 journal 收集，不依赖终端常驻。安装脚本会检查两个服务都已设为开机启用；只有在维护、排错或更新部署时才需要命令行。

`/etc/uav-onboard.env` 默认 `UAV_MODE=control`，有效文件到达后自动进入已有控制状态机；改成 `observe` 可仅做观察。主程序服务每次启动处理一条任务，不在故障退出后自动重新执行磁盘上的旧任务。通过 `journalctl -u uav-onboard -f` 查看等待、文件拒绝和任务启动日志。下次任务需先归档旧文件，再执行 `systemctl restart uav-onboard`。

服务仅接通启动流程，当前真实外参及时间对齐问题仍需解决；这次改动没有放宽这些门槛。

当前开发环境为 x86_64/WSL，RK3588 尚未连接。已具备部署脚本和服务配置，尚未在板上安装或完成实际断电重启验收。

## 离线回归

```bash
cmake -S algorithm/onboard -B .build/check -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build .build/check -j4
(cd .build/check && ctest --output-on-failure)
```

测试包含 MAVLink 编解码/报文更新、PTY 串口收发/断线、生产观察程序的 UDP 上报、任务状态机、KML、校时、感知保护和实测点云包回放入图。测试中的固定报文、假飞控和回放位姿仅供回归使用，不进入生产程序。

2026-10-30 验证：独立实机 Release 构建成功，16/16 回归通过；实机产物没有假飞控/合成场景符号。抓包检查脚本用健康及故障夹具验证了通过/拒绝行为；SDK 补丁可从固定版本源码精确复现当前修改。

本轮环境未接出飞控串口，也未配置雷达主机地址，未进行硬件采集或飞行。此前记录中飞控 `command=1084,result=2`、HOME 查询失败及校时不持续仍需接机复验；协议未明确 1084 与位置目标的映射，不能据此认定控制目标被接受或拒绝。实机闭环还需验证真实外参、遥测实际频率、飞控目标接受/执行和失联处理。历史证据见 `链路与报文核验报告-2026-10-25.md`。
