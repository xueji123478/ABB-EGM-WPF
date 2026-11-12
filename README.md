# ABB-EGM-WPF — ABB EGM 上位机示例

基于 **.NET 8 + WPF** 的 ABB **EGM（Externally Guided Motion，外部引导运动）** 姿态引导示例程序。
程序通过 UDP 与 IRC5 控制器交换 **Protobuf** 编码的 `EgmSensor` / `EgmFeedBack` 消息，
周期性下发目标位姿，并在界面上实时显示机器人反馈位姿、关节角、会话状态与日志。

> 本项目为学习/演示用途，涉及真实机器人运动时请务必先在 RobotStudio 仿真中验证。

---

## 功能

- **手动点动**：平移步长 10 mm（基坐标系 X/Y/Z）、旋转步长 2°（欧拉角 Rx/Ry/Rz）
- **目标位姿输入**：直接填写目标 X/Y/Z/Rx/Ry/Rz，`应用` 后作为发送目标
- **同步目标**：一键把发送目标同步为机器人当前反馈位姿
- **安全限幅**：目标与当前反馈偏差超过 平移 50 mm / 旋转 10° 时拒绝下发
- **会话状态机**：`DISCONNECTED → CONNECTED → RUNNING`，仅 `RUNNING` 时允许移动
- **超时保护**：200 ms 收不到反馈判定断开并暂停发送；目标 500 ms 未被刷新则切换为“保持当前位置”
- **周期发送**：约 4 ms 一帧（实际 60~100 Hz），带递增序列号与毫秒时间戳
- **日志面板**：按级别着色的运行日志，可一键清空

## 目录结构

| 文件 | 说明 |
| --- | --- |
| `App.xaml` / `App.xaml.cs` | 应用入口，`StartupUri` 指向主窗口 |
| `MainWindow.xaml` / `MainWindow.xaml.cs` | 主界面（状态卡片、点动、位姿对比、目标输入、日志）与交互逻辑 |
| `EgmClient.cs` | UDP 收发、会话健康度、后台发送线程与事件回调 |
| `EgmProtocol.cs` | EGM 消息组包（`EgmSensor`）与解析（`EgmFeedBack`） |
| `Egm.cs` | 由 `.proto` 生成的 Google Protobuf 代码（命名空间 `Abb.Egm`） |
| `abb.txt` | 机器人端 RAPID 示例模块（`EGM_POSE_MOVEMENT`） |
| `WPFProject.sln` / `WPFProject.csproj` | 解决方案与项目文件（`net8.0-windows`，`UseWPF`） |

## 环境要求

- Windows 10 / 11
- [.NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0)
- Visual Studio 2022 17.8+（可选，用于调试）
- NuGet 包：`Google.Protobuf` 3.21.7、`Google.Protobuf.Tools` 3.21.7

## 构建与运行

```powershell
dotnet restore
dotnet build -c Release
dotnet run
```

或直接用 Visual Studio 打开 `WPFProject.sln`，按 `F5` 启动。

首次运行请在 **Windows 防火墙** 中放行程序的入站 **UDP 6510** 端口，
否则控制器发出的 EGM 反馈会被拦截，界面会一直停留在 `DISCONNECTED`。

## 使用步骤

1. **机器人端配置 EGM**：在 RobotStudio / 示教器中配置 UDPUC 通信，设备名填 `PC`，
   对端 IP 填运行本程序的电脑 IP；在 RAPID 中运行 `abb.txt` 里的 `EGM_POSE_MOVEMENT`。
2. **启动上位机**：程序在本地 `6510/UDP` 端口监听，等待控制器反馈。
3. **确认状态**：状态依次变为 `CONNECTED`（收到反馈）→ `RUNNING`（`EGMRunPose` 已进入）。
4. **下发目标**：使用点动按钮，或在“目标位姿”区域输入数值后点“应用”；
   `同步` 会把目标重置为当前反馈位姿。
5. **观察对比**：界面同时显示“当前反馈（机器人）”和“当前发送目标”，并给出偏差与关节角。

## 协议要点

- **端口**：上位机侧 UDP `6510` 监听；机器人向该端口发送 `EgmFeedBack`，上位机回发 `EgmSensor`。
- **消息类型**：`EgmHeader.mtype = MSG_TYPE_CORRECTION`，`seqno` 每帧递增，
  `tm` 取 `Environment.TickCount64` 的低 32 位（约 49 天回绕，对控制器无影响）。
- **位置单位**：`pos` 为 mm，`euler` 为度，姿态以欧拉角（ZYX）表示。
- **发送节奏**：EGM 要求外部设备周期性发送修正量，本示例固定 4 ms 周期；
  目标超时（500 ms 未刷新）时持续发送“保持当前位置”，避免机器人停在旧目标上。

## 安全提示

- EGM 会让机器人**跟随外部指令运动**，调试时请降低速度、保持急停可达。
- 代码中的偏差限幅（50 mm / 10°）只是软件保护，不能替代机器人侧的安全配置。
- 建议先在 RobotStudio 虚拟控制器上联调，再上真机。
