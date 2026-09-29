# AS5045 + DRV8825：图形轨迹执行

目标：CY8CKIT-059 / PSoC 5LP，24 MHz，PSoC Creator 4.4。
当前默认入口 `DRV8825.cywrk`，固件标识 `stage2-v7`。实际 XY / 图形执行已开放，可用人工伸直零位替代开关回零。

2026-09-29：当前交付 `deliverables/stage2-workbench-v7-global-motion`，版本目录由 `RELEASE_VERSION` 指定。尺寸、范围与速度统一用于所有图形；关节速度/加速度也覆盖点动和回零，仍受设备各模式上限约束。预览与执行共用时间规划，补检采样点之间的越界；示例不再重置全局参数。须配套更新 `stage2-v7-global1` 固件及完整上位机目录。详见 [WORKBENCH_HARDENING.md](WORKBENCH_HARDENING.md) 和 [实机日志复核](RUN_REVIEW_20260929.md)。

2026-09-27：历史交付 `deliverables/stage2-workbench-v6-upload-retry`。修复静止上传期间的纯 UART 停止位错误恢复：丢弃坏帧、保留已确认节点与参考、由现有有限重传继续；运动和其它 UART 错误仍停止。须烧录本包 `firmware.hex`，现用 v6 分段上位机兼容。详见 [WORKBENCH_UPLOAD_RETRY.md](WORKBENCH_UPLOAD_RETRY.md)。保留此前 600 起点速度、约 ±0.50° 终点容差、按钮视觉反馈和后台自动运行，临时关节范围不变；精度调整见 [WORKBENCH_MOTION_FIRST.md](WORKBENCH_MOTION_FIRST.md)，速度与按钮见 [WORKBENCH_BUTTON_FEEDBACK.md](WORKBENCH_BUTTON_FEEDBACK.md)，参数与范围见 [WORKBENCH_PARAMETERS.md](WORKBENCH_PARAMETERS.md)。旧交付保留。

2026-09-26：历史交付 `deliverables/stage2-workbench-v6-uart`，硬件 UART 与整帧发送升级，保留 UI3 布局、校验重试和停止保护。操作、版本配套与验收见 [STAGE2_HARDWARE_UART.md](STAGE2_HARDWARE_UART.md)。当次未烧录或进行实机通信验证。

2026-09-25：历史 v5 交付 `deliverables/stage2-workbench-v5`，包含重新设计的实验工作台、起点确认、操作防呆、中文故障反馈、arm1 误差曲线及自动实验记录；保留有限重传。完整操作和验收说明见 [WORKBENCH_V5.md](WORKBENCH_V5.md)。已接通“低速移到轨迹起点 → 整条上传并确认 → 双轴定时插补 → 到位检查”，支持直线、矩形、圆；代码和模拟验证完成，未烧录或实机验收。按用户要求设 `PTP_COMMISSIONED=1`，暂用 q1 ±30° / q2 ±60° 软件范围；它们不是已测量的机械范围。默认圆心 (340, 0) mm、半径 5 mm。电流、回零复测和正式标定仍未完成。详见 [STAGE2_EXECUTION.md](STAGE2_EXECUTION.md)。旧交付保留；新版主机兼容 stage1-v3 / stage2-v1 对应基础功能。

当前版本的电机参数、双轴伸直与复测步骤：[STAGE1_V3_NOTES.md](STAGE1_V3_NOTES.md)。此前开关角色修正记录见 [STAGE1_V2_NOTES.md](STAGE1_V2_NOTES.md)。

已有固件支持静态上电自检、人工使能、低速点动、可中止的手动回零、状态与日志；图形执行需要通过人工确认或开关回零建立坐标参考，并保留软件范围、限位、编码器和心跳检查。编译及模拟测试不能代替实机验收。

最新固件：`deliverables/stage2-workbench-v7-global-motion/firmware.hex`（支持全局关节范围与速度，需重新烧录），PTP 与图形执行已开放。旧 `deliverables/stage2-execution` 锁定交付保留。展开偏移仍为 127° / 163°，尚未正式标定。旧 `stage2-offset/firmware.hex` 保留原样，不含本次 PTP 协议。

当前源码将 q1、q2 都定义为相对固定 +X 的连杆角，第二轴电机角为 q2−q1；旧 `global1` 固件按另一坐标约定运行，新上位机只接受 LINK 中同时声明 `fw=stage2-v7-absolute-q2` 和 `q2_absolute=1` 的固件。配套固件与上位机已构建并通过离线闸门（`deliverables/stage2-workbench-v7-absolute-q2`，tag `gate-green/stage2-v7-absolute-q2`），未烧录、未实机验证；旧交付目录不可与新上位机混用。q2 语义变化影响限位怎么填，见「坐标与反馈边界」一节的「关节范围填写规则」。

## 使用顺序

一键软件构建与验证：`powershell -ExecutionPolicy Bypass -File .\build-release.ps1`，不烧录、不连接串口。

**试验阶段无需强制回零：人工把两杆摆直 → 使能 → 点击“人工确认：当前位置为伸直零位” → 移到轨迹起点 → 上传并执行。** 详见 [STAGE2_MANUAL_REFERENCE.md](STAGE2_MANUAL_REFERENCE.md)。以下为仍可选择的开关回零流程。

1. 烧录后上电：两轴保持禁用，约 5 秒静态自检，不自动探测方向、不自动回原点。
2. 运行 `deliverables/stage2-workbench-v7-unified-limits/host/TwoLinkCommissioning.exe`，只做图形或 XY 预览时无需连接。需要控制时选择 KitProg COM 口；连接先请求停止并释放保持力矩，不会自动使能。切窗或最小化时，已启动的回零、起点移动、上传及轨迹执行继续；关闭窗口或断开连接仍请求停止。
3. 读取输入并逐一检查开关。按 C / “使能”后允许低速点动，按住 A/D 控制 arm1，左右键控制 arm2，松开减速停止。手动点动仅在窗口激活时有效；失焦释放点动，回到窗口后需重新按下，不自动恢复。
4. 全部松开且静止后按 R / “开始回零”。本窗口激活时，Space / Esc 可在任意阶段停止；它们不是系统全局快捷键，在其他程序中按下不会发送停止指令。
5. arm1 在 P0[3] 快碰、退开、慢碰后停稳 50 ms，以至少 1 ms 间隔读取 12 个有效编码器角度，处理跨零后取平均作为原点，再反向走 127° 到伸直位；随后 arm2 对 P1[0] NPN 做同样两次触碰并置零，再向左展开约 163°（36,990 微步）并停稳。NPN 安装在 arm1 上，回零时两杆内夹角约 20°；163° 包含用户要求的目测追加 3°，仍待标定。
6. 只有收到固件 `HOME OK` 且 `STATE` 显示 `homed1=1 homed2=1 home=0` 才是流程成功。停止或失败会作废坐标参考；重新使能后可人工摆直确认，也可重新回零。

详细操作、日志预期与实机验收表：[STAGE1_ACCEPTANCE.md](STAGE1_ACCEPTANCE.md)。
接线：[WIRING.md](WIRING.md) 与 [wiring.svg](wiring.svg)。

## GitHub 版本管理

当前项目独立保存在私有仓库 [AGI-Junkie/792-AS5045](https://github.com/AGI-Junkie/792-AS5045)，包含 PSoC 工程与原理图、固件源码、C# WPF 上位机、离线检查和接线/验收文档。保留原有提交历史；本目录 `origin` 指向新仓库，旧 `792_DRV8825` 地址保存在 `legacy`。

构建输出、`Generated_Source`、`deliverables`、本机配置及临时备份不提交。克隆后安装 PSoC Creator 4.4、.NET 10 SDK 和 Visual Studio C++ 工具，再运行 `build-release.ps1` 生成配套固件与上位机。Creator 安装路径不同时使用 `-Creator '<cyprjmgr.exe 的绝对路径>'`；当前离线 C 测试要求 Visual Studio 18 Community 的默认安装路径。

在当前项目目录保存后续修改：

```powershell
Set-Location G:\792\AS5045_enc
git status
git add .
git diff --cached --stat
git commit -m "Describe the change"
git push
```

换电脑时执行 `git clone https://github.com/AGI-Junkie/792-AS5045.git`；更新前先提交本地修改，再执行 `git pull --ff-only`。私有仓库需要登录有访问权限的 GitHub 账号。

## 坐标与反馈边界

- arm1：42 电机接 stepper2，1.8° 整步、1/8 微步、40:1 减速；AS5045 在电机后轴，使用多圈累计。P0[3] 开关触点为零；到伸直位后不再次清零，因此显示约 ±127°，符号取决于实测编码器方向。
- arm2：Anaheim Automation `11YPG302S-LW4-R51` 接 stepper1，步距角 1.8°、减速比 51.06:1、当前 1/8 微步，按减速箱输出轴直接驱动关节计算。NPN 触点是命令零点，伸直后计数为 -36,990，不再次清零。没有编码器，`commanded usteps` 不是实测角度。[厂家参数](https://anaheimautomation.com/11ypg302s-lw4.html)
- 静态自检通过只表示编码器帧和当前 GPIO 电平符合要求，不能证明 VMOT 已有电、马达转动、传感器接线正确或输出轴定位精度。
- INC=DEC=1 保留磁场警告，在其他角度有效条件满足时接受。编码器失效不会自动降级开环继续运动。
- 回零成功不等于完成精度标定，AS5045 在电机侧不能测量减速箱回差。

### 关节范围填写规则（absolute-q2 固件起）

q1、q2 都是相对固定 +X 的连杆绝对角，肘（电机 2）转过的角度是 q2−q1。界面里 q2 的最小/最大限制的是连杆 2 的朝向，**不直接限制肘**。肘的实际范围由四个数联合决定（用真固件离线测过：q1 ±135°、q2 ±155° 时肘可达 −290°…+290°，目标 q1=+135°、q2=−155° 会被固件接受）：

    肘最小 = q2 最小 − q1 最大        肘最大 = q2 最大 − q1 最小

规则：肘最小 ≥ −E，肘最大 ≤ E（伸直为 0，向左为正；NPN 回零点在右侧约 −163°，左侧没有开关）。q1 ±A、q2 ±B 对称时即 A + B ≤ E。**固件和上位机都不检查这条规则，靠操作者。**

E 还没测量，暂按 155°（沿用旧版的肘限位 ±155°，不是实测机械行程）。测出两侧行程后改这里。

例（E = 155°）：

- 通用：q1 ±90°、q2 ±65°。
- 只画 x 260…340 mm、|y| ≤ 50 mm（连杆 180 / 176.38 mm，取肘 >0 的解，需要 q1 约 −53…−7°、q2 约 +7…+54°）：q1 [−60, 5]、q2 [−5, 60]，肘 −10…+120°。

上位机「图形参数」页在关节范围下方有「肘安全行程 E」输入框和一张 q1–q2 平面图，改范围或 E 后约 60 ms 内刷新：蓝框是固件允许的范围，绿带是肘在安全行程内的区域，框内落在绿带外的部分标红，黑线是当前图形需要的关节角；文字给出肘可达范围和超出量。只做显示，不拦截同步；E 不保存，重启恢复 155°。

开机默认（q1 ±30°、q2 ±60°，肘 ±90°）满足规则。上位机启动会自动载入 `%LOCALAPPDATA%\TwoLinkCommissioning\parameters.json` 里保存的限位，并在连接后同步给固件；2026-09-29 保存的是 q1 ±140°、q2 ±155°，在本固件下肘可达 ±295°，**不满足，首次连接前先改这四个框**。

## 保护与调试参数

回零是非阻塞状态机，使用已有加减速算法。两轴快碰和慢碰都持续搜索，直到各自回零开关触发，没有搜索步数或搜索时间上限。整个流程持续接收串口、监测编码器、nFAULT、全局限位及 250 ms 心跳超时。无通信、按停止、回零期间按点动键、编码器失效、跟随误差过大均停止，禁止自动恢复。释放、退开和伸直阶段仍保留有限行程与时间检查。

总停止（原始 0x79，包括主机关闭/断连时的停止请求）禁用两轴驱动，释放保持力矩并作废坐标参考；不单独锁存驱动故障，输入正常、松开按键后可重新使能。使能后仍须回零或重新确认真实伸直参考，不能沿用释放前的坐标。固件自身通信超时、限位和编码器保护停止仍保留保持电流；nFAULT 禁用驱动。禁用驱动不等于切断 VMOT 电源，独立实体急停仍需由硬件实现。

`main.c` 顶部保留可调参数：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `JOG1_RATE_MAX_USTEP_S` | 800 | arm1 低速验收，4.5 输出度/秒 |
| `JOG2_RATE_MAX_USTEP_S` | 600 | arm2 低速验收 |
| `HOME_FAST_RATE_USTEP_S` | 600 | 回零快碰 / 释放 / 退开上限，沿用较低的点动速度上限 |
| `HOME_SLOW_RATE_USTEP_S` | 400 | 二次触碰上限 |
| `HOME_STRAIGHT_RATE_USTEP_S` | 800 | arm1 伸直段上限，沿用 arm1 点动速度上限 |
| `HOME2_STRAIGHT_RATE_USTEP_S` | 600 | arm2 伸直段沿用已测试低速；匀速部分约 61.7 秒，实际受调度和加减速影响，需核实实物对齐 |
| `HOME_ZERO_SAMPLES` / `HOME_ZERO_SAMPLE_MS` | 12 / 1 ms | arm1 停稳后平均采样次数 / 最小间隔；采样期间仍处理心跳和停止 |
| `HOME_ACCEL_USTEP_S2` | 4000 | 回零加减速 |
| `HOME_BACKOFF_USTEPS` | 64 | 释放传感器后的额外退开 |
| `HOME_RELEASE_MAX_USTEPS` | 800 | 传感器必须释放的最大行程 |
| 快碰 / 慢碰结束条件 | 开关触发 | 两轴均不设搜索步数或搜索时间上限 |
| `HOME_PHASE_TIMEOUT_MS` | 120000 | 释放、退开、停稳及伸直阶段时间上限；不用于快碰或慢碰搜索 |
| `ARM1_STRAIGHT_OUT_DEG10` | 1270 | 开关到伸直位偏移 127.0°，需实机确认 |
| `ARM2_STRAIGHT_OUT_DEG10` | 1630 | NPN 到伸直位 163.0°，根据实际姿态微调 |
| `ARM2_STRAIGHT_JOG_DIR` | -1 | 用户确认：向左展开 |
| `ARM2_MOTOR_FULLSTEPS_REV` / `ARM2_GEAR_RATIO_X100` | 200 / 5106 | arm2 独立传动参数；不使用 arm1 的 40:1 |

2026-09-27 回零修复已合入 v7 当前交付，需使用配套固件与上位机。此前 arm2 快碰在 40000 步停止，临时扩大为 52000 步的包已被本包替代。按用户要求，快碰和二次慢碰均改为持续搜索直到开关触发；250 ms 心跳、全局限位、nFAULT、编码器及停止保护保留。若回零开关始终不触发，搜索会持续，直到操作者停止或其他保护触发。离线检查不能证明实际传感器触发或机械行程，实机回零仍待复测。

两轴物理左转均沿用已记录的 DIR=0。编码器极性在人工启动回零后的实际运动中识别；无需上电试走。极性尚未识别时，点动不执行依赖极性的闭环保持及软限位，仍有输入、通信和编码器有效性保护。成功识别后才启用相关监控。

## 串口与脉冲时序

以下为 2026-09-25 软件 UART 历史实现；当前 v7 已使用 TopDesign 中的 UDB 硬件 UART 与中断缓冲，时基和控制协议见 WORKBENCH_HARDENING.md。

2026-09-25：在不改接线与协议的前提下，把两处硬时限收成 SysTick 绝对时刻调度：

1. **主机串口接收**：`wait_until_period` 步进间隙调用 `uart_rx_pump`，探到起始位后按绝对位时刻收完整帧，结果进环形缓冲；`su_rx_service` 只出队。原先只有工作批次之间的 40 µs 听窗，忙起来会丢字节；现在发脉冲之间的等待窗口也收。仍为 115200 8N1、P12[6]/P12[7]、奇数字节、三份重复协议，主机无需改。
2. **步进脉冲**：`pulse_step_pin` / `pulse_hold_us` 用 SysTick 截止时刻保证 `STEP_LOW_US` 宽度，不再叠加相对延时；`delay_us` 只服务短 SSI/DIR 建立，不在此收帧以免拉长脉冲。

该历史版本尚未加入 UDB 硬件 UART，之后已由 v6/v7 替代。历史构建（Flash 约 24.8 KB）及 214 项离线仿真记录不代表当前版本或实机验收。

## 协议与日志

v7 使用 115200 8N1 硬件 UART，RX/TX 中断缓冲各 256 字节。启动与点动均使用 CRC / 控制序号 / epoch 校验帧，具体格式见 [WORKBENCH_HARDENING.md](WORKBENCH_HARDENING.md)。

| 命令 | 行为 |
|---|---|
| 控制帧 type=6 | 方向状态与心跳；mask=0 释放点动，不取消自动回零 |
| 控制帧 type=7 / C | 检查反馈、输入及空闲心跳后使能，无自动运动 |
| 控制帧 type=8 / R | 手动开始回零 |
| 原始 0x79 | 停止并禁用两轴驱动，取消点动、回零、PTP 及轨迹，作废参考 |
| 原始 0x41 | 仅释放点动，不续心跳；其他旧运动字节不能启动 |
| i / e / s | 输入 / 编码器原始帧 / 状态和位置 |

周期状态例：`STATE fw=stage2-v3 enabled=1 home=6 homed1=1 homed2=0 fault=0 enc=1 dir=1`。另有 `PTP_STATE` 报告执行锁、参考有效性、运动阶段、序号和固件关节范围，格式见 STAGE2_PTP.md。图形执行新增 `TRAJ_STATE`，见 STAGE2_EXECUTION.md。
home：0 待机、1 释放、2 额外退开、3 快碰、4 慢碰、5 零点稳定确认、6 伸直运动、7 伸直稳定确认。
`homed1=1` 可以表示 arm1 已触碰置零而整个流程尚未结束；必须同时检查两轴与 home=0。

WPF 日志保存在 `%LOCALAPPDATA%\TwoLinkCommissioning\Logs`，每次连接单独建文件，界面显示完整路径；日志含主机接收时间。无需额外点击保存。
新版上位机支持 `stage1-v3`、`stage2-v1/v2/v3/v4/v5/v6/v7` 对应功能；人工坐标确认要求 stage2-v3 及以上；完整绘制要求 v5/v6/v7 的当前位置反馈，有限重传支持 v4/v5/v6/v7。坐标参考与新鲜状态仍须有效。旧上位机不识别 stage2-v7，需同时更新。WPF 需要本机 .NET 10 Desktop Runtime；当前电脑已运行无串口自测。发布目录需完整保留，不要只复制 exe。

备用脚本：`powershell -ExecutionPolicy Bypass -File G:\792\AS5045_enc\jog.ps1`。启动后 C 使能、R 回零、Space 停止，Q/Esc 退出。脚本退出发送取消回零的停止字节。

## 重建与离线检查

```powershell
& 'D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe' -wrk 'G:\792\AS5045_enc\DRV8825.cywrk' -build
python G:\792\AS5045_enc\tests\test_stage1.py
dotnet build G:\792\AS5045_enc\Commissioning\Commissioning.csproj -c Release
```

固件输出 `DRV8825.cydsn/CortexM3/ARM_GCC_541/Debug/DRV8825.hex`。
离线模拟编译并执行实际固件 C，替换 GPIO/编码器/时钟及硬件 UART API 边界；需要当前安装的 Visual Studio C++ 编译器。含连续收包、溢出和 UART 错误模拟，不能证明真实 USB/串口时序、惯性、负载或物理传感器可靠性。
WPF 的 `--self-test` 检查正逆运动学、轨迹边界、分支连续性、时间规划及导数、速度/加速度限制、CSV、回放状态、无效参数清空预览和原有控制门禁，不打开串口。可加 `--snapshot <绝对路径.png>` 输出预览图、同名带时间规划 CSV，以及独立圆形执行测试向量 `.execution.bin`。当前规划版本未连接硬件验证。

本次修改前文件保存在 `recovered/manual-reference-before-20260924-222237`，保留原有工程未提交改动，未提交或切换 Git 分支，未烧录。
