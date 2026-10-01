# 变更记录

最新在前。每条写“改了什么、验证到哪一步”；“未烧录”指固件镜像没有下载到板子上。

## 2026-10-01（续）：直线 / 矩形 / 单点的默认参数与完整流程（固件未改，未上板）

- **切换图形不再串参数**：`a`、`b` 对不同图形含义不同（半径 / 终点 / 宽高），以前切到直线会把圆的半径 5 当成终点 X，一切换就被“采样点超出规划范围”拒绝。现在每种图形各自记住自己的 a、b；第一次切到某个图形时给一个小示例（直线终点 = 起点 +10, +10；矩形 10×10；圆 r=5），都在默认关节范围内。切回去会恢复该图形自己的值。
- **“载入小圆示例”改名“载入示例”**：载入当前图形的示例（起点 340, 0），不再强制切回圆。
- **新增自测 `ShapeFlowSelfTest`**：单点 / 直线 / 矩形 / 圆各走一遍按钮流程——离起点时只能“低速移到起点”→ 到位后开放“上传图形” → 上传完成变“已上传”（“执行图形”开放、移动锁定）→ 执行 → 完成。闭合图形（矩形、圆）结束在起点，可直接再上传；直线结束在终点，只开放“低速移到起点”，回到起点后再开放上传。另有 `ShapeDefaultsSelfTest` 锁住切换与示例；把切换逻辑改成空操作会立刻红。
- **固件仿真夹具修好**：`tests/test_stage1.py` 的 host-plan 用例以前不下发速度配置，固件停在开机默认 800/600 微步/秒，把按 5/3 °/s 规划的轨迹在第 27 个节点拒了（`trajectory rate/velocity change`）。现在与真实上位机一样，先禁用、写两轴速度上限（type 10）、再使能。整套 **359 项通过**；圆、直线、矩形三份规划各自的 `traj_host_plan / traj_manual_plan / traj_host_plan_offset / traj_host_plan_rx_retry` 全部通过（发布闸门原本就按图形循环，现在能绿）。
- 验证范围：上位机整套离线自检通过；固件是 C 仿真，不含机械惯性、驱动电流和真实 USB 时序。直线 / 矩形 / 单点没有上实机跑过。

## 2026-10-01：上位机拆分“上传图形 / 执行图形”，修复连接前的闪烁（固件未改，未上板）

- **拆成两个按钮**：“上传图形”只送 BEGIN + 全部节点，最后一个节点确认后设备进入“就绪”（`TRAJ_STATE state=2`），上位机停下，不再自动发 RUN；落笔后点“执行图形”才发 RUN。固件本来就把 RUN 当作独立的一包，所以固件和协议都没动。
  - 设备持有已上传图形期间，“低速移到起点”、单点移动和人工确认零位被锁（固件在就绪状态本来就拒绝它们）；只有“执行图形”或“总停止”能离开这个状态。重新点“上传图形”会替换设备里的图形。
  - “执行图形”只在设备里的图形与屏幕上当前检查通过的图形逐点相同、且仍在起点时可点；改了参数后它变灰，状态框提示重新上传。参考代次变化、失能、总停止、断开都会丢掉“已上传”标记。
  - 实验记录里“已上传、未执行”的采样标为 `ready`，不计入误差统计。
- **闪烁根因**：`Tick` 每 40 ms 调 `UpdateControls`，其中 `UpdateActionStatus` / `UpdatePointButton` / `RefreshRangeHelp` 每次都重设按钮 ToolTip、重写文字，并给黄色状态框换一个新画刷。WPF 对引用类型按引用比较，内容相同的新字符串 / 新画刷也算“变了”，于是悬停中的 ToolTip 不断关了又开、状态框不断重绘。改为按内容比较后才写，画刷按状态复用。
- 验证：写了一个探针，对整棵控件树挂属性变更监听，连续调 30 次 `UpdateControls`（未连接、已连接但设备无响应、旧固件三种状态）：改前每个状态有 8 个属性持续变化（黄框背景/前景各 90 次，ToolTip 60–90 次），改后为 0。上位机整套离线自检通过（面板、反馈、范围同步、停止恢复、383 节点上传回放含“上传后 0.6 s 内不发 RUN、执行后才发 RUN”、心跳/参考回放）；1100×760 下三个按钮和状态框仍在一行。没有连接真机，没有实机试过。

## 2026-09-30：`stage2-v7-absolute-q2-r2500`（速度上限 2500；末端速度改为只读；离线自检通过，未烧录）

- 速率上限：`PTP_RATE_MAX`、`TRAJ_RATE_MAX`、`CFG_RATE_MAX_AXIS1/2` 由 1500 改为 **2500 微步/秒**（约 arm1 14.1 °/s、arm2 11.0 °/s）；加速度上限不变（3000 微步/秒²）。`TRAJ_STEP_MIN_US` 随之变为 400 µs。上位机 `DeviceContract.cs` 同步，`FW_BUILD_ID` 改为 `stage2-v7-absolute-q2-r2500`。已烧录过 r1500 的板子要再烧一次才能配这版上位机。
- 上位机：去掉“末端 mm/s”输入，只设电机1 / 电机2 (q2−q1) 的最高速度和加速度。新增只读的“末端最大 mm/s”＝(L1·v1 + L2·(v1+v2))·π/180，v1、v2 取压到上限后的实际生效值；机械臂伸直且两轴同向全速时才达到，实际绘图更慢。规划里的末端速度项固定为 1000 mm/s（校验上限），不再限速；单点移动也不再按末端速度降速。旧参数文件里的 feed 字段读入后被覆盖，保存时仍写 15 个字段以兼容旧上位机。
- 验证：上位机离线自检通过；固件 C 仿真 349 项通过；`DeviceContract.cs` 与 `main.c` 的速度/加速度数值一致。没有上板，2500 微步/秒下的跟随误差和丢步要烧录后看日志。

## 2026-09-30：`stage2-v7-absolute-q2-r1500`（提速，离线闸门见构建报告，未烧录）

- **依据**：一次实机圆形轨迹（1031 节点、约 6 分钟）里，arm1 命令速度 0–0.3 °/s 时 `slip_motor_lsb` 最大 44，3 °/s 以上反而最大 31；跟随误差不随速度增大，说明当时是静态偏移而不是转矩不足。设定的 10 °/s 被 600 微步/秒的上限压成约 3.2 °/s。
- 绘图 / 单点 / 配置速率上限：`PTP_RATE_MAX`、`TRAJ_RATE_MAX`、`CFG_RATE_MAX_AXIS1/2` 由 600/800/600 改为 **1500 微步/秒**（约 arm1 8.4 °/s、arm2 6.6 °/s）；`PTP_ACCEL`、`TRAJ_ACCEL_MAX` 由 800 改为 **3000 微步/秒²**。上位机 `DeviceContract.cs` 同步。
- 轨迹最小步间隔不再写死 1667 µs，改为 `TRAJ_STEP_MIN_US = 1000000 / TRAJ_RATE_MAX`（现 666 µs）。点动/回零的默认速度（800/600）不变；上位机“电机 °/s”填多少，点动就按多少（不超过上限）。
- **新上位机必须配新固件**：旧固件的上限仍是 800/600，会拒绝新上位机的配置。`FW_BUILD_ID` 改为 `stage2-v7-absolute-q2-r1500`。

## 2026-09-30（分支 `six-layers`，本地提交，未推送）

按六层评审（驱动 / 运动原语 / 模式与安全 / 协议 / 上位机 / 交付）逐项处理：

- **固件（行为不变的结构整理，未烧录）**：前台循环拆成 `app_init` / `app_start` / `app_poll`，`main()` = `app_boot()` + `jog_run()`；`service_motion()` 和 `service_encoder()` 由循环与测试共用。删除不可达代码 602 行（`firmware.hex` 逐字节相同），两处裸引脚名改用 `PIN_LIMIT1/2`。拆分后 `firmware.hex` 与之前不同（Flash +120 B，RAM +16 B），上机前要按台架流程复核。`FW_BUILD_ID` 未改。
- **测试**：不再用文本替换改写 `main.c`；AS5045 改成引脚级模拟，真正的位移读取代码进入测试；`cycle()` 与固件共用循环片段，点动打滑保护第一次受测（+2 例）；新增随机事件 fuzz（闸门里 6 个种子 × 两个配置档，逐毫秒检查状态、脉冲、位置记账和跟随误差不变量；离线跑过 360 个种子共 5.4 万次使能、3 万次 PTP、2 万条轨迹，无违例）；新增 `link_capabilities` 用例守住 LINK 能力行。`--emit-c=DIR` 可导出 `check.c` 便于调试。
- **主机与固件共用的数字只有一份定义**：固件里给 20/800/600、100/20000/12000、±18000、50/500/600000 等字面量起了名字（`CFG_*`、`TRAJ_NODE_*`；重建后 `firmware.hex` 逐字节不变），上位机把散落在 10 个文件里的同一批数字收进 `Commissioning/DeviceContract.cs`；发布闸门运行 `TwoLinkCommissioning.exe --print-contract`，把 23 个共用数字逐个与 `main.c` 核对，不一致就不出包。
- **协议版本号**：`LINK` 行新增 `proto=1`（固件 `PROTO_VERSION`，上位机 `DeviceContract.ProtocolVersion`）。上位机改为只看它判断能否运动，不再要求 `fw=` 构建号精确等于某个字符串，所以之后可以随意改构建号；旧的 `q2_absolute=1` 仍保留，让上一版上位机继续可用。`firmware.hex` 因此与上一条不同（LINK 多一个字段）。
- **引脚**：`verify-creator-pins.ps1` 现在也核对 `main.c` 里每个 `PIN_*` 与 TopDesign 表一致，并禁止 `PIN_*` 表之外出现裸 `CYREG_PRT*`。
- **上位机**：关节范围下方新增 q1–q2 平面图（肘范围实时视图，规则见 README）；UI 回放测试新增租约间隔判据：≥ 100 ms 在报告里警告，≥ 200 ms（`LeaseStopMs`）判失败。
- **UART 诊断工具**：`TwoLinkCommissioning.exe --uart-stress COM8 20 报告.txt 备注` 只发送键值为 0 的心跳，用 7 种字节节拍各发 20 秒，读回固件的错误计数增量；固件报告已使能时拒绝运行。解读方法和 KitProg / 外接适配器 A/B 见 `docs/UART_DIAGNOSIS.md`。**尚未在真板子上跑过**（本机没有接开发板）。
- **工具**：`tools/check-log.py` 给实机日志下结论（保护性停机、UART 错误增量、`key_max_gap_ms`、UI 租约、终点误差）。
- **文档**：根目录 24 份逐次交付笔记移入 `docs/history/`（加了“历史记录”提示）；新增 `docs/PROTOCOL.md`、`docs/STATE_MODEL.md`；README 重写，只写当前状态；`BUILD_REPORT.txt` 改为由脚本从源码、ELF 和本次日志生成事实（构建号、协议版本、哈希、Flash/RAM、默认范围、各项测试结果、git 描述），不再手写。

## 2026-09-29：`stage2-v7-absolute-q2`（离线闸门通过，标签 `gate-green/stage2-v7-absolute-q2`，未烧录）

- q1、q2 都定义为相对固定 +X 的连杆绝对角，电机 2 转过 q2 − q1。旧 `limits1`/`global1` 固件按另一种约定运行；上位机现在只接受 `LINK` 中同时有 `fw=stage2-v7-absolute-q2` 和 `q2_absolute=1` 的固件，旧固件会被提示更新。
- 这一改变让 q2 限位不再直接限制肘，见 README“关节范围填写规则”。
- 归档实机验证过的 `stage2-v7-limits1` 镜像（`releases/`，标签 `hw-tested/stage2-v7-limits1`；源码当时没有保存）。
- 全局运动参数：尺寸、范围、速度统一用于所有图形，关节速度/加速度也覆盖点动和回零，仍受设备各模式上限约束；预览与执行共用时间规划，补检采样点之间的越界（[历史笔记](docs/history/WORKBENCH_HARDENING.md)、[实机日志复核](docs/history/RUN_REVIEW_20260929.md)）。

## 更早的交付（`deliverables/` 目录已不再随仓库保存；笔记见 `docs/history/`）

- **2026-09-27 `stage2-workbench-v6-upload-retry`**：静止上传期间纯 UART 停止位错误的恢复：丢弃坏帧、保留已确认节点与参考、由有限重传继续；运动中和其他 UART 错误仍停止（[WORKBENCH_UPLOAD_RETRY](docs/history/WORKBENCH_UPLOAD_RETRY.md)）。同期：起点速度 600、终点容差约 ±0.50°、按钮视觉反馈（[MOTION_FIRST](docs/history/WORKBENCH_MOTION_FIRST.md)、[BUTTON_FEEDBACK](docs/history/WORKBENCH_BUTTON_FEEDBACK.md)、[PARAMETERS](docs/history/WORKBENCH_PARAMETERS.md)）。
- **2026-09-26 `stage2-workbench-v6-uart`**：硬件 UART 与整帧发送，保留 UI3 布局、校验重试和停止保护（[STAGE2_HARDWARE_UART](docs/history/STAGE2_HARDWARE_UART.md)）。
- **2026-09-25 `stage2-workbench-v5`**：重新设计的实验工作台、起点确认、中文故障反馈、arm1 误差曲线与自动实验记录；接通“低速移到轨迹起点 → 整条上传并确认 → 双轴定时插补 → 到位检查”，支持直线、矩形、圆（[WORKBENCH_V5](docs/history/WORKBENCH_V5.md)、[STAGE2_EXECUTION](docs/history/STAGE2_EXECUTION.md)）。
- **2026-09-24**：阶段一验收通过；人工确认伸直零位（[STAGE2_MANUAL_REFERENCE](docs/history/STAGE2_MANUAL_REFERENCE.md)）；展开偏移 127° / 163°（[STAGE1_V3_NOTES](docs/history/STAGE1_V3_NOTES.md)）；开关角色修正（[STAGE1_V2_NOTES](docs/history/STAGE1_V2_NOTES.md)）。
