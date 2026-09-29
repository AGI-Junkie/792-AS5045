# 变更记录

最新在前。每条写“改了什么、验证到哪一步”；“未烧录”指固件镜像没有下载到板子上。

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
