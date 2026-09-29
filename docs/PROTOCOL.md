# 上位机 ↔ 固件 通信协议（stage2-v7-absolute-q2）

本页只写当前源码里**已经存在**的行为，均已对照 `DRV8825.cydsn/main.c` 与 `Commissioning/` 核对；有出入以源码为准。设计过程和现场日志分析见 `docs/history/WORKBENCH_HARDENING.md`。状态机见 [STATE_MODEL.md](STATE_MODEL.md)。

## 1. 线路

- 115200 8N1，UDB 硬件 UART：RX P12[6]、TX P12[7]，经 KitProg USB 串口；RX/TX 各 256 字节中断缓冲。
- **上位机 → 固件**：每个有效字节写两份，两次写入之间至少 0.5 ms（固件对相邻相同字节去重）。数据字节采用交替低/高半字节编码，取值恒为 `0x81…0x9F` / `0xA1…0xBF` 的奇数（最高位为 1），与低于 `0x80` 的原始命令字节、以及 `D1/D3/D5/D7` 界符都不重叠。
- 帧界符：完整帧 `D1 … D3`，紧凑心跳 `D5 … D7`。帧内部出现的原始字节会被丢弃，所以原始停止字节只在两帧之间发送。
- **固件 → 上位机**：文本行（`\r\n` 结尾），见第 5 节。发送队列 2048 字节，满则整条丢弃并累计 `tx_dropped`。

## 2. 完整帧（17 字节载荷，编码后 34 字节 + 2 界符 = 36 线上字节）

小端序：`type:u8 | seq:u32 | epoch:u32 | q1:i16 | q2:i16 | value:u16 | crc16:u16`。CRC-16/CCITT（多项式 0x1021，初值 0xFFFF），覆盖前 15 字节。角度单位是 0.01°（`-3000` = −30°）。

| type | 含义 | q1 / q2 / value | 序号 |
|---|---|---|---|
| 1 | PTP 单点 | q1、q2 目标；value = 速度 20…600 微步/秒 | PTP |
| 2 | 轨迹 BEGIN | q1、q2 = 起点（必须就是当前位姿）；value = 节点数 2…2048 | PTP |
| 3 | 轨迹 NODE | q1、q2；value = 该段时长 50…500 ms | PTP（须连续） |
| 4 | 轨迹 RUN | 全 0 | PTP（须连续） |
| 5 | 人工建立零位参考 | 全 0 | PTP |
| 6 | 心跳 + 点动 | value = 按键掩码 0…15：bit0 arm1 左、bit1 arm1 右、bit2 arm2 左、bit3 arm2 右 | 控制 |
| 7 | 使能 | 全 0 | 控制 |
| 8 | 开关回零 | 全 0 | 控制 |
| 9 | 关节范围 | q1 = 最小，q2 = 最大（0.01°，−18000…0…18000，最小 < 最大）；value = 轴号 0/1 | 控制 |
| 10 | 速度/加速度 | q1 = 最大速度（轴 0：20…800，轴 1：20…600 微步/秒）；q2 = 加速度（轴 0：100…20000，轴 1：100…12000 微步/秒²）；value = 轴号 | 控制 |

规则（`ptp_accept`、`control_accept`、`joint_limits_accept`）：

- CRC 不对、字段非法、序号不大于已接受的序号、代次（`epoch`）不是当前代次 → 拒绝，不改变状态。载荷与上一帧完全相同的重发只重复回执（幂等）。
- type 1…5 用 PTP 序号，type 6…10 用控制序号；两组序号各自递增，所以心跳不会打断轨迹节点的连续性。
- 代次每次作废参考时加一（见 STATE_MODEL）。**唯一的例外**：type 6 心跳只要结构和 CRC 合法，即使代次过期也会续租约——但不应用按键、不接受序号、不执行任何命令；固件随后按 250 ms 节流补发一行 `PTP_EPOCH <n>`。
- type 9/10 只在未使能、无参考、无运动时接受；范围与速度写入后由 `LINK`/`PTP_LIMITS`/`PTP_RATES` 回执确认。

## 3. 紧凑心跳（6 线上字节）

`D5 <lo(keys)> <hi(keys)> <lo(crc8)> <hi(crc8)> D7`，两个半字节编码的数据字节：`keys`（0…15）和 `crc8`（多项式 0x07，初值 0）。不带序号、不带代次，**不消耗控制序号**，也不能启动任何命令。仅当 `LINK` 行含 `short_hb=1` 时上位机才使用；否则退回 36 字节的完整心跳。

## 4. 原始字节与时间常量

| 字节 | 作用 |
|---|---|
| `0x79` | 停止：两轴驱动断电，取消点动/回零/PTP/轨迹，作废参考。任何时候有效 |
| `0x41` | 仅释放点动，**不续租约** |
| 其他 | 不能启动运动（`CONTROL refused: v7 requires framed enable/home/jog`） |

| 常量 | 值 | 位置 |
|---|---|---|
| 固件心跳期限（deadman） | 250 ms | `JOG_DEADMAN_MS`；使能期间无有效心跳即停止 |
| 硬件看门狗 | 256–384 ms | 只有完整主循环喂狗 |
| 上位机心跳间隔 | 20 ms（紧凑）/ 40 ms（完整） | 写线程；空闲从上一拍完成后算起 |
| 按键有效期 | 100 ms | `KeyStaleMs`：界面停止响应超过它，按住的点动键不再被承认 |
| 租约有效期 | 200 ms | `LeaseStopMs`：界面停止响应超过它，写线程停止发心跳 |
| 帧接收窗口 | 250 ms 无新字节 / 2 s 总长 | 超时丢弃并计 `drop_timeout` |
| 验收线 | `key_max_gap_ms ≤ 100` | 0.4 × deadman，见 `tools/check-log.py` |

## 5. 固件文本行

上位机用正则解析下面这些行；改格式等于改协议。

| 行 | 内容 |
|---|---|
| `STATE fw=stage2-v7 enabled home homed1 homed2 fault enc dir` | 固定的 `fw=stage2-v7` 用于选择“帧控制”；`home`：0 待机、1 释放、2 额外退开、3 快碰、4 慢碰、5 零点稳定、6 伸直、7 伸直稳定。`homed1=1` 可能只表示 arm1 已置零，整个流程要看两轴且 `home=0` |
| `LINK uart=hw baud=115200 rx_overflow rx_error tx_dropped fw=<构建号> short_hb=1 joint_limits=1 motion_rates=1 q2_absolute=1 proto=<n>` | 能力声明。上位机要求 `proto=` 等于自己的协议版本才解锁运动（见第 6 节）。计数器自上电起累计，判断“本次是否新增”要看日志内的增量 |
| `PTP_STATE lock ref active epoch seq min1 max1 min2 max2` | 执行锁、参考、运动阶段、代次、序号、固件里的关节范围 |
| `TRAJ_STATE state count total index seq epoch` | 轨迹缓存状态（0 空、1 上传、2 就绪、3 运行、4 完成） |
| `TIMING loop_max_ms tick_lag_max_ms clock wdt key_age_ms key_max_gap_ms` | `key_max_gap_ms`：使能以来运动许可有效期间最长的一次心跳间隔，ENABLE 清零 |
| `ENC_DIAG irq_max_us retries rejected gap_max_ms follow_peak_lsb` | 编码器读取的关中断最长时间等 |
| `PTP_RX short_ok drop_crc drop_short drop_timeout rejected epoch_rejected last_rejected_byte` | 协议层丢帧/拒绝计数；`rx_error` 只统计 UART 级故障 |
| `TELEM t epoch seq ref source p1 p2 e1 err inputs drops` | 位置遥测：`p1/p2` 指令微步，`e1` 编码器，`err` 跟随误差 |
| `CONTROL seq=<n>` | 控制序号（重连同步用） |
| `PTP_EPOCH <n>` | 代次公告 |
| `PTP_CHECK final_lsb follow_lsb` | 终点与跟随误差判据 |
| `PTP_LIMITS / PTP_RATES epoch seq axis min max` | 第 9/10 类帧的回执 |
| `STOP: <原因>; …` | 停止原因，见 STATE_MODEL 第 3 节 |

## 6. 版本与兼容

- 固件标识 `FW_BUILD_ID`（现为 `stage2-v7-absolute-q2`）出现在 `LINK` 行和启动横幅里；每次改固件行为应改它，方便日志回答“这次烧的到底是哪个包”。行为不变的结构性重构（例如 2026-09-30 的循环拆分）没有改它，靠 `firmware.hex` 的 SHA-256 区分。
- **协议版本号**：`LINK` 行末尾的 `proto=<n>`（固件 `PROTO_VERSION`，上位机 `DeviceContract.ProtocolVersion`）。上位机只在两者相等时解锁运动，与 `fw=` 构建号无关，所以构建号可以随时改。**帧格式、字段含义、序号/代次规则或坐标约定一变，两边一起加一**；发布闸门会核对两边的数字。当前为 `proto=1`（即本页描述的协议，含 absolute-q2 坐标）。固件仍同时输出旧的 `q2_absolute=1`，只为让上一版上位机继续可用。
- 上位机对旧固件的处理：`LINK` 里没有 `short_hb` 就用 36 字节完整心跳；没有声明 `joint_limits`、`motion_rates` 或 `q2_absolute` 的能力，相应功能保持锁定并提示更新固件（不会静默忽略）。
