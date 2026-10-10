# 上位机 ↔ 固件 通信协议（stage2-v11-r5000-slip-mref2-ssififo-upstat1-store4-guards2，proto=4）

本页只写当前源码里**已经存在**的行为，均已对照 `DRV8825.cydsn/main.c` 与 `Commissioning/` 核对；有出入以源码为准。设计过程和现场日志分析见 `docs/history/WORKBENCH_HARDENING.md`。状态机见 [STATE_MODEL.md](STATE_MODEL.md)。

## 1. 线路

- 115200 8N1，UDB 硬件 UART：RX P12[6]、TX P12[7]，经 KitProg USB 串口。当前 Creator 配置是 RX 256 字节中断缓冲、TX 4 字节硬件 FIFO（TX 中断关闭）；默认 `HW_TXDMA=0`，CPU 从非阻塞发送队列填充 TX FIFO。旧版 TX 256 字节中断缓冲不是当前配置。
- **上位机 → 固件**：每个有效字节写两份，两次写入之间至少 0.25 ms（`ControlWindow.ControlByteGapMs`；固件对相邻相同字节去重）。本轮保留双发，0.5 ms 历史基线仍可在线路工具中对照；0.25 ms 已过未使能心跳线路筛选及一条 186 节点图形的实机上传，证据见 `UPLOAD_SPEED_OPTIMIZATION.md`。数据字节采用交替低/高半字节编码，取值恒为 `0x81…0x9F` / `0xA1…0xBF` 的奇数（最高位为 1），与低于 `0x80` 的原始命令字节、以及 `D1/D3/D5/D7` 界符都不重叠。
- **USB CDC（J6）**：上位机认出固件自己的 USB 口（`VID_04B4&PID_F232`，`UsbLink.IsFirmwareCdc`）就置 DTR，并改为每帧一次写入、每字节一份、无字节间隔；固件在主机置 DTR 期间把文本行发往 USB，否则仍走 KitProg UART。`LINK` 行的 `link=usb|uart` 说明固件当前走哪条，和上位机判断不一致时上位机记一条警告。协议内容（帧、CRC、状态行）不变。详见 [USB_CDC.md](USB_CDC.md)。
- 帧界符：完整帧 `D1 … D3`，紧凑心跳 `D5 … D7`。帧内部出现的原始字节会被丢弃，所以原始停止字节只在两帧之间发送。
- **固件 → 上位机**：文本行（`\r\n` 结尾），见第 5 节。发送队列 2048 字节，满则整条丢弃并累计 `tx_dropped`。

## 2. 完整帧（17 字节载荷，编码后 34 字节 + 2 界符 = 36 线上字节）

小端序：`type:u8 | seq:u32 | epoch:u32 | q1:i16 | q2:i16 | value:u16 | crc16:u16`。CRC-16/CCITT（多项式 0x1021，初值 0xFFFF），覆盖前 15 字节。角度单位是 0.01°（`-3000` = −30°）。

| type | 含义 | q1 / q2 / value | 序号 |
|---|---|---|---|
| 1 | PTP 单点 | q1、q2 目标；value = 速度 20…5000 微步/秒（`PTP_RATE_MAX`，轨迹每轴速度也不得超过 5000） | PTP |
| 2 | 轨迹 BEGIN | q1、q2 = 起点（必须就是当前位姿）；value = 节点数 2…2048 | PTP |
| 3 | 轨迹 NODE | q1、q2；value = 该段时长 50…500 ms | PTP（须连续） |
| 4 | 轨迹 RUN | 全 0 | PTP（须连续） |
| 5 | 人工建立零位参考 | 全 0 | PTP |
| 6 | 心跳 + 点动 | value = 按键掩码 0…15：bit0 arm1 左、bit1 arm1 右、bit2 arm2 左、bit3 arm2 右 | 控制 |
| 7 | 使能 | 全 0 | 控制 |
| 8 | 开关回零 | 全 0 | 控制 |
| 9 | 关节范围 | q1 = 最小，q2 = 最大（0.01°，−18000…0…18000，最小 < 最大）；value = 轴号：0 = q1，1 = q2，**2 = 肘（q2 − q1，即电机 2 自己的转角）** | 控制 |
| 10 | 速度/加速度 | q1 = 最大速度（两轴均为 20…4000 微步/秒）；q2 = 加速度（轴 0：100…20000，轴 1：100…12000 微步/秒²）；value = 轴号 | 控制 |
| 11 | 遥测速率 | q1 = q2 = 0；value = 每秒流式 `TELEM` 行数：0（只在状态块里带）或 5…20 | 控制 |
| 12 | 轨迹存档（Flash） | q1 = 槽位 0…3（只有查询可用 `-1` = 全部）；q2 = 操作：1 查询、2 把 RAM 里的完整轨迹存入该槽、3 清空该槽；value = 0 | 控制 |
| 13 | 从存档加载轨迹 | q1 = 槽位 0…3；q2 = value = 0 | PTP |

规则（`ptp_accept`、`control_accept`、`joint_limits_accept`、`telem_rate_accept`）：

- CRC 不对、字段非法、序号不大于已接受的序号、代次（`epoch`）不是当前代次 → 拒绝，不改变状态。载荷与上一帧完全相同的重发只重复回执（幂等）。
- type 1…5 用 PTP 序号，type 6…11 用控制序号；两组序号各自递增，所以心跳不会打断轨迹节点的连续性。
- 代次每次作废参考时加一（见 STATE_MODEL）。**唯一的例外**：type 6 心跳只要结构和 CRC 合法，即使代次过期也会续租约——但不应用按键、不接受序号、不执行任何命令；固件随后按 250 ms 节流补发一行 `PTP_EPOCH <n>`。
- type 11（遥测速率）只改变打印什么，**任何时候都接受**（已使能、运动中、上传中也行），代次、序号、幂等规则同 type 9/10；值非 0 且不在 5…20、角度字段非 0、序号为 0 或过期都拒绝（`TELEM_RATE refused: …`）。接受后回执 `TELEM_RATE epoch seq hz`。
- type 12/13（轨迹存档，详见 [TRAJ_STORE.md](TRAJ_STORE.md)；LINK 里 `traj_store=<n>` 非 0 才可用）：12 的代次、控制序号、幂等规则同 type 11，但**重发按字节判断**（心跳会不断推进控制序号）；查询、保存、清空都要求静止（无回零、无 PTP、无运行、两轴不动、没有另一个存档任务），保存还要求 RAM 里有一条完整轨迹（`TRAJ_STATE` 为 2 或 4）和一次成功的 `CySetTemp()`，否则 `TRAJ_STORE refused: …`。保存/清空接受后立刻回一行 `TRAJ_STORE state=saving|erasing …`，之后每 8 行写入再报一次，结束时 `state=idle … last=<saved、erased、failed 或 aborted> seq=<该命令的控制序号>`——主机用 `seq` 对上自己的命令。保存期间 `axes_idle()` 为假：PTP、点动、回零、范围/速度配置都被拒；任何停止、使能、换参考、新上传会丢掉 RAM 轨迹，保存随即 `aborted`，正在写的块保持无效，旧存档不动。type 13 走 PTP 序号和全部 PTP 前置条件（已使能、已回零或人工参考、静止），**还要求两轴正好在存档轨迹的起点**；它只用来填 `g_traj`：节点逐个过 `traj_begin`/`traj_push` 和当前的关节范围、速度、加速度，任何一项不满足就 `PTP refused: archived plan does not fit the current limits` 并清空 RAM 轨迹。成功后回 `TRAJ_LOAD …` 和 `TRAJ_STATE state=2 …`，之后和上传完成一样等 type 4 RUN（序号 = 加载命令的序号 + 1）。
- type 9（范围；三个轴同一规则）只在未使能、无参考、无运动、无缓存轨迹时接受。**肘范围另外作用于运动**：PTP 单点、轨迹 BEGIN/NODE 的目标 q2 − q1 必须在肘范围内（否则 `PTP refused: joint range/rate`），当前位置不在范围内时拒绝新的 PTP，运动中越界则 `STOP: PTP time/joint limit`，点动在肘边界上拒绝继续向外的步进（只能退回）。肘范围是电机 2 的行程，不受 q1 影响，所以 arm1 点动不受它限制。type 10（速度/加速度）在此之外，**已使能、有参考、静止时也接受**（`LINK` 含 `rates_live=1`）；回零、两轴任何一轴在动或在稳定、PTP/轨迹运行或上传中、上传隔离期间一律拒绝（`PTP_LIMITS refused: moving, homing or uploading`）。设备已缓存的轨迹是按旧速度检查过的：type 10 的数值有变化时丢弃它（日志 `TRAJ held plan dropped`，随后 `TRAJ_STATE state=0`），数值没变则保留；上位机据此要重新上传。写入后由 `PTP_LIMITS`/`PTP_RATES` 回执和 `PTP_STATE`/`MOTION_RATES` 确认。

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
| `STATE fw=stage2-v11 enabled home homed1 homed2 fault enc dir enc2 dir2 arm2fb` | `fw=stage2-v11` 用于选择“帧控制”；`enc/dir` 与 `enc2/dir2` 分别报告两路有效性与方向已识别；`home`：0 待机、1 释放、2 额外退开、3 快碰、4 慢碰、5 零点稳定、6 伸直、7 伸直稳定。`homed1=1` 可能只表示 arm1 已置零，整个流程要看两轴且 `home=0` |
| `LINK uart=hw baud=115200 rx_overflow rx_error tx_dropped fw=<构建号> short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 dual_encoder=<0|1> traj_repeat=1 fault_snapshot=1 traj_store=<n> telem_hz=<n> proto=<n>` | 能力声明。上位机要求 `proto=` 等于自己的协议版本才解锁运动（见第 6 节）；`rates_live=1` 表示已使能且静止时也接受 type 10；`dual_encoder` 等于 `ARM2_ENCODER_FEEDBACK`：1 表示本构建要求并读取 arm2 编码器，当前默认 1。计数器自上电起累计，判断“本次是否新增”要看日志内的增量。固件调试缓冲区 `g_dbg` 为 768 字节，须容纳完整 LINK 与双轴故障快照；缓冲区太短会把行尾的 `proto=` 连同换行截掉，`link_capabilities` 测试检查行尾 |
| `PTP_STATE lock ref active epoch seq min1 max1 min2 max2 emin emax` | 执行锁、参考、运动阶段、代次、序号、固件里的关节范围；`emin/emax` 是肘范围（0.01°，开机默认 ∓15500）。**缺这两项的行上位机一律不接受**（旧固件不能证明肘范围） |
| `TRAJ_STATE state count total index seq epoch` | 轨迹缓存状态（0 空、1 上传、2 就绪、3 运行、4 完成） |
| `TIMING loop_max_ms tick_lag_max_ms clock wdt key_age_ms key_max_gap_ms status_max_us traj_lag_max_us` | `key_max_gap_ms`：使能以来运动许可有效期间最长的一次心跳间隔，ENABLE 清零；`traj_lag_max_us`：使能以来轨迹步进相对其到期时刻的最大滞后（微秒），ENABLE 清零；它逼近一整段时长加 20 ms 就会触发“trajectory timing overrun” |
| `ENC_DIAG irq_max_us retries rejected gap_max_ms follow_peak_lsb` | 编码器读取的关中断最长时间等 |
| `PTP_RX short_ok drop_crc drop_short drop_timeout rejected epoch_rejected last_rejected_byte` | 协议层丢帧/拒绝计数；`rx_error` 只统计 UART 级故障 |
| `TELEM t epoch seq ref source p1 p2 e1 err e2 err2 inputs drops` | 位置遥测：`p1/p2` 指令微步，`e1/e2` 两路相对参考的电机轴累计 LSB（逆时针正）；`e2` 是肘而不是绝对 q2，q2=q1+e2×360/(4096×51.06)。`err/err2` 两路原始编码器方向的跟随误差。`ref=1` 要求两路都有效且方向已识别。默认只随 500 ms 状态块出现（约 2 Hz）；type 11 设为 5…20 Hz 后，**已使能期间**另外按该速率单独发送，格式不变。`seq` 是设备最后接受的序号，**收下一条指令（上传的每个节点、RUN）就立刻前进**，而 `PTP_STATE` 仍是每 500 ms 一条：所以流式 `TELEM` 的 `seq` 可以领先主机最近一条 `PTP_STATE`。主机把“`seq` 大于 `PTP_STATE` 的、但不超过自己已发出的最大序号”当作正常（照常使用，只是还不算“已稳定”，`AtStart` 等要等 `PTP_STATE` 追上）；`seq` 落后、超过自己发出过的、或 `epoch` 不同才按“设备反馈无效”处理并停止 |
| `TELEM_RATE epoch seq hz` | type 11 的回执；`LINK` 里的 `telem_hz=` 是当前值 |
| `TRAJ_SLOT slot=<n> valid=0` / `TRAJ_SLOT slot=<n> valid=1 seq=<u32> nodes=<n> duration_ms=<n> q1=<cd> q2=<cd> crc=<8位大写十六进制>` | type 12 查询的结果（每槽一行，之后跟一行 `TRAJ_STORE`）。`valid=1` 表示头和数据 CRC 都过；`seq` 是该槽的保存次数；`q1/q2` 是起点；`crc` 是节点字节流（每点 q1、q2、ms 各 16 位小端，共 6 字节）的 CRC-32/IEEE，主机对自己的计划算同一个值就能认出“这条图形已经存在板上” |
| `TRAJ_STORE state=<idle、saving 或 erasing> slot rows=<done>/<total> seq last=<none、saved、erased、failed 或 aborted> last_slot row_max_us` | type 12 的回执与进度：`seq` 是最近一条被接受的存档命令的控制序号；`rows` 是已写数据行/总数据行；`row_max_us` 是最慢一次写行的微秒数（实板上必须看它）。另有失败原因行 `TRAJ_STORE failed: …`、`TRAJ_STORE aborted: …`、`TRAJ_STORE refused: …` |
| `TRAJ_LOAD slot seq nodes duration_ms` | type 13 加载成功 |
| `TELEM_DIAG hz sent skipped max_us` | 流式遥测的代价：已发送行数、因发送队列过半而跳过的行数、一次格式化最长耗时（微秒）。**跳过而不是丢弃**：流式行从不占用发送队列的后一半，ACK 和停止信息保有余量 |
| `FAULT_SNAPSHOT t epoch seq phase home traj idx ref p1 p2 e1 err inputs hb_age_ms hb_max_gap_ms drv reason="…"` | 每次停止的第一行（在任何状态被清除之前）：停机时的阶段（`home/ptp/traj_run/traj_upload/jog/idle/disabled`）、位置（同 `TELEM`）、输入掩码、心跳年龄。`drv=nfault_undetermined` 表示驱动器 nFAULT 有效，**原因待诊断**：该引脚可由过流、过热或欠压触发，不能据此认定过热；否则 `drv=ok` |
| `CONTROL seq=<n>` | 控制序号（重连同步用） |
| `PTP_EPOCH <n>` | 代次公告 |
| `PTP_CHECK final_lsb follow_lsb` | 终点与跟随误差判据 |
| `PTP_LIMITS / PTP_RATES epoch seq axis min max` | 第 9/10 类帧的回执 |
| `STOP: <原因>; …` | 停止原因，见 STATE_MODEL 第 3 节 |

## 6. 版本与兼容

- 固件标识 `FW_BUILD_ID`（现为 `stage2-v11-r5000-slip-mref2-ssififo-upstat1-store4-guards2`）出现在 `LINK` 行和启动横幅里；每次改固件行为应改它，方便日志回答“这次烧的到底是哪个包”。历史上行为不变的结构性重构靠 `firmware.hex` 的 SHA-256 区分。主机字节间隔从 0.5 ms 改为 0.25 ms 不改帧格式或固件构建号。
- **proto=2（肘范围）**：type 9 新增轴 2（肘），`PTP_STATE` 末尾新增 `emin=<cd> emax=<cd>`，并把肘范围纳入 PTP/轨迹/点动的校验。固件开机默认 ∓155°（`PTP_ELBOW_DEFAULT_CD`，与 `DeviceContract.ElbowDefaultCd` 同值、由发布闸门核对）：这是暂定值，不是实测机械行程。上位机连接后把界面里的肘范围同步给固件，和 q1/q2 范围一样，确认一致才允许运动；proto=1 的固件会被拒绝运动并提示更新。
- **能力位 `traj_store=<n>`（不改 proto）**：`n` 是 Flash 存档的槽位数（现为 4），为 0 或不出现表示不能用（旧固件，或镜像没有用 `cm3gcc_trajstore.ld` 链接，见 TRAJ_STORE.md）；上位机据此决定是否显示存档按钮，不发 type 12/13。
- **能力位（不改 proto）**：`traj_repeat=1` 已运行过的已缓存图形接受再次 RUN，没有它的固件第二次 RUN 会被拒绝并停止，上位机据此在运行一次后要求重新上传；`fault_snapshot=1` 每次停止先打印 `FAULT_SNAPSHOT`；`telem_hz=<n>` 出现表示支持 type 11，值是当前速率。上位机对没声明的固件不发 type 11、不指望快照。
- **proto=3（双编码器历史版本）**：STATE 新增 enc2/dir2；TELEM 与 FAULT_SNAPSHOT 新增 e2/err2；同次 SSI 读取更新两路，任一路失效都会保护停机。人工参考要求 arm2 方向已由本次使能后的回零识别。

- **proto=4（arm2 反馈可选）**：STATE 增加 `arm2fb=0/1`。当前默认 `ARM2_ENCODER_FEEDBACK=1`，启用双编码器强制检查和失效停机；人工参考在 arm2 方向未知时使用 `MANUAL_ENC2_INC_DIR` 的台架标定方向，仍要求编码器有效。显式构建为 0 时，arm2 不要求编码器连接或方向识别，TELEM/FAULT_SNAPSHOT 省略 e2/err2，上位机显示缺测。

- **协议版本号**：`LINK` 行末尾的 `proto=<n>`（固件 `PROTO_VERSION`，上位机 `DeviceContract.ProtocolVersion`）。上位机只在两者相等时解锁运动，与 `fw=` 构建号无关，所以构建号可以随时改。**帧格式、字段含义、序号/代次规则或坐标约定一变，两边一起加一**；发布闸门会核对两边的数字。当前为 `proto=4`（即本页描述的协议，含 absolute-q2 坐标和肘范围）。`q2_absolute=1` 保留坐标说明；proto=2 上位机不能控制本版固件。
- 上位机对旧固件的处理：`LINK` 里没有 `short_hb` 就用 36 字节完整心跳；没有声明 `joint_limits`、`motion_rates` 或 `q2_absolute` 的能力，相应功能保持锁定并提示更新固件（不会静默忽略）。没有 `rates_live=1` 时，已使能下改速度不下发，仍要先总停止、禁用后自动同步、再使能。`rates_live` 是新增的能力位，不改帧格式，所以 `proto` 没有变。

## 停机锁存（v10，proto=4 可选诊断扩展）

FAULT_SNAPSHOT 新增 elapsed_us / plan_us / late_us / done / total。停止后，状态块首先重复 LAST_STOP id=<n>，后面是原始快照全部字段。无新增控制命令，旧未知字段按现有 key=value 解析忽略。上位机去重保存，详见 [STOP_DIAG.md](STOP_DIAG.md)。锁存只在本次开机 RAM 中，停止后的 STOP 不覆盖，后续实际运行的新停机会替换。

## 状态日志调度（v11）

运动时，完整状态块按原有顺序逐行调度，每次前台迭代至多格式化一行，行间隔至少 5 ms，步进已到期时让路。空闲状态同样每次前台迭代至多格式化一行，停机锁存优先。

`TRAJ_UPLOADING` 期间，周期状态块保留 `STATE`、`CONTROL`、`PTP_STATE`、`TRAJ_STATE`、`TELEM`；`LINK`、`TIMING`、`ENC_DIAG` 等完整诊断等上传结束或停止后恢复。已有文本等待发送时不追加周期状态行，减少 ACK 后的积压；接收节点后的直接 `TRAJ_STATE` ACK、独立流式 `TELEM` 和故障/STOP 输出不受该筛选影响。上传中的重复状态请求合并，保留未完成块；新参考代次仍从 `STATE` 重启。周期 `TRAJ_STATE` 仍能补偿失落的 ACK。

`TIMING.status_max_us` 是单条状态行格式化与入队的历史最大耗时，不表示串口传输时间。20/40 ms 心跳间隔、200 ms 主机租约、250 ms 固件 deadman、`proto=4` 和全部运动保护阈值不变。
