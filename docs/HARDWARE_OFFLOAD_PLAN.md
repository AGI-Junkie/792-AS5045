# 下位机硬件卸载计划（PSoC 5LP / CY8CKIT-059）

状态：**计划，未实施，未上板。** 编写日期 2026-10-08。基线：分支 `merge-rate2500-trajh`（82e8adb，PR #7）。

**用户决定（2026-10-08）**
- 时钟走保守的 **48 MHz**（不是 64）。
- USB CDC（P5c）放到最后，其余全部完成后再考虑。
- PR #7 已合入 main；本计划在 worktree `G:\792\.worktrees\hw-offload`（分支 `hw-offload`，基于 main 6610a30）里实施。
- TopDesign 组件由我通过桌面自动化在 PSoC Creator 里放置（已授权，权限等级 full）。
- 每阶段只跑定向用例，完整闸门合并前跑一次；**全部改完再上板**，中途不烧录。

**实施状态（2026-10-08 晚）**——代码和离线验证已完成，TopDesign 组件还没放，没有上板：

| 阶段 | 开关 | 固件代码 | 离线夹具（含变异验证） | 默认构建（开关关） | TopDesign 组件 | 台架 |
|---|---|---|---|---|---|---|
| P0 时钟无关化 | — | ✅ | ✅ 24 / 48 MHz | ✅ | — | — |
| P1 48 MHz | — | ✅（`.cydwr`） | ✅ | ✅ 无时序违例 | — | 待测 |
| P2 SSI | `HW_SSI` | ✅ `hw_ssi.h` | ✅ 5 例 | ✅ | ❌ 待放（§2） | 待测 |
| P3 步进脉冲 | `HW_STEP` | ✅ | ✅ 2 例 | ✅ | ❌ 待放（§3） | 待测 |
| P4 事件定时器 | `HW_EVT` | ✅ | ✅ 3 例 | ✅ | ❌ 待放（§4） | 待测 |
| P4 硬件 deadman | `HW_DEAD` | ✅ | ✅ 1 例 | ✅ | ❌ 待放（§5） | 待测 |
| P5 串口 DMA | `HW_TXDMA` | ✅ | ✅ 2 例 | ✅ | ❌ 待放（§6） | 待测 |
| P6 引脚中断 | `HW_PINIRQ` | ✅ | ✅ 1 例 | ✅ | ❌ 待放（§7） | 待测 |
| P5c USB CDC | — | 按你的决定放到最后，未做 | | | | |
| P6b 纯硬件联锁、P6c EEPROM、P7 DMA 表驱动 | — | 未做（P7 有进入门槛，P6b/c 是可选） | | | | |

与上文计划的差别（实现时按代码和手册改的）：
- **P3**：每个轴一个 1 位 PulseMode Control_Reg（`StepArm1Req`、`StepArm2Req`），不是一个 2 位寄存器——共用一个寄存器时，第二次写入会在 12 MHz 时钟采样之前清掉第一个轴的请求。PWM 和 Control_Reg 用 12 MHz 的 `ClkStep`，10 µs 脉宽 = 120 个时钟，8 位够用。
- **P4**：中断里只做 `traj_poll` 和发脉冲；**没有**搬到 1 kHz 节拍的监督——脉冲不再忙等之后，中断每步只占很少的时间，前台的监督不会被饿死，所以保留在前台，改动更小；改成"前台 40 ms 没跑完一圈，中断自己停下"加硬件 deadman（100 ms）两层兜底。`MsTimer` 因此没有放。
- **P2**：SPI Master 每字 10 位 × 4 字 = 40 个时钟（需要 38 个），读之前抓取指令快照，同步读取（使能、归零、起点检查）接管接口。
- **时钟**：用户定为 48 MHz（不是 64）。

标注约定：**【核实】** = 已对照资料或本机文件读到；**【推断】** = 由资料和代码推出，没有验证；**【台架】** = 必须上板才能确认。

---

## 0. 摘要

1. 现在的固件只用了 CPU、GPIO 和一个 UDB 串口。Creator 报告里 DMA 通道 0/24，固定功能定时器 0/4，USB 0/1，中断 2/32，UDB 约 12%。【核实：`DRV8825.cydsn/DRV8825.rpt` 的 Technology mapping summary】
2. **CPU 时钟建议 64 MHz，回退 48 MHz，本计划不超过 67 MHz。** 依据见第 2 节。当前是 24 MHz（IMO 3 MHz → PLL → BUS_CLK）。
3. 计划分 8 个阶段（P0–P7）。每个阶段只动一类硬件，单独出固件、单独上板、可单独回退。先量后改，后一阶段进入前复测，瓶颈已解决的阶段可以停。
4. 有四件事会改变做法，详见第 6 节：
   - `TopDesign.cysch` 是二进制，**组件只能在 PSoC Creator 里手工放置**，我只能给配方和写 C 代码。
   - 时钟超过约 44 MHz 后，SysTick 24 位回绕（≤ 350 ms）会短于看门狗最长超时（384 ms）。
   - 脉冲交给中断和硬件后，看门狗不再是第一道防线，要加硬件 deadman。
   - UART 的"每字节发两份"workaround（每帧约 36 ms）至今没查出根因，USB 全速（J6）可能是更根本的解法。

---

## 1. 已核实的资料

### 1.1 芯片与板子
- 工程器件 `CY8C5888LTI-LP097`（68-QFN）。【核实：`DRV8825.cyprj`，`infineon-cy8ckit-059-schematics-pcbdesigndata-en.pdf`】
- 数据手册 CY8C58LP（001-84932 Rev *H）表 12-1：该型号 **80 MHz 等级**，256 KB Flash，64 KB SRAM，2 KB EEPROM，24 个 UDB，4 个 16 位 Timer/PWM，全速 USB。【核实】
- 供电：工程里 `CYDEV_VDDD = 5.0`。【核实：`DRV8825.cydwr`】
- 板上只有 32.768 kHz 晶振位（Y1，原理图标 No Load，是否焊接未核实），没有 MHz 晶振，系统时钟全靠 IMO + PLL。【核实原理图文本】

### 1.2 工程现状（Creator 报告）
| 资源 | 已用/总数 | 说明 |
|---|---|---|
| UDB 宏单元 / 数据通路 | 24/192、3/24 | 全是 `HostUart` |
| DMA 通道 | 0/24 | |
| 固定功能 Timer | 0/4 | |
| USB / CAN / I2C | 0/1 各 | |
| 中断 | 2/32 | UART RX/TX，优先级都是 7 |
| SAR ADC / Δ-Σ / 比较器 / 运放 / VIDAC | 全 0 | |
| IO | 25/48 | |

时钟：IMO 3 MHz → PLL 24 MHz → BUS_CLK 24 MHz。`HostUart_IntClock` = 923.077 kHz（24 MHz ÷ 26，8 倍过采样，波特误差 +0.16%）。【核实：`DRV8825_timing.html`、`Generated_Source/PSoC5/cyfitter_cfg.c`、`cyfitter.h`】

### 1.3 USB 接口：全速 USB 是哪个口
- 板上有**两个** USB 接口。【核实：原理图文本 + 商品页"snap-away programmer ... micro-USB header for full-speed USB 2.0"】
  - **J10 `USB Finger Connector`**：在 KitProg 部分，网络 `KP_DP/KP_DM`。你现在上位机通信和烧录用的就是它。UART 桥接到 P12[6]/P12[7]。
  - **J6 `USB Micro-B`**：在目标芯片部分，`VBUS/DM/DP/ID`，对应芯片自己的 USBIO 引脚 P15[6]/P15[7]。**"USB 全速"指的是这个口**，由 PSoC 5LP 片内的 USB 2.0 全速（12 Mbit/s）模块驱动，目前没用上。
- 两条线同时插是这块板设计允许的用法；J6 的 VBUS 经 F1/D2 并入 VTARG。【推断：只读了原理图文本，没逐线核对】
- 你插线前先看一眼板子，确认自己现在插的是哪个口。

### 1.4 KitProg 串口桥
- 官方文档上限 **115200 baud**（KitProg User Guide 001-96359 Rev *I，Table 2-3：1200…115200）。【核实】
- 论坛有人说实际能到 2 Mbaud。我打不开那个帖子（403），**【未核实】**，必须台架实测。

### 1.5 外设时序
- **DRV8825**（TI SLVSA73F，表 7.6）：STEP 最高 250 kHz；STEP 高/低电平各 ≥ 1.9 µs；DIR/微步命令在 STEP 上升沿前后各 ≥ 650 ns；nENBL 到 STEP ≥ 650 ns；nSLEEP 唤醒 1.7 ms。【核实】现行 `STEP_LOW_US=10`、DIR 建立 20 µs 都很保守。
- **AS5045**（ams DS000101 v3-00）：读出时钟 ≤ 1 MHz；CSn 下降沿到第一个 CLK 下降沿 ≥ 500 ns；CLK 上升沿到数据有效 357–394 ns；CSn 高电平 ≥ 500 ns；内部采样率慢速模式（MODE=0/悬空）约 2.61 kHz，快速模式（MODE=1）约 10.4 kHz。【核实】
- 现行软件 SSI：半周期 3 µs（约 167 kHz），在 CLK 上升沿后 2 µs 采样，每台设备读 19 个时钟（18 位数据 + 1 位哑元）。双编码器读 38 个时钟约 360 µs，**读的时候关中断**。【核实：`main.c:416–497`】

### 1.6 组件能力与限制（PSoC Creator 4.4 本机组件手册）
- **SPI Master v2.50**：每字 2–16 位；位速率最高 9 Mbps（高速模式 18 Mbps）；时钟 = 2 × 位速率；四种 SPI 模式；**有硬件 `ss` 输出**：往 TX FIFO 写数据后拉低，数据发完才释放；TX/RX FIFO 各 4 字。【核实】*更正我之前的说法：我说过"SPIM 没有自动片选"，那是错的。*
- **PWM v3.30**：固定功能版**只有连续运行模式**；UDB 版才有 One Shot Single/Multi，但**只能由硬件触发，不能用 API 触发**；有 kill 输入，kill 模式含 Latched。【核实】
- **Control_Reg v1.80**：每位可设 DirectMode / SyncMode / **PulseMode**（CPU 写 1，硬件输出一个时钟宽的脉冲并自动清零）；可由 DMA 写入。【核实】
- **Timer v2.80**：固定功能版 8/16 位，连续或单次，TC 中断，TC 时自动重装周期；有 reset 和 enable 输入。【核实】
- **UART v2.50**：波特率 110–921600，或任意值最高 4 Mbps；RX/TX 缓冲 > 4 字节时靠中断搬运；FIFO 状态可当 DMA 请求源。【核实】
- **USBFS v3.20**：要求 USB 时钟 48 MHz ±0.25%；可用 IMO 24 MHz + 倍频器（Trim with USB）；支持 CDC（USBUART）；端点缓冲通过 DMA 与 RAM 交换，**USB 不能直接和其他外设做 DMA**。【核实】
- **VDAC8**：输出 0–1.02 V（4 mV/位）或 0–4.08 V，**无缓冲**，带载能力有限。**SAR ADC** 12 位，最高 1 Msps。【核实】

### 1.7 引脚与可用余量
- 步进：arm1 = STEP2 `P12[3]`（SIO，低脉冲）/ DIR2 `P12[4]`；arm2 = STEP1 `P2[1]`（板载蓝 LED）/ DIR1 `P2[0]`（高脉冲）。EN1 `P2[7]`、EN2 `P1[7]`。
- AS5045：CSn `P12[1]`、CLK `P12[0]`、DO `P1[3]`。UART `P12[6]`/`P12[7]`。nFAULT `P12[5]`（两路共用）。限位 `P0[3]`、`P0[4]`，NPN `P1[0]`。
- 载板上能接出来的空脚只有：`P1[1]`（J12-6）、`P0[2]`、`P3[2]`（J4）。**任何新外设都要落在这三个脚里。**【核实：`cyfitter.h`、`docs/WIRING.md`、记忆 `drv8825-quad-carrier-pinmap`】

---

## 2. CPU 时钟决策

### 约束
| 约束 | 数值 | 来源 |
|---|---|---|
| CPU / 总线时钟上限 | 80.01 MHz，1.71–5.5 V | 表 11-3 |
| UDB 数据通路 / PLD 上限 | 67.01 MHz | 表 11-61 |
| PLL | 输入 1–48 MHz，中间频率 1–3 MHz，输出 24–80 MHz，锁定 ≤ 250 µs，抖动 250 ps rms | 表 11-89 |
| IMO 精度 | 3 MHz ±1%；24 MHz 非 USB 模式最大 ±4%；24 MHz USB 锁定 ±0.25% | 表 11-81 |
| Flash 等待态 | `<16`→1，`<33`→2，`<50`→3，`<67`→4，`≤83`→5（MHz） | `CyFlash.h`；Creator 在 `cyfitter_cfg.c` 里自动写 WAITPIPE |
| 看门狗 `CYWDT_128_TICKS` | **256–384 ms** | `CyLib.h` |

### 候选
| BUS_CLK | CPU 相对现状 | Flash 等待态 | UDB 上限余量 | SysTick 24 位回绕 | UART 分频与波特误差 | 评价 |
|---|---|---|---|---|---|---|
| 24（现状） | 1.00× | 2 | 无约束 | 699 ms | ÷26，+0.16% | 基线 |
| **48** | 2.00× | 3 | 28% | 350 ms | ÷52，+0.16%（比例与现状完全相同） | **回退值** |
| **64** | 2.67× | 4 | 4.5% | 262 ms | ÷69，+0.64% | **推荐** |
| 67 | 2.79× | 5 | 0.01 MHz | 250 ms | ÷73，−0.42% | 压线，还多一档等待态，不取 |
| 80 | 3.33× | 5 | 超 UDB 上限 | 210 ms | ÷87，−0.22% | 需要所有 UDB 逻辑走独立分频时钟，本计划不取 |

所有行再叠加 IMO 的 ±1%。分频与误差是我按 8 倍过采样 115200 算的，Creator 实际选的分频以报告为准。

### 决定
**目标 64 MHz，回退 48 MHz。** 理由：
1. 低于 UDB 的 67.01 MHz，仍留约 4.5% 余量，同步于 BUS_CLK 的 UDB 逻辑（Control_Reg、状态寄存器）时序可过，不用把每个 UDB 组件都挪到独立时钟。
2. 比 67 MHz 少一档 Flash 等待态。
3. 每微秒 64 个周期，是 2 的幂，`/SU_CY_PER_US` 变成移位。
4. 1 MHz 时钟 = BUS_CLK ÷ 64，整除。
5. 2 Mbaud、1 Mbaud、500 kbaud 都能整除（如果后面要提高波特率）。

**代价与前置工作：**
- `main.c:318` 的 `_Static_assert(BCLK__BUS_CLK__HZ == 24000000u)` 和 `wall_ms()`（`main.c:319–332`，按 24 MHz 写死的常数）必须改成与时钟无关。
- **64 MHz 时 SysTick 24 位约 262 ms 回绕，而看门狗最长 384 ms。** 前台卡 262–384 ms 而看门狗还没复位时，所有靠 `SU_ELAPSED` 判断的间隔会"少算一圈"。现有 250 ms 心跳 deadman 用的是 `wall_ms`（有回绕计数），不受影响；但 `traj_poll` 的 `elapsedCy` 等要逐处审计，规则：**凡是可能跨越 200 ms 的间隔，必须用 `wall_ms` 或回绕计数。** 48 MHz 的回绕是 350 ms，也略短于 384 ms，但"危险窗口"只有约 34 ms，64 MHz 则有约 120 ms；两者都要做这次审计，只有 24 MHz（699 ms）天然安全。
- UART 波特误差从 +0.16% 变 +0.64%，叠加 IMO 的 ±1%，在 8N1 容限内，但**串口至今有未查明的停止位错误**（见 `docs/UART_DIAGNOSIS.md`），所以 P1 验收必须包含 `--uart-stress`，退化就退到 48 MHz。
- 如果以后启用 USB（P5c），IMO 要改 24 MHz。非 USB 锁定状态下 IMO 最大 ±4%，会让 UART 波特误差超限。缓解办法见 P5c。

**这是按规格定的候选值，最终要台架实测确认**（P1 验收）。

---

## 3. 目标架构

### 职责划分
| 功能 | 现在 | 目标 | 用到的硬件 |
|---|---|---|---|
| 编码器 SSI | CPU 位操作，关中断约 360 µs | SPIM 出时钟、CSn、采样；CPU 写 4 字、读 4 字 | UDB SPI Master |
| 步进脉冲宽度 | CPU 忙等 `STEP_LOW_US`，双轴 ≈ 20 µs | CPU 写一个寄存器，硬件出固定宽度脉冲 | Control_Reg（PulseMode）+ UDB PWM 单次 ×2 |
| 步进节拍 | 前台轮询 `traj_poll` | 周期中断：DDA、点火、装下一个间隔 | 固定功能 Timer |
| 毫秒时基 | SysTick + 回绕 ISR + 复杂 `wall_ms` | 1 kHz 中断计数 | 固定功能 Timer |
| 监督（心跳、编码器时效、跟随误差、范围） | 每步一部分 + 前台 | 1 kHz 低优先级节拍 | 同上 |
| 前台存活 | 看门狗 256–384 ms | 40 ms 硬件 deadman，直接切断脉冲 | Timer（单次）+ PWM kill |
| 故障与限位 | 轮询 | 引脚中断 + 硬件 kill | PICU 中断 + UDB |
| 串口发送 | 前台每圈送 1 字节 | DMA 搬运到 UART FIFO | DMA |
| 串口接收 | ISR 256 字节环 | 保留（量小） | — |
| 主机链路 | KitProg 115200 | 同左；可选 USB CDC（J6） | USBFS（可选） |

### 中断优先级（NVIC 0 最高）
| 优先级 | 中断 | 约束 |
|---|---|---|
| 0 | 故障/限位引脚（P6） | 只做"置 halt + 禁驱动" |
| 1 | 步进事件（P4） | 有界、不格式化、不 `printf`、不循环等待 |
| 3 | 1 kHz 节拍与监督（P4） | 可被步进 ISR 抢占 |
| 5 | DMA 完成、USB（P5） | |
| 7 | UART RX（现有） | |
| SysTick 回绕 | 现有 `clock_wrap_isr` | 核对优先级不低于 1 kHz 节拍 |

阶段完成后，**项目里不应再有超过几微秒的 `CyEnterCriticalSection`**（现在的 SSI 读是 360 µs）。

### 资源预算（估计，以 Creator 报告为准）
| 资源 | 现用 | 计划增加 | 合计 / 总数 |
|---|---|---|---|
| UDB | 约 3 个 | SPIM 约 2，PWM ×2 约 2–4，Control_Reg 约 1 | 约 9–11 / 24 |
| 固定功能 Timer | 0 | 事件、1 kHz、deadman | 3 / 4 |
| DMA | 0 | UART TX 1（可选 RX 1，USB 2–4） | ≤ 6 / 24 |
| 中断 | 2 | 事件、1 kHz、引脚、DMA、SysTick | 约 8–10 / 32 |

### 软件接缝（与 `traj.h` 同一做法）
每类硬件一个窄接口，真实实现调组件 API，离线夹具实现一个模型：

| 接缝 | 接口草案 |
|---|---|
| `hw_ssi` | `ssi_begin(clocks)` / `ssi_done()` / `ssi_take(&raw1, &raw2)` |
| `hw_step` | `step_fire(mask)` / `step_dir(axis, level)` / `step_halt(on)` |
| `hw_evt` | `evt_start()` / `evt_set_us(us)` / `evt_stop()`，加 ISR 钩子 |
| `hw_tx` | `tx_enqueue(buf, len)` / `tx_idle()` |

---

## 4. 分阶段计划

顺序与依赖：P0 → P1；P2 与 P3 互相独立；P3 → P4 → P6；P5 独立，但其中的串口 A/B 实验要尽早做。

### P0 基线与时钟无关化（纯软件）
- **做什么**
  - `wall_ms()` 与 `_Static_assert` 改成与 `BCLK__BUS_CLK__HZ` 无关；审计第 2 节说的 `SU_ELAPSED` 长间隔。
  - 把 `trajectory-timer-irq` 里的 `TRAJ_LOAD` 与前台分段耗时（`fg_pass_max_us`、`fg_phase_max_us`）诊断移植过来（只加测量）。
  - 夹具支持非 24 MHz 的时钟参数。
- **验收**：24 MHz 下行为与基线一致（`traj_schedule` 哈希不变，只改测量字段）。
- **上板**：烧一版，记录基线：`loopMaxMs`、`traj_lag_max_us`、编码器读耗时、`isr/fg` 占用，五边形和圆各一次，存进 `docs/`。
- **规模**：S

### P1 CPU 时钟 64 MHz
- **做什么**：Creator Design-Wide Resources → Clocks：IMO 3 MHz，PLL 输出 64 MHz，BUS_CLK = PLL。
- **检查**：构建后 `DRV8825_timing.html` 无时序违例；`HostUart_IntClock` 分频与波特误差落在 §2 预期；Flash 等待态由 Creator 写入 WAITPIPE。
- **台架**：自检通过；`--uart-stress` 各节拍 `rx_error` 不新增；编码器、点动、回零、轨迹照常；与 P0 基线对比 `loopMaxMs`、`traj_lag_max_us`。
- **回退**：`--uart-stress` 或时序报告不过 → 48 MHz；再不行回 24。
- **预期方向**：纯计算部分提速，但不会线性（Flash 等待态 2→4），只报测量值。
- **规模**：S

### P2 SSI 换成 UDB SPI Master
- **Creator 配方**（实例名 `Ssi`）
  - SPI Master v2.50，数据位 **10**（4 字 = 40 个时钟，刚好填满 4 字 TX FIFO；需要 38 个）。
  - 位速率 **250 kHz**（软件版约 167 kHz，AS5045 允许 1 MHz；先保守）。
  - 模式 **CPOL=1，CPHA=0（模式 2）**【台架确认】：AS5045 在 CLK 上升沿移出数据，下降沿采样；CSn 拉低时 DO 先变高，所以第一个采样点是哑元位。
  - `sclk`→`P12[0]`，`miso`→`P1[3]`，`ss`→`P12[1]`（硬件片选），`mosi` 不接。删掉原来的 `CLK`/`CSN`/`AS5045_DO` 三个 Pins。
  - MISO 输入的同步选项按组件手册设置。
- **代码**
  - `as5045_read_clocks` 改成 `ssi_begin` / `ssi_done` / `ssi_take` 三步，前台轮询，不再关中断。
  - 位对齐：SPIM 采样序列 j（j=0 是 CSn 刚拉低时的哑元）等于软件版第 j−1 位；`raw1` = SPIM 位 1–18，`raw2` = 位 20–37。
  - 在 `ssi_begin` 之前原子抓取"指令步数快照"（`cmdUsteps`、`cmdLsbX100`），用于跟随误差。快照与 CSn 下降沿相差约 1 µs，对 4000 步/s 是 0.004 步。
  - 删除 `JOG_ENC_READ_US` 等"在步与步之间找空档读编码器"的逻辑（`jog_slot_free` 里的编码器预算）。
- **验收**
  - 逻辑分析仪或示波器：CSn→首个下降沿 ≥ 500 ns；时钟 250 kHz；DO 在上升沿后 ≤ 394 ns 稳定。
  - 同一磁铁位置下，软件版与硬件版连读的原始 18 位帧一致（允许 ±1–2 LSB 噪声；奇偶、OCF/COF/LIN 等状态位一致）。无分析仪时：做一条调试命令，同一时刻先软件后硬件各读一次并打印。
  - `enc_irq_max_us` 从数百 µs 降到个位数；`fg_pass_max_us` 不增加；闭环跟随误差统计与基线同级。
- **回退**：`HW_SSI` 宏切回软件读。
- **规模**：M

### P3 步进脉冲交给硬件
- **Creator 配方**
  - Clock `ClkUs` = 1 MHz（BUS_CLK ÷ 64）。
  - Control_Reg `StepReq`，2 位，**PulseMode**，时钟 `ClkUs`：bit0 → arm1 脉冲，bit1 → arm2 脉冲。
  - PWM `StepArm1`、`StepArm2`：**UDB 实现**，8 位，Run Mode = One Shot (Single)，Trigger = Rising edge，时钟 `ClkUs`，kill 模式 Latched；脉宽参数 W µs，**初值 10 µs（与现状一致）**，台架确认后再降到 4 µs（DRV8825 最小 1.9 µs）。
  - `StepArm1` 输出 → `P12[3]`，**低有效、空闲高**（输出取反）；`StepArm2` 输出 → `P2[1]`，**高有效、空闲低**。引脚设置为"HW connection"。
  - DIR 仍是 GPIO。CPU 先写 DIR，再写 `StepReq`；硬件触发后至少延迟 1 个 `ClkUs` 周期，满足 DRV8825 DIR 建立时间 650 ns。
- **代码**
  - 所有出脉冲的路径统一走 `step_fire(mask)`：点动、回零、点到点斜坡、轨迹。删掉 `pulse_step_pin`、`pulse_hold_us`，DIR 建立改成"方向变化时才等 ≥ 2 µs"（原 `DIR_SETUP_US` 20 µs）。
  - `stepper*_disable()` 里对 STEP 空闲电平的写法，改成 `step_halt(on)`。
  - `jog_step` 里的安全检查（`FAULT_ACTIVE`、`HARD_LIMIT_ACTIVE`、回零开关方向、`joint_step_allowed`）原样保留。
  - 可选 P3b：给每路脉冲加一个 UDB 计数器，核对"发出的脉冲数 = 请求数"。
- **离线验证**：`step_fire` 的夹具模型在同一模拟时刻记录脉冲，**`traj_schedule` 逐脉冲哈希不应变化**，这是等价性证明。
- **台架验收**
  - 示波器：两路脉宽、极性、空闲电平；上电、禁用、复位期间 STEP 脚没有毛刺。没有示波器就用编码器跟随误差、`done` 计数与基线对比。
  - 速率阶梯：在不改其他东西的前提下，把 `PlanRateMax` 从 2500 逐档升到 3000、3500，看 `traj_lag_max_us`、`traj_slip_us`、编码器跟随误差。
- **回退**：`HW_STEP` 宏切回软件脉冲。
- **规模**：M

### P4 节拍进定时器中断，监督分层，加硬件 deadman
**不要把旧的 `trajectory-timer-irq` 分支合并进来**：它基于 `traj.h` 之前的 `main.c`，冲突很大。应在合并线的 `traj.h` 之上重做，只取它的诊断和夹具。

- **Creator 配方**
  - Timer（固定功能）`EvtTimer`：16 位，时钟 `ClkUs`，连续，TC 中断，优先级 1。
  - Timer（固定功能）`MsTimer`：时钟 1 kHz（BUS_CLK ÷ 64000），TC 中断，优先级 3。
  - Timer（固定功能）`DeadTimer`：单次，周期 40 ms，reset 输入接 Control_Reg `AliveReq`（PulseMode）；TC 输出接两个步进 PWM 的 kill（Latched）。
- **事件 ISR**（`evt_isr`）要点
  - 不 Stop/Start 定时器；用"TC 时自动重装周期"，ISR 里只写下一个间隔。v12 每次事件停止、等 2 µs、清状态、重启，约 4 µs 白白浪费。
  - 流程：读 `SU_NOW` → 复用现有纯逻辑 `traj_poll` → `step_dir` + `step_fire` → 更新 `usteps` 和编码器指令计数 → 写下一个间隔（用相邻两次绝对到期时刻的差，不累积舍入）。
  - 下一个事件超过 60 ms 时，写一个 1 ms 的维护周期。
  - **ISR 里不调 `stage_stop`（它会格式化日志）。** 改成 `stop_request(reason)`：置 halt、禁驱动、记原因码，前台再调用 `stage_stop` 收尾。
- **步进引脚归属**：同一时刻只有一个所有者（前台或 ISR），用状态变量保证。点动和回零是低速路径，可以先留在前台，用 `step_fire` 出脉冲。
- **监督分层**：心跳、编码器时效（`TRAJ_ENCODER_MAX_MS=20`）、前台存活（`TRAJ_FOREGROUND_MAX_MS=40`）、跟随误差、关节范围全部搬到 1 kHz 节拍。每步的限位和 nFAULT 检查继续留在 `jog_step`（几个寄存器读）。
- **硬件 deadman**：前台每圈写 `AliveReq`；超过 40 ms 没写，`DeadTimer` 的 TC 通过 kill 锁死两路脉冲，直到软件显式重新启动 PWM（对应现有"必须人工 C 重新使能"）。**脉冲由中断产生后，前台卡死不再自动让脉冲停下，这一层是必须的。**
- **并发审计清单**：`g_ptp.*`、`g_arm*.usteps`、编码器指令字段、`g_traj`。原则是单写者（ISR 写，前台读快照）；跨 32 位的读用临界区或序列号。
- **清理**：删除 `jog_slot_free` 及其调用（编码器读和串口送字节不再需要避开步进）。
- **验收**
  - `TRAJ_LOAD`：`isr_avg_us` 和 `isr_busy_permille`；目标是 4000 事件/s 时 ISR 占用 ≤ 10%（**这是我提的目标，不是规格**）。
  - 之前必定停机的五边形（r2 = 3426 步/s）能跑完，不出现 `encoder stale`；`enc_age_max_ms` 留 2 倍余量。
  - 逻辑分析仪测脉冲间隔抖动。
- **回退**：保留前台轮询路径，开关 `TRAJ_IRQ`。
- **规模**：L

### P5 串口
- **P5a 先做实验，不改固件结构**
  - 用已有的 `TwoLinkCommissioning.exe --uart-stress` 做 A/B：KitProg 桥 vs 外部 3.3 V USB-UART 适配器（`docs/UART_DIAGNOSIS.md` 已写好步骤）。结论决定 P5b/P5c 的意义。
  - 波特率扫描：实验固件在运行时用 `HostUart_IntClock_SetDividerValue()` 改分频，主机同步改波特率，测 230400 / 460800 / 921600 / 1M / 2M。KitProg 文档上限是 115200，实测为准。
- **P5b 发送改 DMA**
  - `HostUart` 的 TX 缓冲改为 4（只用硬件 FIFO），DMA 通道从 `g_txq` 环形区搬到 `HostUart_TXDATA`，请求源用"TX FIFO 未满"。
  - 按连续区段建 TD，DMA 完成后推进尾指针并启动下一段。删除 `txq_send_one`、每圈送 1 字节的逻辑，TX ISR 也不再需要。
  - 验收：`tx_dropped` 不增加；60 字符一行不再占前台；遥测 `g_telemSkipped` 对比基线。
- **P5c 可选：USB CDC（J6）**
  - 动机不是带宽，而是**可靠性**：如果 P5a 证明 KitProg 桥是错误来源，USB CDC 可以去掉"每字节发两份"，17 字节载荷从约 36 ms 缩到约 2 ms，心跳余量大增。
  - 配方：USBFS v3.20 + CDC；IMO 24 MHz，倍频器给 USB，"Trim with USB"；PLL 从 IMO 24 MHz 取 64 MHz（Q=12 → 2 MHz，×32）。
  - **风险**：USB 未连接时 IMO 24 MHz 最大 ±4%，UART 波特会漂。缓解：USB 链路作为主链路，KitProg UART 只做调试输出；或者 P5c 阶段才改 IMO，并重做串口验收。
  - 主机侧只是多一个 COM 口，协议不变。
- **规模**：P5a S，P5b M，P5c L

### P6 安全硬件（依赖 P3、P4）
- **P6a 引脚中断**：`P12[5]` nFAULT 下降沿、`P0[4]` 全局限位上升沿，优先级 0，ISR 只做"置 halt + 禁驱动"。保留前台轮询作为第二层。
- **P6b 硬件联锁（可选）**：把 nFAULT（低有效）和全局限位（高有效）经反相和或逻辑接到两个步进 PWM 的 kill，**不经过 CPU**。只放与方向无关的信号；回零开关和 NPN 与方向有关，留在软件。
- **P6c 可选：片内 EEPROM（2 KB）**存标定值（肘角范围、编码器原点等），目前这些在主机端。与卸载 CPU 无关，只是"没用上的硬件"。
- **规模**：M

### P7 选做：DMA 表驱动发步（有门槛）
- 内容：DMA 在定时器 TC 时把"掩码字节"写入 Control_Reg，另一路 DMA 重装定时器周期，CPU 只在 50 ms 级别填表，步进期间零中断。
- **进入条件**：P4 完成后 ISR 占用仍 > 10%，或目标步频 > 6000 步/s。
- **难点**：位置核算要改成读硬件计数，安全检查粒度变成"一个表块"，表空时 DMA 链自然终止（这一点是故障安全的）。风险最高，不默认做。
- **规模**：L

### 建议的上板顺序
| 烧录 | 内容 | 目的 |
|---|---|---|
| #1 | P0 + P1 | 基线 + 时钟 |
| #2 | P2 | SSI |
| #3 | P3 | 脉冲 |
| #4 | P4 | 中断 + deadman |
| #5 | P5b（P5a 实验固件另算） | 串口 |
| #6 | P6 | 安全硬件 |

---

## 5. 不打算用的硬件与理由
| 硬件 | 理由 |
|---|---|
| CAN 2.0b | 没有 CAN 总线 |
| 固定功能 I2C | 没有 I2C 器件，`P12[0]/[1]` 已被 SSI 占用 |
| LCD 段驱动、CapSense | 没有显示和触摸 |
| 运放、比较器、SC/CT、DFB、LPF | 没有模拟信号链 |
| QuadDec | AS5045 在本系统走 SSI，没有 ABI 输出 |
| CRC/PRS 硬件 | 协议 CRC 只有几微秒，不是瓶颈（若 P0 测量显示不是则不做） |
| SAR / Δ-Σ ADC、VDAC8 | 可用引脚只有 `P1[1]`、`P0[2]`、`P3[2]`；VREF 要从驱动模块飞线才能测或设，属载板改造，不在本计划内，之后可作为独立选项 |

---

## 6. 贯穿事项

### 6.1 TopDesign 只能在 PSoC Creator 里改
`TopDesign/TopDesign.cysch` 是二进制（.NET 序列化，内含 UTF-16 字符串）。**我不能可靠地手改它。** 分工：
- 我：写每个阶段的"配方"（组件名、参数、引脚）、写 C 代码、写夹具模型、写测试。
- 你：在 Creator 里按配方放置组件、连线、分配引脚，构建通过后告诉我。
- 桌面自动化不适合这件事：Creator 属于 IDE 类应用，只给点击权限，不能输入文字。

### 6.2 夹具与测试
- 离线夹具（`tests/test_stage1.py`）对每个接缝加模型，保持"行为不变用哈希证明"的做法：P3 的 `traj_schedule` 哈希是等价性证明。
- P4 需要夹具能在模拟时刻调用 `evt_isr`。`trajectory-timer-irq` 分支已经做了 ISR 模拟（`tests/test_stage1.py` 约 +441 行），可复用。
- 夹具只给寄存器读取计时，**量不出 CPU 占用**；性能结论一律靠台架测量。
- 完整闸门约 8 分钟，只在你要求时才跑；每阶段先跑定向用例。

### 6.3 仓库脚本要跟着改
- `verify-creator-pins.ps1` 现在要求 `main.c` 里每个 `PIN_*` 与 TopDesign 一致，并禁止 `PIN_*` 表之外出现裸 `CYREG_PRT*`。引脚改为硬件驱动后（`STEP`、`CLK`、`CSN`、`DO`），这个脚本和 `PIN_*` 表要同步改。
- `build-release.ps1` 的源码清单、`docs/PROTOCOL.md` 的 `LINK` 字段（建议加 `hw=` 标志位）、CHANGELOG、每阶段独立的 `FW_BUILD_ID`。

### 6.4 回退与发布
每个阶段一个分支或 worktree，一个 PR。软件路径用宏保留到台架验证通过，之后再删掉，避免长期双份代码。

### 6.5 测量清单（每阶段前后对比）
`loopMaxMs`、`traj_lag_max_us`、`traj_slip_us`、`enc_irq_max_us`、`fg_pass_max_us`、`isr_avg_us`、`isr_busy_permille`、`enc_age_max_ms`、`rx_error`、`tx_dropped`、`g_telemSkipped`，加编码器跟随误差峰值 `follow_peak_lsb`。

### 6.6 有示波器或逻辑分析仪更好
P2、P3、P4 的验收有示波器或逻辑分析仪最直接。没有的话，以编码器闭环数据和日志计数作为间接证据，结论的可信度会低一档，计划里会标出来。

---

## 7. 需要你决定的事
1. **时钟**：接受 64 MHz（回退 48），还是先走更保守的 48 MHz？
2. **USB（J6）**：要不要做 P5c？需要第二根线，并接受 IMO 改 24 MHz 带来的取舍。建议等 P5a 的 A/B 结果出来再定。
3. **仪器**：有没有示波器或逻辑分析仪？
4. **分支策略**：每个阶段基于 PR #7 的分支叠加，还是等 PR #7 合并后再开始？
5. **闸门**：每阶段只跑定向用例，完整闸门只在合并前跑一次，可以吗？

---

## 8. 未核实清单与来源

**未核实**
- 各组件的实际 UDB 占用与布线（需 Creator 构建报告）。
- SIO 引脚（`P12[x]`）经 DSI 驱动 UDB 输出、接 UDB 输入的布线可行性。
- BUS_CLK 超过 67 MHz 的可行性（本计划不取）。
- KitProg 桥的真实波特上限（官方 115200，论坛称 2 Mbaud）。
- SPI 模式与位偏移（由数据手册和现有代码推导，需台架确认）。
- J6 的 VBUS 并入 VTARG 与双线供电行为。
- 现有 ISR 版本"每事件约 155 µs"是由遥测反推（`TELEM_DIAG`），diag2a 的 `TRAJ_LOAD` 实测结果我没有看到。

**来源**
- CY8C58LP 数据手册 001-84932 Rev *H（SparkFun 镜像），表 11-3、11-61、11-81、11-89、12-1，§4.3.2，§6.1。
- DRV8825 数据手册 SLVSA73F（TI），表 7.6。
- AS5045 数据手册 DS000101 v3-00（项目内 `AS5045_DS000101_3-00.pdf`）。
- CY8CKIT-059 原理图（项目内 `infineon-cy8ckit-059-schematics-pcbdesigndata-en.pdf`）。
- KitProg User Guide 001-96359 Rev *I，Table 2-3。
- PSoC Creator 4.4 组件手册：SPI_Master v2.50、PWM v3.30、Timer v2.80、UART v2.50、USBFS v3.20、VDAC8 v1.90、ADC_SAR v3.10、CyControlReg v1.80（`D:\Psoc Creator\...\CyComponentLibrary.cylib\`）。
- 工程文件：`DRV8825.rpt`、`DRV8825_timing.html`、`cyfitter.h`、`cyfitter_cfg.c`、`CyFlash.h`、`CyLib.h`、`DRV8825.cydwr`。
