# TopDesign 配方：把硬件卸载的组件放进 PSoC Creator

固件代码和离线夹具已经就绪（见 `HARDWARE_OFFLOAD_PLAN.md`），默认所有 `HW_*` 开关都是 0，工程现在用 Creator 直接构建是通过的。
要打开某个开关，必须先在 `TopDesign/TopDesign.cysch` 里放对应的组件。`.cysch` 是二进制，只能在 PSoC Creator 里改。
本文是每个组件的名字、参数、连线和引脚，名字不能改：固件按它们生成的 API 名字（`<实例名>_Write()` 等）调用。

标注：**【核实】** 来自本机 Creator 4.4 的组件手册；**【待核】** 我没法在不打开 Creator 的情况下确认，放置时请对照。

## 0. 总体顺序
按阶段放，每放一阶段就构建一次，通过后再把 `main.c` 里对应开关的默认值改成 `(1u)`：

| 阶段 | 开关 | 本文小节 |
|---|---|---|
| P2 | `HW_SSI` | §2 |
| P3 | `HW_STEP` | §3 |
| P4 | `HW_EVT`、`HW_DEAD` | §4、§5（依赖 §3） |
| P5 | `HW_TXDMA` | §6 |
| P6 | `HW_PINIRQ` | §7（依赖 §3） |

每一阶段构建后检查：`DRV8825_timing.html` 无时序违例；`DRV8825.rpt` 的 Technology mapping 里 UDB 数量；无 Warning。
命令行构建（不用开 Creator 图形界面）：`cyprjmgr.exe -wrk DRV8825.cywrk -build -c Debug`，约 20 秒。

## 1. 时钟（先做）
`DRV8825.cydwr` 里 BUS_CLK 已是 48 MHz（IMO 3 MHz → PLL）。在 TopDesign 里放三个 **Clock** 组件（Digital，Source 默认，Tolerance 1%）：

| 实例名 | 频率 | 用在 |
|---|---|---|
| `ClkStep` | 12 MHz | §3 的 Control_Reg 和 PWM |
| `ClkUs` | 1 MHz | §4 的 EvtTimer 和 EvtRestart |
| `ClkMs` | 1 kHz | §5 的 DeadTimer 和 AliveReq |

## 2. P2：编码器 SSI → `Ssi`（SPI Master）
先删掉原来的三个引脚组件 `CLK`、`CSN`、`AS5045_DO` 的软件连接，改成由 SPI Master 驱动，**引脚位置保持不变**。

- 组件：**SPI Master** v2.50，实例名 **`Ssi`**，UDB。
  - 模式：CPHA = 0，CPOL = 1（SPI 模式 2）。【待核】数据手册推出来的：AS5045 在 CLK 上升沿移出数据，下降沿采样，时钟空闲为高。台架上要用示波器或逻辑分析仪对一次波形。
  - 数据位数：**10**。位速率：**250 kbps**。MSB 在前。High Speed Mode 关闭。
  - TX/RX 缓冲区大小：**4**（只用硬件 FIFO，不要内部中断）。
- 连线：`sclk` → 引脚 `CLK`（P12[0]）；`ss` → 引脚 `CSN`（P12[1]，硬件片选，低有效）；`miso` ← 引脚 `AS5045_DO`（P1[3]）；`mosi` 不接。
- 引脚设置：`CLK`、`CSN` 勾选 "HW connection"（由硬件驱动输出），强驱动；`AS5045_DO` 数字输入，高阻，同步模式选 **Transparent**。【待核】
- 固件：`main.c` 里 `#define HW_SSI (1u)`。

验收（台架）：CSn 下降沿到第一个 CLK 下降沿 ≥ 500 ns；CLK 250 kHz；`ENC_DIAG irq_max_us` 从几百 µs 降到个位数。

## 3. P3：STEP 脉冲 → `StepArm1Req/StepArm2Req` + `StepArm1/StepArm2`
- 两个 **Control Register**（Control_Reg v1.80），实例名 **`StepArm1Req`**、**`StepArm2Req`**：1 个输出，模式 **PulseMode**，时钟 `ClkStep`，不用外部复位。【核实】PulseMode：CPU 写 1，输出一个时钟周期宽的脉冲并自动清零。
- 两个 **PWM**（v3.30），实例名 **`StepArm1`**、**`StepArm2`**：
  - Implementation：**UDB**（固定功能版没有单次模式）。【核实】分辨率 8 位，PWM Mode：One Output，时钟 `ClkStep`。
  - Run Mode：**One Shot (Single)**；Trigger Mode：**Rising Edge**；`trigger` ← 对应的 `StepArmNReq` 输出。【核实】PWM 只能硬件触发，不能用 API 触发。
  - 脉宽：**120 个时钟 = 10 µs**（与 `main.c` 的 `STEP_LOW_US` 一致）。Period / Compare / 比较类型的组合请对照 PWM 手册里的波形图选，使 `pwm` 输出有效电平恰好持续 120 个 `ClkStep` 周期，其余时间无效。【待核】台架上用示波器确认，之后可以缩到 4 µs（DRV8825 最小 1.9 µs）。
  - Kill Mode：只放 §3 时选 **Disabled**；放 §5/§7 时选 **Latched**。
  - 无死区，无中断。
- 引脚：
  - `StepArm1` 的 `pwm` → **取反（放一个 Not 门）** → 引脚 `STEP2`（P12[3]）：arm1 是低电平脉冲，空闲为高。引脚初始电平 1。
  - `StepArm2` 的 `pwm` → 引脚 `STEP1`（P2[1]，也是板载蓝 LED）：arm2 是高电平脉冲，空闲为低。引脚初始电平 0。
  - 两个引脚勾选 "HW connection"，强驱动。
- 固件：`#define HW_STEP (1u)`。

## 4. P4：事件定时器 → `EvtTimer`、`EvtRestart`、`EvtIsr`
- **Timer**（v2.80），实例名 **`EvtTimer`**：Implementation **Fixed Function**，分辨率 16 位，Period 1000，Run Mode **One Shot**，Enable Mode **Software Only**，Capture Mode None，Trigger Mode None，中断源只选 **TC**；时钟 `ClkUs`。
  - `reset` ← `EvtRestart` 的输出；`interrupt` → `EvtIsr`。
  - 【核实】固定功能单次模式：停下之后必须用硬件复位重新装载并开始（复位释放到开始计数有两个时钟的延迟）。固件用 `EVT_RESTART_LAG_US`（默认 3 µs）从每个延迟里扣掉，这是台架上要校准的唯一数字。
- **Control Register**，实例名 **`EvtRestart`**：1 位，PulseMode，时钟 `ClkUs`。
- **isr**，实例名 **`EvtIsr`**：中断类型保持默认。固件设优先级 1。
- 固件：`#define HW_EVT (1u)`（需要 `HW_STEP`）。

## 5. P4：硬件 deadman → `AliveReq`、`DeadTimer`、kill 汇总
- **Control Register**，实例名 **`AliveReq`**：1 位，PulseMode，时钟 `ClkMs`。
- **Timer**（v2.80），实例名 **`DeadTimer`**：**Fixed Function**，16 位，Period **100**（时钟 1 kHz，所以 100 ms），One Shot，Enable Mode Software Only，时钟 `ClkMs`；`reset` ← `AliveReq`；`tc` → kill 汇总。
- **kill 汇总**：一个 **Or** 门（Digital Logic 库）：输入 `DeadTimer.tc` 和（若放了 §7）`StepKill`，输出同时接 `StepArm1.kill` 和 `StepArm2.kill`。PWM 的 Kill Mode 必须是 **Latched**。【核实】Latched：kill 之后一直关闭，直到 PWM 被复位；固件的 `deadman_arm()` 用 `StepArmN_Stop()` + `StepArmN_Start()` 复位。【待核】请确认 Stop+Start 确实清掉锁存。
- 固件：`#define HW_DEAD (1u)`（需要 `HW_STEP`）。

## 6. P5：串口发送改 DMA → `TxDma`
- 把 `HostUart` 的 **TX Buffer Size 改为 4**（只用硬件 FIFO），RX 保持 256，RX 中断照旧。TX 中断源选 **TX FIFO Not Full**，`tx_interrupt` 输出作 DMA 请求。【核实】UART 状态输出可以当 DMA 请求源。
- **DMA Wizard**（Creator 菜单里的 DMA Wizard，不是组件）：名字 **`TxDma`**；Hardware request：把 `HostUart` 的 `tx_interrupt` 连到它的 `drq`；Bytes per burst 1，Requests per burst 1；源 SRAM，目的 Peripheral；优先级默认。生成的 `TxDma_dma.h` 里有 `TxDma_DmaInitialize()`，固件调用它。
- 固件：`#define HW_TXDMA (1u)`。

## 7. P6：引脚中断 → `StepKill`、`PinIsr`
- **Control Register**，实例名 **`StepKill`**：1 位，**DirectMode**，输出接 §5 的 Or 门（没放 §5 就建一个只有它的 Or 门）。
- 引脚 `FAULT`（P12[5]）：中断设为 **Falling edge**；`LIMIT2`（P0[4]）：中断设为 **Rising edge**。两者的中断连到 **isr** 实例 **`PinIsr`**。【待核】Pins 组件如何把中断接到 isr 组件（有的版本经 PICU 端口中断，没有单独的终端），请按 Creator 的 Pins 手册放置；固件需要的只是 `PinIsr_StartEx()`、`FAULT_ClearInterrupt()`、`LIMIT2_ClearInterrupt()`。
- 固件：`#define HW_PINIRQ (1u)`（需要 `HW_STEP`）。

## 8. 全部放完之后
1. 构建通过后，把 `main.c` 里对应开关的默认值改成 `(1u)`，再跑 `tests/test_stage1.py`（含 `--sim-mhz=48`）。
2. 夹具里 `traj_schedule` 的期望哈希要按最终开关组合重新记录（见 `CHANGELOG.md`，用 `--define=SCHED_RECORD=1`）。
3. `verify-creator-pins.ps1` 已经在 main 上失败（`Missing instance: Pin_17 ...`，与本改动无关），引脚改成硬件驱动后，`STEP*`/`CLK`/`CSN`/`AS5045_DO` 不再是普通 GPIO，这个脚本要一并修。
4. 上板前先读 `HWOFFLOAD` 状态行（`bus_mhz`、各开关、`evt_isr_max_us`、`evt_isr_busy_permille`）确认烧对了版本。
