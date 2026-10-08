# TopDesign 组件放置规格（交给执行者的完整说明）

## 0. 任务一句话
在 PSoC Creator 4.4 里，给工程 `G:\792\.worktrees\hw-offload\DRV8825.cywrk` 的 `TopDesign` 按下面的规格放组件、连线、设参数，每放完一个阶段就构建并改一个开关。固件 C 代码已经写好，只缺组件。

## 1. 先弄清楚的三件事
1. **组件会参与固件编译。** 每个组件放进 TopDesign 后，Creator 做两件事：配置芯片硬件（UDB、时钟、引脚、中断连线，写进 `cyfitter_cfg.c`），并生成 C 驱动文件（例如实例名 `Ssi` 生成 `Ssi.c/.h`，里面有 `Ssi_Start()`、`Ssi_WriteTxData()`），和 `main.c` 一起编译。`main.c` 按这些**生成的函数名**调用，所以**实例名必须和本文完全一致**。
2. **开关和组件必须一起改。** `main.c` 顶部有 6 个开关 `HW_SSI`、`HW_STEP`、`HW_EVT`、`HW_DEAD`、`HW_PINIRQ`、`HW_TXDMA`，默认 `(0u)`。开关是 1 但没放组件会编译失败；组件放了但开关是 0 更危险，引脚被硬件接管后软件版写不动引脚。所以**放哪个阶段的组件，就在同一次提交里把对应开关改成 `(1u)`**。
3. **不要改别的。** 只动 `TopDesign/TopDesign.cysch`，和 `main.c` 里六个 `#define HW_xxx (0u)` 的默认值。不要改 `DRV8825.cydwr`（BUS_CLK 已是 48 MHz）、不要改 `main.c` 的其他部分、不要改任何已有引脚的位置。

## 2. 通用规则与命令
- 工作区：`G:\792\.worktrees\hw-offload\DRV8825.cywrk`。用 PSoC Creator 4.4（`D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\psoc_creator.exe`）打开。
- 每个阶段放完保存后，用命令行构建（约 20 秒）：
  `D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe -wrk DRV8825.cywrk -build -c Debug`（在 `G:\792\.worktrees\hw-offload` 下运行）
- 构建通过的标准：最后一行 `Build Succeeded`；无 Warning；`DRV8825.cydsn\DRV8825_timing.html` 里 "No Timing Violations"；`DRV8825.cydsn\DRV8825.rpt` 里 UDB 数量没超过 24 个。
- 构建会改写 `DRV8825.cydsn\DRV8825.cyprj`（只是重排格式），提交前用 `git checkout -- DRV8825.cydsn/DRV8825.cyprj` 还原，不要提交它。
- 每个阶段一个 git 提交，信息写阶段号和开关名。出问题可以单独回退一个阶段。
- 组件库里的名字以 Creator 的 Component Catalog 为准；下面写的是组件名和版本。
- 引脚位置全部沿用现有的，不要重新分配。

## 3. 阶段 1：时钟（不改开关）
在 TopDesign 里放 3 个 **Clock** 组件（Component Catalog → System → Clock），频率用 Configure 里的 Frequency 填写，Source 保持默认，Tolerance 填 5%：

| 实例名 | 频率 | 后面用在 |
|---|---|---|
| `ClkStep` | 12 MHz | 阶段 3 |
| `ClkUs` | 1 MHz | 阶段 4 |
| `ClkMs` | 1 kHz | 阶段 5 |

## 4. 阶段 2：编码器 SSI（开关 `HW_SSI`）
**要做的事**：用 SPI Master 接管 AS5045 的三根线，原来的三个引脚组件 `CLK`、`CSN`、`AS5045_DO` 保留（引脚位置 P12[0]、P12[1]、P1[3] 不变），只改成由硬件驱动。

1. 放 **SPI Master**（Digital → Communications → SPI → SPI Master，v2.50），实例名 `Ssi`，Configure：
   - Mode：**CPHA = 0，CPOL = 1**（表格里第 2 行）。
   - Data Lines：4-wire（MOSI + MISO）。
   - Data Bits：**10**。
   - Shift Direction：MSB First。
   - Bit Rate：**250000**（Hz）。Advanced 页 Clock Selection：Internal Clock（它会自己用 48 MHz ÷ 96 = 500 kHz）。
   - Advanced 页 RX Buffer Size：**4**，TX Buffer Size：**4**，不要用内部中断。
2. 连线：
   - `Ssi.sclk` → 引脚 `CLK`
   - `Ssi.ss` → 引脚 `CSN`
   - `Ssi.miso` ← 引脚 `AS5045_DO`
   - `Ssi.mosi` 不接。
3. 引脚组件设置：
   - `CLK`、`CSN`：打开 "HW connection"（Output 由硬件驱动），驱动模式强驱动；`CSN` 初始电平 1，`CLK` 初始电平 1。
   - `AS5045_DO`：数字输入，高阻；若有 "Input synchronization" 选项，选 **Transparent**。
4. 构建，通过后把 `main.c` 里 `#define HW_SSI (0u)` 改成 `(1u)`，再构建一次。提交。

## 5. 阶段 3：步进脉冲（开关 `HW_STEP`）
**要做的事**：每个轴一个"请求寄存器"加一个"单次 PWM"，CPU 写一次寄存器，PWM 输出一个 10 µs 的脉冲。arm1 的 STEP 是 `STEP2`（P12[3]，低电平脉冲，空闲为高），arm2 的 STEP 是 `STEP1`（P2[1]，高电平脉冲，空闲为低）。

1. 放两个 **Control Register**（Digital → Registers → Control Register，v1.80），实例名 `StepArm1Req`、`StepArm2Req`：
   - Number of outputs：1；Bit 0 mode：**PulseMode**；Initial value：0；不要打开外部复位。
   - 时钟输入接 `ClkStep`。
2. 放两个 **PWM**（Digital → Functions → PWM，v3.30），实例名 `StepArm1`、`StepArm2`，Configure：
   - Implementation：**UDB**（不能选 Fixed Function）。
   - Resolution：8 bit。PWM Mode：One Output。
   - Period：**120**；Compare Value 1：**120**；Compare Type 1：**Less**。（含义：计数器从 120 减到 0，小于 120 时输出有效，共 120 个 12 MHz 时钟 = 10 µs，之后输出回到 0。）
   - Run Mode：**One Shot (Multi Trigger)**。Trigger Mode：**Rising Edge**。
   - Kill Mode：先选 **Disabled**（阶段 5 才改）。Dead Band：Disabled。Capture：None。不要中断。
   - 时钟输入接 `ClkStep`。`trigger` 输入分别接 `StepArm1Req`、`StepArm2Req` 的输出。
3. 输出到引脚：
   - `StepArm2.pwm` → 引脚 `STEP1`（P2[1]），初始电平 0。
   - `StepArm1.pwm` → 先放一个 **Not**（Digital → Logic → Not）门 → 引脚 `STEP2`（P12[3]），初始电平 1。（PWM 空闲输出是 0，取反后空闲为高，脉冲时为低。）
   - `STEP1`、`STEP2` 引脚组件：打开 "HW connection"，强驱动。
4. 构建，通过后把 `HW_STEP` 改成 `(1u)`，再构建。提交。

## 6. 阶段 4：事件定时器（开关 `HW_EVT`，要求阶段 3 已完成）
1. 放 **Timer**（Digital → Functions → Timer，v2.80），实例名 `EvtTimer`，Configure：
   - Implementation：**Fixed Function**。Resolution：16 bit。Period：1000。
   - Run Mode：**One Shot**。Enable Mode：**Software Only**。Trigger Mode：None。Capture Mode：None。
   - Interrupt：只勾 **TC**。
   - 时钟输入接 `ClkUs`。
2. 放 **Control Register**，实例名 `EvtRestart`：1 位，**PulseMode**，时钟 `ClkUs`；输出接 `EvtTimer.reset`。
3. 放 **isr**（System → Interrupt），实例名 `EvtIsr`，Interrupt Type：**LEVEL**；`EvtTimer.interrupt` → `EvtIsr`。
4. 构建，通过后把 `HW_EVT` 改成 `(1u)`，再构建。提交。

## 7. 阶段 5：硬件 deadman（开关 `HW_DEAD`，要求阶段 3 已完成）
1. 放 **Control Register**，实例名 `AliveReq`：1 位，**PulseMode**，时钟 `ClkMs`。
2. 放 **Timer**（v2.80），实例名 `DeadTimer`：**Fixed Function**，16 bit，Period **100**，Run Mode **One Shot**，Enable Mode **Software Only**，Trigger None，Capture None，Interrupt 勾 **TC**（但 `interrupt` 输出不接任何东西，固件靠读状态寄存器），时钟 `ClkMs`；`reset` ← `AliveReq` 输出。
3. 放 **Control Register**，实例名 `StepRearm`：1 位，**PulseMode**，时钟 `ClkStep`；输出同时接 `StepArm1.reset` 和 `StepArm2.reset`。
4. 把两个 PWM（`StepArm1`、`StepArm2`）的 **Kill Mode 改成 Latched**，kill 输入都接 `DeadTimer.tc`。（阶段 7 完成后，这里要经一个 Or 门，见阶段 7。）
5. 构建，通过后把 `HW_DEAD` 改成 `(1u)`，再构建。提交。

## 8. 阶段 6：串口发送改 DMA（开关 `HW_TXDMA`）
1. 打开已有的 `HostUart`（UART v2.50）的 Configure：
   - TX Buffer Size 改成 **4**（RX Buffer Size 保持 256，RX 中断保持）。
   - Interrupt On 里取消 TX 的其他项，只勾 **TX - On FIFO Not Full**。
   - 这样 `HostUart.tx_interrupt` 输出在 FIFO 未满时为高，可作 DMA 请求。
2. 用 Creator 的 **DMA Wizard**（菜单或 DMA 组件，名字固定为 `TxDma`）：
   - 名字：`TxDma`；Hardware request：`Direct`/DRQ；Bytes per burst：1；Requests per burst：1。
   - 源：SRAM（上 16 位用 `HI16(CYDEV_SRAM_BASE)`），目的：Peripheral（`HI16(CYDEV_PERIPH_BASE)`）。优先级默认。
   - 把 `HostUart.tx_interrupt` 连到 `TxDma` 的 `drq`。
   - 不要在 Wizard 里建 TD，固件自己建。
3. 构建，确认生成了 `TxDma_dma.h`；通过后把 `HW_TXDMA` 改成 `(1u)`，再构建。提交。

## 9. 阶段 7：引脚中断（开关 `HW_PINIRQ`，要求阶段 3 和阶段 5 已完成）
1. 放 **Control Register**，实例名 `StepKill`：1 位，**DirectMode**，Initial value 0；它的输出和 `DeadTimer.tc` 一起接一个 **Or** 门（Digital → Logic → Or），Or 门输出接两个 PWM 的 kill 输入（取代阶段 5 里直接接 `DeadTimer.tc` 的连线）。
2. 打开已有引脚组件 `FAULT`（P12[5]）的 Configure：Interrupt 选 **Falling Edge**，并勾选 **Use dedicated port interrupt**（这样会出现一个 `irq` 终端）。引脚组件 `LIMIT2`（P0[4]）：Interrupt 选 **Rising Edge**，同样勾选 dedicated port interrupt。
3. 放两个 **isr**，实例名 `FaultIsr`、`LimitIsr`，Interrupt Type 保持默认；`FAULT.irq` → `FaultIsr`；`LIMIT2.irq` → `LimitIsr`。
4. 构建，通过后把 `HW_PINIRQ` 改成 `(1u)`，再构建。提交。

## 10. 全部完成后
- 在 `G:\792\.worktrees\hw-offload` 下跑：`python tests/test_stage1.py --define=HW_SSI=1u --define=HW_STEP=1u --define=HW_EVT=1u --define=HW_DEAD=1u --define=HW_PINIRQ=1u --define=HW_TXDMA=1u`，应当全部通过。
- 遇到问题的处理：某个阶段放不成（例如构建报错、时序违例、UDB 不够），**保持那个开关为 0 并如实报告**，其他阶段不受影响；唯一的依赖是阶段 4、5、7 要先完成阶段 3。
- 不要烧录、不要推送、不要改 `main` 分支。

## 11. 上板前才会确认的事（执行者不用管，但要如实写进报告）
SPI 模式与位偏移；PWM 实际脉宽是否 10 µs；`EVT_RESTART_LAG_US`（`main.c`，默认 3 µs）；PWM 的 Latched kill 是否能被 `StepRearm` 清除；定时器单次模式重启延迟。
