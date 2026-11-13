# USB CDC 链路（P5c）

目标：上位机经芯片自带的 USB（J6，P15[6]/P15[7]）通信，不再只靠 KitProg 的 UART 桥。出处：`docs/HARDWARE_OFFLOAD_PLAN.md` P5c。

## 状态

当前代码核对（2026-10-19）：`HW_USB=1`、`HW_TXDMA=0`，协议 proto=6。下表构建与上板结果是此前 USB CDC 交付的记录，不能替代后续完整固件包的验收；本次未连接设备或复测。

| 部分 | 状态 |
|---|---|
| 固件链路层（`HW_USB`，默认 1） | 已做；仿真 5 个用例（`usb_tx` / `usb_fallback` / `usb_stall` / `usb_rx` / `usb_link`） |
| TopDesign：`UsbCdc`（USBUART CDC 宏，USBFS v3.20） | 已放；引脚 Dm=P15[7]、Dp=P15[6] |
| 时钟 | 已改；Creator 构建 0 错误 0 警告，时序 No Timing Violations |
| 上位机：认出 USB 口、置 DTR、取消双发和字节间隔（`UsbLink.cs`） | 已做；自检 `USB link`（`--self-test`）通过 |
| 历史上板记录（当时版本） | 插着 J6 时 `link=usb`、能使能、无复位无停机、`key_max_gap_ms=53`（KitProg 时 66–134）；拔掉 J6 后回退 UART（`link=uart`、`ENABLE OK`、无停机）；用 USB 跑完整轨迹无复位无停机；拔掉再插回 J6 自动切回 USB；只接 J6 一根线可供电使用。以上不代表当前全部速度、负载和故障条件通过 |

## 固件行为

- “USB 链路已打开” = 已枚举（configured）、主机置了 DTR，**并且 100 ms 内看到过总线活动**（`UsbCdc_CheckActivity()`，主机连着时每毫秒一个 SOF）。
  没有最后一条时，拔掉 J6 后“已枚举”和 DTR 都不会清除（没开 VBUS 监测），固件会一直把输出塞给没人读的端点，KitProg 的 UART 就是哑的（上板时 COM8 连不上、`tx_dropped` 飙到上千就是这个原因）。
- 已打开：发送队列经 CDC IN 端点发出，每包最多 64 字节；未打开：仍走 KitProg UART（和以前一样）。
- 接收是合并的：USB 和 UART 的字节都进同一个解析器。
- 主机停止读取（端点一直不就绪）时，队列写满后整条丢弃并计入 `tx_dropped`，`txq_flush_blocking` 100 ms 后放弃，和 UART 卡住时一样。
- `LINK` 行新增 `link=usb|uart`，上位机据此知道当前走哪条链路。
- `HW_USB` 与 `HW_TXDMA` 不能同时为 1（编译期报错）：TX DMA 仍然关着，且尚未修复。

## 时钟改动（`DRV8825.cydwr`）

| 项 | 之前 | 现在 |
|---|---|---|
| IMO | 3 MHz ±1% | 24 MHz，开倍频器，USB 锁定时 ±0.25% |
| PLL | 输入 IMO 3 MHz | 输入 IMO 24 MHz，输出仍是 48 MHz |
| USB_CLK | 关 | 开，IMOx2 = 48 MHz |
| ILO | 1 kHz | 1 kHz 与 100 kHz 同时使能，Route 选 100 kHz |
| 总线 / 主时钟 | 48 MHz | 48 MHz（不变） |
| `HostUart_IntClock` | 923.077 kHz（÷52） | 不变 |

ILO Route 改为 100 kHz 是 USBFS 的硬性要求（Creator 报 `fit.M0031`）。看门狗用的是 ILO 1 kHz 输出，只要求它保持使能（`CyWdtStart` 说明），所以看门狗周期按设计不变；这一点没有上板验证。

## 风险（上板前要知道）

1. **USB 没连、IMO 没被锁定时精度只有 ±4%**（数据手册，非 USB 模式）。总线、步进脉冲、µs 计时、UART 波特率都会跟着偏。USB 连着且在传 SOF 时是 ±0.25%。KitProg UART 在没有 USB 锁定时可能失配；这是计划里写过的取舍。
2. 只用 KitProg 线（不插 J6）时，应核对未锁定时钟对计时精度的影响。编码器提供电机侧位置监测和保护，PTP/轨迹没有连续误差纠偏控制律，不能保证“兜住位置”；步频、deadman 等时间相关限值也会随时钟偏差变化。
3. J6 单线供电（VBUS 经 F1/D2 并入 VTARG）：已在板上验证可用。
4. 烧录和调试仍然走 J10（KitProg）。

## 上板验证步骤

1. 在 Creator 里构建并烧录（J10）。
2. 再插 J6：Windows 应出现一个新的 COM 口（“USB 串行设备”）。
3. 用串口工具以任意波特率打开新 COM 口并置 DTR，发 `s`：应收到完整的 STATE/LINK 行，当前源码的 `LINK` 应含 `link=usb`、`proto=6`；固件标识须与所选构建包一致。
4. 关闭该 COM 口：下一次 `s` 走 KitProg 的 COM 口，`link=uart`。
5. 拔掉 J6、只留 J10：行为应和以前一致；记录此时 `--uart-stress` 的结果。
6. 观察看门狗：固件不应反复复位（反复出现开机信息即为复位）。

## 排查备忘

- 连 COM8 报 `Access to the path 'COM8' is denied`：别的程序（PuTTY 等串口终端）占着这个口，先关掉它。
- 拔掉 J6 后 Windows 的端口列表里还会残留旧的 COM10，选它会报 `Could not find file`，是注册表里的历史记录，不影响功能。
