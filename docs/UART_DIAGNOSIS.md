# UART 线路诊断

## 已知事实

- 2026-09-27 现场日志：两轴未使能、没有电机动作时，上位机连续发送整帧，固件就会出现 `flags=0x08`（停止位/帧错误）；同一 COM 口、同一固件。
- 历史台架对照（记录在旧笔记 `docs/history/WORKBENCH_HARDENING.md`）：连续整帧两轮各出现 7 / 4 次停止位错误；每字节单份、间隔 1 ms 消除了停止位错误，但仍有不完整帧；**每字节发两份、相邻写入间隔 ≥ 0.5 ms** 后，两轮各 10 秒 128 / 127 帧和随后 60 秒都是 0 错误。0.5 ms 是历史稳定基线，不能据此认定当前硬件仍需要这个最小间隔。
- 固件的 `rx_error` 自上电起累计（部分历史实机日志长期停在 129），所以判断“这次有没有新增”要看日志内的增量（`tools/check-log.py` 就是这么做的）。

**历史接收错误的根因仍未查明**。本轮保留双发，将相邻写入间隔从 0.5 ms 改为 0.25 ms，由 `ControlWindow.ControlByteGapMs` 同时用于生产发送和线路工具。17 字节载荷仍占 72 个线上字节；最小节拍等待由 35.5 ms 降至 17.75 ms，紧凑心跳由 5.5 ms 降至 2.75 ms，不包含串口写入、USB 和线程调度耗时。波特率、帧格式和心跳租约不变。已完成未使能完整及紧凑心跳线路筛选，以及一条 186 节点图形的实机上传，不能宣称历史链路根因已解决。

## 用工具把现象量化

`TwoLinkCommissioning.exe` 里有一个只读的实验模式，**只发送键值为 0 的心跳帧**（不含任何能让电机动的帧），并且在固件报告 `enabled=1` 时拒绝运行：

```powershell
.\host\TwoLinkCommissioning.exe --uart-stress COM8 20 D:\uart-kitprog.txt KitProg
```

参数依次是串口、每种节拍的持续秒数、报告文件、备注。运行前：固件已上电且未使能（按过 Space），上位机主窗口已关闭（否则串口被占用）。它依次用 8 种字节节拍各发 20 秒，读回固件每 500 ms 打印的 `LINK` / `PTP_RX` 行，报告每种节拍下这些计数器的增量；新增 0.5 ms 基线位于枚举末尾，原有节拍编号保持不变：

| 节拍 | 含义 |
|---|---|
| `Production` | 当前候选：每字节两份，写入间隔 0.25 ms，随生产常量变化 |
| `BurstSingle` | 整帧 36 字节一次写出，字节背靠背 |
| `BurstDouble` | 每字节复制一份，72 字节一次写出 |
| `PacedSingle1ms` | 每字节一份，间隔 1 ms |
| `PacedDouble025ms` | 每字节两份，间隔 0.25 ms |
| `PacedDouble1ms` | 每字节两份，间隔 1 ms |
| `CompactProduction` | 紧凑心跳（6 字节 × 2 份），间隔 0.25 ms，随生产常量变化 |
| `PacedDouble05ms` | 历史基线：每字节两份，固定间隔 0.5 ms |

报告里的 `rx_error`（UART 硬件标志：停止位、奇偶、break、溢出）、`drop_crc`、`drop_short`（帧不完整）、`drop_timeout` 是要看的列。

`wire_bytes` 按成功的串口写入字节数累计，包括双份；完整心跳双发为 72 字节，紧凑心跳双发为 12 字节。历史工具按编码帧长度累计，双发模式的该列少计一半；历史报告不修改，比较时应按帧数和模式重新计算。

## 2026-10-09：0.25 ms 候选线路筛选

操作者停止并断开上位机后，`fw=stage2-v11-r2500-slip-mref2-ssififo-upstat1` 在未使能状态完成旧工具的 7 种节拍，每种 8 秒。报告保存在 `out/upload-status-optimization/uart-stress-upstat1.txt`：所有模式的 `rx_error`、`rx_overflow`、CRC、短帧、超时和 `tx_dropped` 增量均为 0。0.25 ms 双发模式发出 103 个完整心跳；旧 `CompactProduction` 的 0.5 ms 紧凑心跳发出 134 个、`short_ok` 增加 134。

这是短时、未使能的线路筛选。完整心跳没有逐帧接收计数，不能仅由零错误断言全部收到；0.25 ms 的紧凑心跳尚未在该报告中覆盖。

随后新候选工具完成 8 种节拍、每种 5 秒，报告 `out/upload-status-optimization/uart-stress-025-candidate.txt`。新增固定 `PacedDouble05ms` 保留 0.5 ms 对照；`Production` 与 `CompactProduction` 使用生产常量 0.25 ms。紧凑心跳发出 101 个、`short_ok` 增加 101；8 种节拍均未新增 RX 错误/溢出、CRC/短帧/超时或发送丢弃。新工具 `wire_bytes` 统计成功写出的物理字节，包括双发；旧工具统计编码长度，少算了双发副本。

操作者随后使用新入口，上传与 0.5 ms 完全相同的 186 节点计划（两个 `plan.csv` SHA-256 一致）。上传 5.972 秒，进入 READY；无重传/心跳告警，RX/CRC/短帧/超时、发送丢弃和 epoch 拒绝均无上传期增量，上传前后 `key_max_gap_ms=69`。这是一条计划的实机上传验证，未执行本次轨迹，尚无长期突发/不同 USB 链路验证。记录见 `UPLOAD_SPEED_OPTIMIZATION.md`。

## 怎么解读

| 现象 | 说明 | 下一步 |
|---|---|---|
| 只有 `Burst*` 报错，`Production` 全 0 | 字节背靠背时链路有问题，当前节拍在这次采集中未报错 | 对比 `PacedDouble05ms` 历史基线，并持续采集生产心跳作为通信验收 |
| `Production` 也报错 | 0.25 ms 候选在这次采集中不可靠 | 先对比 `PacedDouble05ms`；0.5 ms 也报错时再换线/换 USB 口/查共地，做下面的 A/B 对照 |
| `PacedSingle1ms` 有 `drop_short` 而 `PacedDouble*` 没有 | 会丢单个字节（不是帧格式问题） | 双份是必要的；重点查丢字节发生在桥接还是 PSoC 一侧 |
| 所有节拍都是 0 | 台架上现在复现不了 | 在正常使能/运动流程中采集日志，比较有无电机干扰；线路工具仍只允许未使能状态 |

## A/B：换掉 USB 串口桥

要判断问题出在 KitProg 桥还是 PSoC 一侧，需要把同样的实验换一个串口桥再做一遍（外部 3.3 V USB‑UART 适配器，共地）。固件的 UART 固定在 P12[6]（RX）/ P12[7]（TX），KitProg 的串口线也接在这一对上：**不要让两个驱动源同时接在同一根线上**。怎么断开 KitProg 那一侧，请对照 CY8CKIT-059 原理图确认；如果不能断开，替代办法是在 TopDesign 里把 `HostUart` 改到另一对引脚（这会改固件，需要重新过闸门和上机，不在本轮范围）。

对照的结论：

- 外部适配器下 `BurstSingle` 也是 0 → 问题在 KitProg 桥（或它与 PSoC 之间的线），PSoC 的 UART 接收没问题。
- 外部适配器下 `BurstSingle` 同样报错 → 问题在 PSoC 一侧（UDB UART 配置、时钟、中断），应该改固件，而不是改协议。

无论哪种，记得把报告文件和 `LINK` 行原文一起留档。

## 2026-10-09：hw-offload TX DMA 隔字丢失

烧录六项卸载全开的固件后，COM8 可以打开，上位机一直等不到可识别的固件应答。关闭上位机后直接用 115200 8N1 读取 COM8，同样收到 `SAE`、`IK` 等损坏标识；固定状态前缀恰好只保留原文的隔一个字节。带一个 `s` 查询的 2.2 秒采集收到 3117 字节，纯被动 1.2 秒采集收到 1812 字节，两次均无完整 `STATE` / `LINK`。

只将 `HW_TXDMA` 关闭、保留其余五项卸载和 48 MHz 配置，经 PSoC Creator 4.4 编译并由操作者烧录后，同样的 2.2 秒检查收到 6636 字节，完整状态行通过：`fw=stage2-v11-r2500-slip-mref2-txdiag0`、`proto=4`、`rx_error=0`、`tx_dropped=0`、`ssi=1 step=1 evt=1 dead=1 pinirq=1 txdma=0`。此对照确认 UART TX DMA 启用路径是连接故障的触发项；尚未分辨 FIFO/DRQ 时序和 KitProg 接收行为各自的影响。

当前默认 `HW_TXDMA=0`，沿用现有 CPU 非阻塞发送路径和 4 字节 TX FIFO。修改不涉及 UART 波特率、协议、运动保护或其余卸载。引脚检查允许 CPU 填硬件 FIFO，同时保留原 DMA 配置的拒绝用例。重新开启 DMA 前需要新的实机通信证据；恢复连接不代表运动、SSI 或发送吞吐已验收。

本次采集和对照构建保存在 `out/uart-link-diagnosis/`；572 项 actual-C 检查属于离线证据。

随后操作者确认上位机已连接；`stage1-20261009-141659-629.log` 持续收到完整状态和版本行，关节范围同步完成，未出现固件应答超时。启动配置/状态输出期间发送队列整条消息丢弃计数由 0 升至 17，后续采样保持 17，`rx_error=0 rx_overflow=0`。这说明连接恢复，但发送吞吐尚未验收。
