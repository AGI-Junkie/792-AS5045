# 2026-10-09 硬件 SSI 接收完成判断

串口修复版 `stage2-v11-r2500-slip-mref2-txdiag0` 可以连接，但上位机报告 `ENABLE refused: encoder invalid; inspect E diagnostics`。COM8 仅查询 `s/e/f` 的结果存于 `out/uart-link-diagnosis/encoder-before.txt`：单编码器和双编码器帧均为 `raw=0x00000`；查询时 `enabled=0`。

## 已确认的代码错误

Creator 生成的 `Ssi.c` 在硬件 RX FIFO 模式下，`Ssi_GetRxBufferSize()` 只根据 `Ssi_STS_RX_FIFO_NOT_EMPTY` 返回 0 或 1。当前 RX Buffer Size 为 4，不使用软件缓冲。旧 `ssi_done()` 却等待返回值达到本次发送的 2 或 4 个字，条件永远不能满足，随后同步/异步调用超时，返回零帧。因此日志里的 `DO stuck LOW` 不能证明物理 DO 引脚真的为低。

原测试替身返回 FIFO 实际字数，未模拟这个 API 限制。将替身改成真实的 0/1 行为后，`python tests/test_stage1.py origin_retry ssi_fifo` 在 `enc_set_origin()` 失败，记录于 `out/uart-link-diagnosis/ssi-fifo-red.log`。

## 修复范围

- `hw_ssi.h` 在 `ssi_done()` 中取出当前可读的字，累计到本帧的接收数组；只有收到全部 2/4 个字才完成。
- `ssi_take()` 继续使用同一位对齐解码；异步前台读取、同步初始化、重试和中止共用该修复。
- 测试替身使用真实空/非空语义。新增 `ssi_fifo` 检查逐字到达、重复轮询、完整双编码器帧和后续单编码器帧。
- 固件标识改为 `stage2-v11-r2500-slip-mref2-ssififo`。其余 SSI 配置和编码器有效性保护保持原值；UART TX DMA 仍为 0。

21 项针对性 actual-C 模拟检查通过，含读取、快照、重试、超时、同步接管、原点重试、坏编码器使能拒绝和启动；完整测试为 574 项通过。日志：`out/uart-link-diagnosis/ssi-fifo-green.log`、`ssi-fifo-full-tests.log`。这些是离线控制流证据。

## 用户烧录与板上验证

用 PSoC Creator 打开 `G:\792\.worktrees\hw-offload\DRV8825.cywrk`，重新编译并烧录。保持上位机断开后查询 COM8，要求 LINK 标识以 `ssififo` 结尾，且两路编码器原始帧通过状态和校验检查。随后由操作者在上位机使能，核对 `ENABLE OK` 和 `STATE enabled=1 enc=1 enc2=1`。

操作者烧录后，COM8 报告正确的 `ssififo` 标识和 `STATE enabled=0 enc=1 enc2=1`。三次单路加三次双路查询，共 9 个原始帧通过现有有效性门槛：偶校验正确、OCF=1、COF=LIN=0；板 1 示例 `0x2AA26`，板 2 示例 `0x219A6`。记录：`out/encoder-ssi-fix/encoder-after.txt`。板 1 的 INC=DEC=1 被现有分类器标为磁场警告、角度有效，并未为本次修复修改该规则。初始查询脚本只接受文字 `ENCODER OK`，误拒了该合法警告状态；随后按固件现有原始帧有效性条件复核，9 帧全部通过。

操作者随后确认“已使能”。上位机会话 `stage1-20261009-142827-284.log` 出现 `ENABLE OK: no automatic movement`，后续持续报告 `enabled=1 enc=1 enc2=1 fault=0` 和正确的 `ssififo` 标识；快照为 `out/encoder-ssi-fix/host-enabled-after.log`。静止反馈采样的 `ENC_DIAG retries=0 rejected=0`，当前最大间隔为 17 ms。初始配置/状态输出使 `tx_dropped` 增至 19 后稳定，RX 错误和溢出均为 0；该计数是消息队列丢弃，不是前次 DMA 的隔字丢失。

串口连接及本次使能拒绝均已在板上恢复。运动、回零、SSI 电气时序和吞吐尚未验证；板 1 磁场告警仍存在。代理仅发送查询，不执行使能或运动。

组件背景：[Infineon SPI Master v2.50 数据手册](https://www.infineon.com/assets/row/public/documents/cross-divisions/396/infineon-component-spi-v2.50-software-module-datasheets-en-09018a90808603f2.pdf)。API 返回值以本工程生成的 `Ssi.c` 为直接依据。
