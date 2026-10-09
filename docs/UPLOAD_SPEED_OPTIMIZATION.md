# 上传速度优化

## 实机基线

`out/upload-heartbeat-diagnosis/host-upload-after.log` 中，176 节点从 14:45:17.950 上传到 14:45:28.896，耗时 10.946 秒，约 16.08 节点/秒。节点确认间隔中位数 49 ms；24 个间隔超过 80 ms，其中 22 个包含周期大状态块。没有 `HOST PTP retry`。

逐节点 ACK 与周期诊断共用固件文本发送队列。修改前的真实 C 回归 `status_upload_backlog` 中，58 字节 ACK 已排队后，周期服务把待发文本扩到 1311 字节；新增文本按 115200、8N1 换算需要约 108.8 ms。记录：`out/upload-status-optimization/red-backlog.log`。`status_upload_volume` 也确认上传时原本仍发送完整诊断块，记录：`red-volume.log`。

## 修改

固件 `FW_BUILD_ID=stage2-v11-r2500-slip-mref2-ssififo-upstat1`。

- 仅在 `TRAJ_UPLOADING` 阶段缩减周期状态，保留 `STATE`、`CONTROL`、`PTP_STATE`、`TRAJ_STATE`、`TELEM`。现有上位机需要持续收到状态、参考和位置反馈；周期 `TRAJ_STATE` 也能补偿一次即时 ACK 丢失。
- 发送队列非空时，周期状态服务暂缓追加文本；已有完整消息继续发送，不丢弃诊断或故障证据。
- 周期状态请求保留尚未完成的上传状态块，避免每 500 ms 反复重置到第一行。参考 epoch 改变仍会重启状态块。
- READY、RUNNING、DONE、STOP 恢复完整周期诊断。逐节点即时 ACK、独立遥测流、故障快照和 STOP 仍走原有路径。

主机和协议版本无需修改。波特率、每字节双发、0.5 ms 字节间隔、心跳与运动保护阈值继续采用当前已上板配置。本阶段不实现批量上传。

## 离线结果

`python tests/test_stage1.py status_upload_volume status_upload_backlog status_upload_partial status_upload_stream status_upload_restore status_upload_stop status_upload_fault` 已通过两种编码器配置，共 14 项实际 C 仿真检查。候选固件同一夹具中：完整周期状态 1261 字节，上传状态 374 字节；单编码器配置为 1249 / 362 字节。ACK 入队 58 字节，周期服务后仍为 58 字节；若 STATE 已先入队，ACK 前的该状态行是 99 字节，按线速换算约 8.6 ms。记录：`out/upload-status-optimization/green.log`。

连续流检查用真实报文解析、状态服务和发送队列，按每模拟毫秒最多排出 11 字节的预算，上传完整 176 节点。上传中 STATE/PTP_STATE 最大间隔为 507 ms，TRAJ_STATE 为 94 ms，10 Hz TELEM 为 101 ms；队列峰值 148 字节，丢消息数 0。重复请求也保持未完成周期块，并通过周期 TRAJ_STATE 补确认。

完整 `python tests/test_stage1.py` 已通过 588 项实际 C 仿真检查，记录：`out/upload-status-optimization/full-actual-c.log`。Creator 引脚验证及 12 项错误配置拒绝检查也已通过。以上仅证明固件控制流、消息数量和队列行为；线速换算不是 USB/UART 或实机耗时测量。

## 上板验证

由操作者用 PSoC Creator 打开 `G:\792\.worktrees\hw-offload\DRV8825.cywrk`，重新编译并烧录。使用现有 `G:\792\.worktrees\hw-offload\out\host-heartbeat-fix\TwoLinkCommissioning.exe`，重新建立参考、移到起点，上传同一条 176 节点图形，先不执行轨迹。

核对新 `LINK fw`、开始/完成耗时、节点间隔、STATE/PTP_STATE/TELEM 新鲜度、RX/CRC/短帧/重传及心跳情况。上传前已经排入的完整诊断仍可能拖慢最初几个节点。

## 第一阶段实机结果

操作者重新编译、烧录并回复“上传成功”。日志 `stage1-20261009-151458-653.log` 首条 LINK 已确认 `stage2-v11-r2500-slip-mref2-ssififo-upstat1`。本次实际计划为 **186 节点**，15:15:18.557 开始、15:15:28.078 完成，耗时 **9.521 秒**，吞吐 **19.54 节点/秒**。此前 176 节点、10.946 秒、16.08 节点/秒；按节点数计算，本次吞吐提高约 **21.5%**。两次节点数不同，这不是完全相同图形的受控 A/B。

节点确认间隔中位数仍为 49 ms，P90 从 126 ms 降至 55 ms，最大值从 161 ms 降至 86 ms；超过 80 ms 的间隔从 24 个降至 1 个。上传最终 `TRAJ_STATE state=2 count=186 total=186`，参考、使能与两路编码器正常。STATE/PTP_STATE/TELEM 实机最大间隔分别为 526 / 521 / 174 ms。

上传前后 RX 错误和溢出、CRC/短帧/超时均为 0；`tx_dropped` 保持在上传前已有的 22，`epoch_rejected` 保持在已有的 4，没有上传期增量。记录中没有重传、STOP、故障快照或主机心跳间隔告警。上传后 `key_max_gap_ms=71`，ENC retries/rejected 为 0。快照：`out/upload-status-optimization/host-upload-after.log`。

上述上传验收区间在 READY 结束，没有自动执行。完整日志后续记录操作者于 15:15:34.904 请求 RUN，并最终到达 `state=4 index=186`；这不作为绘图精度验收。本阶段没有改动主机字节间隔，缩短间隔需要下一阶段线路对照与实际上传验证。

## 第二阶段：0.25 ms 主机候选

仅缩短现有生产写入函数的相邻字节间隔，从 0.5 ms 改为 0.25 ms；仍保留每字节双发、逐节点 ACK、原波特率、心跳与输入租约。生产发送、线路工具和传输自测使用同一 `ControlByteGapMs` 常量，线路工具另保留固定 0.5 ms 对照。无需改帧格式或重烧固件。

直接调用已发布主机程序集的实际 `WriteControlFrame`，对同一 36 字节编码帧检查逐字节双发和最小间隔。旧主机 72 次写入中位耗时 35.80 ms，超过 25 ms 检查预算；新候选中位耗时 17.81 ms，并通过同一检查。记录：`out/upload-status-optimization/pace-red.log`、`pace-green.log`。这是离线实际生产函数测量，不是整条轨迹上传时间。

操作者停止并断开上位机后，COM8 未使能空键值心跳对照完成。旧工具每模式 8 秒的筛选中，0.25 ms 双发发送 103 个完整心跳，无新增错误。新候选每模式 5 秒、共 8 模式：0.25 ms 紧凑心跳发出 101 个、收到计数增加 101；全部模式未新增 RX 错误/溢出、CRC/短帧/超时或发送丢弃。完整心跳没有逐帧接收计数，短时零错误不能证明持续突发上传可靠。报告：`uart-stress-upstat1.txt`、`uart-stress-025-candidate.txt`。

候选完整主机 self-test 结束，生成 `host-025-self-test/workbench.png`、控制流和执行轨迹，无 self-test error 文件。真实生产写入的离线上传回放完成 383 节点与 RUN 确认；该上传回放中，完整发送的紧凑心跳最大间隔 40.6 ms，UI 租约最大间隔 63.5 ms，丢弃一个心跳帧后仍完成上传。兼容旧固件完整控制帧的空闲/编辑检查最大心跳间隔分别为 110.2 / 101.1 ms，不能把上传回放数值推广到所有模式。5000 条历史记录收尾检查 dispatcher 最大间隔 30.9 ms。此处 RUN 是离线模拟，未访问串口或驱动电机。结果：`host-025-self-test/heartbeat-tests.txt`，语言自测同时通过。

新候选导出的 `workbench.control.bin` 已通过真实 C `control_host` / `control_host_single_copy` × 双反馈/可选单反馈，共 4 项窄回归。记录：`host-025-actual-c.log`。

主机目录：`G:\792\.worktrees\hw-offload\out\host-upload-pace-025`，入口 `TwoLinkCommissioning.exe`，SHA-256 `D0235007E8FCB37DD25B46477D9261CD2E69B2A972B1DBC34B47F4E267963EA8`。

## 第二阶段实机结果

操作者回复“新上位机也可以上传成功”。日志 `stage1-20261009-153631-158.log` 确认主机入口和 `>=0.25 ms spacing`，固件仍为 `stage2-v11-r2500-slip-mref2-ssififo-upstat1`。本次 186 节点从 **15:37:13.860** 到 **15:37:19.832**，耗时 **5.972 秒**，吞吐 **31.15 节点/秒**。相较第一阶段 9.521 秒，耗时减少 **37.3%**，吞吐增加 **59.4%**。相较最初 176 节点、10.946 秒，按节点数归一化的吞吐增加 **93.7%**；最初计划不同，不能把它作为完全相同图形的 A/B。

第一阶段 `20261009-151518-554-trajectory-3/plan.csv` 与本次 `20261009-153713-858-trajectory-192/plan.csv` 的 SHA-256 均为 `2BF33FFE10213C9D53C2922D092438212D034977B3CE57E190F8D2D872120853`，两次计划逐字节相同。运动参数和图形时长同为 45.975 秒；上传提速没有修改运动速度。

186 个节点均收到确认并进入 `state=2 count=186 total=186 index=0 seq=377 epoch=12`。去重后的节点确认间隔中位数 **30 ms**、P90 **38 ms**（nearest-rank）、最大 **55 ms**，超过 80 ms 的间隔 **0** 个。上传期 STATE/PTP_STATE/TELEM 最大间隔分别为 **503 / 512 / 109 ms**。

上传前最近 LINK 的 `tx_dropped=56`，上传后第一条仍为 56；`rejected/epoch_rejected=5` 同样无增量。RX 错误/溢出、CRC/短帧/超时均为 0。连接初始化和移到起点阶段已有计数增加，这些不算上传错误。上传前后 `key_max_gap_ms=69`，两路 ENC retries/rejected 为 0；上传区间无 HOST 重传、心跳间隔告警、STOP 或故障快照。

日志快照：`out/upload-status-optimization/host-upload-025-after.log`；测量结果：`host-upload-025-metrics.json`。本轮验证到 READY，未执行新主机下的轨迹，也未验证长期突发/不同 USB 链路。保留双发与逐节点确认；本轮到此结束，不新增批量协议。
