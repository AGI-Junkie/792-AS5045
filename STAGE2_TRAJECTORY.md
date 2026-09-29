# 阶段二 B：图形选择、轨迹检查与规划数据导出

后续时间规划与回放版见 [STAGE2_TIMING.md](STAGE2_TIMING.md)，最新上位机为 `deliverables/stage2-timing/host`。下文保留几何预览版的记录与检查边界。

运行 `deliverables/stage2-trajectory/host/TwoLinkCommissioning.exe`，保留完整 host 文件夹。需要 .NET 10 Desktop Runtime（当前电脑已有）。本版只更新上位机，无需烧录；仍兼容 stage1-v3。保留原有控制、回零、停止、日志及单点预览。

## 离线操作

无需连接串口或给电机上电。在“图形 / XY 预览”页选择：

| 图形 | 输入 | 轨迹顺序 |
|---|---|---|
| 单点 | X、Y | 原来的双逆解预览 |
| 直线 | 起点 X/Y、终点 X/Y | 起点至终点 |
| 矩形 | 左下角 X/Y、宽、高 | 向右、向上、向左、向下闭合 |
| 圆 | 圆心 X/Y、半径 | 最右侧起点，逆时针闭合 |

示例：圆心 (220, 20) mm、半径 60 mm、肘角 q2−q1 ≥ 0 分支。L1=180 mm、L2=176.38 mm；q1、q2 均相对固定 +X 零方向。拖动下方滑块查看各采样点对应的机械臂姿态、X/Y、q1/q2 和累计路程。图形输入只在当前窗口保留。

关节范围未测定时留空，界面明确标为未检查；测定后须完整填写两轴四个边界。选择分支后整条轨迹保持同一肘部构型；q1 跨 ±180° 时连续展开，不能用归一化掩盖实际超限。规划不会自动换分支以绕过超限。

## 检查范围

- 相邻采样点不超过 1 mm，最多 5000 点。圆按均匀角度采样后形成闭合折线。
- 检查每段线的最近原点位置以及端点半径，防止端点可达而中间穿过内侧不可达区；圆还检查解析圆周的最小/最大半径。
- 拒绝触及完全伸直、完全折叠的奇异位形，提示接近奇异位形的采样点。
- 对所有采样点检查用户填写的关节范围。这不是机械碰撞、线缆干涉、连续运动或速度/加速度可行性验证，也不包含从当前姿态到起点的引入路径。
- 非有限参数、无效尺寸、不可达、超限时清空旧图并禁用滑块和导出，避免导出上一次成功结果。

## CSV 含义

“导出规划 CSV”使用系统保存窗口，输出 UTF-8 BOM CSV，数字小数点固定为 `.`。每行含 `preview_only`、图形、L1/L2、分支、关节检查状态和范围、点号（从 0 起）、累计路程 mm、目标 X/Y mm、规划 q1/q2 度。

`limits_status=unmeasured` 表示机械范围未填写；`samples_checked` 表示在填写的范围内检查了全部采样点。允许保存未标定的几何方案，但文件不是运动指令，不能据此执行机械臂。

本版没有实测坐标、跟踪误差或实验结果，不能把规划数据当作实测数据。尚未实现速度规划、串口轨迹协议、自动绘制、闭环 XY 误差采集。arm2 仍无编码器，命令步数不能充当实测角度。

## 当前硬件状态

此前阶段一曾由用户确认验收通过；后续临时展开偏移更新为 arm1 127°（22577 微步）、arm2 163°（36990 微步），仍待正式标定。最新上位机没有改变这些值，固件包仍是 `deliverables/stage2-offset/firmware.hex`。

临时偏移版本随后出现 arm2 慢触碰 NPN 后未持续有效而停止的日志，尚未完成对应实机复测。用户报告 arm1 运动后过热，怀疑电流偏大，当前尚无新 VREF、采样电阻和接线实测结论。用户要求继续软件工作，本次不修改驱动电流、保持逻辑或回零状态机。

实际 XY 接入仍需确认上述硬件状态、回零重复性、伸直零偏和两轴可用范围；本版没有实际 XY 执行按钮。

## 修改与验证

新增 `Commissioning/DrawingTrajectory.cs`；更新 `Commissioning/XYPreviewPanel.cs`、`Commissioning/App.cs`、README、STAGE2_PREVIEW，并增加本文档。修改前备份：`recovered/trajectory-before-20260924-205113`。

离线复现：

```powershell
dotnet publish Commissioning/Commissioning.csproj -c Release -r win-x64 --self-contained false -o deliverables/stage2-trajectory/host --nologo
$exe = Join-Path (Get-Location) 'deliverables/stage2-trajectory/host/TwoLinkCommissioning.exe'
$image = Join-Path (Get-Location) 'deliverables/stage2-trajectory/preview.png'
$process = Start-Process -FilePath $exe -ArgumentList '--self-test','--snapshot',$image -WindowStyle Hidden -Wait -PassThru
$process.ExitCode
```

自测涵盖三种图形、双分支正逆回算、闭合与间距、线段内部不可达、圆周越界/奇异点、±180° 连续性、范围拒绝、非有限输入、CSV 列与区域格式，以及 UI 旧数据失效和原有控制门禁。自测不打开串口；`preview.png` 和同名 CSV 是离线示例，非实机结果。
