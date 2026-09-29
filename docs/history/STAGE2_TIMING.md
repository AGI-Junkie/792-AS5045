# 阶段二 C：离线时间规划与屏幕回放

> **历史记录**：记录当时那次交付的改动和验证，可能与当前源码不一致（版本目录、固件标识、坐标约定都已变）。当前说明见 [README](../../README.md)、[协议](../PROTOCOL.md)、[状态模型](../STATE_MODEL.md)；以源码为准。

运行 `deliverables/stage2-timing/host/TwoLinkCommissioning.exe`，保留完整 host 文件夹。本次不修改固件、不需要重新烧录，兼容 stage1-v3。全部新增功能可在不连接串口、不上电时使用。

## 操作

1. 在“图形 / XY 预览”选择直线、矩形或圆，填写坐标、尺寸和逆解分支。原有单点预览仍可使用。
2. 填写离线规划上限：末端速度 mm/s、两轴关节速度 °/s、两轴关节加速度 °/s²。默认分别为 20、5、3、10、6，都是预览参数，未验证为电机能力，也不会写入驱动器或固件。
3. 界面显示预计运动时间。“播放预览”按规划时间 1× 播放，可暂停、拖动时间滑块或回到起点。当前 XY、关节角、末端速度、两轴速度及加速度随动画更新。
4. 修改参数会停止回放并从头计算；无效参数、越界或超限会清空旧方案，禁用播放和导出。播放到末尾自动停止，再按播放会从头开始。窗口失焦、切走该页或按总停止按钮会暂停动画。
5. 导出 CSV 包含时间、规划位置、速度和加速度，仍标为 `preview_only`，并用 `timing_status=sampled_estimate` 表明时间规划是采样估算。未测定关节范围时 `limits_status=unmeasured`。

## 规划方式与边界

- 直线和圆为一段，矩形四条边分别一段。每段使用五次进度曲线，首尾关节速度和加速度为零；矩形在拐角停稳后再转向。圆从最右点开始，按解析圆周计算，不在每个几何采样点停顿。
- 用逆运动学雅可比解析计算关节速度、加速度。每段在归一化时间上检查 1001 个位置，根据末端速度及两轴速度/加速度上限延长运动时间，再增加 5% 时间余量。
- 这不是连续区间上的极值证明，余量也不能替代实机验证；接近奇异位形时尤其不能把估算结果用于电机执行。固件接入前仍需连续约束验证、脉冲时序与负载试验。
- 最大预览时间 600 秒，用于限制输入和导出大小。合法速度/加速度输入范围为 0.01～1000；这一软件范围不是硬件规格。
- CSV 按 20 ms 网格输出，另外包含所有段端点/矩形拐角，所以端点附近间隔不一定恰好 20 ms。时间是规划时间；UI 约每 33 ms 刷新并依据单调时钟追赶，不能作为实时控制时钟。
- 圆的时间规划长度按解析圆周算；旧版静态预览长度按采样折线算，两者会有很小差异。
- 几何分支、连续展开 q1、关节范围输入及检查规则沿用上一版；时间规划还检查新增时间采样点的关节范围。未检查机械碰撞、线缆、到起点的引入路径或硬件执行能力。

本版仍没有实际 XY 指令、自动绘图、实测末端位置或跟踪误差。arm2 的命令步数不是实测角度。电流/温升、NPN 回零复测、正式零偏标定和关节可用范围仍待实机确认，状态详见 STAGE2_TRAJECTORY.md。

## 验证与复现

```powershell
dotnet publish Commissioning/Commissioning.csproj -c Release -r win-x64 --self-contained false -o deliverables/stage2-timing/host --nologo
$exe = Join-Path (Get-Location) 'deliverables/stage2-timing/host/TwoLinkCommissioning.exe'
$image = Join-Path (Get-Location) 'deliverables/stage2-timing/preview.png'
$process = Start-Process -FilePath $exe -ArgumentList '--self-test','--snapshot',$image -WindowStyle Hidden -Wait -PassThru
$process.ExitCode
```

自测不打开串口。检查三种图形、两个分支的正逆回算，起止/拐角零速度和零加速度，独立差分核对导数，更密集时间采样核对上限，降速后的时长缩放，圆一整圈的角度连续性，CSV 列数，以及播放、暂停、结束、无效参数使旧方案失效。已有几何与控制门禁自测一并保留。

修改前备份：`recovered/timing-before-20260924-210246`。新增 `Commissioning/TimedTrajectory.cs`；更新几何模型的分段数据、预览 UI、主窗口的动画暂停与自测入口、README 和文档。发布包带 BUILD_REPORT、构建/自测日志、校验值、示例 PNG/CSV。没有烧录或实机运动测试。
