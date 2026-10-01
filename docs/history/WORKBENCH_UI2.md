# Two-Link Studio v5 UI2

> **历史记录**：记录当时那次交付的改动和验证，可能与当前源码不一致（版本目录、固件标识、坐标约定都已变）。当前说明见 [README](../../README.md)、[协议](../PROTOCOL.md)、[状态模型](../STATE_MODEL.md)；以源码为准。

本次为上位机界面更新，兼容已有 stage2-v5 固件。交付目录为 `deliverables/stage2-workbench-v5-ui2`，只发布上位机，不包含新固件；不需要因本次界面更新重新烧录。

## 启动与操作

关闭旧上位机，运行完整 `host` 文件夹内的 `TwoLinkCommissioning.exe`。保留旧 v5 交付目录以便回退。仍需要 .NET 10 Desktop Runtime。

- 左侧“图形参数”用于选图形、填写尺寸、检查和保存参数；“设备 / 回零”用于使能、人工参考、回零、点动和输入状态。连接成功后自动切到设备页。
- 中央画布独立伸缩，不再嵌套在表单滚动区域。参数较多时只滚动侧栏。
- “适合轨迹”聚焦当前图形，默认隐藏预览连杆；空间足够时右上显示全臂概览和当前视野框。“全臂”显示离线预览姿态和理论可达圆，理论可达圆不代表机械安全范围。
- 滚轮围绕鼠标位置缩放；左键或中键拖动平移；点击“框选”后拖出矩形放大。全臂 / 适合轨迹可恢复视野。遥测和回放不会重置手动视野；修改图形参数会重新适合图形。
- 坐标等比例显示，含自适应毫米网格、刻度、比例尺和鼠标 XY。局部放大不代表测量精度增加。
- 青色线为规划路径，橙色点为离线预览，紫色十字为命令角度计算的 XY；不是实测笔尖位置。全臂概览中的蓝色连杆也属于离线预览。
- 播放与时间滑块仅用于离线预览；实机上传和执行进度在顶部。两个实际运动按钮保留在画布下方。查看画布不会发送运动命令。
- 底部反馈 / 实验 / 日志可展开；展开时收起并暂停屏幕回放，为反馈保留空间。完成、中止及结果保存失败时会自动展开。日志保持原生选择和 Ctrl+C 复制。
- 右上固定总停止，Space / Esc 仍用于停止。既有参考检查、限位、反馈新鲜度、到位判断和互斥控制沿用原逻辑。

## 离线验证

在工程目录执行：

```powershell
dotnet publish Commissioning/Commissioning.csproj -c Release -r win-x64 --self-contained false -o deliverables/stage2-workbench-v5-ui2/host
$p = Start-Process -FilePath .\deliverables\stage2-workbench-v5-ui2\host\TwoLinkCommissioning.exe -ArgumentList @('--self-test','--snapshot','G:\792\AS5045_enc\deliverables\stage2-workbench-v5-ui2\preview.png') -WindowStyle Hidden -Wait -PassThru
$p.ExitCode
```

自测包含既有规划、协议、参数、执行按钮与记录检查；新增光标锚定缩放、缩放边界、平移坐标、框选和误点击、反馈不改变视野、视图操作不产生运动请求，以及两种窗口尺寸的画布和按钮边界检查。

截图覆盖普通、紧凑、反馈展开和全臂视图。截图为程序离线渲染，包含明确标注的模拟数据。未连接串口、未烧录、未进行实际机械臂运动；不能据此宣称实机轨迹精度已经验证。

## 参考

- RoboDK：主视图、工具栏和状态分区。https://robodk.com/doc/en/Interface.html
- UGS：路径可视化与独立控制面板。https://winder.github.io/ugs_website/
- LightBurn：鼠标锚定缩放、平移和适合选区。https://docs.lightburnsoftware.com/latest/Reference/ZoomPan/
- MoveIt：规划、起始、目标和当前状态分层。https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html
