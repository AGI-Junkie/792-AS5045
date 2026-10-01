# Two-Link Studio v5 UI3

> **历史记录**：记录当时那次交付的改动和验证，可能与当前源码不一致（版本目录、固件标识、坐标约定都已变）。当前说明见 [README](../../README.md)、[协议](../PROTOCOL.md)、[状态模型](../STATE_MODEL.md)；以源码为准。

本版只更新上位机，兼容 stage2-v5 固件，无需因界面更新重新烧录。关闭旧程序，运行 `deliverables/stage2-workbench-v5-ui3/host/TwoLinkCommissioning.exe`，保留完整 host 文件夹。UI2 交付目录保留，可回退。

## 本次调整

- 执行按钮旁统一显示状态：绿色“可执行 / 可移动”、黄色“等待中”、红色“被拒绝”。上传点数、绘制进度和等待确认也在此处显示；长信息可悬停查看。
- 范围数值、低速执行计划、完整设备原因及规划检查全部收进左侧“查看规划检查详情”。顶部重复指引卡片删除，静态流程放进执行按钮的提示。
- 蓝色表示连接、使能、参考建立和运动等主流程操作；普通工具为白底描边；总停止为红色。连接不使能、不自动运动，人工零位确认也不驱动电机。
- 禁用按钮使用淡灰底、灰字及 0.5 不透明度，即使原本是蓝色按钮也有明显禁用外观。
- 反馈折叠标题与实际两个页签对应；页签简写为“原始日志”，复制说明移至提示，仍支持原生 Ctrl+C。
- 画布最低高度 260，保留既有缩放、平移、框选、图层、全臂概览和毫米刻度。

既有运动门禁与固件协议保持不变。状态条依据按钮判定与实时设备状态生成；不凭“到起点”提示放宽坐标参考、新鲜反馈或限位检查。

## 验证与边界

Release 编译、完整上位机自测、两种窗口尺寸渲染均通过。新增检查覆盖等待起点、可执行、越界拒绝、故障覆盖就绪、上传进度、旧忙状态清除、反馈失效撤销就绪、禁用主按钮外观恢复，以及画布和按钮布局。

`workbench.png` 为模拟就绪状态，`workbench-waiting.png` 为模拟未使能状态。其它截图覆盖紧凑窗口、反馈展开和全臂视图。所有截图均为离线渲染，不是实机验收证据；本次没有连接串口、烧录或运行电机。

## 重建

```powershell
dotnet publish Commissioning/Commissioning.csproj -c Release -r win-x64 --self-contained false -o deliverables/stage2-workbench-v5-ui3/host
$p = Start-Process -FilePath .\deliverables\stage2-workbench-v5-ui3\host\TwoLinkCommissioning.exe -ArgumentList @('--self-test','--snapshot','G:\792\AS5045_enc\deliverables\stage2-workbench-v5-ui3\preview.png') -WindowStyle Hidden -Wait -PassThru
$p.ExitCode
```
