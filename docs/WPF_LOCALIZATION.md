# WPF 中英文切换

右上角 `EN` 切换到英文，`中` 切换到中文，即时生效，无需重启。
语言记在 `%LOCALAPPDATA%\TwoLinkCommissioning\settings.json`，重启后恢复。
保存采用临时文件替换，保留其他设置；损坏的设置会备份为 `.bak` 后恢复。

切换覆盖按钮、分组、属性名称、下拉选项、提示、画布、无障碍名称，以及已有状态、错误和实验摘要。
原始固件日志、协议标识和 CSV 字段保留原格式；输入小数点始终使用 `.`。
切换不会重新检查/应用参数，不重建规划，不改变预览播放位置、相机、已上传图形或执行状态，也不请求运动或配置同步。

语言对集中在 `Commissioning/Lang.cs`。静态标题按键绑定到控件属性，只有译文变化时才更新；使用弱引用，不让语言表持有已关闭的界面。避免资源字典更新连带重置文本编辑器样式造成停顿。
保留在界面上的动态文案保存格式化函数；规划验证错误保留键和参数，切换后重新显示文案。
异常分支按错误键判断，避免译文影响轨迹重定时。图形和逆解下拉使用固定的 `ComboBoxItem`，仅修改显示文字。

检查命令（在工程目录执行）：

```powershell
dotnet build Commissioning/Commissioning.csproj -c Release
Commissioning/bin/Release/net10.0-windows/TwoLinkCommissioning.exe --language-self-test
Commissioning/bin/Release/net10.0-windows/TwoLinkCommissioning.exe --self-test --snapshot deliverables/host-bilingual-check/preview.png
```

`--language-self-test` 检查语言表、参数/规划保留、已有错误/反馈、上传/待执行/执行中状态、命令队列、数字格式和偏好恢复。
完整自检还包含串口模拟回放，上传期间每 500 ms 切换一次语言，检查心跳和 UI 租约阈值。
所有自检关闭语言偏好写入；使用模拟设备，不打开串口、不发送真实电机命令。
截图覆盖 1100×760 和 1380×930 两种窗口尺寸。

交付：`deliverables/host-bilingual/TwoLinkCommissioning.exe`，保留同目录依赖文件。
需要 Windows x64 和 .NET 10 Windows Desktop Runtime；与当前 `DeviceContract.cs` / `main.c` 对应的固件配套使用。
本次只发布上位机，未烧录固件、未进行实机双语切换测试。
