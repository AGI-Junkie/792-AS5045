# 项目约定

## WPF 中英文文案同步

- 后续新增、修改或删除 UI 文案时，必须在同一次修改中同步维护 `Commissioning/Lang.cs` 中的中文和英文文本，包括按钮、标签、下拉选项、提示、状态、错误、画布及无障碍名称。
- 界面文案统一通过现有语言表和绑定机制显示，避免硬编码单一语言；动态文案也必须支持即时切换。原始固件日志、协议标识和 CSV 字段保留原格式。
- 修改后构建并运行 `TwoLinkCommissioning.exe --language-self-test`，检查两种语言下的显示与布局。实现和检查命令见 [WPF_LOCALIZATION.md](docs/WPF_LOCALIZATION.md)。
