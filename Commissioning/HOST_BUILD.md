# 上位机编译命令

在 Windows PowerShell 中执行，需要安装 **.NET 10 SDK**。

## 仅编译并发布上位机

```powershell

Set-Location -LiteralPath 'G:\792\AS5045_enc'
dotnet publish .\Commissioning\Commissioning.csproj -c Release -r win-x64 --self-contained false -o .\out\host-dev --nologo


输出：
G:\792\AS5045_enc\out\host-dev\TwoLinkCommissioning.exe

启动命令（编译成功后执行）：

```powershell
& 'G:\792\AS5045_enc\out\host-dev\TwoLinkCommissioning.exe'
```

此命令只发布上位机，不运行完整发布闸门。重新编译前关闭该输出目录中的上位机；拷贝时保留整个 `host-dev` 文件夹。目标电脑需要 **.NET 10 Windows Desktop Runtime（x64）**。

## 构建完整发布包

```powershell
Set-Location -LiteralPath 'G:\792\AS5045_enc'
powershell -ExecutionPolicy Bypass -File .\build-release.ps1
```

此脚本构建固件和上位机，执行上位机自检、固件 C 离线测试、引脚核对及参数一致性检查；全部通过后更新固定目录 `out\latest`，并保留 `out\<RELEASE_VERSION>` 版本目录。

正式上位机路径：`G:\792\AS5045_enc\out\latest\host\TwoLinkCommissioning.exe`。版本与检查结果见 `G:\792\AS5045_enc\out\latest\BUILD_REPORT.txt`；发布时保留完整的 `host` 文件夹。

完整构建还需要 PSoC Creator 4.4、Visual Studio C++ 工具和 Python。Creator 默认路径为 `D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe`；安装位置不同时，给上述脚本命令追加 `-Creator '实际的 cyprjmgr.exe 绝对路径'`。重新构建前关闭对应输出目录中的上位机。

完整构建不会烧录固件或连接串口；离线检查通过不代表实机验收通过。
