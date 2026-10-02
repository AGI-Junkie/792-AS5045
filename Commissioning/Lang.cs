using System.Globalization;
using System.IO;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text;
using System.Windows;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

/// <summary>One-click zh/en UI strings. Each key holds both languages so the pair cannot drift apart.</summary>
internal static class Lang
{
    public static bool IsEn { get; private set; }
    public static event Action? Changed;
    internal static bool PersistChanges = true;
    private static readonly List<(WeakReference<FrameworkElement> Element, DependencyProperty Property, string Key)> bindings = [];
    internal static TElement Bind<TElement>(TElement element, DependencyProperty property, string key) where TElement : FrameworkElement
    {
        element.SetCurrentValue(property, T(key));
        bindings.Add((new(element), property, key));
        return element;
    }
    // Retain the formatter, not yesterday's rendered status/error. No global control subscriptions.
    private static readonly DependencyProperty TextSourceProperty = DependencyProperty.RegisterAttached(
        "TextSource", typeof(Func<string>), typeof(Lang));
    internal static void Text(FrameworkElement element, Func<string> source)
    {
        element.SetValue(TextSourceProperty, source);
        RefreshText(element);
    }
    internal static void RefreshText(FrameworkElement element)
    {
        if (element.GetValue(TextSourceProperty) is not Func<string> source) return;
        var property = element is TextBox ? TextBox.TextProperty : TextBlock.TextProperty;
        string text = source();
        if (!Equals(element.GetValue(property), text)) element.SetValue(property, text);
    }
    internal sealed class ValidationException(string key, object[] args) : ArgumentException
    {
        public string Key { get; } = key;
        public override string Message => T(Key, args.Select(arg => arg is Func<string> text ? text() : arg).ToArray());
    }
    internal static ArgumentException Error(string key, params object[] args) => new ValidationException(key, args);

    private static string SettingsPath => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "settings.json");

    public static string T(string key) => Lookup(key);
    public static string T(string key, params object[] args) =>
        string.Format(CultureInfo.InvariantCulture, Lookup(key), args);

    private static string Lookup(string key) =>
        Table.TryGetValue(key, out var pair) ? (IsEn ? pair.En : pair.Zh) : throw new KeyNotFoundException("Missing UI translation: " + key);
    public static void Load(string? path = null)
    {
        try
        {
            path ??= SettingsPath;
            var saved = File.Exists(path) ? JsonSerializer.Deserialize<Settings>(File.ReadAllText(path)) : null;
            Set(saved?.Language == "en", persist: false);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException) { Set(false, persist: false); }
    }

    public static void Toggle() => Set(!IsEn);

    public static void Set(bool english, bool persist = true)
    {
        if (IsEn == english) return;
        IsEn = english;
        if (persist && PersistChanges) Save(SettingsPath);
        // Set only translated properties. ResourceDictionary changes also invalidate unrelated text-editor styles.
        for (int i = bindings.Count - 1; i >= 0; i--) {
            var binding = bindings[i];
            if (!binding.Element.TryGetTarget(out var element)) { bindings.RemoveAt(i); continue; }
            string value = T(binding.Key);
            if (!Equals(element.GetValue(binding.Property), value)) element.SetCurrentValue(binding.Property, value);
        }
        Changed?.Invoke();
    }

    private static void Save(string path)
    {
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            JsonObject saved;
            try { saved = File.Exists(path) ? JsonNode.Parse(File.ReadAllText(path)) as JsonObject ?? new() : new(); }
            catch (JsonException) { File.Copy(path, path + ".bak", true); saved = new(); }
            saved["Language"] = IsEn ? "en" : "zh";
            File.WriteAllText(path + ".tmp", saved.ToJsonString(), new UTF8Encoding(false));
            File.Move(path + ".tmp", path, true);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { }
    }

    private sealed class Settings
    {
        public string? Language { get; set; }
    }

    internal static void EnsureTableIntegrity()
    {
        foreach (var (key, pair) in Table)
        {
            if (string.IsNullOrWhiteSpace(key) || string.IsNullOrWhiteSpace(pair.Zh) || string.IsNullOrWhiteSpace(pair.En))
                throw new InvalidOperationException($"Lang entry '{key}' must have both zh and en text.");
            if (CompositeFormat.Parse(pair.Zh).MinimumArgumentCount != CompositeFormat.Parse(pair.En).MinimumArgumentCount)
                throw new InvalidOperationException("Translation arguments differ: " + key);
        }
    }

    /// <summary>Self-test: switching language must rewrite known UI strings.</summary>
    internal static void SelfTest()
    {
        EnsureTableIntegrity();
        bool start = IsEn;
        try
        {
            Set(false, persist: false);
            if (T("btn.connect") != "连接") throw new InvalidOperationException("zh string missing");
            Set(true, persist: false);
            if (T("btn.connect") != "Connect") throw new InvalidOperationException("en string missing");
            string zhTitle = Table["app.title"].Zh, enTitle = Table["app.title"].En;
            if (zhTitle == enTitle) throw new InvalidOperationException("app.title must differ between languages");
            int toggles = 0;
            void Count() => toggles++;
            Changed += Count;
            try { Set(false, false); Set(true, false); Set(false, false); Set(false, false); }
            finally { Changed -= Count; }
            if (toggles != 3) throw new InvalidOperationException("Lang.Changed must fire on every effective toggle");
            string temp = Path.Combine(Path.GetTempPath(), "TwoLinkLanguage-" + Guid.NewGuid());
            Directory.CreateDirectory(temp);
            try {
                string path = Path.Combine(temp, "settings.json");
                File.WriteAllText(path, "{\"OtherSetting\":42}");
                Set(true, false); Save(path); Set(false, false); Load(path);
                if (!IsEn || JsonNode.Parse(File.ReadAllText(path))?["OtherSetting"]?.GetValue<int>() != 42)
                    throw new InvalidOperationException("Language must persist without losing other settings");
                File.WriteAllText(path, "invalid JSON"); Load(path);
                if (IsEn) throw new InvalidOperationException("Corrupt preferences must fall back to Chinese");
                Set(true, false); Save(path); Load(path);
                if (!IsEn || File.ReadAllText(path + ".bak") != "invalid JSON")
                    throw new InvalidOperationException("Corrupt preferences must recover with a backup");
                Load(Path.Combine(temp, "missing.json"));
                if (IsEn) throw new InvalidOperationException("First launch must default to Chinese");
            } finally { Directory.Delete(temp, recursive: true); }
        }
        finally { Set(start, persist: false); }
    }

    private static readonly Dictionary<string, (string Zh, string En)> Table = new()
    {
        // window / chrome
        ["app.title"] = ("Two-Link Studio · 轨迹工作台 · v7 全局运动参数", "Two-Link Studio · Trajectory Workbench · v7 Global Motion"),
        ["lang.toggle"] = ("EN", "中"),
        ["lang.toggle.tip"] = ("Switch to English（切换到英文）", "切换到中文（Switch to Chinese）"),

        // toolbar
        ["btn.stop"] = ("■  总停止   Space / Esc", "■  STOP ALL   Space / Esc"),
        ["btn.refreshPorts"] = ("刷新", "Refresh"),
        ["btn.connect"] = ("连接", "Connect"),
        ["btn.disconnect"] = ("断开", "Disconnect"),
        ["btn.enable"] = ("使能 / 重新检查 (C)", "Enable / Recheck (C)"),
        ["btn.home"] = ("开始回零 (R)", "Start Homing (R)"),
        ["btn.manualRef"] = ("人工确认当前位置为伸直零位", "Manual Zero: Links Straight"),
        ["btn.manualRef.tip"] = ("先人工把两杆沿 +X 摆直，再确认 q1=q2=0；只建立坐标，不驱动电机。",
            "Straighten both links along +X by hand, then confirm q1=q2=0. Establishes coordinates only; does not drive motors."),
        ["btn.readInputs"] = ("读取限位 / 编码器", "Read Limits / Encoders"),
        ["btn.openData"] = ("打开实验数据目录", "Open Experiment Folder"),
        ["btn.sample"] = ("载入示例", "Load Example"),
        ["btn.sample.tip"] = ("载入当前图形的小示例（起点约 340, 0 mm，在默认关节范围内）；只改图形参数。",
            "Load a small sample of the current shape (start near 340, 0 mm, inside default joint range). Changes shape fields only."),
        ["btn.preview"] = ("检查并预览", "Check & Preview"),
        ["btn.save"] = ("保存", "Save"),
        ["btn.export"] = ("导出规划 CSV", "Export Plan CSV"),
        ["btn.movePoint"] = ("低速移到此 XY 点", "Slow Move to XY"),
        ["btn.moveToStart"] = ("移到起点", "Move to Start"),
        ["btn.upload"] = ("上传图形", "Upload Shape"),
        ["btn.run"] = ("执行图形", "Run Shape"),
        ["btn.play"] = ("▶ 预览", "▶ Preview"),
        ["btn.playPause"] = ("Ⅱ 暂停预览", "Ⅱ Pause Preview"),
        ["btn.rewind"] = ("↺ 重播", "↺ Rewind"),
        ["btn.box"] = ("框选", "Box Zoom"),
        ["btn.boxActive"] = ("框选中…", "Boxing…"),

        // tabs / sections
        ["tab.feedback"] = ("实时反馈与实验", "Live Feedback & Experiments"),
        ["tab.log"] = ("原始日志", "Raw Log"),
        ["tab.log.tip"] = ("日志可选中文字，用 Ctrl+C 复制。", "Select text in the log and copy with Ctrl+C."),
        ["diag.header"] = ("实时反馈与实验 / 原始日志", "Live Feedback & Experiments / Raw Log"),
        ["section.properties"] = ("属性", "Properties"),
        ["section.joints"] = ("关节状态", "Joint Status"),
        ["section.inputs"] = ("输入 / 状态", "Inputs / Status"),
        ["section.jog"] = ("键盘控制", "Keyboard Control"),
        ["section.geometry"] = ("图形与几何", "Shape & Geometry"),
        ["section.transform"] = ("图形变换", "Shape Transform"),
        ["section.dimensions"] = ("机构尺寸", "Mechanism Size"),
        ["section.limits"] = ("关节范围", "Joint Range"),
        ["section.speeds"] = ("运动参数（全局）", "Motion Parameters (Global)"),
        ["section.checks"] = ("查看规划检查详情", "Planning Check Details"),

        // status bar
        ["status.ref"] = ("坐标参考：", "Ref: "),
        ["status.ref.on"] = ("已建立", "Established"),
        ["status.ref.off"] = ("未建立", "Not set"),
        ["status.range"] = ("关节范围：", "Range: "),
        ["status.range.on"] = ("已同步", "Synced"),
        ["status.range.off"] = ("未同步", "Not synced"),
        ["badge.offline"] = ("未连接 · 离线预览", "Offline · Preview"),
        ["badge.v7"] = ("在线 · v7 校验控制", "Online · v7 framed control"),
        ["badge.v6"] = ("在线 · v6 硬件串口", "Online · v6 hardware UART"),
        ["badge.v5"] = ("在线 · v5 兼容慢速串口", "Online · v5 slow UART"),
        ["badge.old"] = ("在线 · 旧固件，建议更新", "Online · Old firmware, update advised"),

        // joint / led labels
        ["joint.q1cmd"] = ("q1 命令", "q1 cmd"),
        ["joint.q1enc"] = ("q1 编码器", "q1 encoder"),
        ["joint.q1err"] = ("q1 误差", "q1 error"),
        ["joint.q2cmd"] = ("q2 命令", "q2 cmd"),
        ["joint.x"] = ("末端 X（估算）", "Tip X (est)"),
        ["joint.y"] = ("末端 Y（估算）", "Tip Y (est)"),
        ["joint.drops"] = ("累计丢包", "Drops"),
        ["led.enable"] = ("使能", "Enable"),
        ["led.ref"] = ("坐标参考", "Ref"),
        ["led.limits"] = ("全局限位", "Limit"),
        ["led.arm1"] = ("arm1 传感器", "arm1 sensor"),
        ["led.arm2"] = ("arm2 NPN", "arm2 NPN"),
        ["led.fault"] = ("驱动故障", "Drive fault"),
        ["lbl.telemRate"] = ("遥测速率", "Telemetry rate"),
        ["telem.status"] = ("仅状态块（约 2 Hz）", "Status block only (~2 Hz)"),
        ["telem.hz5"] = ("5 Hz", "5 Hz"),
        ["telem.hz10"] = ("10 Hz", "10 Hz"),
        ["telem.hz20"] = ("20 Hz", "20 Hz"),
        ["snap.phase.home"] = ("回零", "homing"),
        ["snap.phase.ptp"] = ("点位移动", "point move"),
        ["snap.phase.traj_run"] = ("图形执行", "drawing"),
        ["snap.phase.traj_upload"] = ("上传图形", "uploading a plan"),
        ["snap.phase.jog"] = ("点动", "jogging"),
        ["snap.phase.idle"] = ("使能空闲", "enabled, idle"),
        ["snap.phase.disabled"] = ("未使能", "disabled"),
        ["snap.phase.other"] = ("其他", "other"),
        ["snap.noRef"] = ("无参考", "no reference"),
        ["snap.summary"] = ("停机快照（固件在停机瞬间记录）：阶段 {0}；指令角 q1/q2 {1}；arm1 编码器误差 {2}；输入掩码 {3}；心跳年龄 {4} ms（最长间隔 {5} ms）。",
            "Stop snapshot (recorded by the firmware as it stopped): phase {0}; commanded q1/q2 {1}; arm1 encoder error {2}; input mask {3}; heartbeat age {4} ms (longest gap {5} ms)."),
        ["snap.drvUndetermined"] = ("驱动器 nFAULT 有效：原因待诊断。nFAULT 可由过流、过热或欠压触发，引脚不能区分，不能据此认定是过热。",
            "Driver nFAULT was active: cause not yet diagnosed. nFAULT can come from over-current, over-temperature or under-voltage and the pin does not say which, so it cannot be taken as overheating."),
        ["snap.drvOk"] = ("驱动器 nFAULT 未触发。", "Driver nFAULT was not active."),
        ["err.telemRate"] = ("遥测速率须是 0（仅状态块）或 5～20 Hz，且不带角度。", "The telemetry rate must be 0 (status block only) or 5-20 Hz, with no angles."),
        ["lbl.encoder"] = ("编码器", "Encoder"),
        ["lbl.driveFault"] = ("驱动故障", "Drive fault"),
        ["lbl.ref"] = ("坐标参考", "Reference"),
        ["lbl.colon"] = ("{0}：{1}", "{0}: {1}"),

        // default live texts
        ["live.waiting"] = ("等待 stage2-v5 实时反馈。", "Waiting for stage2-v5 live feedback."),
        ["live.noRefYet"] = ("坐标参考尚未建立；角度与误差不作有效读数。",
            "Reference not established; angle/error readings not valid yet."),
        ["live.experimentHint"] = ("每次单点 / 图形实验自动保存计划、反馈 CSV 和结果摘要。",
            "Each point / shape experiment auto-saves the plan, feedback CSV and summary."),
        ["live.none"] = ("—", "—"),

        // notes
        ["note.jogManual"] = ("可人工摆直确认，也可开关回零。人工确认时两杆必须已伸直。",
            "Confirm straight by hand, or switch-homing. Both links must already be straight for manual confirm."),
        ["note.jogStop"] = ("点动松开或切换窗口即停；回零和轨迹可在后台继续。",
            "Jog stops on release or when the window loses focus; homing and trajectories may continue."),
        ["note.splitter"] = ("拖动调整栏宽", "Drag to resize columns"),
        ["note.chart"] = ("约 2 Hz；arm2 无编码器，XY 为估算。", "~2 Hz; arm2 has no encoder; XY is estimated."),
        ["chart.title"] = ("arm1 编码器 − 命令角度 / °", "arm1 encoder − command angle / °"),

        // connection / device state
        ["state.disconnected"] = ("未连接", "Not connected"),
        ["state.inputsEmpty"] = ("输入尚未读取", "Inputs not read yet"),
        ["inputs.raw"] = ("输入电平（0＝触发）：", "Input levels (0 = active): "),
        ["state.needPort"] = ("请选择 KitProg 的 COM 口", "Select the KitProg COM port first"),
        ["state.logSaved"] = ("日志自动保存：", "Log auto-saved: "),
        ["state.connected"] = ("已连接，等待 stage1-v3 / stage2 固件状态；不会自动使能",
            "Connected, waiting for stage1-v3 / stage2 firmware state; will not auto-enable"),
        ["state.connectFail"] = ("连接失败：", "Connect failed: "),
        ["state.fwTimeout"] = ("固件状态超时 / 版本不匹配；已请求停止，请检查设备",
            "Firmware state timeout / version mismatch; stop requested, check the device"),
        ["state.enableReq"] = ("使能已请求，等待固件检查结果", "Enable requested, waiting for firmware checks"),
        ["state.homeReq"] = ("回零已请求，等待固件报告；Space / Esc 停止",
            "Homing requested, waiting for firmware; Space / Esc to stop"),
        ["state.manualRef"] = ("正在确认人工伸直零位；此指令不驱动电机。",
            "Confirming manual straight zero; this command does not drive motors."),
        ["state.uploadStart"] = ("正在启动图形执行；等待设备确认", "Starting shape run; waiting for device ACK"),
        ["state.uploading"] = ("轨迹上传：{0} / {1} 点；等待确认，上传时不运动",
            "Uploading trajectory: {0} / {1} points; waiting ACK, no motion while uploading"),
        ["state.uploadedRan"] = ("图形已执行完（{0} 点仍在设备中）；回到起点、落笔后可再点“执行图形”",
            "Shape finished ({0} points still on device); return to start, pen down, then Run Shape again"),
        ["state.uploaded"] = ("图形已上传（{0} 点，未运动）；落笔后点“执行图形”",
            "Shape uploaded ({0} points, not moving); pen down then Run Shape"),
        ["state.executing"] = ("图形执行：{0} / {1} 段\narm2 为命令步数；末端位置未实测。",
            "Shape run: {0} / {1} segments\narm2 is commanded steps; tip pose not measured."),
        ["state.stopReq"] = ("停止已请求，等待固件确认", "Stop requested, waiting for firmware ACK"),
        ["state.pointSettle"] = ("XY 单点：停稳并检查到位", "XY point: settling / checking position"),
        ["state.pointMove"] = ("XY 单点：双轴协调移动中", "XY point: two-axis coordinated move"),
        ["state.pointWait"] = ("XY 单点：等待固件接收", "XY point: waiting for firmware"),
        ["state.pointNote"] = ("\narm1 有编码器反馈；arm2 为命令步数，末端位置未实测。",
            "\narm1 has encoder feedback; arm2 is commanded steps; tip pose not measured."),
        ["state.enabled"] = ("已使能", "Enabled"),
        ["state.motionOff"] = ("运动禁用", "Motion disabled"),
        ["state.refEstablished"] = ("已建立", "Established"),
        ["state.refMissing"] = ("未建立", "Not set"),
        ["state.encOk"] = ("可用", "OK"),
        ["state.encBad"] = ("不可用", "Unavailable"),
        ["state.faultOn"] = ("有（锁定）", "Yes (locked)"),
        ["state.faultOff"] = ("无", "None"),
        ["state.selectPort"] = ("请选择 KitProg 的 COM 口", "Select the KitProg COM port first"),

        // homing phases
        ["phase.0"] = ("待机", "Idle"),
        ["phase.1"] = ("释放开关", "Release switch"),
        ["phase.2"] = ("额外退开", "Extra back-off"),
        ["phase.3"] = ("快速触碰", "Fast touch"),
        ["phase.4"] = ("慢速触碰", "Slow touch"),
        ["phase.5"] = ("确认零点", "Confirm zero"),
        ["phase.6"] = ("移到伸直位（轴号见日志）", "Move to straight (axis in log)"),
        ["phase.7"] = ("确认伸直位置", "Confirm straight position"),

        // enable/home pending tokens used in status (keep tokens stable for logic)
        ["ctl.enable"] = ("使能", "Enable"),
        ["ctl.home"] = ("回零", "Home"),
        ["ctl.unknown"] = ("未知", "Unknown"),

        // faults / lastStop
        ["fault.badPtp"] = ("运动状态回执格式无效，已取消请求；检查通信与固件版本。",
            "Invalid motion-state reply; request cancelled. Check link and firmware version."),
        ["fault.epochChanged"] = ("等待指令期间坐标参考已改变；已取消请求，请重新确认零位。",
            "Coordinate reference changed while waiting; request cancelled. Re-confirm zero."),
        ["fault.manualRefAck"] = ("人工零位回执无效；设备未建立可用参考，请重新使能后再确认。",
            "Invalid manual-zero ACK; no usable reference. Enable again and re-confirm."),
        ["fault.notAtStart"] = ("尚未确认到达这条轨迹的起点。请先低速移到起点，等待到位反馈，再执行。",
            "Not yet at this trajectory's start. Slow-move to the start, wait for in-position feedback, then run."),
        ["fault.seqExhausted"] = ("运动序号用尽，请重启设备。", "Motion sequence exhausted; restart the device."),
        ["fault.seqExhaustedHome"] = ("运动序号用尽，请重启设备后回零。", "Motion sequence exhausted; restart and home."),
        ["fault.seqExhaustedRef"] = ("PTP 序号用尽，请重启设备后重新建立坐标参考。",
            "PTP sequence exhausted; restart and re-establish coordinates."),
        ["fault.pointTimeout"] = ("设备未确认指令 {0}（已发送 {1} 次）。已停止；请检查串口和丢包日志，再重新使能。",
            "Device did not ACK command {0} (sent {1} times). Stopped; check serial/drop log, then enable again."),
        ["fault.controlTimeout"] = ("{0}未收到确认。已停止；检查串口和输入状态后重新使能。",
            "{0}: no ACK. Stopped; check serial and inputs, then enable again."),
        ["fault.telemetryStale"] = ("反馈已过期；等待新的设备数据。", "Feedback expired; waiting for new device data."),
        ["fault.badTelemetry"] = ("设备反馈无效或参考代次不匹配；已停止，请重新建立坐标。",
            "Invalid feedback or reference epoch mismatch; stopped. Re-establish coordinates."),
        ["fault.saveSummary"] = ("已请求停止；结果摘要保存失败：", "Stop requested; failed to save summary: "),
        ["fault.saveSummary2"] = ("实验摘要保存失败：", "Failed to save experiment summary: "),
        ["fault.ioClosed"] = ("串口已关闭", "Serial port closed"),
        ["fault.opFail"] = ("操作失败：", "Operation failed: "),
        ["fault.opFailSuffix"] = ("；请确认下位机已停止", "; confirm the device has stopped"),
        ["fault.unplug"] = ("已断开；请重新连接设备", "Disconnected; reconnect the device"),
        ["fault.windowClose"] = ("窗口关闭", "Window closed"),
        ["fault.unplugReason"] = ("设备已断开；当前位置反馈无效。", "Device disconnected; position feedback invalid."),
        ["fault.inputsInvalid"] = ("输入状态已失效，需重新连接读取", "Input status stale; reconnect to read again"),
        ["fault.paramFormat"] = ("参数文件格式不兼容或字段无效。", "Incompatible parameter file or invalid fields."),
        ["fault.paramTooBig"] = ("参数文件过大。", "Parameter file too large."),
        ["fault.paramSave"] = ("保存参数失败：", "Failed to save parameters: "),
        ["fault.paramRestore"] = ("恢复参数失败：", "Failed to restore parameters: "),
        ["fault.openFolder"] = ("无法打开目录：", "Cannot open folder: "),
        ["fault.experimentMissingFeedback"] = ("缺少有效的当前位置反馈；请等待读数后重试。",
            "Missing valid position feedback; wait for a reading and retry."),

        // ExplainFault
        ["explain.uart"] = ("串口接收出错或缓冲溢出，已停止并作废坐标参考。检查 USB 连接及通信诊断，再使能并确认零位。",
            "UART receive error/overflow: stopped and reference invalidated. Check USB and diagnostics, enable and re-zero."),
        ["explain.range"] = ("目标超出软件范围：缩小图形或调整位置。", "Target outside software range: shrink the shape or move it."),
        ["explain.start"] = ("起点不符：先低速移到当前轨迹起点。", "Start mismatch: slow-move to the current trajectory start."),
        ["explain.encoder"] = ("编码器或跟随误差异常：检查负载、驱动与编码器，再使能并建立零位。",
            "Encoder/following error: check load, drive and encoder, enable and re-zero."),
        ["explain.limit"] = ("限位 / 传感器触发：检查实际位置及输入，再重新使能。",
            "Limit/sensor triggered: check position and inputs, then enable again."),
        ["explain.heartbeat"] = ("心跳超时：保持窗口响应，检查串口后重新使能。",
            "Heartbeat timeout: keep the window responsive, check serial, enable again."),
        ["explain.stale"] = ("坐标参考或序号失效：重新使能并人工确认零位，也可开关回零。",
            "Stale reference/sequence: enable and manual-zero, or switch-homing."),
        ["explain.operatorStop"] = ("已停止。继续前重新使能，再人工确认零位或回零。",
            "Stopped. Enable again, then manual-zero or home before continuing."),
        ["explain.fault"] = ("驱动故障：检查驱动供电 / 故障输入，再重新使能。",
            "Drive fault: check drive power / fault input, then enable again."),
        ["explain.other"] = ("请求已停止或被拒绝；检查以下设备原因，解决后再操作。",
            "Request stopped or refused; fix the device reason below, then continue."),

        // UpdateControls reasons
        ["reason.needStage2"] = ("图形执行需要 stage2-v2 固件；当前只可预览。",
            "Shape execution needs stage2-v2 firmware; preview only."),
        ["reason.lockedFw"] = ("当前板上仍是锁定固件；请烧录 stage2-manual-reference 版本，再人工确认零位或回零。",
            "Board still runs locked firmware; flash stage2-manual-reference, then manual-zero or home."),
        ["reason.busy"] = ("运动指令发送 / 执行中；总停止按钮可中止。",
            "Motion command in flight / running; STOP ALL can abort."),
        ["reason.waitStop"] = ("等待停止确认；随后重新使能，再人工确认零位或回零。",
            "Waiting for stop ACK; then enable, manual-zero or home."),
        ["reason.notEnabled"] = ("未使能：请点“使能 / 重新检查”，再人工确认零位或回零。",
            "Not enabled: click Enable / Recheck, then manual-zero or home."),
        ["reason.homing"] = ("正在回零：等待两轴完成并报告 HOME OK。",
            "Homing: waiting for both axes and HOME OK."),
        ["reason.needManualRef"] = ("坐标参考未建立：把两杆摆直，点击工具栏“人工确认当前位置为伸直零位”；也可选择开关回零。",
            "No reference: straighten links and click Manual Zero in the toolbar, or switch-home."),
        ["reason.needHome"] = ("坐标参考未建立：当前固件需开关回零；人工确认功能需要 stage2-v3。",
            "No reference: this firmware needs switch-homing; manual zero needs stage2-v3."),
        ["reason.notReady"] = ("驱动、编码器或方向状态未就绪，请查看状态和停止原因。",
            "Drive/encoder/direction not ready; check status and stop reason."),
        ["reason.badRef"] = ("XY 参考无效：重新摆直并人工确认零位，或执行开关回零。",
            "XY reference invalid: straighten and manual-zero, or switch-home."),
        ["reason.stalePtp"] = ("XY 固件状态超过 2 秒未更新，请检查通信。",
            "XY firmware state stale (>2 s); check the link."),
        ["reason.waitTelem"] = ("等待 v5 当前角度反馈；超过 2 秒请检查通信。",
            "Waiting for v5 angle feedback; if >2 s check the link."),
        ["reason.inputsHit"] = ("传感器或驱动输入触发；请查看右侧输入 / 状态。",
            "Sensor/drive input active; see Inputs / Status on the right."),
        ["reason.keysDown"] = ("请松开所有点动按键。", "Release all jog keys."),
        ["reason.readyTrajectory"] = ("设备已就绪：目标须通过下方范围检查，再移到轨迹起点，上传图形并落笔后执行。固件范围为临时软件限制。",
            "Device ready: target must pass the range check, move to start, upload shape, pen down, then run. Firmware range is a temporary software limit."),
        ["reason.pointOnly"] = ("当前固件仅支持单点移动；图形执行需要 stage2-v2 固件。",
            "Firmware supports single moves only; shape run needs stage2-v2."),
        ["reason.needRef"] = ("单点执行：等待有效坐标参考、编码器反馈和新鲜固件状态。",
            "Point move: waiting for valid reference, encoder feedback and fresh firmware state."),

        // why-not on canvas
        ["why.homing"] = ("回零中，设备位置未知", "Homing; device pose unknown"),
        ["why.noRef"] = ("设备坐标参考未建立，位置未知", "No device reference; pose unknown"),
        ["why.waitFeed"] = ("等待设备反馈", "Waiting for device feedback"),

        // status messages (SetDeviceStatus)
        ["msg.stopSent"] = ("停止请求已发送，等待设备确认", "Stop sent; waiting for device ACK"),
        ["msg.uploadRun"] = ("正在启动图形执行，等待设备确认", "Starting shape run; waiting for device ACK"),
        ["msg.uploadPoint"] = ("上传 {0} / {1} 点，上传期间不运动", "Uploading {0} / {1} points, no motion while uploading"),
        ["msg.drawing"] = ("正在绘制 {0} / {1} 段", "Drawing {0} / {1} segments"),
        ["msg.manualRef"] = ("正在确认人工零位；等待设备确认，不运动", "Confirming manual zero; waiting for ACK, no motion"),
        ["msg.ctl"] = ("正在{0}，等待设备确认", "{0} in progress; waiting for device ACK"),
        ["msg.homing"] = ("正在开关回零", "Switch-homing"),
        ["msg.slowMove"] = ("正在低速移动 / 到位检查", "Slow move / in-position check"),
        ["msg.connectFirst"] = ("连接设备；离线可预览", "Connect the device; offline preview available"),
        ["msg.pointOnly"] = ("当前固件仅支持点动，完整绘图需 v5", "Firmware jog-only; full drawing needs v5"),
        ["msg.enableBtn"] = ("请点工具栏“使能 / 重新检查”", "Click Enable / Recheck in the toolbar"),
        ["msg.homeOrRef"] = ("请点工具栏确认人工零位或回零", "Confirm manual zero or home from the toolbar"),
        ["msg.eta"] = (" · 预计剩余 {0:F0} 秒", " · ~{0:F0} s left"),

        // experiment results
        ["exp.failMark"] = ("失败", "fail"),
        ["exp.savedWaiting"] = ("已保存 {0} 条反馈；等待运动样本，暂不计算误差统计。",
            "Saved {0} feedback rows; waiting for motion samples before error stats."),
        ["exp.live"] = ("本次采样 {0} · arm1 RMSE {1:F4}° · 峰值 {2:F4}°\n数据正在自动保存。",
            "Samples {0} · arm1 RMSE {1:F4}° · peak {2:F4}°\nAuto-saving data."),
        ["exp.finished"] = ("实验{0} · {1} 个样本 · ", "Experiment {0} · {1} samples · "),
        ["exp.finishedPrefix.complete"] = ("实验完成", "Experiment complete"),
        ["exp.finishedPrefix.aborted"] = ("实验中止", "Experiment aborted"),
        ["exp.complete"] = ("完成", "complete"),
        ["exp.aborted"] = ("中止", "aborted"),
        ["exp.rmse"] = ("RMSE {0:F4}°\n", "RMSE {0:F4}°\n"),
        ["exp.noMotion"] = ("无有效运动样本\n", "No valid motion samples\n"),
        ["exp.savedNote"] = ("已保存计划、反馈和摘要。arm2 / 末端误差未实测。",
            "Plan, feedback and summary saved. arm2 / tip error not measured."),

        // XYPreviewPanel
        ["xy.branchPos"] = ("肘角 ≥ 0 分支", "Elbow ≥ 0"),
        ["xy.branchNeg"] = ("肘角 ≤ 0 分支", "Elbow ≤ 0"),
        ["xy.shape0"] = ("单点", "Point"),
        ["xy.shape1"] = ("直线", "Line"),
        ["xy.shape2"] = ("矩形", "Rectangle"),
        ["xy.shape3"] = ("圆", "Circle"),
        ["xy.shape4"] = ("椭圆", "Ellipse"),
        ["xy.shape5"] = ("圆弧", "Arc"),
        ["xy.shape6"] = ("圆角矩形", "Rounded rectangle"),
        ["xy.shape7"] = ("“8”字 / 利萨如", "Figure 8 / Lissajous"),
        ["xy.shape8"] = ("正多边形", "Regular polygon"),
        ["xy.shape9"] = ("螺旋", "Spiral"),
        ["xy.shape10"] = ("自定义轮廓", "Custom contour"),
        ["xy.shape"] = ("形状", "Shape"),
        ["xy.branch"] = ("逆解分支", "IK branch"),
        ["xy.l1"] = ("L1", "L1"),
        ["xy.l2"] = ("L2", "L2"),
        ["xy.q1min"] = ("q1 最小", "q1 min"),
        ["xy.q1max"] = ("q1 最大", "q1 max"),
        ["xy.q2min"] = ("q2 最小", "q2 min"),
        ["xy.q2max"] = ("q2 最大", "q2 max"),
        ["xy.elbowMin"] = ("肘 最小", "Elbow min"),
        ["xy.elbowMax"] = ("肘 最大", "Elbow max"),
        ["xy.motor1v"] = ("电机1 速度", "Motor1 speed"),
        ["xy.motor2v"] = ("电机2 速度", "Motor2 speed"),
        ["xy.motor1a"] = ("电机1 加速度", "Motor1 accel"),
        ["xy.motor2a"] = ("电机2 加速度", "Motor2 accel"),
        ["xy.motor1v.name"] = ("电机1 °/s", "Motor1 °/s"),
        ["xy.motor2v.name"] = ("电机2 (q2−q1) °/s", "Motor2 (q2−q1) °/s"),
        ["xy.motor1a.name"] = ("电机1 °/s²", "Motor1 °/s²"),
        ["xy.motor2a.name"] = ("电机2 (q2−q1) °/s²", "Motor2 (q2−q1) °/s²"),
        ["xy.motor2.tip"] = ("电机2 转的是 q2−q1", "Motor2 turns q2−q1"),
        ["xy.feed"] = ("末端最大", "Tip max"),
        ["xy.feed.name"] = ("末端最大 mm/s（只读）", "Tip max mm/s (read-only)"),
        ["xy.feed.tip"] = ("由杆长和两个电机速度算出的末端速度上限：机械臂伸直、两轴都以最高速度同向转动时。只读；实际绘图更慢。",
            "Tip speed ceiling from link lengths and motor speeds (arm straight, both axes max same direction). Read-only; drawing is slower."),
        ["xy.unit.mm"] = ("mm", "mm"),
        ["xy.unit.deg"] = ("°", "°"),
        ["xy.unit.degs"] = ("°/s", "°/s"),
        ["xy.unit.degs2"] = ("°/s²", "°/s²"),
        ["xy.unit.mms"] = ("mm/s", "mm/s"),
        ["xy.help.name"] = ("关节范围说明", "Joint range help"),
        ["xy.speedHelp.name"] = ("运动参数说明", "Motion parameters help"),
        ["xy.automation.field"] = ("{0} / {1}", "{0} / {1}"),
        ["xy.rangeField.tip"] = ("{0} / °；留空使用 {1}°，填写后使用你的值。",
            "{0} / °; empty uses {1}°, otherwise your value."),
        ["xy.speedsGlobal"] = ("全局：杆长用于全部图形、命令 XY 与记录；关节速度/加速度用于绘图、定位、点动和回零。\n上限：电机1 ≤{0:0.##} °/s、≤{1:0.##} °/s²；电机2 (q2−q1) ≤{2:0.##} °/s、≤{3:0.##} °/s²。\n回零：快碰和走到伸直位按这里的速度和加速度；释放、退开（固定 600 微步/秒）和慢碰（固定 400 微步/秒）不跟随，这里的速度更低时才取这里的。\n到设备：速度和加速度改动后自动同步，已使能也行（运动、回零、上传中要等结束）；已上传还没执行的图形会作废，要重新上传。关节范围只在未使能时同步，已使能时改范围要先总停止。需要固件声明 rates_live=1。",
            "Global: link lengths for all shapes, commanded XY and logging; joint speed/accel for drawing, positioning, jog and homing.\nCaps: Motor1 ≤{0:0.##} °/s, ≤{1:0.##} °/s²; Motor2 (q2−q1) ≤{2:0.##} °/s, ≤{3:0.##} °/s².\nHoming: the fast touch and the move to straight use these speeds and accelerations; release and backoff (fixed 600 usteps/s) and the slow touch (fixed 400 usteps/s) do not follow, and only take these values if they are lower.\nTo the device: speed and accel changes sync automatically, even while enabled (a move, homing or upload has to finish first); a plan already uploaded but not run is dropped and must be uploaded again. Joint range syncs only while disabled; if you change it while enabled, STOP ALL first. Needs firmware advertising rates_live=1."),
        ["xy.effective"] = ("实际生效：电机1/电机2 {0:F2}/{1:F2} °/s，加速度 {2:F2}/{3:F2} °/s²。填得比上限大会被压到上限。",
            "Effective: motor1/motor2 {0:F2}/{1:F2} °/s, accel {2:F2}/{3:F2} °/s². Values above the cap are clamped."),

        // range help
        ["xy.rangeHelp1"] = ("关节范围 / °：q1、q2 两个关节角的规划范围（两轴逆时针为正，0° 为两杆伸直的零位）。\n· q1 ＝ arm1：主动臂（经 40:1 减速箱、带 AS5045 编码器反馈）。\n· q2 ＝ arm2 相对固定 +X 零方向的夹角；第二轴电机实际转角为 q2−q1（无编码器反馈）。\n· 肘 ＝ q2−q1 ＝ 第二轴电机自己的转角（伸直为 0，向左为正），有单独的“肘 最小 / 最大”：预览、规划、单点移动和固件执行都按它检查，不再只靠 q2 范围。请按实测的机械行程填写；默认 ±155° 只是暂定值。\n· 不确定哪个是哪个：按住点动 \"arm1 ← / D →\" 看状态栏，或轻推对应杆件，读数变化的那一路就是它。\n· 每个输入框留空时使用对应默认值：q1 ∈ [−30°, +30°]，q2 ∈ [−60°, +60°]，肘 ∈ [−155°, +155°]。\n· 填写后以你填的值为准：",
            "Joint range / °: planning range for q1 and q2 (CCW positive; 0° = links straight).\n· q1 = arm1: driven arm (40:1 gearbox, AS5045 encoder feedback).\n· q2 = arm2 angle vs fixed +X; the second motor actually turns q2−q1 (no encoder).\n· Elbow = q2−q1 = the second motor's own angle (0 straight, positive left). It has its own \"Elbow min / max\": preview, planning, point moves and firmware execution all check it, not just the q2 range. Enter the measured mechanical stroke; the ±155° default is only provisional.\n· Not sure which is which: hold jog \"arm1 ← / D →\" and watch the status bar, or nudge a link.\n· Empty fields use defaults: q1 ∈ [−30°, +30°], q2 ∈ [−60°, +60°], elbow ∈ [−155°, +155°].\n· After you fill values, yours win: "),
        ["xy.rangeHelp2"] = ("\n· 透明红色表示在此范围和连杆长度下，两个逆解分支都无法到达的区域；修改后实时更新。\n· 红区、轨迹规划和设备执行共用这组范围。未使能时自动同步到支持范围设置的固件，确认一致后才允许运动。\n· q1、q2 和肘三个范围都须在 −180°～180° 内并包含 0°，最多两位小数。修改已使能设备的范围后，先总停止再同步（肘范围同样）。\n· 这是软件关节范围，不是实测碰撞边界。",
            "\n· Translucent red = unreachable by both IK branches at this range and link length; updates live.\n· Red region, planning and device execution share this range. When disabled it syncs to range-capable firmware; motion allowed only after match.\n· The q1, q2 and elbow ranges must each lie within −180°…180° and contain 0°, at most 2 decimals. After editing a live device, STOP ALL then sync (the elbow range too).\n· This is a software joint range, not a measured collision bound."),
        ["xy.envelopeInvalid"] = ("当前输入无效，请检查上下限", "Invalid input; check min/max"),
        ["xy.envelopeEmpty"] = ("未填写", "Not set"),
        ["xy.envelopeFormat"] = ("q1 [{0:F2}, {1:F2}]°、q2 [{2:F2}, {3:F2}]°、肘 [{4:F2}, {5:F2}]°", "q1 [{0:F2}, {1:F2}]°, q2 [{2:F2}, {3:F2}]°, elbow [{4:F2}, {5:F2}]°"),
        ["xy.rangeInvalid"] = ("范围无效：须包含 0°、上下限有序，最多两位小数。",
            "Invalid range: must contain 0°, min < max, ≤2 decimals."),
        ["xy.rangePreviewOnly"] = ("红区与规划使用上方范围；设备确认后才能运动。",
            "Red region and planning use the range above; motion waits for device ACK."),
        ["xy.rangeMatched"] = ("设备已确认：红区、规划、执行范围一致。",
            "Device confirmed: red region, planning and execution ranges match."),
        ["xy.rangeUnsynced"] = ("设备范围尚未同步；运动已锁定。\n", "Device range not synced; motion locked.\n"),
        ["xy.deviceRange"] = ("设备当前：", "Device now: "),

        // shape labels
        ["xy.x"] = ("X / mm", "X / mm"),
        ["xy.y"] = ("Y / mm", "Y / mm"),
        ["xy.centerX"] = ("中心 X / mm", "Center X / mm"),
        ["xy.centerY"] = ("中心 Y / mm", "Center Y / mm"),
        ["xy.originX"] = ("原点 X / mm", "Origin X / mm"),
        ["xy.originY"] = ("原点 Y / mm", "Origin Y / mm"),
        ["xy.cornerX"] = ("左下角 X / mm", "Corner X / mm"),
        ["xy.cornerY"] = ("左下角 Y / mm", "Corner Y / mm"),
        ["xy.startX"] = ("起点 X / mm", "Start X / mm"),
        ["xy.startY"] = ("起点 Y / mm", "Start Y / mm"),
        ["xy.radius"] = ("半径 / mm", "Radius / mm"),
        ["xy.width"] = ("宽 / mm", "Width / mm"),
        ["xy.height"] = ("高 / mm", "Height / mm"),
        ["xy.endX"] = ("终点 X / mm", "End X / mm"),
        ["xy.endY"] = ("终点 Y / mm", "End Y / mm"),
        ["xy.semiX"] = ("X 半轴 / mm", "X semi-axis / mm"),
        ["xy.semiY"] = ("Y 半轴 / mm", "Y semi-axis / mm"),
        ["xy.startAngle"] = ("起始角 / °", "Start angle / °"),
        ["xy.sweep"] = ("扫过角度 / °", "Sweep / °"),
        ["xy.cornerRadius"] = ("圆角半径 / mm", "Corner radius / mm"),
        ["xy.halfWidth"] = ("宽度半幅 / mm", "Half width / mm"),
        ["xy.halfHeight"] = ("高度半幅 / mm", "Half height / mm"),
        ["xy.freq"] = ("Y 向频率比（2＝8字）", "Y frequency (2 = figure 8)"),
        ["xy.circumRadius"] = ("外接圆半径 / mm", "Circumradius / mm"),
        ["xy.sides"] = ("边数（3＝三角形，最多 12）", "Sides (3 = triangle, max 12)"),
        ["xy.startRadius"] = ("起始半径 / mm", "Start radius / mm"),
        ["xy.endRadius"] = ("终止半径 / mm", "End radius / mm"),
        ["xy.turns"] = ("圈数", "Turns"),
        ["xy.rotation"] = ("旋转角（逆时针）", "Rotation (CCW)"),
        ["xy.scale"] = ("缩放", "Scale"),
        ["xy.direction"] = ("方向", "Direction"),
        ["xy.dirCcw"] = ("逆时针", "Counter-clockwise"),
        ["xy.dirCw"] = ("顺时针", "Clockwise"),
        ["xy.customName"] = ("自定义轮廓（点列或 SVG 路径）", "Custom contour (points or SVG path)"),
        ["xy.customHelp"] = ("每行一个点“x y”（mm，相对原点），或粘贴一条 SVG 路径（M L H V C S Q T Z，SVG 的 y 向下，已自动翻转）。只支持一条连续轮廓：机器没有抬笔，多个独立轮廓之间会留下连接线。点列的每个点都是停顿角；曲线只在明显转折处停顿。",
            "One point \"x y\" per line (mm, relative to the origin), or paste ONE SVG path (M L H V C S Q T Z; SVG's y points down and is flipped). Only a single contour is supported: there is no pen lift, so separate contours would be joined by a line. Every listed point is a dwell corner; curves only dwell at sharp turns."),

        // validation messages
        ["err.number"] = ("{0}：请输入有效数值，小数点使用 .，不能留空或输入 NaN/Infinity。",
            "{0}: enter a valid number (use '.'), no empty / NaN / Infinity."),
        ["err.limitDecimals"] = ("关节范围最多保留两位小数（0.01°）。", "Joint range allows at most 2 decimals (0.01°)."),
        ["err.limitBounds"] = ("关节范围（q1、q2、肘）须位于 -180°～180° 内、包含 0°，且最小值小于最大值；留空项使用默认值。",
            "Each joint range (q1, q2, elbow) must lie within -180°…180°, contain 0°, and min < max; empty fields use defaults."),
        ["err.needPreview"] = ("参数已修改，请点击“检查并预览”或“保存”后应用。",
            "Parameters changed; click Check & Preview or Save to apply."),

        // plan notices
        ["notice.noRange"] = ("关节范围未测定；机械范围未检查。CSV 为规划数据，不能直接驱动机械臂。",
            "Joint range not set; mechanical range unchecked. CSV is planning data only; do not drive the arm from it."),
        ["notice.checked"] = ("已检查全部采样点的关节范围；未验证机械碰撞、线缆干涉或从当前位置到起点的路径。",
            "All samples checked against the joint range; mechanical collision, cable interference and approach path are not verified."),
        ["notice.pointNoRange"] = ("两轴左转＝逆时针已确认；此处机械范围未填写。实际单点移动另受固件标定锁和固件范围限制。",
            "Left = CCW confirmed; mechanical range not filled. Actual moves also limited by firmware calibration lock and firmware range."),
        ["notice.pointChecked"] = ("目标按当前关节范围检查，设备确认同一范围后才允许运动；尚未验证机械碰撞或线缆干涉。",
            "Target checked against the current joint range; motion needs the device to confirm the same range. Collision/cable not verified."),
        ["plan.trajOk"] = ("整条轨迹检查通过：使用当前关节范围", "Trajectory check passed: using the current joint range"),
        ["plan.trajStats"] = ("{0} 点；路径长度 {1:F2} mm；预计 {2:F2} 秒。\nq1：{3:F2}～{4:F2}°；q2：{5:F2}～{6:F2}°；肘：{7:F2}～{8:F2}°。\n预览与执行共用设备上限内的时间规划；执行另含每段末尾 50 ms 停留。",
            "{0} points; path {1:F2} mm; ~{2:F2} s.\nq1: {3:F2}…{4:F2}°; q2: {5:F2}…{6:F2}°; elbow: {7:F2}…{8:F2}°.\nPreview and run share the in-limit timing; run adds 50 ms at each segment end."),
        ["plan.nearSing"] = ("\n提示：部分姿态接近奇异位形，微小 XY 变化可能引起较大关节变化。",
            "\nNote: some poses near singularity; small XY changes may cause large joint motion."),
        ["plan.target"] = ("目标 ({0:F2}, {1:F2}) mm；几何半径 {2:F2}～{3:F2} mm\n",
            "Target ({0:F2}, {1:F2}) mm; geometric radius {2:F2}…{3:F2} mm\n"),
        ["plan.solution"] = ("解 {0}：q1={1:F2}°，q2={2:F2}°，肘={3:F2}°；", "Solution {0}: q1={1:F2}°, q2={2:F2}°, elbow={3:F2}°; "),
        ["plan.inRange"] = ("在填写的范围内", "inside the entered range"),
        ["plan.outRange"] = ("超出填写的范围", "outside the entered range"),
        ["plan.noRangeCheck"] = ("机械范围未检查", "mechanical range unchecked"),
        ["plan.singular"] = ("\n当前是伸直或完全折叠的奇异位形；不能据此认为连续轨迹可执行。",
            "\nSingular (straight or fully folded); do not treat a continuous trajectory as executable."),
        ["plan.execInfo"] = ("低速执行计划 · {0} 点 · {1:F2} 秒 · 每轴 ≤{2} 微步/秒",
            "Slow-run plan · {0} points · {1:F2} s · ≤{2} µsteps/s per axis"),
        ["plan.execFail"] = ("执行检查未通过：", "Execution check failed: "),

        // move / action status
        ["move.default"] = ("低速实机试运行：连接后人工确认伸直零位，或执行开关回零。",
            "Slow live trial: after connect, manual-zero straight or switch-home."),
        ["move.waitConnect"] = ("等待连接并建立人工或开关坐标参考。",
            "Waiting to connect and establish manual or switch-homing reference."),
        ["move.currentRange"] = ("\n当前软件范围：", "\nCurrent software range: "),
        ["move.locked"] = ("\n设备尚未确认当前范围/速度，运动已锁定；预览仍按输入设置显示。",
            "\nDevice has not confirmed range/speed; motion locked. Preview still follows your settings."),
        ["move.inFw"] = ("目标在固件范围内", "Target inside firmware range"),
        ["move.outFw"] = ("目标超出固件范围，不能移动", "Target outside firmware range; cannot move"),
        ["move.outFwStart"] = ("轨迹起点超出固件范围，不能移动", "Trajectory start outside firmware range; cannot move"),
        ["move.fwBounds"] = ("\n{0}：q1={1:F2}°，q2={2:F2}°，肘={7:F2}°；固件允许 q1 [{3:F2}, {4:F2}]°、q2 [{5:F2}, {6:F2}]°、肘 [{8:F2}, {9:F2}]°。",
            "\n{0}: q1={1:F2}°, q2={2:F2}°, elbow={7:F2}°; firmware allows q1 [{3:F2}, {4:F2}]°, q2 [{5:F2}, {6:F2}]°, elbow [{8:F2}, {9:F2}]°."),
        ["move.planFail"] = ("\n参数检查未通过：", "\nPlan check failed: "),
        ["move.ran"] = ("\n图形已执行完，仍在设备中；回到起点、落笔后可再次“执行图形”。改动参数则需重新上传。",
            "\nShape finished and still on device; return to start, pen down, Run Shape again. Parameter edits need re-upload."),
        ["move.uploaded"] = ("\n图形已上传；手动落笔后点“执行图形”。", "\nShape uploaded; pen down then Run Shape."),
        ["move.atStart"] = ("\n已到当前轨迹起点；点“上传图形”（上传时不运动），落笔后再点“执行图形”。",
            "\nAt the trajectory start; click Upload Shape (no motion while uploading), pen down, then Run Shape."),
        ["move.needStart"] = ("\n图形执行：先移到起点并等待到位反馈（需 v5 固件）。",
            "\nShape run: move to start first and wait for in-position feedback (needs v5 firmware)."),
        ["move.otherPlan"] = ("\n设备里已有另一份图形：重新上传，或先总停止。",
            "\nDevice already holds another shape: re-upload, or STOP ALL first."),
        ["move.needCheck"] = ("先完成图形参数检查。", "Finish the shape parameter check first."),
        ["move.uploadTip"] = ("\n流程：准备 → 抬笔移到起点 → 上传图形 → 落笔 → 执行图形 → 保存结果。",
            "\nFlow: prepare → pen up, move to start → upload shape → pen down → run shape → save results."),

        // action summary
        ["act.rejected"] = ("被拒绝 · ", "Rejected · "),
        ["act.waiting"] = ("等待中 · ", "Waiting · "),
        ["act.ready"] = ("可执行 · ", "Ready · "),
        ["act.readyRerun"] = ("可再次执行 · ", "Ready to rerun · "),
        ["act.canUpload"] = ("可上传 · ", "Can upload · "),
        ["act.canMove"] = ("可移动 · ", "Can move · "),
        ["act.opFail"] = ("操作失败 · ", "Failed · "),
        ["act.paramsDirty"] = ("等待中 · 参数已修改，请检查或保存后应用", "Waiting · Parameters changed; check or save to apply"),
        ["act.outOfRange"] = ("被拒绝 · 目标超出范围，调整图形参数", "Rejected · Target out of range; adjust shape parameters"),
        ["act.unsynced"] = ("等待中 · 设备尚未确认当前范围/速度", "Waiting · Device has not confirmed range/speed"),
        ["act.trajFail"] = ("被拒绝 · 整条轨迹未通过检查", "Rejected · Trajectory check failed"),
        ["act.planOnDevice"] = ("等待中 · 设备里的图形与当前参数不一致，请重新上传",
            "Waiting · Device shape differs from current parameters; re-upload"),
        ["act.uploadReady"] = ("可上传 · 已到起点；上传时不运动", "Can upload · At start; no motion while uploading"),
        ["act.pointReady"] = ("可移动 · 目标检查通过", "Can move · Target check passed"),
        ["act.noTraj"] = ("等待中 · 当前固件不支持图形执行", "Waiting · Firmware does not support shape run"),
        ["act.needPen"] = ("等待中 · 请先抬笔，点“移到起点”", "Waiting · Pen up first, then Move to Start"),
        ["act.runAgain"] = ("可再次执行 · 图形仍在设备中，落笔后点“执行图形”",
            "Ready to rerun · Shape still on device; pen down then Run Shape"),
        ["act.runNow"] = ("可执行 · 图形已上传，请落笔后点“执行图形”",
            "Ready · Shape uploaded; pen down then Run Shape"),
        ["act.details"] = ("\n完整参数见左侧“查看规划检查详情”。", "\nFull parameters under Planning Check Details on the left."),

        // viewport
        ["vp.liveLayer"] = ("设备姿态", "Device pose"),
        ["vp.liveLayer.tip"] = ("运动期间，两根连杆跟随设备回报的命令步数。arm2 和末端位置为模型估算，非实测；离线播放使用规划姿态。",
            "While moving, links follow commanded steps from the device. arm2 and tip are model estimates; offline playback uses the planned pose."),
        ["vp.unreachable"] = ("不可达区域", "Unreachable"),
        ["vp.box.tip"] = ("点击后在画布上拖出矩形，放大选区；仅改变视图。", "Drag a rectangle on the canvas to zoom the selection; view only."),
        ["vp.zoomOut"] = ("缩小视图；双击画布回到全臂视图。", "Zoom out; double-click canvas for the full-arm view."),
        ["vp.zoomIn"] = ("放大视图；也可滚轮缩放。", "Zoom in; mouse wheel also zooms."),
        ["vp.unreachable.tip"] = ("透明红色＝按当前关节范围与连杆长度算出的不可达区域。\n留空使用默认 q1 [−30°, +30°]、q2 [−60°, +60°]；填写后按你的范围实时更新。\n红区、规划与设备执行使用同一范围；设备确认同步后才允许运动。红色边界是栅格近似，鼠标位置使用精确逆解检查。",
            "Translucent red = unreachable at the current joint range and link length.\nEmpty uses default q1 [−30°, +30°], q2 [−60°, +60°]; filled values update live.\nRed region, planning and execution share this range; motion after device sync. Red edge is a grid approximation; pointer uses exact IK."),
        ["vp.hint"] = ("滚轮缩放 · 拖动平移 · 双击复位 · 单位 mm", "Wheel zoom · drag pan · double-click reset · mm"),
        ["vp.hintRed"] = ("滚轮缩放 · 拖动平移 · 双击复位 · 单位 mm · 红色＝按当前关节范围不可达",
            "Wheel zoom · drag pan · double-click reset · mm · red = unreachable at current range"),
        ["vp.reach"] = (" · 可达", " · reachable"),
        ["vp.unreach"] = (" · 不可达（红色区域）", " · unreachable (red)"),
        ["vp.canvasName"] = ("轨迹视图：滚轮缩放，拖动平移，框选按钮放大局部；不控制机械臂运动。",
            "Trajectory view: wheel zoom, drag pan, box-zoom. Does not move the arm."),
        ["vp.mouse"] = ("鼠标 X {0:F2}  Y {1:F2} mm", "Mouse X {0:F2}  Y {1:F2} mm"),
        ["vp.start"] = ("起点", "Start"),
        ["vp.startEnd"] = ("起点 / 收笔点", "Start / pen-up point"),
        ["vp.end"] = ("终点", "End"),
        ["vp.cornerPause"] = ("角 {0} · 停顿", "Corner {0} · pause"),
        ["vp.ghostPlan"] = ("规划姿态", "Planned pose"),
        ["vp.ghostDevice"] = ("设备当前", "Device now"),
        ["vp.legend1"] = ("━━ arm1", "━━ arm1"),
        ["vp.legend2"] = ("━━ arm2", "━━ arm2"),
        ["vp.legend3"] = ("─ 规划轨迹", "─ planned path"),
        ["vp.legend4"] = ("● 末端", "● tip"),
        ["vp.srcDevice"] = ("设备命令姿态 · 非实测", "Device command pose · not measured"),
        ["vp.srcOfflineFollow"] = ("离线预览 · 未跟随设备（点“检查并预览”回到设备姿态）",
            "Offline preview · not following device (Check & Preview returns to device pose)"),
        ["vp.srcOfflineLayer"] = ("离线规划姿态 · 已关闭“设备姿态”", "Offline planned pose · Device pose layer off"),
        ["vp.srcOffline"] = ("离线规划姿态", "Offline planned pose"),
        ["vp.srcEstXY"] = ("＋ 命令估算 XY（非实测）", "＋ commanded XY estimate (not measured)"),
        ["vp.mini"] = ("轨迹局部 · ", "Trajectory detail · "),
        ["vp.sampleDevice"] = ("设备命令姿态（非实测） · ", "Device command pose (not measured) · "),
        ["vp.sampleOffline"] = ("离线规划目标 · ", "Offline planned target · "),
        ["vp.samplePose"] = ("q1={0:F2}° · q2={1:F2}° · X={2:F2}，Y={3:F2} mm",
            "q1={0:F2}° · q2={1:F2}° · X={2:F2}, Y={3:F2} mm"),
        ["vp.sampleElbow"] = (" · 肘={0:F2}°（距下限 {1:F1}° / 距上限 {2:F1}°）", " · elbow={0:F2}° (to min {1:F1}° / to max {2:F1}°)"),
        ["vp.sampleElbowOut"] = ("【已超出范围】", " [OUT OF RANGE]"),
        ["vp.sampleTooltipDevice"] = ("按设备命令步数和连杆长度估算；arm2 / 末端没有实测反馈。",
            "Estimated from commanded steps and link lengths; arm2 / tip not measured."),
        ["vp.sampleTooltipOffline"] = ("仅屏幕预览，不代表执行进度。路程 {0:F2} mm；末端速度 {1:F2} mm/s。",
            "Screen preview only; not execution progress. Path {0:F2} mm; tip speed {1:F2} mm/s."),
        ["vp.offlineTime"] = ("离线预览 · 时间 {0:F2}/{1:F2} s · ", "Offline preview · time {0:F2}/{1:F2} s · "),
        ["vp.savedPlan"] = ("已保存规划数据：", "Plan data saved: "),
        ["vp.saveFail"] = ("保存失败：", "Save failed: "),
        ["vp.saveDialog"] = ("保存规划轨迹（非实测数据）", "Save planned trajectory (not measured)"),
        ["vp.csvFilter"] = ("CSV 文件 (*.csv)|*.csv", "CSV files (*.csv)|*.csv"),

        // link diagnostics tooltip
        ["link.diag"] = ("通信诊断（累计计数，重启设备清零）：\n", "Link diagnostics (cumulative, cleared on device reset):\n"),

        // joint-limit sync status
        ["limits.protoMismatch"] = ("固件协议版本不匹配（固件 proto={0}，上位机需要 proto={1}）：请更新固件，并使用同一交付里的固件与上位机；运动已锁定。",
            "Firmware protocol mismatch (device proto={0}, host needs proto={1}): update firmware and use the matched host; motion locked."),
        ["limits.invalidProfile"] = ("机构尺寸或全局速度无效，请检查设置。", "Invalid mechanism size or global speeds; check settings."),
        ["limits.noRates"] = ("固件没有声明全局速度支持（motion_rates=1）；请更新固件后重新连接。",
            "Firmware does not advertise motion_rates=1; update firmware and reconnect."),
        ["limits.invalidRange"] = ("关节范围输入无效；请填写 ±180° 内、包含零位的有效范围（最多两位小数）。",
            "Invalid joint range: enter values within ±180° containing 0° (max 2 decimals)."),
        ["limits.awaitDevice"] = ("等待设备确认关节范围；确认范围前不能使能或运动。",
            "Waiting for device to confirm joint range; no enable/motion until confirmed."),
        ["limits.mismatch"] = ("关节范围或全局速度与设备不一致：先总停止，禁用后自动同步，再重新使能。",
            "Range/speed differs from device: STOP ALL, let it sync while disabled, then enable again."),
        ["limits.rangeMismatch"] = ("关节范围与设备不一致：先总停止，禁用后自动同步，再重新使能。速度和加速度不用停。",
            "Joint range differs from device: STOP ALL, let it sync while disabled, then enable again. Speed and accel need no stop."),
        ["limits.liveRates"] = ("速度/加速度与设备不一致，正在自动同步；确认前运动已锁定，运动、回零或上传中要等它结束。",
            "Speed/accel differ from the device; syncing automatically. Motion is locked until confirmed, and waits for any move, homing or upload to finish."),
        ["limits.fwMismatch"] = ("当前固件的关节范围与输入不一致；请更新支持范围同步的固件。",
            "Firmware joint range differs from input; update to range-sync firmware."),
        ["limits.syncing"] = ("正在同步关节范围与全局速度，等待设备确认；确认前不能使能或运动。",
            "Syncing joint range and global speeds; waiting for device ACK; no enable/motion yet."),
        ["limits.syncTimeout"] = ("范围/速度同步未获完整确认（设备可能正在运动或回零）；请稍后改一下设置重试，或先总停止，必要时重新连接。",
            "Range/speed sync not fully confirmed (the device may be moving or homing); change a setting to retry, or STOP ALL first, and reconnect if it persists."),

        // domain validation messages that surface in the UI
        ["err.linkLength"] = ("连杆长度须为 0～10000 mm 之间的正数。", "Link lengths must be positive within 0…10000 mm."),
        ["err.angleFinite"] = ("角度须为有限数值。", "Angles must be finite."),
        ["err.targetFinite"] = ("目标坐标须为 ±20000 mm 内的有限数值。", "Target must be finite within ±20000 mm."),
        ["err.targetUnreachable"] = ("目标几何不可达：距原点 {0:F2} mm，可达半径 {1:F2}～{2:F2} mm。",
            "Target geometrically unreachable: {0:F2} mm from origin, reachable radius {1:F2}…{2:F2} mm."),
        ["err.equalFold"] = ("等长连杆在原点完全折叠时解不唯一，请选择其他目标。",
            "Equal links fully folded at the origin have no unique IK; pick another target."),
        ["err.shapeRange"] = ("图形参数须为 ±20000 内的有限数值，分支须有效。",
            "Shape parameters must be finite within ±20000; branch must be valid."),
        ["err.jointRange"] = ("关节范围无效。", "Invalid joint range."),
        ["err.tooManyPoints"] = ("轨迹过长：最多 5000 个预览点，请缩小图形。",
            "Trajectory too long: at most 5000 preview points; shrink the shape."),
        ["err.lineDegenerate"] = ("直线起点与终点不能重合。", "Line start and end must not coincide."),
        ["err.rectSize"] = ("矩形宽、高至少为 0.001 mm。", "Rectangle width/height must be at least 0.001 mm."),
        ["err.circleRadius"] = ("圆半径至少为 0.001 mm。", "Circle radius must be at least 0.001 mm."),
        ["err.shapeScale"] = ("缩放须在 0.01～100 之间。", "Scale must be between 0.01 and 100."),
        ["err.ellipseSize"] = ("椭圆两个半轴都至少为 0.001 mm。", "Both ellipse semi-axes must be at least 0.001 mm."),
        ["err.arcRadius"] = ("圆弧半径至少为 0.001 mm。", "Arc radius must be at least 0.001 mm."),
        ["err.arcSweep"] = ("圆弧扫过角度须大于 0°、不超过 360°。", "Arc sweep must be above 0° and at most 360°."),
        ["err.roundRadius"] = ("圆角半径须在 0.001 mm 与较短边一半之间。", "Corner radius must be between 0.001 mm and half the shorter side."),
        ["err.figureSize"] = ("“8”字的宽、高半幅至少为 0.001 mm。", "Figure-8 half width/height must be at least 0.001 mm."),
        ["err.figureRatio"] = ("Y 向频率比须是 2～6 的整数（2 是“8”字）。", "The y frequency must be a whole number from 2 to 6 (2 is a figure 8)."),
        ["err.polygonSize"] = ("外接圆半径至少为 0.001 mm。", "Circumradius must be at least 0.001 mm."),
        ["err.polygonSides"] = ("边数须是 3～12 的整数。", "Sides must be a whole number from 3 to 12."),
        ["err.spiralRadius"] = ("螺旋的起止半径不能为负，且至少一个大于 0.001 mm。", "Spiral radii cannot be negative and at least one must exceed 0.001 mm."),
        ["err.spiralTurns"] = ("螺旋圈数须大于 0、不超过 20。", "Spiral turns must be above 0 and at most 20."),
        ["err.customEmpty"] = ("自定义轮廓至少需要两个不重合的点。", "A custom contour needs at least two distinct points."),
        ["err.customPointLine"] = ("自定义点列第 {0} 行不是“x y”两个数。", "Custom point list: line {0} is not two numbers \"x y\"."),
        ["err.customNumber"] = ("自定义轮廓里的数值无效或超出 ±20000：{0}", "Invalid or out-of-range (±20000) number in the custom contour: {0}"),
        ["err.customSyntax"] = ("自定义轮廓无法解析：请检查 SVG 路径语法（须以 M 开头，命令后跟足够的数）。", "The custom contour cannot be parsed: check the SVG path syntax (start with M, enough numbers after each command)."),
        ["err.customContours"] = ("自定义轮廓只能有一条连续路径：机器没有抬笔，多个独立轮廓之间会留下连接线。", "A custom contour must be one continuous path: there is no pen lift, so separate contours would be joined by a line."),
        ["err.customCommand"] = ("自定义轮廓不支持 SVG 命令 {0}（支持 M L H V C S Q T Z）。", "SVG command {0} is not supported in a custom contour (supported: M L H V C S Q T Z)."),
        ["err.circleTooLong"] = ("圆周过长：最多 5000 个预览点，请缩小半径。",
            "Circle too long: at most 5000 preview points; reduce the radius."),
        ["err.geomUnreachable"] = ("轨迹经过几何不可达区域；请移动或缩小图形。",
            "Trajectory crosses a geometrically unreachable region; move or shrink the shape."),
        ["err.singularPath"] = ("轨迹触及完全伸直或折叠的奇异位形；请留出余量。",
            "Trajectory touches a fully straight/folded singularity; leave margin."),
        ["err.sampleOutOfRange"] = ("第 {0} 个采样点超出规划范围：q1={1:F2}°，q2={2:F2}°，肘={7:F2}°；允许 q1 [{3:F2},{4:F2}]°、q2 [{5:F2},{6:F2}]°、肘 [{8:F2},{9:F2}]°。红区、规划与设备执行共用当前关节范围，请调整图形或关节范围。",
            "Sample {0} outside planning range: q1={1:F2}°, q2={2:F2}°, elbow={7:F2}°; allowed q1 [{3:F2},{4:F2}]°, q2 [{5:F2},{6:F2}]°, elbow [{8:F2},{9:F2}]°. Red region, planning and execution share this range; adjust the shape or range."),
        ["err.betweenSamples"] = ("图形在采样点之间超出当前关节范围：q1={0:F6}°，q2={1:F6}°，肘={2:F6}°；请调整图形或关节范围。",
            "Shape leaves the current joint range between samples: q1={0:F6}°, q2={1:F6}°, elbow={2:F6}°; adjust the shape or range."),
        ["err.execCapacity"] = ("执行轨迹超出 {0} 点容量或时间间隔范围；请缩短路径。",
            "Execution trajectory exceeds {0} nodes or dt limits; shorten the path."),
        ["err.chord"] = ("执行轨迹的弦线近似误差超过 0.15 mm；请降低速度或调整图形。",
            "Chord approximation error exceeds 0.15 mm; lower speed or adjust the shape."),
        ["err.execNodes"] = ("执行轨迹超过 {0} 点容量。", "Execution trajectory exceeds {0} nodes."),
        ["err.execHead"] = ("执行轨迹容量或首点无效。", "Invalid execution capacity or first node."),
        ["err.execRange"] = ("第 {0} 点超出当前关节范围：q1={1:F2}°，q2={2:F2}°，肘={7:F2}°；允许 q1 [{3:F2},{4:F2}]°、q2 [{5:F2},{6:F2}]°、肘 [{8:F2},{9:F2}]°。请调整图形或关节范围。",
            "Point {0} outside current joint range: q1={1:F2}°, q2={2:F2}°, elbow={7:F2}°; allowed q1 [{3:F2},{4:F2}]°, q2 [{5:F2},{6:F2}]°, elbow [{8:F2},{9:F2}]°. Adjust the shape or range."),
        ["err.execTime"] = ("执行时间无效。", "Invalid execution timing."),
        ["err.execPulse"] = ("执行轨迹超过脉冲速度或相邻段速度变化上限；请降低速度。",
            "Execution exceeds pulse-rate or segment speed-change limits; lower the speed."),
        ["err.speedRange"] = ("速度与加速度须为 0～1000 内的有限正数。", "Speed/accel must be finite positive within 0…1000."),
        ["err.belowMin"] = ("arm{0} 设置低于设备支持的下限 {1:F3}（速度 °/s 或加速度 °/s²）。",
            "arm{0} is below the device minimum {1:F3} (speed °/s or accel °/s²)."),
        ["err.planTooLong"] = ("满足设备上限后的规划超过 {0} 秒，请缩短路径或调整参数。",
            "Planning under device caps exceeds {0} s; shorten the path or adjust parameters."),
        ["err.planRates"] = ("规划速度与加速度须为 0.01～1000 内的有限正数（不是驱动实测能力）。",
            "Plan speed/accel must be finite positive within 0.01…1000 (not a measured drive limit)."),
        ["err.planSlow"] = ("离线规划超过 {0} 秒或接近数值奇异点；请调整图形或规划参数。",
            "Offline plan exceeds {0} s or near a numeric singularity; adjust the shape or plan parameters."),
        ["err.planRange"] = ("时间规划采样点超出填写的关节范围，请调整图形或分支。",
            "Timing samples leave the entered joint range; adjust the shape or branch."),
        ["err.planSingular"] = ("时间规划经过奇异位形。", "Timing plan passes a singularity."),
        ["err.previewTime"] = ("预览时间须为有限数值。", "Preview time must be finite."),
        ["err.keyMask"] = ("心跳按键掩码无效。", "Invalid heartbeat key mask."),
        ["err.pointPacket"] = ("单点目标/序号/速度无效。", "Invalid point target/sequence/rate."),
        ["err.motionPacket"] = ("运动数据包无效。", "Invalid motion packet."),
        ["err.ctrlPacket"] = ("控制数据包无效。", "Invalid control packet."),
        ["err.limitDecimals2"] = ("关节范围最多保留两位小数。", "Joint range allows at most 2 decimals."),
        ["err.rateConfig"] = ("设备速度配置无效。", "Invalid device rate configuration."),
        ["err.limitContainsZero"] = ("关节范围须包含零位，且下限小于上限。",
            "Joint range must contain zero, with min < max."),
        ["err.seqExhaustedCtrl"] = ("控制序号用尽，请重启设备后重新连接。",
            "Control sequence exhausted; restart the device and reconnect."),
        ["ref.manual"] = ("人工伸直零位", "Manual straight zero"),
        ["ref.home"] = ("开关回零", "Switch homing"),
        ["ref.none"] = ("未建立", "Not set"),
    };
}
