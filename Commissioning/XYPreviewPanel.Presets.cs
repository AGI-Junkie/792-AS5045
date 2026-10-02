using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace TwoLinkCommissioning;

// Named parameter sets, and the small status rows that say where each group of parameters stands:
// saved on this computer, confirmed by the device, used by the plan that is on the device.
public sealed partial class XYPreviewPanel
{
    // ---- named presets ------------------------------------------------------------------------------------------
    // A preset sets the shape and its sizes, the transform, the branch and the four motion rates. It never touches the
    // link lengths or the joint/elbow ranges: those describe the machine, not the drawing, and changing them needs a STOP.
    private static readonly string[] PresetKeys = ["shape", "branch", "x", "y", "a", "b", "c", "rotation", "scale", "direction", "custom", "v1", "v2", "a1", "a2"];
    private static readonly (string NameKey, Dictionary<string, string> Values)[] BuiltInPresets = [
        ("preset.smallCircle", new() { ["shape"] = "3", ["branch"] = "0", ["x"] = "340", ["y"] = "0", ["a"] = "8", ["b"] = "10", ["c"] = "", ["rotation"] = "0", ["scale"] = "1", ["direction"] = "0", ["v1"] = "5", ["v2"] = "3", ["a1"] = "10", ["a2"] = "6" }),
        ("preset.lineTest", new() { ["shape"] = "1", ["branch"] = "0", ["x"] = "335", ["y"] = "0", ["a"] = "350", ["b"] = "0", ["c"] = "", ["rotation"] = "0", ["scale"] = "1", ["direction"] = "0", ["v1"] = "5", ["v2"] = "3", ["a1"] = "10", ["a2"] = "6" }),
        ("preset.repeatability", new() { ["shape"] = "0", ["branch"] = "0", ["x"] = "340", ["y"] = "0", ["a"] = "5", ["b"] = "10", ["c"] = "", ["rotation"] = "0", ["scale"] = "1", ["direction"] = "0", ["v1"] = "3", ["v2"] = "2", ["a1"] = "6", ["a2"] = "4" }),
    ];
    private sealed record UserPreset(string Name, Dictionary<string, string> Values);
    private sealed record PresetFile(int Schema, List<UserPreset> Presets);
    internal static string? PresetsPathOverride;      // tests keep the real file untouched
    private static string PresetsPath => PresetsPathOverride ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "presets.json");
    private const int MaxPresets = 50, MaxPresetName = 40;
    private List<UserPreset> userPresets = [];
    private readonly ComboBox presetBox = new() { MinHeight = 24 };
    private readonly Button presetSave = new() { Margin = new Thickness(2), Padding = new Thickness(10, 2, 10, 2) }, presetDelete = new() { Margin = new Thickness(2), Padding = new Thickness(10, 2, 10, 2) };
    private readonly TextBlock presetNote = new() { TextWrapping = TextWrapping.Wrap, FontSize = 11, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(10, 2, 10, 4) };
    private bool loadingPreset;

    private string FieldText(string key) => key switch {
        "shape" => shape.SelectedIndex.ToString(), "branch" => branch.SelectedIndex.ToString(), "direction" => direction.SelectedIndex.ToString(),
        "x" => x.Text, "y" => y.Text, "a" => a.Text, "b" => b.Text, "c" => c.Text, "rotation" => rotation.Text, "scale" => scale.Text, "custom" => custom.Text,
        "v1" => v1.Text, "v2" => v2.Text, "a1" => acc1.Text, "a2" => acc2.Text, _ => throw new ArgumentException(key) };

    private void ApplyPreset(IReadOnlyDictionary<string, string> values)
    {
        updating = true;
        try
        {
            foreach (var (key, value) in values)
            {
                if (!PresetKeys.Contains(key)) continue;
                switch (key)
                {
                    case "shape": shape.SelectedIndex = int.Parse(value); shownShape = shape.SelectedIndex; break;
                    case "branch": branch.SelectedIndex = int.Parse(value); break;
                    case "direction": direction.SelectedIndex = int.Parse(value); break;
                    case "x": x.Text = value; break; case "y": y.Text = value; break; case "a": a.Text = value; break; case "b": b.Text = value; break;
                    case "c": c.Text = value; break; case "rotation": rotation.Text = value; break; case "scale": scale.Text = value; break; case "custom": custom.Text = value; break;
                    case "v1": v1.Text = value; break; case "v2": v2.Text = value; break; case "a1": acc1.Text = value; break; case "a2": acc2.Text = value; break;
                }
            }
            if (shownShape >= 0) shapeDims[shownShape] = [a.Text, b.Text, c.Text];
        }
        finally { updating = false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }

    private static bool ValidPresetValues(Dictionary<string, string>? values) =>
        values is not null && values.Count > 0 && values.Count <= PresetKeys.Length &&
        values.All(pair => PresetKeys.Contains(pair.Key) && pair.Value is not null && pair.Value.Length <= (pair.Key == "custom" ? 4096 : 32)) &&
        (!values.TryGetValue("shape", out var s) || int.TryParse(s, out int shapeIndex) && shapeIndex is >= 0 and <= 10) &&
        (!values.TryGetValue("branch", out var br) || br is "0" or "1") && (!values.TryGetValue("direction", out var d) || d is "0" or "1");

    private void LoadUserPresets()
    {
        userPresets = [];
        try
        {
            if (!File.Exists(PresetsPath)) return;
            if (new FileInfo(PresetsPath).Length > 600_000) throw Lang.Error("preset.err.file");
            var file = JsonSerializer.Deserialize<PresetFile>(File.ReadAllText(PresetsPath));
            if (file is null || file.Schema != 1 || file.Presets is null || file.Presets.Count > MaxPresets ||
                file.Presets.Any(p => string.IsNullOrWhiteSpace(p.Name) || p.Name.Length > MaxPresetName || !ValidPresetValues(p.Values))) throw Lang.Error("preset.err.file");
            userPresets = file.Presets;
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException or ArgumentException)
        { Lang.Text(presetNote, () => Lang.T("preset.err.read") + ex.Message); }
    }

    private void WriteUserPresets()
    {
        string path = PresetsPath;
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path + ".tmp", JsonSerializer.Serialize(new PresetFile(1, userPresets), new JsonSerializerOptions { WriteIndented = true }), new UTF8Encoding(false));
        File.Move(path + ".tmp", path, true);
    }

    private void RebuildPresetList(string? select = null)
    {
        loadingPreset = true;
        try
        {
            presetBox.Items.Clear();
            presetBox.Items.Add(Lang.Bind(new ComboBoxItem { Tag = "" }, ContentControl.ContentProperty, "preset.none"));
            foreach (var (key, _) in BuiltInPresets) presetBox.Items.Add(Lang.Bind(new ComboBoxItem { Tag = "builtin:" + key }, ContentControl.ContentProperty, key));
            foreach (var preset in userPresets) presetBox.Items.Add(new ComboBoxItem { Content = preset.Name, Tag = "user:" + preset.Name });
            presetBox.SelectedIndex = Math.Max(0, presetBox.Items.Cast<ComboBoxItem>().ToList().FindIndex(i => (string)i.Tag == select));
            presetDelete.IsEnabled = presetBox.SelectedItem is ComboBoxItem { Tag: string tag } && tag.StartsWith("user:", StringComparison.Ordinal);
        }
        finally { loadingPreset = false; }
    }

    // Choosing a preset loads it and runs the check; nothing moves: the move, upload and run buttons stay the only way to do that.
    private void PresetChosen()
    {
        if (loadingPreset || presetBox.SelectedItem is not ComboBoxItem { Tag: string tag }) return;
        presetDelete.IsEnabled = tag.StartsWith("user:", StringComparison.Ordinal);
        if (tag == "") return;
        if (tag.StartsWith("builtin:", StringComparison.Ordinal))
        {
            var preset = BuiltInPresets.First(p => p.NameKey == tag[8..]);
            ApplyPreset(preset.Values); Lang.Text(presetNote, () => Lang.T("preset.loaded", Lang.T(preset.NameKey)));
        }
        else if (userPresets.FirstOrDefault(p => p.Name == tag[5..]) is { } user)
        { ApplyPreset(user.Values); Lang.Text(presetNote, () => Lang.T("preset.loaded", user.Name)); }
    }

    internal void ChoosePresetForSelfTest(string tag)
    {
        int index = presetBox.Items.Cast<ComboBoxItem>().ToList().FindIndex(i => (string)i.Tag == tag);
        if (index < 0) throw new InvalidOperationException("no preset " + tag);
        presetBox.SelectedIndex = index;
    }

    // Keeps what is on screen under a name. Returns false (and says why) for a bad or reserved name.
    internal bool SavePresetNamed(string? name)
    {
        name = name?.Trim() ?? "";
        if (name.Length == 0 || name.Length > MaxPresetName || name.Any(char.IsControl)) { Lang.Text(presetNote, () => Lang.T("preset.err.name", MaxPresetName)); return false; }
        if (BuiltInPresets.Any(p => string.Equals(Lang.T(p.NameKey), name, StringComparison.OrdinalIgnoreCase)) || name == Lang.T("preset.none")) { Lang.Text(presetNote, () => Lang.T("preset.err.builtin")); return false; }
        if (userPresets.Count >= MaxPresets && userPresets.All(p => !string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase))) { Lang.Text(presetNote, () => Lang.T("preset.err.full", MaxPresets)); return false; }
        var values = PresetKeys.ToDictionary(key => key, FieldText);
        if (shape.SelectedIndex != 10) values.Remove("custom");
        userPresets.RemoveAll(p => string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase));
        userPresets.Add(new UserPreset(name, values));
        try { WriteUserPresets(); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { Lang.Text(presetNote, () => Lang.T("preset.err.write") + ex.Message); return false; }
        RebuildPresetList("user:" + name);
        Lang.Text(presetNote, () => Lang.T("preset.saved", name));
        return true;
    }

    internal bool DeleteSelectedPreset()
    {
        if (presetBox.SelectedItem is not ComboBoxItem { Tag: string tag } || !tag.StartsWith("user:", StringComparison.Ordinal)) return false;
        string name = tag[5..];
        userPresets.RemoveAll(p => p.Name == name);
        try { WriteUserPresets(); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { Lang.Text(presetNote, () => Lang.T("preset.err.write") + ex.Message); }
        RebuildPresetList(); Lang.Text(presetNote, () => Lang.T("preset.deleted", name));
        return true;
    }

    private string? AskPresetName()
    {
        var box = new TextBox { MinWidth = 260, Margin = new Thickness(0, 6, 0, 6), MaxLength = MaxPresetName };
        var ok = new Button { Content = Lang.T("btn.ok"), IsDefault = true, MinWidth = 70, Margin = new Thickness(4) };
        var cancel = new Button { Content = Lang.T("btn.cancel"), IsCancel = true, MinWidth = 70, Margin = new Thickness(4) };
        var dialog = new Window { Title = Lang.T("preset.saveTitle"), SizeToContent = SizeToContent.WidthAndHeight, ResizeMode = ResizeMode.NoResize, ShowInTaskbar = false,
            WindowStartupLocation = WindowStartupLocation.CenterOwner, Owner = Window.GetWindow(this), Background = WorkbenchTheme.Bar };
        ok.Click += (_, _) => dialog.DialogResult = true;
        dialog.Content = new StackPanel { Margin = new Thickness(14), Children = { new TextBlock { Text = Lang.T("preset.namePrompt", MaxPresetName) }, box,
            new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Children = { ok, cancel } } } };
        dialog.Loaded += (_, _) => box.Focus();
        return dialog.ShowDialog() == true ? box.Text : null;
    }

    private UIElement BuildPresetControls()
    {
        var label = WorkbenchTheme.LocalLabel("preset.label");
        System.Windows.Automation.AutomationProperties.SetName(presetBox, Lang.T("preset.label"));
        presetSave.Content = Lang.T("preset.save"); presetDelete.Content = Lang.T("preset.delete"); presetDelete.IsEnabled = false;
        presetSave.Click += (_, _) => { if (AskPresetName() is { } name) SavePresetNamed(name); };
        presetDelete.Click += (_, _) => DeleteSelectedPreset();
        presetBox.SelectionChanged += (_, _) => PresetChosen();
        Lang.Text(presetNote, () => Lang.T("preset.hint"));
        LoadUserPresets(); RebuildPresetList();   // a damaged file replaces the hint with its message
        editControls.Add(presetBox); editControls.Add(presetSave);   // deleting a preset changes no parameter, so it stays available
        var buttons = new WrapPanel { Margin = new Thickness(8, 0, 8, 0), Children = { presetSave, presetDelete } };
        return new StackPanel { Children = { WorkbenchTheme.Row(label, presetBox), buttons, presetNote } };
    }

    private void RefreshPresetLanguage()
    {
        presetSave.Content = Lang.T("preset.save"); presetDelete.Content = Lang.T("preset.delete");
        Lang.RefreshText(presetNote);
    }

    // ---- where each group of parameters stands ----------------------------------------------------------------------
    // Positions in the snapshot: the 21 text fields in ParameterFields order, then shape, branch, direction.
    private static readonly int[] ShapeGroup = [2, 3, 4, 5, 17, 18, 19, 20, 21, 22, 23], SizeGroup = [0, 1], RangeGroup = [6, 7, 8, 9, 15, 16], SpeedGroup = [11, 12, 13, 14];
    private string[]? savedFields;                       // what was last written to, or read from, the parameter file
    private bool rangeConfirmed = true, speedConfirmed = true;
    private bool planHeld;                               // the plan on the device is the one on screen
    private readonly List<(int[] Group, bool HasDevice, TextBlock[] Lamps)> lampRows = [];

    private string[] SnapshotFields() => ParameterFields.Select(f => f.Text).Concat([shape.SelectedIndex.ToString(), branch.SelectedIndex.ToString(), direction.SelectedIndex.ToString()]).ToArray();
    private void RememberSaved() { savedFields = SnapshotFields(); UpdateLamps(); }

    // The device's own report of the joint ranges and of the speeds, each against what is typed here.
    internal void SetParameterConfirmation(bool ranges, bool speeds)
    {
        if (rangeConfirmed == ranges && speedConfirmed == speeds) return;
        rangeConfirmed = ranges; speedConfirmed = speeds; UpdateLamps();
    }

    private Border BuildLamps(int[] group, bool hasDevice)
    {
        var lamps = Enumerable.Range(0, hasDevice ? 3 : 2).Select(_ => new TextBlock { FontSize = 11, Margin = new Thickness(0, 0, 10, 0) }).ToArray();
        lampRows.Add((group, hasDevice, lamps));
        var row = new WrapPanel { Margin = new Thickness(10, 3, 10, 3) };
        foreach (var lamp in lamps) row.Children.Add(lamp);
        return new Border { Background = Brushes.White, BorderBrush = WorkbenchTheme.Faint, BorderThickness = new Thickness(0, 0, 0, 1), Child = row };
    }

    private static readonly Brush LampOn = WorkbenchTheme.Ok, LampOff = WorkbenchTheme.Brush("#B45309"), LampNeutral = WorkbenchTheme.Idle;
    private void UpdateLamps()
    {
        if (lampRows.Count == 0) return;
        var now = SnapshotFields();
        foreach (var (group, hasDevice, lamps) in lampRows)
        {
            bool saved = savedFields is { } sf && group.All(i => sf[i] == now[i]);
            bool? used = uploadedPlan is null ? null : planHeld;
            bool deviceOk = ReferenceEquals(group, RangeGroup) ? rangeConfirmed : speedConfirmed;
            void Lamp(int index, string text, string tip, Brush colour)
            {
                var lamp = lamps[index];
                if (lamp.Text != text) lamp.Text = text;
                lamp.Foreground = colour;
                if (!Equals(lamp.ToolTip, tip)) lamp.ToolTip = tip;
            }
            Lamp(0, saved ? Lang.T("lamp.saved") : Lang.T("lamp.unsaved"), Lang.T("lamp.saved.tip"), saved ? LampOn : LampOff);
            int next = 1;
            if (hasDevice) { Lamp(next++, deviceOk ? Lang.T("lamp.device") : Lang.T("lamp.noDevice"), Lang.T("lamp.device.tip"), deviceOk ? LampOn : LampOff); }
            Lamp(next, used is null ? Lang.T("lamp.noPlan") : used.Value ? Lang.T("lamp.used") : Lang.T("lamp.differs"), Lang.T("lamp.used.tip"), used is null ? LampNeutral : used.Value ? LampOn : LampOff);
        }
    }

    // Presets, and the parameter lamps, through the real controls.
    private static void PresetSelfTest()
    {
        static void Fail(string message) => throw new InvalidOperationException("方案与参数状态：" + message);
        string temp = Path.Combine(Path.GetTempPath(), "arm-presets-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(temp);
        PresetsPathOverride = Path.Combine(temp, "presets.json");
        try
        {
            var panel = new XYPreviewPanel();
            int requests = 0;
            panel.PointMoveRequested += _ => requests++; panel.TrajectoryRequested += _ => requests++; panel.ExecuteRequested += () => requests++;
            string[] machine = [panel.l1.Text, panel.l2.Text, panel.min1.Text, panel.max1.Text, panel.min2.Text, panel.max2.Text, panel.elbowMin.Text, panel.elbowMax.Text];
            string[] Machine() => [panel.l1.Text, panel.l2.Text, panel.min1.Text, panel.max1.Text, panel.min2.Text, panel.max2.Text, panel.elbowMin.Text, panel.elbowMax.Text];
            if (panel.presetBox.Items.Count != 1 + BuiltInPresets.Length) Fail("方案列表应只含“未选”和内置方案");
            // Each built-in loads, passes the check at the default ranges, and leaves the machine settings alone.
            foreach (var (key, values) in BuiltInPresets)
            {
                panel.ChoosePresetForSelfTest("builtin:" + key);
                int index = int.Parse(values["shape"]);
                if (panel.shape.SelectedIndex != index || panel.x.Text != values["x"] || panel.v1.Text != values["v1"] || panel.acc2.Text != values["a2"]) Fail($"{key}：字段没有载入");
                if (panel.parametersDirty || (index == 0 ? panel.pointTarget is null : panel.execution is null)) Fail($"{key}：载入后应已检查通过：" + panel.result.Text);
                if (!Machine().SequenceEqual(machine)) Fail($"{key}：不得改动杆长或关节/肘范围");
                if (!panel.presetNote.Text.Contains("没有运动")) Fail($"{key}：应说明只载入和检查");
            }
            if (requests != 0) Fail("载入方案不得请求任何运动");
            // Mine: saved under a name, listed, loadable, deletable; bad and reserved names refused.
            panel.ChoosePresetForSelfTest("builtin:preset.smallCircle");
            panel.rotation.Text = "20"; panel.scale.Text = "1.2"; panel.v1.Text = "6.5"; panel.Refresh();
            if (!panel.SavePresetNamed("My circle 20°") || !File.Exists(PresetsPathOverride)) Fail("保存方案失败");
            foreach (string bad in new[] { "", "   ", new string('x', 41), "bad\nname", Lang.T("preset.smallCircle"), Lang.T("preset.none") })
                if (panel.SavePresetNamed(bad)) Fail($"名称“{bad}”不应被接受");
            panel.ChoosePresetForSelfTest("builtin:preset.lineTest");
            if (panel.shape.SelectedIndex != 1) Fail("切换到直线方案");
            panel.ChoosePresetForSelfTest("user:My circle 20°");
            if (panel.shape.SelectedIndex != 3 || panel.rotation.Text != "20" || panel.scale.Text != "1.2" || panel.v1.Text != "6.5" || panel.execution is null) Fail("载入我的方案应还原全部字段并通过检查");
            if (!panel.presetDelete.IsEnabled) Fail("我的方案应能删除");
            // A new window reads the file back, names compared without case.
            var other = new XYPreviewPanel();
            if (other.presetBox.Items.Count != 2 + BuiltInPresets.Length) Fail("重启后应读回我的方案");
            if (!panel.SavePresetNamed("MY CIRCLE 20°") || panel.userPresets.Count != 1 || panel.userPresets[0].Name != "MY CIRCLE 20°") Fail("同名（忽略大小写）应覆盖而不是新增");
            panel.ChoosePresetForSelfTest("user:MY CIRCLE 20°");
            if (!panel.DeleteSelectedPreset() || panel.userPresets.Count != 0 || new XYPreviewPanel().presetBox.Items.Count != 1 + BuiltInPresets.Length) Fail("删除方案失败");
            panel.ChoosePresetForSelfTest("builtin:preset.lineTest");
            if (panel.DeleteSelectedPreset()) Fail("内置方案不能删除");
            // A damaged file is ignored with a message, never trusted.
            foreach (string junk in new[] { "not json", "{\"Schema\":2,\"Presets\":[]}", "{\"Schema\":1,\"Presets\":[{\"Name\":\"x\",\"Values\":{\"shape\":\"99\"}}]}", "{\"Schema\":1,\"Presets\":[{\"Name\":\"x\",\"Values\":{\"L1\":\"1\"}}]}" })
            {
                File.WriteAllText(PresetsPathOverride, junk);
                var broken = new XYPreviewPanel();
                if (broken.presetBox.Items.Count != 1 + BuiltInPresets.Length || broken.presetNote.Text.Length == 0 || !broken.presetNote.Text.Contains("方案")) Fail("损坏的方案文件应被忽略并提示：" + junk);
            }
            File.Delete(PresetsPathOverride);

            // The lamps. Nothing saved yet: unsaved. After a save the group is saved; editing one group only un-saves that group.
            var p = new XYPreviewPanel();
            string Lamps(int[] group) => string.Join("|", p.lampRows.First(r => ReferenceEquals(r.Group, group)).Lamps.Select(l => l.Text));
            if (!Lamps(SpeedGroup).StartsWith("○ 未保存")) Fail("没有保存过时应显示未保存：" + Lamps(SpeedGroup));
            string file = Path.Combine(temp, "parameters.json");
            p.SaveCurrentParameters(file);
            if (!Lamps(SpeedGroup).StartsWith("● 已保存到电脑") || !Lamps(RangeGroup).StartsWith("● 已保存到电脑") || !Lamps(ShapeGroup).StartsWith("● 已保存到电脑")) Fail("保存后各组应显示已保存");
            p.v1.Text = "4";
            if (!Lamps(SpeedGroup).StartsWith("○ 未保存") || !Lamps(RangeGroup).StartsWith("● 已保存到电脑")) Fail("改速度只应让速度组变成未保存：" + Lamps(SpeedGroup));
            p.v1.Text = "5";
            if (!Lamps(SpeedGroup).StartsWith("● 已保存到电脑")) Fail("改回原值应又是已保存");
            p.rotation.Text = "15";
            if (!Lamps(ShapeGroup).StartsWith("○ 未保存")) Fail("改旋转角应让图形组未保存");
            p.rotation.Text = "0"; p.Refresh();
            // The device: ranges and speeds are confirmed separately, which is what a speed edit needs to show.
            p.SetParameterConfirmation(true, true);
            if (!Lamps(SpeedGroup).Contains("● 设备已确认") || !Lamps(RangeGroup).Contains("● 设备已确认")) Fail("两者都确认时应显示设备已确认");
            p.SetParameterConfirmation(true, false);
            if (!Lamps(SpeedGroup).Contains("○ 设备未确认") || !Lamps(RangeGroup).Contains("● 设备已确认")) Fail("只有速度未确认时只应标速度组：" + Lamps(SpeedGroup));
            if (Lamps(ShapeGroup).Split('|').Length != 2) Fail("图形组与设备无关，不应有设备一栏");
            // The plan on the device: used while it is the plan on screen, "differs" as soon as an edit changes the plan.
            if (!Lamps(SpeedGroup).EndsWith("– 设备上没有图形")) Fail("设备上没有图形时应说明：" + Lamps(SpeedGroup));
            p.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, p.execution!.Nodes[0].Pose), ExecutionTrajectory.Counts(1, p.execution.Nodes[0].Pose), 0, 0, 0, 0));
            p.SetPointMoveState(true, p.RequestedLimits, "ok", true, true, p.execution, false);
            if (!Lamps(SpeedGroup).EndsWith("● 已用于上传的图形") || !Lamps(ShapeGroup).EndsWith("● 已用于上传的图形")) Fail("设备持有的就是屏幕上的图形时应显示已用于：" + Lamps(SpeedGroup));
            p.v1.Text = "1"; p.v2.Text = "1"; p.Refresh();   // slow enough to change the plan itself (a small edit can leave it identical)
            if (!Lamps(SpeedGroup).EndsWith("○ 与设备上的图形不同") || !Lamps(RangeGroup).EndsWith("○ 与设备上的图形不同")) Fail("改速度后重新检查，设备上的图形就和屏幕上的不同了：" + Lamps(SpeedGroup));
            // Every lamp has an explanation for the pointer.
            if (p.lampRows.Any(r => r.Lamps.Any(l => l.ToolTip is not string tip || tip.Length < 10))) Fail("每个状态灯都应有说明");
        }
        finally { PresetsPathOverride = null; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
}
