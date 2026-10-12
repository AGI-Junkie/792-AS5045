using System.Windows;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

// The board's flash archive in the panel: a star on a recent run means "this figure is stored on the board".  ☆ puts the
// run's figure there (load it, upload it, save it into a free slot, or into the slot the operator picks when all are used);
// ★ takes it off again.  A figure on the board is loaded from its slot instead of uploaded: the Upload button says so.
// Nothing here moves the arm or talks to the device; the window does that and reports the slots back through SetBoard.
public sealed partial class XYPreviewPanel
{
    public event Action<ExecutionTrajectory, int>? SaveToBoardRequested;   // plan, slot: upload it if the device does not hold it, then save
    public event Action<int>? EraseBoardRequested;

    private int boardSlotCount;                    // the slots the device advertises (traj_store=<n>); 0 = no archive
    private bool boardKnown, boardBusy;            // the inventory is complete; a save/erase/upload-then-save is in progress
    private bool boardSupported => boardSlotCount > 0;
    private IReadOnlyList<BoardSlot?> boardSlots = new BoardSlot?[DeviceContract.TrajStoreSlots];
    private IReadOnlyList<LedgerEntry> boardLedger = [];
    private Func<string>? boardMessage;
    private BoardSlot? planOnBoard;                    // the slot that holds exactly the plan on screen
    internal static bool SuppressMenus;                // tests drive the menu items without opening a popup
    internal static ContextMenu? LastMenu;
    private static void ShowMenu(ContextMenu menu) { LastMenu = menu; if (!SuppressMenus) menu.IsOpen = true; }
    internal string BoardNoteText => boardNote.Text;
    internal string RecentListText => string.Join("\n", recentList.Children.Cast<UIElement>().Select(c => c is TextBlock t ? t.Text : c is DockPanel d ? string.Join(" ", d.Children.OfType<Button>().Select(b => b.Content as TextBlock is { } label ? label.Text : b.Content?.ToString())) : ""));
    // One clear button per stored slot, so a figure saved from another computer (or whose record was lost) can be cleared too.
    private readonly WrapPanel boardClear = new() { Margin = new Thickness(10, 0, 10, 6) };
    private readonly ContentControl boardClearHost = new() { IsTabStop = false, Focusable = false };   // locked with the other edit controls while the arm moves
    internal IReadOnlyList<Button> BoardClearButtons => boardClear.Children.OfType<Button>().ToList();
    private readonly TextBlock boardNote = new() { TextWrapping = TextWrapping.Wrap, FontSize = 11, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(10, 2, 10, 6) };

    // known: every slot has been reported on this link.  Until then no slot is assumed empty, and saving stays off.
    internal int BoardRebuilds;                    // how often the board state really changed (the controls are rebuilt only then)
    public void SetBoard(int slotCount, bool known, bool busy, IReadOnlyList<BoardSlot?> slots, IReadOnlyList<LedgerEntry> ledger, Func<string>? message)
    {
        // The device repeats its LINK line every 500 ms: unchanged state must not tear down the buttons the operator is using
        // (a confirmation menu opened from one of them would be lost).
        slotCount = Math.Clamp(slotCount, 0, DeviceContract.TrajStoreSlots);
        var newSlots = slots.ToArray(); var newLedger = ledger.ToList();
        if (slotCount == boardSlotCount && known == boardKnown && busy == boardBusy && newSlots.SequenceEqual(boardSlots) && newLedger.SequenceEqual(boardLedger) && ReferenceEquals(message, boardMessage)) return;
        BoardRebuilds++;
        boardSlotCount = slotCount; boardKnown = known; boardBusy = busy; boardSlots = newSlots; boardLedger = newLedger; boardMessage = message;
        ShowRecentRuns();
        UpdateBoardNote();
        UpdateBoardClear();
        UpdatePointButton();
    }

    private void UpdateBoardClear()
    {
        boardClear.Children.Clear();
        if (!boardSupported || !boardKnown) return;
        foreach (var slot in boardSlots.Take(boardSlotCount).OfType<BoardSlot>().Where(s => s.Valid))
        {
            var button = new Button { Content = Lang.T("board.clearButton", slot.Slot + 1), ToolTip = Lang.T("board.clearButton.tip", slot.Slot + 1),
                Margin = new Thickness(0, 0, 6, 0), Padding = new Thickness(8, 1, 8, 1), IsEnabled = !boardBusy };
            ToolTipService.SetShowOnDisabled(button, true);
            button.Click += (_, _) => { if (button.IsEnabled) StoredStarClicked(button, slot); };
            boardClear.Children.Add(button);
        }
    }

    // The slot holding the plan on screen, cached per plan and slot contents: the check is a CRC over the node bytes.
    private (ExecutionTrajectory? Plan, IReadOnlyList<BoardSlot?> Slots, BoardSlot? Found) boardLookup;
    private BoardSlot? PlanOnBoard()
    {
        if (!boardSupported || !boardKnown || execution is null) return null;   // a half-read inventory is not used to decide "already on the board"
        if (!ReferenceEquals(boardLookup.Plan, execution) || !ReferenceEquals(boardLookup.Slots, boardSlots))
            boardLookup = (execution, boardSlots, TrajectoryArchive.SlotFor(boardSlots, execution.Nodes));
        return boardLookup.Found;
    }

    private static bool SameFigure(IReadOnlyDictionary<string, string> a, IReadOnlyDictionary<string, string> b) =>
        PresetKeys.All(key => string.Equals(a.GetValueOrDefault(key), b.GetValueOrDefault(key), StringComparison.Ordinal));

    // The slot a recent run is stored in: this computer recorded those settings for it and the device still reports that plan.
    private BoardSlot? StarSlot(IReadOnlyDictionary<string, string>? values)
    {
        if (!boardSupported || values is null) return null;
        foreach (var slot in boardSlots)
            if (BoardLedger.For(slot, boardLedger) is { } entry && RunValues(entry.Settings) is { } stored && SameFigure(stored, values)) return slot;
        return null;
    }

    private string SlotLabel(BoardSlot? slot)
    {
        if (slot is not { Valid: true }) return Lang.T("board.empty");
        string size = Lang.T("board.size", slot.Nodes, slot.Seconds);
        if (BoardLedger.For(slot, boardLedger) is not { } entry) return Lang.T("board.unknown") + size;
        var values = RunValues(entry.Settings);
        string name = values is not null && int.TryParse(values["shape"], out int index) && index is >= 0 and <= 10 ? Lang.T("xy.shape" + index) : Lang.T("board.custom");
        return name + size;
    }

    private void UpdateBoardNote()
    {
        Lang.Text(boardNote, () =>
        {
            if (!boardSupported) return Lang.T("board.unsupported");
            if (!boardKnown) return Lang.T("board.reading") + (boardMessage is { } pending ? "\n" + pending() : "");
            string slots = string.Join("\n", Enumerable.Range(0, boardSlotCount).Select(i => Lang.T("board.slot", i + 1, SlotLabel(boardSlots.ElementAtOrDefault(i)))));
            return Lang.T("board.summary", slots) + (boardMessage is { } message ? "\n" + message() : "");
        });
    }

    // ☆: the figure goes to the first free slot; with none free the operator chooses which one to replace.
    private void StarClicked(Button star, ExperimentRun run, Dictionary<string, string> values)
    {
        if (TrajectoryArchive.FreeSlot(boardSlots, boardSlotCount) is { } free) { SaveRunToBoard(run, values, free); return; }
        var menu = new ContextMenu { PlacementTarget = star };
        for (int i = 0; i < boardSlotCount; i++)
        {
            int slot = i;
            var item = new MenuItem { Header = Lang.T("board.replace", slot + 1, SlotLabel(boardSlots.ElementAtOrDefault(slot))) };
            item.Click += (_, _) => SaveRunToBoard(run, values, slot);
            menu.Items.Add(item);
        }
        ShowMenu(menu);
    }

    // ★: one menu item, so a stray click clears nothing.
    private void StoredStarClicked(Button star, BoardSlot slot)
    {
        var item = new MenuItem { Header = Lang.T("board.clear", slot.Slot + 1) };
        item.Click += (_, _) => EraseBoardRequested?.Invoke(slot.Slot);
        ShowMenu(new ContextMenu { PlacementTarget = star, Items = { item } });
    }

    // Loads the run's figure exactly as clicking the run would, then asks for it to be saved; the window uploads it first if the
    // device does not hold it.  A figure that fails its checks is not sent anywhere.
    private void SaveRunToBoard(ExperimentRun run, Dictionary<string, string> values, int slot)
    {
        ApplyPreset(values);
        if (execution is not { } plan) { Lang.Text(recentNote, () => Lang.T("board.err.plan") + result.Text); return; }
        Lang.Text(recentNote, () => Lang.T("board.saveStarted", RunSummary(run), slot + 1));
        SaveToBoardRequested?.Invoke(plan, slot);
    }

    private void UpdateUploadCaption()
    {
        planOnBoard = PlanOnBoard();
        string caption = planOnBoard is null ? Lang.T("btn.upload") : Lang.T("btn.loadBoard");
        if (!Equals(upload.Content, caption)) upload.Content = caption;
    }
}
