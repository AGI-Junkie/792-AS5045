using System.IO;

namespace TwoLinkCommissioning;

// The board's flash archive from the window's side (docs/TRAJ_STORE.md): ask which figures are stored, save the plan the device
// holds into a slot, clear a slot, and load a stored plan instead of uploading it node by node.  Slot commands are control
// type 12 (one at a time, retried byte for byte like the other control requests); loading is type 13 on the PTP sequence.
public sealed partial class ControlWindow
{
    private sealed class StoreJob(StoreOp op, int slot, Action<bool, string>? done)
    {
        public StoreOp Op { get; } = op;
        public int Slot { get; } = slot;
        public Action<bool, string>? Done { get; } = done;
        public DateTime RequestedAt { get; set; } = DateTime.UtcNow;
        public DateTime LastSeenAt { get; set; } = DateTime.UtcNow;
        public int Attempts { get; set; } = 1;
        public uint Sequence { get; set; }     // control sequence of the frame, known once the writer has sent it
        public bool Acked { get; set; }
    }

    private int storeSlotCount;                                           // traj_store=<n> in LINK; 0 = this firmware cannot archive
    private readonly BoardSlot?[] boardSlots = new BoardSlot?[DeviceContract.TrajStoreSlots];
    private bool boardKnown;                                              // a COMPLETE inventory (every slot reported) has arrived on this link
    private int boardSeen;                                                // bit per slot reported by the query in flight
    private (ExecutionTrajectory Plan, int Slot, DateTime Since)? deferredBoardSave;   // asked while the device was still confirming new speeds
    private StoreStatus? storeStatus;
    private StoreJob? storeJob;
    private StoreOp storeRequestOp;
    private int storeRequestSlot;
    private byte[] storePacket = [];
    private bool storeResend;
    private DateTime lastBoardQueryAt;
    private List<LedgerEntry> boardLedger = [];
    private Func<string>? boardMessage;
    private (ExecutionNode[] Nodes, int Slot)? pendingBoardSave;          // upload first, then save into the slot
    private int? loadingSlot;                                             // a type 13 load is in flight for the plan in `uploading`
    private const double StoreAnswerSeconds = 1.0, StoreWorkSeconds = 12;

    private bool StoreAvailable => storeSlotCount > 0 && framedControl && compatible && protocolMatches && reportedEpoch != 0 && !waitingStopEpoch && !stopPending;
    private bool StoreBusy => storeJob is { Op: StoreOp.Save or StoreOp.Erase };

    private void ReadStoreCapability(string[] words)
    {
        string? slots = words.FirstOrDefault(w => w.StartsWith("traj_store=", StringComparison.Ordinal));
        storeSlotCount = slots is not null && int.TryParse(slots.AsSpan(11), out int n) && n is > 0 and <= DeviceContract.TrajStoreSlots ? n : 0;
        if (storeSlotCount == 0) { boardKnown = false; boardSeen = 0; Array.Clear(boardSlots); }
        PublishBoard();
    }

    private void ResetBoard()
    {
        storeSlotCount = 0; boardKnown = false; boardSeen = 0; Array.Clear(boardSlots); storeStatus = null; storeJob = null; storePacket = []; storeResend = false;
        pendingBoardSave = null; deferredBoardSave = null; loadingSlot = null; boardMessage = null; lastBoardQueryAt = default;
        lock (wireSync) { if (pendingControlType == 12) pendingControlType = 0; }
        PublishBoard();
    }

    private void PublishBoard() => xyPreview.SetBoard(storeSlotCount, boardKnown, storeJob is not null && storeJob.Op != StoreOp.Query || pendingBoardSave is not null || deferredBoardSave is not null,
        boardSlots, boardLedger, boardMessage);
    private void SetBoardMessage(Func<string>? message) { boardMessage = message; PublishBoard(); }

    private bool StartStoreJob(StoreOp op, int slot, Action<bool, string>? done)
    {
        if (!StoreAvailable || storeJob is not null) return false;
        lock (wireSync)
        {
            if (pendingControlType != 0) return false;
            storeJob = new StoreJob(op, slot, done);
            storeRequestOp = op; storeRequestSlot = slot; storePacket = []; storeResend = false; pendingControlType = 12;
            if (op == StoreOp.Query) boardSeen = 0;   // only a query that reports every slot completes the inventory
        }
        Append($"HOST archive request {op} slot={(slot < 0 ? "all" : slot.ToString())}");
        PublishBoard();
        return true;
    }

    private void FinishStoreJob(bool ok, string message)
    {
        if (storeJob is not { } job) return;
        storeJob = null;
        lock (wireSync) { if (pendingControlType == 12) pendingControlType = 0; }
        Append($"HOST archive {job.Op} slot={(job.Slot < 0 ? "all" : job.Slot.ToString())} {(ok ? "done" : "failed: " + message)}", ok ? null : LogLevel.Warning);
        job.Done?.Invoke(ok, message);
        // A change on the board is read back, so the picture shown is the device's and not what was hoped for.
        if (ok && job.Op != StoreOp.Query) StartStoreJob(StoreOp.Query, -1, null);
        PublishBoard();
    }

    // Reads the archive once the link is up and the machine is still, and again after the device lost the answer.
    private void SynchronizeBoard(DateTime now)
    {
        if (!StoreAvailable || boardKnown || storeJob is not null || pendingBoardSave is not null || (now - lastBoardQueryAt).TotalSeconds < 3) return;
        if (PointBusy || homing || keyboardMask != 0 || mouseMask != 0 || ptp is not { Active: 0 }) return;
        lastBoardQueryAt = now;
        StartStoreJob(StoreOp.Query, -1, null);
    }

    // Unanswered requests are repeated byte for byte (the device only repeats its receipt); a saving device is given time.
    private void PollStoreJob(DateTime now)
    {
        if (storeJob is not { } job) return;
        double quiet = (now - job.LastSeenAt).TotalSeconds;
        if (!job.Acked)
        {
            if ((now - job.RequestedAt).TotalSeconds > 8 || (job.Attempts >= 3 && quiet >= StoreAnswerSeconds * 3)) { FinishStoreJob(false, Lang.T("board.err.noAnswer")); return; }
            if (job.Sequence != 0 && job.Attempts < 3 && quiet >= StoreAnswerSeconds * job.Attempts)
            {
                lock (wireSync) { if (pendingControlType != 0) return; storeResend = true; pendingControlType = 12; }
                job.Attempts++;
                Append($"HOST archive retry attempt={job.Attempts}/3; identical packet", LogLevel.Warning);
            }
        }
        else if (quiet > (job.Op == StoreOp.Query ? 3 : StoreWorkSeconds)) FinishStoreJob(false, Lang.T("board.err.stalled"));
    }

    private void ObserveStoreLine(string line)
    {
        if (line.StartsWith("TRAJ_SLOT ", StringComparison.Ordinal))
        {
            if (BoardSlot.Parse(line) is { } slot && slot.Slot < storeSlotCount)
            {
                boardSlots[slot.Slot] = slot; boardSeen |= 1 << slot.Slot;
                if (boardSeen == (1 << storeSlotCount) - 1) boardKnown = true;
                if (storeJob is { } q) q.LastSeenAt = DateTime.UtcNow;
                PublishBoard();
            }
            else Append("HOST malformed or unexpected archive slot line", LogLevel.Warning);
            return;
        }
        if (storeJob is not { } job) return;
        if (StoreStatus.Parse(line) is { } status)
        {
            storeStatus = status;
            if (job.Sequence == 0 || status.Seq != job.Sequence) return;
            job.Acked = true; job.LastSeenAt = DateTime.UtcNow;
            if (status.Busy) { PublishBoard(); return; }
            bool ok = job.Op switch
            {
                StoreOp.Query => true,
                StoreOp.Save => status.Last == "saved" && status.LastSlot == job.Slot,
                _ => status.Last == "erased" && status.LastSlot == job.Slot,
            };
            FinishStoreJob(ok, ok ? "" : Lang.T("board.err.device", status.Last));
        }
        else if (line.StartsWith("TRAJ_STORE refused:", StringComparison.Ordinal) && !job.Acked)
            FinishStoreJob(false, Lang.T("board.err.refused", line[19..].Trim()));
    }

    // ---- what the panel asks for ----------------------------------------------------------------------------------------
    private void HookBoardPanel()
    {
        xyPreview.SaveToBoardRequested += (plan, slot) => SafeAction(() => RequestSaveToBoard(plan, slot));
        xyPreview.EraseBoardRequested += slot => SafeAction(() => RequestEraseBoardSlot(slot));
        boardLedger = BoardLedger.Load();
    }

    // The plan the device holds is what gets saved.  A figure that is not held yet is uploaded first, then saved: one click.
    private void RequestSaveToBoard(ExecutionTrajectory plan, int slot)
    {
        if (slot < 0 || slot >= storeSlotCount) return;
        if (!StoreAvailable || !boardKnown || storeJob is not null || pendingBoardSave is not null || PointBusy) { SetBoardMessage(() => Lang.T("board.err.busy")); return; }
        if (uploaded is { } held && held.Nodes.SequenceEqual(plan.Nodes)) { deferredBoardSave = null; BeginBoardSave(plan.Nodes, slot); return; }
        // A figure with other speeds makes the panel re-send them first; the upload has to wait until the device confirmed them.
        if (!LimitsReady) { deferredBoardSave = (plan, slot, DateTime.UtcNow); SetBoardMessage(() => Lang.T("board.waitSync", slot + 1)); return; }
        deferredBoardSave = null;
        pendingBoardSave = (plan.Nodes, slot);
        RequestTrajectory(plan, allowBoard: false);
        if (uploading is null) { pendingBoardSave = null; SetBoardMessage(() => Lang.T("board.err.needStart")); return; }
        SetBoardMessage(() => Lang.T("board.uploadThenSave", slot + 1));
    }

    // The save that was waiting for the speed synchronisation goes ahead once the device confirmed it, or is dropped after 15 s.
    private void RetryDeferredBoardSave(DateTime now)
    {
        if (deferredBoardSave is not { } deferred) return;
        if ((now - deferred.Since).TotalSeconds > 15) { deferredBoardSave = null; SetBoardMessage(() => Lang.T("board.err.needStart")); return; }
        if (!LimitsReady || PointBusy) return;
        deferredBoardSave = null;
        RequestSaveToBoard(deferred.Plan, deferred.Slot);
    }

    private void BeginBoardSave(ExecutionNode[] nodes, int slot)
    {
        uint crc = TrajectoryArchive.PlanCrc(nodes);
        string settings = xyPreview.SettingsDescription;
        if (!StartStoreJob(StoreOp.Save, slot, (ok, message) =>
        {
            if (ok) { RememberBoard(slot, crc, settings); SetBoardMessage(() => Lang.T("board.saved", slot + 1)); }
            else SetBoardMessage(() => Lang.T("board.saveFailed", slot + 1, message));
        })) { SetBoardMessage(() => Lang.T("board.err.busy")); return; }
        SetBoardMessage(() => Lang.T("board.saving", slot + 1));
    }

    private void RememberBoard(int slot, uint crc, string? settings)
    {
        try { BoardLedger.Set(slot, crc, settings); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { Append("HOST could not write the board ledger: " + ex.Message, LogLevel.Warning); }
        boardLedger = BoardLedger.Load();
    }

    private void RequestEraseBoardSlot(int slot)
    {
        if (slot < 0 || slot >= storeSlotCount) return;
        if (!StoreAvailable || storeJob is not null || PointBusy) { SetBoardMessage(() => Lang.T("board.err.busy")); return; }
        if (StartStoreJob(StoreOp.Erase, slot, (ok, message) =>
        {
            if (ok) { RememberBoard(slot, 0, null); SetBoardMessage(() => Lang.T("board.erased", slot + 1)); }
            else SetBoardMessage(() => Lang.T("board.eraseFailed", slot + 1, message));
        })) SetBoardMessage(() => Lang.T("board.erasing", slot + 1));
    }

    // ---- loading a stored plan ------------------------------------------------------------------------------------------
    private void SendLoadPacket()
    {
        if (uploading is null || loadingSlot is not { } slot) return;
        pendingPoint = ++pointSequence; pointRequestedAt = DateTime.UtcNow;
        QueuePointPacket(PtpProtocol.EncodeLoad(pendingPoint, trajectoryEpoch, slot));
        ShowDeviceState(); UpdateControls();
    }

    // The device holds the plan now: the same state as after a complete upload, and still no automatic run.
    private bool CompleteBoardLoad(uint seq, uint epoch, uint mode, uint count, uint total)
    {
        if (loadingSlot is not { } slot || uploading is not { } plan || seq != pendingPoint || epoch != trajectoryEpoch) return false;
        if (mode != 2 || count != plan.Nodes.Length || total != plan.Nodes.Length) { Append("HOST archive load acknowledgement mismatch", LogLevel.Error); Stop(); return true; }
        pendingPoint = 0; lock (wireSync) { pointWire.Clear(); }
        uploading = null; uploaded = plan; uploadedRan = false; loadingSlot = null;
        Append($"HOST plan loaded from board slot {slot}; waiting for the operator to start it (no automatic run)");
        SetBoardMessage(() => Lang.T("board.loaded", slot + 1));
        ShowDeviceState(); UpdateControls();
        return true;
    }

    // The device would not take the stored plan (changed limits or speeds, damaged slot): fall back to the normal upload of the
    // same plan.  The refused load did not use up a device sequence number, so the next frames simply continue from ours.
    private void FallBackToUpload(string reason)
    {
        if (loadingSlot is not { } slot || uploading is null) return;
        Append($"HOST board slot {slot} could not be loaded ({reason}); uploading the plan instead", LogLevel.Warning);
        loadingSlot = null; pendingPoint = 0; lock (wireSync) { pointWire.Clear(); }
        boardKnown = false; boardSeen = 0; uploadNext = 0;
        SetBoardMessage(() => Lang.T("board.loadFallback", slot + 1));
        SendNextTrajectoryPacket();
    }

    // The plan is on the device and a save was waiting for it.
    private void ContinueBoardSave(ExecutionTrajectory plan)
    {
        if (pendingBoardSave is not { } pending || !pending.Nodes.SequenceEqual(plan.Nodes)) return;
        pendingBoardSave = null;
        BeginBoardSave(plan.Nodes, pending.Slot);
    }

    // The archive's message handling, checked against the lines the firmware prints.
    private static void StoreSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Archive: " + reason); }
        string ledger = Path.Combine(Path.GetTempPath(), "arm-board-" + Guid.NewGuid().ToString("N") + ".json");
        BoardLedger.PathOverride = ledger;
        var w = new ControlWindow { selfTesting = true };
        var frames = new List<byte[]>();
        int sent12 = 0;
        static int Type(byte[] f) => ((f[1] >> 1) & 15) | (((f[2] >> 1) & 15) << 4);
        static int Field(byte[] f, int i) => ((f[1 + 2 * i] >> 1) & 15) | (((f[2 + 2 * i] >> 1) & 15) << 4);
        void Write(byte[] data, int offset, int count)
        {
            if (offset == 0 && data.Length == 36 && Type(data) == 12) sent12++;
            if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data);
        }
        byte[]? Next(int type)
        {
            w.PublishHeartbeatLease();
            for (int i = 0; i < 8 && w.pendingControlType != 0; i++) w.WireWriterStep(Write);
            return frames.LastOrDefault(f => f.Length == 36 && Type(f) == type);
        }
        string Link(string extra) => $"LINK uart=hw baud=115200 fw=stage2-v11-test short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 traj_repeat=1 fault_snapshot=1{extra} proto={DeviceContract.ProtocolVersion}";
        try
        {
            w.HandleLine("STATE fw=stage2-v11 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=3 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            // A firmware without the capability (or linked with the stock script, traj_store=0) is never asked.
            w.HandleLine(Link(""));
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(10));
            Check(w.storeSlotCount == 0 && w.pendingControlType == 0 && !w.StartStoreJob(StoreOp.Query, -1, null), "an old firmware was asked about its archive");
            w.HandleLine(Link(" traj_store=0 telem_hz=0"));
            Check(w.storeSlotCount == 0, "traj_store=0 must mean unavailable");
            // A capable one is queried once, when the machine is still.
            w.HandleLine(Link(" traj_store=4 telem_hz=0"));
            Check(w.storeSlotCount == 4 && w.StoreAvailable, "the capability was not read");
            Next(11);   // the stream-rate request that the same LINK line triggered goes out first
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(10));
            Check(w.pendingControlType == 12 && w.storeJob is { Op: StoreOp.Query }, $"no query was queued (available={w.StoreAvailable} slots={w.storeSlotCount} framed={w.framedControl} busy={w.PointBusy} pending={w.pendingControlType})");
            var frame = Next(12);
            Check(frame is not null && Field(frame, 0) == 12 && (short)(Field(frame, 9) | Field(frame, 10) << 8) == -1 && Field(frame, 11) == 1 && Field(frame, 12) == 0 && Field(frame, 13) == 0, "query frame payload");
            uint seq = w.storeJob!.Sequence;
            int writesPerFrame = sent12;   // how many write calls one frame takes on this transport
            Check(seq != 0 && w.pendingControlType == 0, "the query frame was not sent");
            // No answer: the same bytes again after a second, and the identical sequence.
            w.PollStoreJob(DateTime.UtcNow.AddSeconds(1.5));
            Check(w.pendingControlType == 12 && w.storeResend, "an unanswered query was not repeated");
            int before = sent12;
            var again = Next(12);
            Check(sent12 == before + writesPerFrame && writesPerFrame > 0 && again is not null && again.AsSpan().SequenceEqual(frame) && w.storeJob.Sequence == seq,
                $"the retry must be the identical packet, sent once (sent {sent12 - before} write calls for {writesPerFrame}, same bytes {again is not null && again.AsSpan().SequenceEqual(frame)}, seq {w.storeJob.Sequence}/{seq})");
            // The slots, then the status that ends it.  A status for another sequence is not ours.
            w.HandleLine("TRAJ_SLOT slot=0 valid=0");
            w.HandleLine("TRAJ_SLOT slot=1 valid=1 seq=2 nodes=485 duration_ms=60480 q1=0 q2=0 crc=BD3EB746");
            w.HandleLine("TRAJ_SLOT slot=2 valid=0");
            Check(!w.boardKnown && TrajectoryArchive.FreeSlot(w.boardSlots, w.storeSlotCount) == 0, "a partial inventory was taken as complete");
            w.HandleLine("TRAJ_SLOT slot=3 valid=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={seq + 5} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is not null, "a status for another command ended the job");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={seq} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is null && w.boardKnown && w.boardSlots[1] is { Valid: true, Crc: 0xBD3EB746 } && w.boardSlots[0] is { Valid: false }, "the query answer was not taken");
            // Saving waits for the complete inventory, and for the device to confirm new speeds: a figure with other speeds makes the panel
            // re-send them, and the upload that follows is held back instead of being dropped.
            w.xyPreview.SelectShapeForSelfTest(3); w.xyPreview.LoadExampleForSelfTest();
            var shown = w.xyPreview.Plan ?? throw new InvalidOperationException("Archive: the example figure has no plan");
            Check(!w.LimitsReady, "the fixture was expected to be waiting for the device's ranges");
            w.RequestSaveToBoard(shown, 0);
            Check(w.deferredBoardSave is { Slot: 0 } && w.uploading is null && w.pendingBoardSave is null && w.storeJob is null, "a save asked during speed synchronisation was not held back");
            w.RetryDeferredBoardSave(DateTime.UtcNow.AddSeconds(2));
            Check(w.deferredBoardSave is not null, "the held save was dropped before its time");
            w.RetryDeferredBoardSave(DateTime.UtcNow.AddSeconds(16));
            Check(w.deferredBoardSave is null && w.uploading is null, "a save that never became possible was not dropped");
            w.RequestSaveToBoard(shown, 7);
            Check(w.deferredBoardSave is null && w.pendingBoardSave is null, "a slot the device does not have was accepted");
            // A plan on the device is saved into the free slot; the receipt, progress and the end are matched by sequence.
            var nodes = new[] { new ExecutionNode(new JointPose(0, 0), 0), new ExecutionNode(new JointPose(0.5, -0.25), 100), new ExecutionNode(new JointPose(1, -0.5), 100), new ExecutionNode(new JointPose(1, -0.5), 50) };
            Check(TrajectoryArchive.FreeSlot(w.boardSlots) == 0 && TrajectoryArchive.SlotFor(w.boardSlots, nodes) is null, "slot choice before saving");
            bool done = false, ok = false;
            Check(w.StartStoreJob(StoreOp.Save, 0, (success, _) => { done = true; ok = success; }) && w.StoreBusy, "a save was not queued");
            Check(!w.StartStoreJob(StoreOp.Erase, 1, null), "a second command was queued while one is in flight");
            Next(12); uint saveSeq = w.storeJob!.Sequence;
            Check(saveSeq > seq, "the control sequence did not advance");
            w.HandleLine($"TRAJ_STORE state=saving slot=0 rows=0/1 seq={saveSeq} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is { Acked: true } && !done, "the receipt did not mark the job acknowledged");
            w.PollStoreJob(DateTime.UtcNow.AddSeconds(5)); Check(w.storeJob is not null, "a saving device was given up on after five seconds");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=1/1 seq={saveSeq} last=saved last_slot=0 row_max_us=4000");
            Check(done && ok && w.storeJob is { Op: StoreOp.Query }, "the end of the save was not taken and read back");
            // A failed save, a refusal, silence: each ends the job as failed; a refusal needs no waiting.
            w.FinishStoreJob(true, "");
            done = false; Check(w.StartStoreJob(StoreOp.Save, 2, (success, _) => { done = true; ok = success; }), "save 2 queued"); Next(12); uint failSeq = w.storeJob!.Sequence;
            w.HandleLine($"TRAJ_STORE state=idle slot=2 rows=3/12 seq={failSeq} last=failed last_slot=2 row_max_us=0");
            Check(done && !ok, "a failed save was reported as done");
            w.FinishStoreJob(true, "");
            done = false; Check(w.StartStoreJob(StoreOp.Erase, 1, (success, _) => { done = true; ok = success; }), "erase queued");
            w.HandleLine("TRAJ_STORE refused: stage not idle");
            Check(done && !ok && w.storeJob is null, "a refusal did not end the job");
            done = false; Check(w.StartStoreJob(StoreOp.Erase, 1, (success, _) => { done = true; ok = success; }), "erase queued again"); Next(12);
            w.PollStoreJob(DateTime.UtcNow.AddSeconds(30)); w.PollStoreJob(DateTime.UtcNow.AddSeconds(31)); w.PollStoreJob(DateTime.UtcNow.AddSeconds(32)); w.PollStoreJob(DateTime.UtcNow.AddSeconds(33));
            Check(done && !ok && w.storeJob is null, "silence did not end the job");
            // Disconnecting forgets the board; a stop forgets a pending save.
            w.pendingBoardSave = (nodes, 0);
            w.ResetBoard();
            w.HandleLine(Link(" traj_store=2 telem_hz=0"));
            Check(w.storeSlotCount == 2, "a smaller slot count was not read");
            w.StartStoreJob(StoreOp.Query, -1, null); Next(12);
            w.HandleLine("TRAJ_SLOT slot=0 valid=0"); w.HandleLine("TRAJ_SLOT slot=1 valid=0"); w.HandleLine("TRAJ_SLOT slot=3 valid=0");
            Check(w.boardKnown && w.boardSlots[3] is null, "the advertised slot count was not honoured");
            w.ResetBoard();
            Check(w.storeSlotCount == 0 && !w.boardKnown && w.boardSlots.All(s => s is null) && w.pendingBoardSave is null && w.pendingControlType == 0, "the board state survived a reset");
        }
        finally { w.Close(); BoardLedger.PathOverride = null; if (File.Exists(ledger)) File.Delete(ledger); }
    }
}
