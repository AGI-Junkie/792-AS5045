using System.IO;

namespace TwoLinkCommissioning;

// The board's flash archive from the window's side (docs/TRAJ_STORE.md): ask which figures are stored, save the plan the device
// holds into a slot, clear a slot, and load a stored plan instead of uploading it node by node.  Slot commands are control
// type 12 (one at a time, retried byte for byte like the other control requests); loading is type 13 on the PTP sequence.
public sealed partial class ControlWindow
{
    private sealed class StoreJob(StoreOp op, int slot, Action<bool, Func<string>>? done)
    {
        public StoreOp Op { get; } = op;
        public int Slot { get; } = slot;
        public Action<bool, Func<string>>? Done { get; } = done;
        public DateTime RequestedAt { get; set; } = DateTime.UtcNow;
        public DateTime LastSeenAt { get; set; } = DateTime.UtcNow;
        public int Attempts { get; set; } = 1;
        public uint Sequence { get; set; }     // control sequence of the frame, known once the writer has sent it
        public bool Acked { get; set; }
    }

    private int storeSlotCount;                                           // traj_store=<n> in LINK; 0 = this firmware cannot archive
    private int storeMetaMax;                                             // traj_meta=<n> in LINK: bytes of description it keeps with a plan; 0 = none
    private bool storeReadSupported;                                      // traj_read=1 in LINK: it sends a slot's nodes back (type 12 op 5)
    private int storeRequestFirst;                                        // first node asked for by the op 5 request in flight
    // A slot's nodes being read back, forty at a time, to show what the board really holds.
    private sealed class NodeRead(int slot, uint seq, int total, int durationMs, uint crc, string? settings)
    {
        public int Slot { get; } = slot;
        public uint Seq { get; } = seq;                                   // the slot's save counter when the read began: a re-save changes it
        public int Total { get; } = total;
        public int DurationMs { get; } = durationMs;
        public uint Crc { get; } = crc;
        public string? Settings { get; } = settings;                      // the description stored with the plan, if any (link lengths)
        public byte[] Bytes { get; } = new byte[total * 6];
        public int Next { get; set; }                                     // nodes taken so far
        public int AskedAt { get; set; }                                  // Next when the request in flight was made
        public int Retries { get; set; }                                  // requests for the same nodes that got only their status
    }
    private NodeRead? nodeRead;
    private const int NodeRetries = 3;
    private readonly BoardSlot?[] boardSlots = new BoardSlot?[DeviceContract.TrajStoreSlots];
    private readonly BoardMeta?[] boardMeta = new BoardMeta?[DeviceContract.TrajStoreSlots];   // read back from the board, one slot at a time
    private readonly uint?[] metaAsked = new uint?[DeviceContract.TrajStoreSlots];             // the slot's save counter when its description was last asked for
    private readonly uint?[] metaFailSeq = new uint?[DeviceContract.TrajStoreSlots];           // ...and the one the failures below were counted for
    private readonly int[] metaFailures = new int[DeviceContract.TrajStoreSlots];
    private const int MetaRetries = 3;
    private readonly bool[] metaEmpty = new bool[DeviceContract.TrajStoreSlots];   // the device answered "no description" to the read in flight
    private void ForgetMeta(int slot) { boardMeta[slot] = null; metaAsked[slot] = null; metaFailSeq[slot] = null; metaFailures[slot] = 0; metaEmpty[slot] = false; }
    private byte[][] storeMetaChunks = [];                                // the description of the plan being saved: sent first, then the Save frame
    private int storeMetaNext;
    private bool boardKnown;                                              // a COMPLETE inventory (every slot reported) has arrived on this link
    private int boardSeen;                                                // bit per slot reported by the query in flight
    private (ExecutionTrajectory Plan, int Slot, DateTime Since, string Settings)? deferredBoardSave;   // asked while the device was still confirming new speeds
    private StoreStatus? storeStatus;
    private StoreJob? storeJob;
    private StoreOp storeRequestOp;
    private int storeRequestSlot;
    private byte[] storePacket = [];
    private bool storeResend;
    private DateTime lastBoardQueryAt;
    private List<LedgerEntry> boardLedger = [];
    private Func<string>? boardMessage;
    private (ExecutionNode[] Nodes, int Slot, string Settings)? pendingBoardSave;          // upload first, then save into the slot
    private int? loadingSlot;                                             // a type 13 load is in flight for the plan in `uploading`
    private const double StoreAnswerSeconds = 1.0, StoreWorkSeconds = 12;

    private bool StoreAvailable => storeSlotCount > 0 && framedControl && compatible && protocolMatches && reportedEpoch != 0 && !waitingStopEpoch && !stopPending;
    private bool StoreBusy => storeJob is { Op: StoreOp.Save or StoreOp.Erase };

    private void ReadStoreCapability(string[] words)
    {
        string? slots = words.FirstOrDefault(w => w.StartsWith("traj_store=", StringComparison.Ordinal));
        int before = storeSlotCount, metaBefore = storeMetaMax; bool readBefore = storeReadSupported;
        storeSlotCount = slots is not null && int.TryParse(slots.AsSpan(11), out int n) && n is > 0 and <= DeviceContract.TrajStoreSlots ? n : 0;
        string? meta = words.FirstOrDefault(w => w.StartsWith("traj_meta=", StringComparison.Ordinal));
        storeMetaMax = storeSlotCount > 0 && meta is not null && int.TryParse(meta.AsSpan(10), out int m) && m > 0 ? Math.Min(m, BoardMeta.MaxBytes) : 0;
        storeReadSupported = storeSlotCount > 0 && words.Contains("traj_read=1");
        if (storeSlotCount == 0) { boardKnown = false; boardSeen = 0; Array.Clear(boardSlots); Array.Clear(boardMeta); Array.Clear(metaAsked); Array.Clear(metaFailSeq); Array.Clear(metaFailures); Array.Clear(metaEmpty); }
        // The LINK line comes every 500 ms; the panel is only told when what it shows depends on changes (a repeated line that finally
        // carries a token an earlier one lost must reach it too).
        if (storeSlotCount != before || storeMetaMax != metaBefore || storeReadSupported != readBefore) PublishBoard();
    }

    private void ResetBoard()
    {
        storeSlotCount = 0; storeMetaMax = 0; storeReadSupported = false; nodeRead = null; wantedPathSlot = null; boardKnown = false; boardSeen = 0; Array.Clear(boardSlots); Array.Clear(boardMeta); Array.Clear(metaAsked); Array.Clear(metaFailSeq); Array.Clear(metaFailures); Array.Clear(metaEmpty); storeStatus = null; storeJob = null; storePacket = []; storeResend = false;
        pendingBoardSave = null; deferredBoardSave = null; loadingSlot = null; boardMessage = null; lastBoardQueryAt = default;
        lock (wireSync) { if (pendingControlType is 12 or 14) pendingControlType = 0; storeMetaChunks = []; storeMetaNext = 0; }
        PublishBoard();
    }

    private void PublishBoard() => xyPreview.SetBoard(storeSlotCount, boardKnown, storeJob is { Op: StoreOp.Save or StoreOp.Erase } || pendingBoardSave is not null || deferredBoardSave is not null,
        boardSlots, BoardLedger.Merge(boardSlots, boardMeta, boardLedger), boardMessage, storeReadSupported);
    private void SetBoardMessage(Func<string>? message) { boardMessage = message; PublishBoard(); }

    private bool StartStoreJob(StoreOp op, int slot, Action<bool, Func<string>>? done, int first = 0)
    {
        if (!StoreAvailable || storeJob is not null) return false;
        lock (wireSync)
        {
            if (pendingControlType != 0) return false;
            storeJob = new StoreJob(op, slot, done);
            if (op != StoreOp.Save) storeMetaChunks = [];
            // A save that carries a description sends it first (type 14, one chunk per frame); the Save frame follows the last chunk.
            storeRequestOp = op; storeRequestSlot = slot; storeRequestFirst = first; storePacket = []; storeResend = false; storeMetaNext = 0;
            pendingControlType = storeMetaChunks.Length > 0 ? (byte)14 : (byte)12;
            if (op == StoreOp.Query) boardSeen = 0;   // only a query that reports every slot completes the inventory
        }
        Append($"HOST archive request {op} slot={(slot < 0 ? "all" : slot.ToString())}");
        PublishBoard();
        return true;
    }

    private void FinishStoreJob(bool ok, Func<string> message)
    {
        if (storeJob is not { } job) return;
        storeJob = null;
        lock (wireSync) { if (pendingControlType is 12 or 14) pendingControlType = 0; storeMetaChunks = []; storeMetaNext = 0; }
        Append($"HOST archive {job.Op} slot={(job.Slot < 0 ? "all" : job.Slot.ToString())} {(ok ? "done" : "failed")}", ok ? null : LogLevel.Warning);   // the raw log stays raw; the reason goes to the panel
        job.Done?.Invoke(ok, message);
        // A path asked for while this read-only job ran starts now (once the inventory is complete again, if it was the inventory).
        // It waits for a COMPLETE inventory: a status that arrived without all the slot lines leaves it queued for the next read.
        if (wantedPathSlot is { } waiting && storeJob is null && (job.Op is StoreOp.Meta || job.Op is StoreOp.Query && boardKnown)) { wantedPathSlot = null; RequestBoardPath(waiting); }
        // A change on the board is read back, so the picture shown is the device's and not what was hoped for.  A FAILED save or erase
        // is read back too: it may have changed flash before it failed (erase clears block A before block B), or have finished with its
        // final status lost.  From the moment a save or erase ends, until the answer is complete, the old slot contents are not trusted:
        // nothing is saved, cleared or loaded from them (a stale entry could pick the wrong plan), and if the read-back is lost the
        // periodic synchronisation asks again.
        if (job.Op is StoreOp.Save or StoreOp.Erase)
        {
            boardKnown = false; boardSeen = 0;
            StartStoreJob(StoreOp.Query, -1, null);
        }
        PublishBoard();
    }

    // Reads the archive once the link is up and the machine is still, and again after the device lost the answer.
    private void SynchronizeBoard(DateTime now)
    {
        // A waiting path that no complete inventory ever came for is given up, with a reason, instead of leaving the panel "reading" for good.
        if (wantedPathSlot is { } stale && (now - wantedPathSince).TotalSeconds > 15)
        {
            wantedPathSlot = null; Func<string> gone = () => Lang.T("board.readFailed", stale + 1, Lang.T("board.err.noAnswer"));
            SetBoardMessage(gone); xyPreview.EndBoardPath(stale, gone);
        }
        if (!StoreAvailable || storeJob is not null || pendingBoardSave is not null || (now - lastBoardQueryAt).TotalSeconds < (boardKnown ? 0.3 : 3)) return;
        if (PointBusy || homing || keyboardMask != 0 || mouseMask != 0 || ptp is not { Active: 0 }) return;
        if (!boardKnown) { lastBoardQueryAt = now; StartStoreJob(StoreOp.Query, -1, null); return; }
        // Then the descriptions the slots carry, one slot per query (four at once would not fit the device's text queue), every valid slot:
        // nothing on the slot line says whether it has one, and a line that is lost must not hide a description.  Each is asked for once
        // per save counter, so a slot that cannot be read does not keep the link busy.
        if (storeMetaMax == 0) return;
        for (int i = 0; i < storeSlotCount; i++)
        {
            if (boardSlots[i] is not { Valid: true } slot || metaAsked[i] == slot.Seq) continue;
            StartMetaRead(i, slot, now);
            return;
        }
    }

    // One slot's description.  A read that failed (a refusal), or ended with its status but without the description (the long line was
    // lost), is tried again a few times, a little later; then the slot stays "saved elsewhere".  `then` runs when the read has ended,
    // whatever came of it (but not when STOP cancelled it).
    private bool StartMetaRead(int i, BoardSlot slot, DateTime now, Action? then = null)
    {
        if (metaFailSeq[i] != slot.Seq) { metaFailSeq[i] = slot.Seq; metaFailures[i] = 0; }
        lastBoardQueryAt = now; metaAsked[i] = slot.Seq; boardMeta[i] = null; metaEmpty[i] = false;
        int asked = i; uint askedSeq = slot.Seq;
        bool started = StartStoreJob(StoreOp.Meta, i, (ok, _) =>
        {
            bool got = boardMeta[asked] is { } meta && meta.Seq == askedSeq || metaEmpty[asked];   // an answer, even "none"
            if (!((ok && got) || metaFailures[asked] >= MetaRetries || metaAsked[asked] != askedSeq))
            {
                metaFailures[asked]++; metaAsked[asked] = null;
                // A path waiting for this description asks again at once, up to the same limit, instead of reading the nodes without it (the
                // saved link lengths could not be added to a view that is already built).
                if (then is not null && !cancellingStores && boardSlots[asked] is { Valid: true } again && again.Seq == askedSeq && StartMetaRead(asked, again, DateTime.UtcNow, then)) return;
                lastBoardQueryAt = DateTime.UtcNow.AddSeconds(2);
            }
            if (!cancellingStores) then?.Invoke();
        });
        if (!started) metaAsked[i] = null;   // never sent: ask again later
        return started;
    }
    private bool cancellingStores;   // STOP is ending the archive jobs that only read

    // Unanswered requests are repeated byte for byte (the device only repeats its receipt); a saving device is given time.
    private void PollStoreJob(DateTime now)
    {
        if (storeJob is not { } job) return;
        double quiet = (now - job.LastSeenAt).TotalSeconds;
        if (!job.Acked)
        {
            if ((now - job.RequestedAt).TotalSeconds > 8 || (job.Attempts >= 3 && quiet >= StoreAnswerSeconds * 3)) { FinishStoreJob(false, () => Lang.T("board.err.noAnswer")); return; }
            if (job.Sequence != 0 && job.Attempts < 3 && quiet >= StoreAnswerSeconds * job.Attempts)
            {
                lock (wireSync) { if (pendingControlType != 0) return; storeResend = true; pendingControlType = 12; }
                job.Attempts++;
                Append($"HOST archive retry attempt={job.Attempts}/3; identical packet", LogLevel.Warning);
            }
        }
        else if (quiet > (job.Op is StoreOp.Query or StoreOp.Meta or StoreOp.Nodes ? 3 : StoreWorkSeconds)) FinishStoreJob(false, () => Lang.T("board.err.stalled"));
    }

    private void ObserveStoreLine(string line)
    {
        if (line.StartsWith("TRAJ_SLOT ", StringComparison.Ordinal))
        {
            if (BoardSlot.Parse(line) is { } slot && slot.Slot < storeSlotCount)
            {
                // The save counter alone does not name a plan: erasing a slot and saving into it starts it at 1 again.  An emptied slot, or one
                // whose counter or plan changed, loses what was known about its description.
                if (!slot.Valid || boardSlots[slot.Slot] is { } before && (before.Seq != slot.Seq || before.Crc != slot.Crc)) ForgetMeta(slot.Slot);
                boardSlots[slot.Slot] = slot; boardSeen |= 1 << slot.Slot;
                if (boardSeen == (1 << storeSlotCount) - 1) boardKnown = true;
                if (storeJob is { } q) q.LastSeenAt = DateTime.UtcNow;
                PublishBoard();
            }
            else Append("HOST malformed or unexpected archive slot line", LogLevel.Warning);
            return;
        }
        if (line.StartsWith("TRAJ_NODES ", StringComparison.Ordinal)) { TakeNodes(line); return; }
        if (line.StartsWith("TRAJ_META ", StringComparison.Ordinal))
        {
            if (BoardMeta.Parse(line) is { } meta && storeJob is { Op: StoreOp.Meta } asking && asking.Slot == meta.Slot) { boardMeta[meta.Slot] = meta; asking.LastSeenAt = DateTime.UtcNow; PublishBoard(); }
            else if (storeJob is { Op: StoreOp.Meta } none && line == $"TRAJ_META slot={none.Slot} len=0") { metaEmpty[none.Slot] = true; none.LastSeenAt = DateTime.UtcNow; }
            else if (line.StartsWith("TRAJ_META refused:", StringComparison.Ordinal) && storeJob is { Op: StoreOp.Save, Acked: false }) FinishStoreJob(false, () => Lang.T("board.err.refused", line[18..].Trim()));
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
                StoreOp.Query or StoreOp.Meta or StoreOp.Nodes => true,
                StoreOp.Save => status.Last == "saved" && status.LastSlot == job.Slot,
                _ => status.Last == "erased" && status.LastSlot == job.Slot,
            };
            FinishStoreJob(ok, () => ok ? "" : Lang.T("board.err.device", status.Last));
        }
        else if (line.StartsWith("TRAJ_STORE refused:", StringComparison.Ordinal) && !job.Acked)
            FinishStoreJob(false, () => Lang.T("board.err.refused", line[19..].Trim()));
    }

    // ---- what the panel asks for ----------------------------------------------------------------------------------------
    private void HookBoardPanel()
    {
        xyPreview.SaveToBoardRequested += (plan, slot) => SafeAction(() => RequestSaveToBoard(plan, slot));
        xyPreview.EraseBoardRequested += slot => SafeAction(() => RequestEraseBoardSlot(slot));
        xyPreview.BoardPathRequested += slot => SafeAction(() => RequestBoardPath(slot));
        boardLedger = BoardLedger.Load();
    }

    // The plan the device holds is what gets saved.  A figure that is not held yet is uploaded first, then saved: one click.
    // `settings` is the description of the figure AT THE MOMENT it was asked for: a save held back or uploading first keeps it, so the
    // ledger never labels this plan with settings typed meanwhile.
    private void RequestSaveToBoard(ExecutionTrajectory plan, int slot, string? settings = null)
    {
        settings ??= xyPreview.SettingsDescription;
        if (slot < 0 || slot >= storeSlotCount) return;
        if (!StoreAvailable || !boardKnown || storeJob is not null || pendingBoardSave is not null || PointBusy) { SetBoardMessage(() => Lang.T("board.err.busy")); return; }
        if (uploaded is { } held && held.Nodes.SequenceEqual(plan.Nodes)) { deferredBoardSave = null; BeginBoardSave(plan.Nodes, slot, settings); return; }
        // A figure with other speeds makes the panel re-send them first; the upload has to wait until the device confirmed them.
        if (!LimitsReady) { deferredBoardSave = (plan, slot, DateTime.UtcNow, settings); SetBoardMessage(() => Lang.T("board.waitSync", slot + 1)); return; }
        deferredBoardSave = null;
        // The marker comes AFTER the upload has started: it counts as busy, and RequestTrajectory refuses to start while busy.
        RequestTrajectory(plan, allowBoard: false);
        if (uploading is null) { SetBoardMessage(() => Lang.T("board.err.needStart")); return; }
        pendingBoardSave = (plan.Nodes, slot, settings);
        SetBoardMessage(() => Lang.T("board.uploadThenSave", slot + 1));
    }

    // The save that was waiting for the speed synchronisation goes ahead once the device confirmed it, or is dropped after 15 s.
    private void RetryDeferredBoardSave(DateTime now)
    {
        if (deferredBoardSave is not { } deferred) return;
        if ((now - deferred.Since).TotalSeconds > 15) { deferredBoardSave = null; SetBoardMessage(() => Lang.T("board.err.needStart")); return; }
        if (!LimitsReady || PointBusyCore) return;   // PointBusy itself includes this held save
        deferredBoardSave = null;
        RequestSaveToBoard(deferred.Plan, deferred.Slot, deferred.Settings);
    }

    private void BeginBoardSave(ExecutionNode[] nodes, int slot, string settings)
    {
        uint crc = TrajectoryArchive.PlanCrc(nodes);
        string? description = storeMetaMax > 0 ? BoardMeta.TextFor(settings, storeMetaMax) : null;
        bool dropped = storeMetaMax > 0 && description is null;   // too long or not plain text: saved without one, and the operator is told
        storeMetaChunks = description is not null ? BoardMeta.Chunks(description, crc) : [];
        if (!StartStoreJob(StoreOp.Save, slot, (ok, message) =>
        {
            if (ok) { RememberBoard(slot, crc, settings); SetBoardMessage(() => Lang.T(dropped ? "board.savedNoDescription" : "board.saved", slot + 1)); }
            else SetBoardMessage(() => Lang.T("board.saveFailed", slot + 1, message()));
        })) { storeMetaChunks = []; SetBoardMessage(() => Lang.T("board.err.busy")); return; }
        SetBoardMessage(() => Lang.T("board.saving", slot + 1));
    }

    private void RememberBoard(int slot, uint crc, string? settings)
    {
        try { BoardLedger.Set(slot, crc, settings); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { Append("HOST could not write the board ledger: " + ex.Message, LogLevel.Warning); }
        boardLedger = BoardLedger.Load();
    }

    // ---- reading a slot's nodes back ------------------------------------------------------------------------------------
    // What the board really holds, for the preview: the nodes come back forty at a time (one type 12 op 5 request each, retried like any
    // other), and only a complete stream that matches the slot's CRC, point count and duration is shown.  Nothing moves.
    private int? wantedPathSlot;   // a path asked for while a read-only archive job (the scan) was running: started when that ends, not dropped
    private DateTime wantedPathSince;

    private void RequestBoardPath(int slot)
    {
        if (slot < 0 || slot >= storeSlotCount) return;
        if (!StoreAvailable || !storeReadSupported || nodeRead is not null || PointBusy) { RefuseBoardPath(slot); return; }
        if (storeJob is { Op: StoreOp.Query or StoreOp.Meta }) { wantedPathSlot = slot; wantedPathSince = DateTime.UtcNow; return; }   // the scan in flight: start it when that ends
        if (!boardKnown || storeJob is not null || boardSlots[slot] is not { Valid: true } held) { RefuseBoardPath(slot); return; }
        // The slot's description (the link lengths it was saved with) comes first: the periodic read may not have reached this slot yet, and
        // the node reads would keep it out until the whole path was in, so the view would be drawn with this computer's lengths.
        if (storeMetaMax > 0 && metaAsked[slot] != held.Seq && StartMetaRead(slot, held, DateTime.UtcNow, () => BeginNodeRead(slot, held.Seq))) return;
        BeginNodeRead(slot, held.Seq);
    }

    private void RefuseBoardPath(int slot)
    {
        Func<string> busy = () => Lang.T("board.err.busy");
        SetBoardMessage(busy);
        xyPreview.EndBoardPath(slot, busy);
    }

    private void BeginNodeRead(int slot, uint seq)
    {
        if (boardSlots[slot] is not { Valid: true } held || held.Seq != seq || !StoreAvailable || !storeReadSupported || !boardKnown || storeJob is not null || nodeRead is not null || PointBusy)
        { RefuseBoardPath(slot); return; }
        string? settings = BoardLedger.For(held, BoardLedger.Merge(boardSlots, boardMeta, boardLedger))?.Settings;
        nodeRead = new NodeRead(slot, held.Seq, held.Nodes, held.DurationMs, held.Crc, settings);
        ReadNextNodes();
    }

    private void ReadNextNodes()
    {
        if (nodeRead is not { } read) return;
        read.AskedAt = read.Next;
        SetBoardMessage(() => Lang.T("board.readingPath", read.Slot + 1, read.Next, read.Total));
        if (!StartStoreJob(StoreOp.Nodes, read.Slot, (ok, message) => NodesJobEnded(read, ok, message), read.Next))
            EndNodeRead(read, () => Lang.T("board.readFailed", read.Slot + 1, Lang.T("board.err.busy")));
    }

    private void EndNodeRead(NodeRead read, Func<string> message)
    {
        if (ReferenceEquals(nodeRead, read)) nodeRead = null;
        SetBoardMessage(message);
        xyPreview.EndBoardPath(read.Slot, message);   // the panel's note must not keep saying "reading" for a read that is over
    }

    // One run of nodes: taken only when it is the next one of the slot being read, with the same save counter and size.
    private void TakeNodes(string line)
    {
        if (nodeRead is not { } read || storeJob is not { Op: StoreOp.Nodes } job || job.Slot != read.Slot) return;
        if (BoardNodes.Parse(line) is not { } run) { if (!line.StartsWith($"TRAJ_NODES slot={read.Slot} count=0", StringComparison.Ordinal)) Append("HOST malformed archive node line", LogLevel.Warning); return; }
        if (run.Slot != read.Slot) return;
        if (run.Seq != read.Seq || run.Total != read.Total)
        {
            boardKnown = false; boardSeen = 0;   // the slot was written meanwhile: read the inventory again
            EndNodeRead(read, () => Lang.T("board.readChanged", read.Slot + 1));
            return;
        }
        if (run.First != read.Next) return;      // the repeated answer to a retried request: already taken
        run.Bytes.CopyTo(read.Bytes, run.First * 6); read.Next += run.Count; read.Retries = 0; job.LastSeenAt = DateTime.UtcNow;
    }

    private void NodesJobEnded(NodeRead read, bool ok, Func<string> message)
    {
        if (!ReferenceEquals(nodeRead, read)) return;
        if (!ok) { EndNodeRead(read, () => Lang.T("board.readFailed", read.Slot + 1, message())); return; }
        if (read.Next == read.AskedAt)
        {
            // The long line may simply have been lost while the short status got through (the device drops a message that does not fit its
            // text queue whole): ask for the same nodes again, a few times, before giving the whole read up.
            if (++read.Retries <= NodeRetries) { ReadNextNodes(); return; }
            EndNodeRead(read, () => Lang.T("board.readFailed", read.Slot + 1, Lang.T("board.err.noNodes")));
            return;
        }
        if (read.Next < read.Total) { ReadNextNodes(); return; }
        var nodes = TrajectoryArchive.Nodes(read.Bytes);
        if (TrajectoryArchive.Crc32(read.Bytes) != read.Crc || nodes[0].Milliseconds != 0 || nodes.Sum(n => (int)n.Milliseconds) != read.DurationMs)
        { EndNodeRead(read, () => Lang.T("board.readBad", read.Slot + 1)); return; }
        nodeRead = null;
        SetBoardMessage(() => Lang.T("board.readDone", read.Slot + 1));
        xyPreview.ShowBoardPath(read.Slot, nodes, read.Settings);
    }

    private void RequestEraseBoardSlot(int slot)
    {
        if (slot < 0 || slot >= storeSlotCount) return;
        if (!StoreAvailable || storeJob is not null || PointBusy) { SetBoardMessage(() => Lang.T("board.err.busy")); return; }
        if (StartStoreJob(StoreOp.Erase, slot, (ok, message) =>
        {
            if (ok) { RememberBoard(slot, 0, null); SetBoardMessage(() => Lang.T("board.erased", slot + 1)); }
            else SetBoardMessage(() => Lang.T("board.eraseFailed", slot + 1, message()));
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
    // The device says which plan a load put in its RAM: slot, save counter, size and duration.  Another host may have rewritten the slot
    // since the plan was previewed, and the load takes what is in it NOW, so the acknowledgment alone (a node count) proves nothing.
    private (int Slot, uint Seq, int Nodes, int DurationMs)? loadedSlot;
    private (uint Seq, int DurationMs)? loadingExpect;
    private static readonly System.Text.RegularExpressions.Regex LoadPattern = new(@"^TRAJ_LOAD slot=(\d+) seq=(\d+) nodes=(\d+) duration_ms=(\d+)$", System.Text.RegularExpressions.RegexOptions.CultureInvariant);
    private void ObserveLoadLine(string line)
    {
        var m = LoadPattern.Match(line);
        loadedSlot = m.Success && int.TryParse(m.Groups[1].Value, out int slot) && uint.TryParse(m.Groups[2].Value, out uint seq) && int.TryParse(m.Groups[3].Value, out int nodes) &&
            int.TryParse(m.Groups[4].Value, out int duration) ? (slot, seq, nodes, duration) : null;
    }

    private bool CompleteBoardLoad(uint seq, uint epoch, uint mode, uint count, uint total)
    {
        if (loadingSlot is not { } slot || uploading is not { } plan || seq != pendingPoint || epoch != trajectoryEpoch) return false;
        if (mode != 2 || count != total) { Append("HOST archive load acknowledgement mismatch", LogLevel.Error); Stop(); return true; }
        // Only the plan that was previewed may be held: the load line must name this slot, the save counter and the duration it had when the
        // plan was matched to it, and the device must hold as many nodes as the plan has.  Anything else (the slot rewritten meanwhile,
        // possibly with another size, or the line lost) is not trusted: the plan is uploaded instead, which replaces whatever the load put
        // in the device.
        if (loadedSlot is not { } got || got.Slot != slot || loadingExpect is not { } want || got.Seq != want.Seq || got.DurationMs != want.DurationMs ||
            got.Nodes != plan.Nodes.Length || count != plan.Nodes.Length)
        {
            FallBackToUpload("it is not the plan that was previewed: the slot was rewritten meanwhile, or its load line was lost", "board.loadChanged");
            return true;
        }
        pendingPoint = 0; lock (wireSync) { pointWire.Clear(); }
        uploading = null; uploaded = plan; uploadedRan = false; loadingSlot = null;
        Append($"HOST plan loaded from board slot {slot}; waiting for the operator to start it (no automatic run)");
        SetBoardMessage(() => Lang.T("board.loaded", slot + 1));
        ShowDeviceState(); UpdateControls();
        return true;
    }

    // The device would not take the stored plan (changed limits or speeds, damaged slot): fall back to the normal upload of the
    // same plan.  The refused load did not use up a device sequence number, so the next frames simply continue from ours.
    private void FallBackToUpload(string reason, string message = "board.loadFallback")
    {
        if (loadingSlot is not { } slot || uploading is null) return;
        Append($"HOST board slot {slot} could not be loaded ({reason}); uploading the plan instead", LogLevel.Warning);
        loadingSlot = null; pendingPoint = 0; lock (wireSync) { pointWire.Clear(); }
        boardKnown = false; boardSeen = 0; uploadNext = 0;
        SetBoardMessage(() => Lang.T(message, slot + 1));
        SendNextTrajectoryPacket();
    }

    // The plan is on the device and a save was waiting for it.
    private void ContinueBoardSave(ExecutionTrajectory plan)
    {
        if (pendingBoardSave is not { } pending || !pending.Nodes.SequenceEqual(plan.Nodes)) return;
        pendingBoardSave = null;
        BeginBoardSave(plan.Nodes, pending.Slot, pending.Settings);
    }

    // The archive's message handling, checked against the lines the firmware prints.
    internal static void StoreSelfTest()
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
            // The LINK line repeats every 500 ms: with nothing changed the panel's controls are not rebuilt.
            int rebuilds = w.xyPreview.BoardRebuilds;
            w.HandleLine(Link(" traj_store=4 telem_hz=0")); w.HandleLine(Link(" traj_store=4 telem_hz=0"));
            Check(w.xyPreview.BoardRebuilds == rebuilds, "a repeated LINK line rebuilt the board controls");
            // Saving waits for the complete inventory, and for the device to confirm new speeds: a figure with other speeds makes the panel
            // re-send them, and the upload that follows is held back instead of being dropped.
            w.xyPreview.SelectShapeForSelfTest(3); w.xyPreview.LoadExampleForSelfTest();
            var shown = w.xyPreview.Plan ?? throw new InvalidOperationException("Archive: the example figure has no plan");
            Check(!w.LimitsReady, "the fixture was expected to be waiting for the device's ranges");
            w.RequestSaveToBoard(shown, 0);
            Check(w.deferredBoardSave is { Slot: 0 } && w.uploading is null && w.pendingBoardSave is null && w.storeJob is null, "a save asked during speed synchronisation was not held back");
            // While it waits, editing is locked (the plan and the settings it will save must not change), and it keeps the settings of the moment.
            string heldSettings = w.deferredBoardSave!.Value.Settings;
            Check(w.PointBusy && heldSettings.Contains("shape=3"), "a held-back save did not lock editing or capture its settings");
            w.xyPreview.SelectShapeForSelfTest(2);
            Check(w.deferredBoardSave!.Value.Settings == heldSettings, "the held settings changed with the panel");
            w.xyPreview.SelectShapeForSelfTest(3);
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
            Check(done && ok && w.storeJob is { Op: StoreOp.Query } && !w.boardKnown, "the end of the save was not taken and read back with the inventory distrusted meanwhile");
            // A failed save, a refusal, silence: each ends the job as failed; a refusal needs no waiting.
            w.FinishStoreJob(true, () => "");
            done = false; Check(w.StartStoreJob(StoreOp.Save, 2, (success, _) => { done = true; ok = success; }), "save 2 queued"); Next(12); uint failSeq = w.storeJob!.Sequence;
            w.HandleLine($"TRAJ_STORE state=idle slot=2 rows=3/12 seq={failSeq} last=failed last_slot=2 row_max_us=0");
            Check(done && !ok, "a failed save was reported as done");
            Check(w.storeJob is { Op: StoreOp.Query } && !w.boardKnown, "a failed save did not trigger a read-back and distrust of the inventory");
            w.FinishStoreJob(true, () => "");
            done = false; Check(w.StartStoreJob(StoreOp.Erase, 1, (success, _) => { done = true; ok = success; }), "erase queued");
            w.HandleLine("TRAJ_STORE refused: stage not idle");
            Check(done && !ok && w.storeJob is { Op: StoreOp.Query }, "a refusal did not end the job (and read the board back)");
            w.FinishStoreJob(true, () => "");
            done = false; Check(w.StartStoreJob(StoreOp.Erase, 1, (success, _) => { done = true; ok = success; }), "erase queued again"); Next(12);
            for (int later = 30; later < 40 && !done; later++) w.PollStoreJob(DateTime.UtcNow.AddSeconds(later));   // stops at the first poll that gives up
            Check(done && !ok && w.storeJob is { Op: StoreOp.Query }, "silence did not end the job (and read the board back)");
            w.FinishStoreJob(true, () => "");
            // Disconnecting forgets the board; a stop forgets a pending save.
            w.pendingBoardSave = (nodes, 0, "shape=3");
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

    // The description kept with a plan on the board: asked for slot by slot once the inventory is complete, sent in front of a save
    // (type 14 frames, then the Save frame), refused saves, and what the panel shows for a slot another computer saved.
    internal static void BoardMetaFlowSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Archive description: " + reason); }
        static int Type(byte[] f) => ((f[1] >> 1) & 15) | (((f[2] >> 1) & 15) << 4);
        static int Field(byte[] f, int i) => ((f[1 + 2 * i] >> 1) & 15) | (((f[2 + 2 * i] >> 1) & 15) << 4);
        string ledger = Path.Combine(Path.GetTempPath(), "arm-board-meta-" + Guid.NewGuid().ToString("N") + ".json");
        BoardLedger.PathOverride = ledger;
        bool language = Lang.IsEn; Lang.Set(false, false);
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false };
        var frames = new List<byte[]>();
        void Write(byte[] data, int offset, int count) { if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data); }
        void Pump() { for (int i = 0; i < 600 && w.pendingControlType != 0; i++) { w.PublishHeartbeatLease(); w.WireWriterStep(Write); } }
        List<byte[]> Sent() => frames.Where(f => f.Length == 36 && Type(f) is 12 or 14).ToList();
        string Link(string extra) => $"LINK uart=hw baud=115200 fw=stage2-v11-test short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 traj_repeat=1 fault_snapshot=1{extra} proto={DeviceContract.ProtocolVersion}";
        try
        {
            const string settings = "shape=2; branch=0; L1=180; L2=176.38; x=340; y=0; a=10; b=6; feed=1; v1=6.5; v2=3; a1=10; a2=6; min1=-1; max1=1; emin=-1; emax=1; c=; rot=20; scale=1.2; dir=cw; custom_chars=0";
            string text = BoardMeta.TextFor(settings) ?? throw new InvalidOperationException("Archive description: the fixture settings were not accepted");
            var nodes = new[] { new ExecutionNode(new JointPose(0, 0), 0), new ExecutionNode(new JointPose(0.5, -0.25), 100), new ExecutionNode(new JointPose(1, -0.5), 100), new ExecutionNode(new JointPose(1, -0.5), 50) };
            uint planCrc = TrajectoryArchive.PlanCrc(nodes);
            w.HandleLine("STATE fw=stage2-v11 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=3 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            // A firmware that archives but keeps no descriptions (traj_meta absent or 0) is never asked for one and is sent none.
            w.HandleLine(Link(" traj_store=4 telem_hz=0"));
            Check(w.storeSlotCount == 4 && w.storeMetaMax == 0, "a firmware without descriptions was assumed to have them");
            w.HandleLine(Link(" traj_meta=256 traj_store=4 telem_hz=0"));
            Check(w.storeMetaMax == 256, "the description size was not read");
            Pump();   // the stream-rate request that the LINK line triggered goes out first
            // The inventory first; the slot that carries a description is then asked for, once.
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(10));
            Check(w.storeJob is { Op: StoreOp.Query }, "no inventory query");
            Pump(); uint querySeq = w.storeJob!.Sequence;
            w.HandleLine("TRAJ_SLOT slot=0 valid=0");
            w.HandleLine($"TRAJ_SLOT slot=1 valid=1 seq=2 nodes=4 duration_ms=250 q1=0 q2=0 crc={planCrc:X8}");
            w.HandleLine("TRAJ_SLOT slot=2 valid=1 seq=1 nodes=9 duration_ms=900 q1=0 q2=0 crc=11111111");
            w.HandleLine("TRAJ_SLOT slot=3 valid=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={querySeq} last=none last_slot=0 row_max_us=0");
            Check(w.boardKnown && w.storeJob is null, "the inventory was not taken");
            frames.Clear();
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(20));
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 } && w.pendingControlType == 12, "the first valid slot was not asked for its description");
            Pump();
            var ask = Sent();
            Check(ask.Count == 1 && Field(ask[0], 0) == 12 && (short)(Field(ask[0], 9) | Field(ask[0], 10) << 8) == 1 && Field(ask[0], 11) == 4, "description query frame (slot 1, op 4)");
            uint metaSeq = w.storeJob!.Sequence;
            // A line for another slot is not taken; the right one is; the status then ends the job without a re-read of the inventory.
            byte[] body = System.Text.Encoding.ASCII.GetBytes(text);
            string Line(int slot) => $"TRAJ_META slot={slot} seq=2 len={body.Length} crc={TrajectoryArchive.Crc32(body):X8} hex={Convert.ToHexString(body)}";
            w.HandleLine(Line(2));
            Check(w.boardMeta.All(m => m is null), "a description for a slot that was not asked for was taken");
            w.HandleLine(Line(1));
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={metaSeq} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is null && w.boardKnown && w.boardMeta[1] is { Seq: 2 } && w.boardMeta[1]!.Text == text, "the description was not kept");
            Check(w.xyPreview.BoardNoteText.Contains("槽2：矩形") && w.xyPreview.BoardNoteText.Contains("槽3：别处存的图形"), "the panel does not name the figures from the board's own descriptions: " + w.xyPreview.BoardNoteText);
            // The next valid slot is asked too: nothing on a slot line says whether it carries a description.  It carries none.
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(30));
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 2 }, "a valid slot was not asked for its description");
            Pump(); w.HandleLine("TRAJ_META slot=2 len=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is null && w.boardMeta[2] is null, "a slot without a description left a job behind");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(35));
            Check(w.storeJob is null, "a description was asked for twice");
            // The slot is saved again (its counter moves): the old description is not trusted and is asked for again.
            w.HandleLine($"TRAJ_SLOT slot=1 valid=1 seq=3 nodes=4 duration_ms=250 q1=0 q2=0 crc={planCrc:X8}");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(40));
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 } && w.boardMeta[1] is null, "a slot saved again was not asked again");
            // A read whose long line was lost (the status arrives, the description does not) is not a read: tried again like a failed one.
            Pump(); w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is null && w.boardMeta[1] is null, "a read without its description was taken for done");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(45));
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 }, "a read that ended without its description was not tried again");
            // A read that fails (a refusal, a timeout) is tried again, a few times, and then given up: three retries after the first ask.
            for (int attempt = 1; attempt <= 2; attempt++)
            {
                w.FinishStoreJob(false, () => "no answer");
                Check(w.storeJob is null, "a failed description read left a job behind");
                w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(45 + attempt * 5));
                Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 }, $"a failed description read was not tried again (retry {attempt + 1})");
            }
            w.FinishStoreJob(false, () => "no answer");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(100));
            Check(w.storeJob is null, "a description read that kept failing was asked for forever");
            // The slot is emptied and written anew: its save counter starts at 1 again (here it simply repeats), but nothing known about the
            // old plan's description carries over, and the new one is read.
            w.HandleLine("TRAJ_SLOT slot=1 valid=0");
            Check(w.boardMeta[1] is null && w.metaAsked[1] is null && w.metaFailures[1] == 0, "an emptied slot kept what was known about its description");
            w.HandleLine($"TRAJ_SLOT slot=1 valid=1 seq=3 nodes=4 duration_ms=250 q1=0 q2=0 crc={planCrc:X8}");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(110));
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 }, "a slot written anew (same save counter) was not asked for its description");
            Pump();
            w.HandleLine($"TRAJ_META slot=1 seq=3 len={body.Length} crc={TrajectoryArchive.Crc32(body):X8} hex={Convert.ToHexString(body)}");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.storeJob is null && w.boardMeta[1] is { Seq: 3 } && w.boardMeta[1]!.Text == text, "the description of the new plan was not read");
            // The same counter with another plan, without the empty state ever being seen, also drops it.
            w.HandleLine("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=4 duration_ms=250 q1=0 q2=0 crc=22222222");
            Check(w.boardMeta[1] is null && w.metaAsked[1] is null, "a slot that came back as another plan kept the old description");
            w.HandleLine($"TRAJ_SLOT slot=1 valid=1 seq=3 nodes=4 duration_ms=250 q1=0 q2=0 crc={planCrc:X8}");

            // A save carries its description: chunks first (type 14, consecutive numbers), the Save frame after the last one.
            frames.Clear();
            Check(w.StoreAvailable && w.boardKnown, "the archive is not available for the save");
            w.BeginBoardSave(nodes, 0, settings);
            Check(w.storeJob is { Op: StoreOp.Save, Slot: 0 } && w.pendingControlType == 14, "the save did not start with its description");
            Pump();
            var sent = Sent();
            var expected = BoardMeta.Chunks(text, planCrc);
            Check(sent.Count == expected.Length + 1 && sent.Take(expected.Length).All(f => Type(f) == 14) && Type(sent[^1]) == 12, $"frames: {string.Join(",", sent.Select(Type))}");
            for (int i = 0; i < expected.Length; i++)
            {
                var f = sent[i];
                Check(Field(f, 13) == i && Field(f, 14) == 0 && Enumerable.Range(0, 4).Select(k => (byte)Field(f, 9 + k)).SequenceEqual(expected[i]), $"chunk {i} payload");
            }
            Check(Enumerable.Range(1, sent.Count - 1).All(i => Seq(sent[i]) > Seq(sent[i - 1])) && w.storeJob!.Sequence == Seq(sent[^1]) && w.pendingControlType == 0, "sequence numbers or the end of the chunk run");
            Check((short)(Field(sent[^1], 9) | Field(sent[^1], 10) << 8) == 0 && Field(sent[^1], 11) == 2, "the Save frame (slot 0, op 2)");
            // The device saved it: the job ends and the board is read back.
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=1/1 seq={w.storeJob!.Sequence} last=saved last_slot=0 row_max_us=0");
            Check(w.storeJob is { Op: StoreOp.Query } && w.storeMetaChunks.Length == 0 && w.boardMessage?.Invoke().Contains("已存到板上槽1") == true, "the end of the save");
            w.FinishStoreJob(true, () => "");
            // The device refuses (a chunk was lost): the job fails with its reason and the read-back follows.
            w.BeginBoardSave(nodes, 0, settings);
            Pump();
            w.HandleLine("TRAJ_STORE refused: description incomplete");
            Check(w.storeJob is { Op: StoreOp.Query } && w.boardMessage?.Invoke().Contains("失败") == true && w.boardMessage.Invoke().Contains("description incomplete"), "a refused save was not reported as failed: " + w.boardMessage?.Invoke());
            w.FinishStoreJob(true, () => "");
            // A refusal while the chunks are still going out stops them.
            w.BeginBoardSave(nodes, 0, settings);
            Check(w.pendingControlType == 14, "the second save did not start with chunks");
            w.HandleLine("TRAJ_META refused: arguments");
            Check(w.storeJob is { Op: StoreOp.Query } && w.storeMetaChunks.Length == 0 && w.pendingControlType == 12, "chunks kept going after the device refused them");
            w.FinishStoreJob(true, () => "");
            // STOP while the description is still going out: the request never left, nothing will ever answer it, so it ends at once instead of
            // keeping the archive busy (and editing locked) until it times out.
            w.BeginBoardSave(nodes, 0, settings);
            Check(w.storeJob is { Op: StoreOp.Save, Sequence: 0 } && w.pendingControlType == 14, "the save did not start with chunks");
            w.ResetControlTransport();
            Check(w.storeJob is not { Op: StoreOp.Save } && w.storeMetaChunks.Length == 0 && w.pendingControlType != 14 && !w.StoreBusy && w.boardMessage?.Invoke().Contains("总停止") == true,
                "STOP left the save waiting: " + w.boardMessage?.Invoke());
            if (w.storeJob is not null) w.FinishStoreJob(true, () => "");
            // A description that does not fit what the firmware keeps is not sent: the save goes ahead without one.
            w.storeMetaMax = 20;
            w.BeginBoardSave(nodes, 0, settings);
            Check(w.storeJob is { Op: StoreOp.Save } && w.pendingControlType == 12, "an oversized description was sent");
            Pump();
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=1/1 seq={w.storeJob!.Sequence} last=saved last_slot=0 row_max_us=0");
            Check(w.boardMessage?.Invoke().Contains("没有存描述") == true, "the operator was not told the description was left out: " + w.boardMessage?.Invoke());
            w.ResetBoard();
            Check(w.pendingControlType == 0 && w.boardMeta.All(m => m is null) && w.storeMetaMax == 0, "the description state survived a reset");
        }
        finally { w.Close(); Lang.Set(language, false); BoardLedger.PathOverride = null; if (File.Exists(ledger)) File.Delete(ledger); }
        static uint Seq(byte[] f) => (uint)(Field(f, 1) | Field(f, 2) << 8 | Field(f, 3) << 16 | Field(f, 4) << 24);
    }

    // Reading a slot's plan back to show it: one request per forty nodes, only a complete stream that matches the slot is shown, and a
    // slot rewritten meanwhile, a damaged stream or a silent device end the read with a reason.  Driven by clicking the slot's row.
    internal static void BoardNodesFlowSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Archive nodes: " + reason); }
        static int Type(byte[] f) => ((f[1] >> 1) & 15) | (((f[2] >> 1) & 15) << 4);
        static int Field(byte[] f, int i) => ((f[1 + 2 * i] >> 1) & 15) | (((f[2 + 2 * i] >> 1) & 15) << 4);
        string ledger = Path.Combine(Path.GetTempPath(), "arm-board-nodes-" + Guid.NewGuid().ToString("N") + ".json");
        BoardLedger.PathOverride = ledger;
        bool language = Lang.IsEn; Lang.Set(false, false);
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false };
        var frames = new List<byte[]>();
        void Write(byte[] data, int offset, int count) { if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data); }
        void Pump() { for (int i = 0; i < 600 && w.pendingControlType != 0; i++) { w.PublishHeartbeatLease(); w.WireWriterStep(Write); } }
        string Link(string extra) => $"LINK uart=hw baud=115200 fw=stage2-v11-test short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 traj_repeat=1 fault_snapshot=1{extra} proto={DeviceContract.ProtocolVersion}";
        try
        {
            // A plan of 100 nodes: three runs of forty, forty and twenty.
            var nodes = Enumerable.Range(0, 100).Select(i => new ExecutionNode(new JointPose(Math.Round(10 * Math.Sin(i / 9.0), 2), Math.Round(30 + i * 0.2, 2)), (ushort)(i == 0 ? 0 : 100))).ToArray();
            byte[] stream = TrajectoryArchive.Stream(nodes); uint crc = TrajectoryArchive.PlanCrc(nodes); int total = nodes.Length, ms = nodes.Sum(n => (int)n.Milliseconds);
            string Run(int first, uint seq = 2, byte[]? bytes = null)
            {
                int count = Math.Min(BoardNodes.MaxPerLine, total - first);
                return $"TRAJ_NODES slot=1 seq={seq} total={total} first={first} count={count} hex={Convert.ToHexString((bytes ?? stream).AsSpan(first * 6, count * 6))}";
            }
            w.HandleLine("STATE fw=stage2-v11 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=3 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            // Without the capability a slot nobody described is not a row, and no read is started.
            w.HandleLine(Link(" traj_store=4 telem_hz=0"));
            Check(w.storeSlotCount == 4 && !w.storeReadSupported, "a firmware that cannot send nodes was assumed to");
            w.HandleLine(Link(" traj_meta=256 traj_read=1 traj_store=4 telem_hz=0"));
            Check(w.storeReadSupported, "the capability was not read");
            Check(w.xyPreview.BoardCanRead, "a capability that arrived on a repeated LINK line (same slot count) did not reach the panel");
            Pump();
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(10)); Pump(); uint query = w.storeJob!.Sequence;
            w.HandleLine("TRAJ_SLOT slot=0 valid=0");
            w.HandleLine($"TRAJ_SLOT slot=1 valid=1 seq=2 nodes={total} duration_ms={ms} q1=0 q2=3000 crc={crc:X8}");
            w.HandleLine("TRAJ_SLOT slot=2 valid=0"); w.HandleLine("TRAJ_SLOT slot=3 valid=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={query} last=none last_slot=0 row_max_us=0");
            Check(w.boardKnown && w.storeJob is null, "the inventory was not taken");
            Check(w.xyPreview.RecentListText.Contains("槽2：别处存的图形"), "the undescribed slot is not a row: " + w.xyPreview.RecentListText);
            void Click() => w.xyPreview.BoardRowLoadButton(0)!.RaiseEvent(new System.Windows.RoutedEventArgs(System.Windows.Controls.Primitives.ButtonBase.ClickEvent));
            // A request for the first forty; the answer is taken, the status ends the job, the next forty are asked for.
            frames.Clear(); Click();
            // The periodic read has not reached the slot yet: its description (the link lengths it was saved with) is read first, so the view
            // is not drawn with this computer's lengths for want of it.
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 } && w.nodeRead is null, "the slot's description was not read before its nodes");
            Pump();
            byte[] saved = System.Text.Encoding.ASCII.GetBytes("shape=3;L1=181.5;L2=176.38");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");   // the long description line was lost
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 } && w.nodeRead is null, "a lost description line was not retried before the nodes were read");
            Pump();
            w.HandleLine($"TRAJ_META slot=1 seq=2 len={saved.Length} crc={TrajectoryArchive.Crc32(saved):X8} hex={Convert.ToHexString(saved)}");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            frames.Clear();
            Check(w.nodeRead is { Slot: 1, Next: 0 } && w.storeJob is { Op: StoreOp.Nodes, Slot: 1 } && w.pendingControlType == 12, "the click did not start a read");
            Pump();
            var f0 = frames.Last(f => f.Length == 36 && Type(f) == 12);
            Check((short)(Field(f0, 9) | Field(f0, 10) << 8) == 1 && Field(f0, 11) == 5 && (Field(f0, 13) | Field(f0, 14) << 8) == 0, "first request (slot 1, op 5, from node 0)");
            Check(w.PointBusy, "the arm is not locked while the nodes are read");
            // The nodes line is lost while the short status gets through: the same nodes are asked for again.
            w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.nodeRead is { Next: 0 } && w.storeJob is { Op: StoreOp.Nodes } && w.pendingControlType == 12, "a node request that got only its status was not asked again");
            Pump();
            w.HandleLine(Run(0)); w.HandleLine(Run(0));   // a retried request is answered twice: counted once
            Check(w.nodeRead!.Next == 40, "a repeated run was counted twice");
            w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.nodeRead is { Next: 40 } && w.storeJob is { Op: StoreOp.Nodes } && w.pendingControlType == 12 && w.boardMessage?.Invoke().Contains("40/100") == true, "the second request did not follow: " + w.boardMessage?.Invoke());
            frames.Clear(); Pump();
            var f1 = frames.Last(f => f.Length == 36 && Type(f) == 12);
            Check((Field(f1, 13) | Field(f1, 14) << 8) == 40, "second request asks from node 40");
            w.HandleLine(Run(40));
            w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Pump(); w.HandleLine(Run(80));
            w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.nodeRead is null && w.storeJob is null && w.xyPreview.ShowsBoardPath && Math.Abs(w.xyPreview.BoardPathDuration - ms / 1000.0) < 1e-9 && w.boardMessage?.Invoke().Contains("已读完板上槽2") == true,
                "the complete plan was not shown: " + w.boardMessage?.Invoke());
            Check(!w.PointBusy, "the arm stayed locked after the read");
            Check(w.xyPreview.ResultText.Contains("181.5"), "the view was not drawn with the link lengths the slot was saved with: " + w.xyPreview.ResultText);

            // A stream that does not match the slot's CRC is not shown.
            var damaged = (byte[])stream.Clone(); damaged[6 * 50 + 1] ^= 4;
            Click(); Pump();
            w.HandleLine(Run(0)); w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0"); Pump();
            w.HandleLine(Run(40, bytes: damaged)); w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0"); Pump();
            w.HandleLine(Run(80, bytes: damaged)); w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.nodeRead is null && w.boardMessage?.Invoke().Contains("没有通过校验") == true, "a damaged stream was accepted: " + w.boardMessage?.Invoke());
            Check(!w.xyPreview.RecentNoteText.Contains("正在读取") && w.xyPreview.RecentNoteText.Contains("没有通过校验"), "the panel still says it is reading after the read failed: " + w.xyPreview.RecentNoteText);

            // A device that ends the request without sending anything, and one that refuses.
            Click(); Pump();
            for (int retry = 1; retry <= 3; retry++)
            {
                w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
                Check(w.nodeRead is not null && w.storeJob is { Op: StoreOp.Nodes }, $"a silent answer gave the read up at once (retry {retry})");
                Pump();
            }
            w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.nodeRead is null && w.boardMessage?.Invoke().Contains("设备没有给出节点") == true, "a device that stayed silent did not end the read: " + w.boardMessage?.Invoke());
            Click(); Pump();
            w.HandleLine("TRAJ_STORE refused: stage not idle");
            Check(w.nodeRead is null && w.boardMessage?.Invoke().Contains("stage not idle") == true, "a refusal did not end the read: " + w.boardMessage?.Invoke());

            // STOP while a read is under way: the new reference epoch may turn the answer away, so the read ends at once (no waiting for the
            // timeout with the editing locked), whatever the frame's sequence number says.
            Click(); Pump();
            Check(w.nodeRead is not null && w.storeJob is { Op: StoreOp.Nodes, Sequence: > 0 } && w.PointBusy, "the read did not start");
            w.ResetControlTransport();
            Check(w.nodeRead is null && w.storeJob is null && !w.PointBusy && w.boardMessage?.Invoke().Contains("总停止") == true, "STOP left the node read waiting: " + w.boardMessage?.Invoke());
            // The slot is rewritten between two runs: its save counter changes, nothing is shown, and the inventory is read again.
            Click(); Pump();
            w.HandleLine(Run(0)); w.HandleLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0"); Pump();
            w.HandleLine(Run(40, seq: 3));
            Check(w.nodeRead is null && !w.boardKnown && w.boardMessage?.Invoke().Contains("被改写") == true, "a rewritten slot was not noticed: " + w.boardMessage?.Invoke());
            w.FinishStoreJob(true, () => "");
            // Leaving the link drops a read in progress.
            w.ResetBoard();
            Check(w.nodeRead is null && !w.storeReadSupported, "the read survived a reset");
        }
        finally { w.Close(); Lang.Set(language, false); BoardLedger.PathOverride = null; if (File.Exists(ledger)) File.Delete(ledger); }
    }

    // A path asked for while the periodic scan has an archive job running is not dropped: it starts when that job ends; STOP drops it.
    internal static void BoardPathQueueSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Archive path queue: " + reason); }
        string ledger = Path.Combine(Path.GetTempPath(), "arm-board-queue-" + Guid.NewGuid().ToString("N") + ".json");
        BoardLedger.PathOverride = ledger;
        bool language = Lang.IsEn; Lang.Set(false, false);
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false };
        void Pump() { for (int i = 0; i < 600 && w.pendingControlType != 0; i++) { w.PublishHeartbeatLease(); w.WireWriterStep((_, _, _) => { }); } }
        string Link(string extra) => $"LINK uart=hw baud=115200 fw=stage2-v11-test short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 traj_repeat=1 fault_snapshot=1{extra} proto={DeviceContract.ProtocolVersion}";
        try
        {
            w.HandleLine("STATE fw=stage2-v11 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=3 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.HandleLine(Link(" traj_meta=256 traj_read=1 traj_store=4 telem_hz=0"));
            Pump();
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(10)); Pump(); uint query = w.storeJob!.Sequence;
            w.HandleLine("TRAJ_SLOT slot=0 valid=0");
            w.HandleLine("TRAJ_SLOT slot=1 valid=1 seq=2 nodes=100 duration_ms=9900 q1=0 q2=3000 crc=0000ABCD");
            w.HandleLine("TRAJ_SLOT slot=2 valid=0"); w.HandleLine("TRAJ_SLOT slot=3 valid=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={query} last=none last_slot=0 row_max_us=0");
            Check(w.boardKnown && w.storeJob is null, "the inventory was not taken");
            // The periodic description read is running when the click comes: the request waits, and runs when the read has ended.
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(15)); Pump();
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 }, "the periodic description read did not start");
            w.RequestBoardPath(1);
            Check(w.wantedPathSlot == 1 && w.nodeRead is null && w.storeJob is { Op: StoreOp.Meta }, "a path asked for during the scan was dropped or started too early");
            w.HandleLine("TRAJ_META slot=1 len=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.wantedPathSlot is null && w.nodeRead is { Slot: 1 } && w.storeJob is { Op: StoreOp.Nodes, Slot: 1 }, "the waiting path did not start when the scan's job ended");
            w.ResetControlTransport();   // STOP ends the read
            Check(w.nodeRead is null && w.storeJob is null, "STOP left the read");
            // The slot is written again (its counter moves); the description read starts, the request waits, and STOP drops the request.
            w.HandleLine("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=100 duration_ms=9900 q1=0 q2=3000 crc=0000ABCD");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(30)); Pump();
            Check(w.storeJob is { Op: StoreOp.Meta, Slot: 1 }, "the description read of the rewritten slot did not start");
            w.RequestBoardPath(1);
            Check(w.wantedPathSlot == 1, "the second request was not kept");
            w.ResetControlTransport();
            Check(w.wantedPathSlot is null && w.nodeRead is null && w.storeJob is null, "STOP did not drop the waiting request");
            // A path asked for while the inventory is being read again waits for a COMPLETE inventory: a status that comes without all the slot
            // lines does not make one, and must not drop the request.
            w.boardKnown = false; w.boardSeen = 0;
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(40)); Pump();
            Check(w.storeJob is { Op: StoreOp.Query }, "the inventory was not read again");
            w.RequestBoardPath(1);
            Check(w.wantedPathSlot == 1, "a path asked for during the inventory was not queued");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");   // the slot lines were lost
            Check(!w.boardKnown && w.wantedPathSlot == 1 && w.storeJob is null, "the waiting path was dropped by an inventory that did not complete");
            w.lastBoardQueryAt = default; w.SynchronizeBoard(DateTime.UtcNow); Pump();   // (real time: the request has not been waiting long)
            Check(w.storeJob is { Op: StoreOp.Query }, "the incomplete inventory was not read again");
            w.HandleLine("TRAJ_SLOT slot=0 valid=0");
            w.HandleLine("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=100 duration_ms=9900 q1=0 q2=3000 crc=0000ABCD");
            w.HandleLine("TRAJ_SLOT slot=2 valid=0"); w.HandleLine("TRAJ_SLOT slot=3 valid=0");
            w.HandleLine($"TRAJ_STORE state=idle slot=0 rows=0/0 seq={w.storeJob!.Sequence} last=none last_slot=0 row_max_us=0");
            Check(w.boardKnown && w.wantedPathSlot is null && w.storeJob is { Op: StoreOp.Meta or StoreOp.Nodes }, "the waiting path did not start once the inventory was complete: " + $"known={w.boardKnown} wanted={w.wantedPathSlot} job={w.storeJob?.Op} nodeRead={w.nodeRead is not null} busy={w.PointBusy} avail={w.StoreAvailable} msg={w.boardMessage?.Invoke()}");
            w.ResetControlTransport();
            // And one that no complete inventory ever comes for is given up, with a reason.
            w.boardKnown = false; w.boardSeen = 0;
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(60)); Pump();
            w.RequestBoardPath(1);
            Check(w.wantedPathSlot == 1, "the request was not queued the last time");
            w.SynchronizeBoard(DateTime.UtcNow.AddSeconds(90));
            Check(w.wantedPathSlot is null && w.boardMessage?.Invoke().Contains("设备没有回应") == true, "a path that never got a complete inventory was kept for ever: " + w.boardMessage?.Invoke());
            w.ResetBoard();
        }
        finally { w.Close(); Lang.Set(language, false); BoardLedger.PathOverride = null; if (File.Exists(ledger)) File.Delete(ledger); }
    }

    // A plan read back from the board and run: it replaces what this computer planned, only when this machine accepts it, and then is
    // loaded from its slot by the same Upload button (the slot's CRC is its CRC); a plan this machine's speeds do not allow stays a picture.
    internal static void BoardRunFlowSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Archive run: " + reason); }
        string ledger = Path.Combine(Path.GetTempPath(), "arm-board-run-" + Guid.NewGuid().ToString("N") + ".json");
        BoardLedger.PathOverride = ledger;
        bool language = Lang.IsEn; Lang.Set(false, false);
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false };
        try
        {
            var p = w.xyPreview;
            p.SelectShapeForSelfTest(2); p.LoadExampleForSelfTest();
            var held = p.Plan ?? throw new InvalidOperationException("Archive run: no rectangle plan");   // what the board holds
            p.SelectShapeForSelfTest(3); p.LoadExampleForSelfTest();                                      // what this computer has on screen
            var onScreen = p.Plan ?? throw new InvalidOperationException("Archive run: no circle plan");
            Check(!held.Nodes.SequenceEqual(onScreen.Nodes), "the two plans must differ");
            byte[] stream = TrajectoryArchive.Stream(held.Nodes); uint crc = TrajectoryArchive.PlanCrc(held.Nodes); int total = held.Nodes.Length, ms = held.Nodes.Sum(n => (int)n.Milliseconds);
            var start = held.Nodes[0].Pose;
            w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            w.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=4 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.TestLine($"LINK fw=stage2-v7-absolute-q2 proto={DeviceContract.ProtocolVersion} motion_rates=1 joint_limits=1 traj_read=1 traj_store=4");
            w.TestLine($"TELEM t=10 epoch=4 seq=1 ref=1 source=2 p1={ExecutionTrajectory.Counts(0, start)} p2={ExecutionTrajectory.Counts(1, start)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            w.TestLine("INPUTS: J4-1/P0[4]=1 J4-2/P0[3]=1 NPN/P1[0]=1 nFAULT=1");
            w.TestLine("TRAJ_SLOT slot=0 valid=0");
            w.TestLine($"TRAJ_SLOT slot=1 valid=1 seq=2 nodes={total} duration_ms={ms} q1=0 q2=0 crc={crc:X8}");
            w.TestLine("TRAJ_SLOT slot=2 valid=0"); w.TestLine("TRAJ_SLOT slot=3 valid=0");
            if (w.storeJob is { } initial) { initial.Sequence = 30; w.TestLine("TRAJ_STORE state=idle slot=0 rows=0/0 seq=30 last=none last_slot=0 row_max_us=0"); }
            Check(w.boardKnown && w.storeJob is null && w.storeReadSupported, "the inventory was not taken");
            uint fake = 40;
            // Reads the slot's nodes as the device sends them, forty at a time.
            void ReadSlot()
            {
                w.xyPreview.BoardRowLoadButton(0)!.RaiseEvent(new System.Windows.RoutedEventArgs(System.Windows.Controls.Primitives.ButtonBase.ClickEvent));
                Check(w.nodeRead is not null, "the click did not start a read");
                while (w.nodeRead is { } read)
                {
                    int first = read.Next, count = Math.Min(BoardNodes.MaxPerLine, total - first);
                    w.storeJob!.Sequence = ++fake;
                    w.TestLine($"TRAJ_NODES slot=1 seq=2 total={total} first={first} count={count} hex={Convert.ToHexString(stream.AsSpan(first * 6, count * 6))}");
                    w.TestLine($"TRAJ_STORE state=idle slot=1 rows=0/0 seq={fake} last=none last_slot=0 row_max_us=0");
                }
            }
            // This machine accepts it: it is now the plan on screen, and Upload says it loads from the slot.
            ReadSlot();
            Check(p.Plan is { Timing: null } shown && shown.Nodes.SequenceEqual(held.Nodes), "the plan read back is not the plan on screen");
            w.TestLine($"TELEM t=11 epoch=4 seq=1 ref=1 source=2 p1={ExecutionTrajectory.Counts(0, start)} p2={ExecutionTrajectory.Counts(1, start)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            Check(p.UploadButton.IsEnabled && (string)p.UploadButton.Content == "从板上加载" && !p.RunButton.IsEnabled && p.SettingsDescription.Contains("board_slot=2"),
                $"the board's plan cannot be loaded from its slot: enabled={p.UploadButton.IsEnabled} caption={p.UploadButton.Content} note={p.RecentNoteText}");
            // Slower speeds in the boxes than the plan was made for: the same nodes are no longer accepted, and only a picture is left.
            p.SetSpeedsForSelfTest("1", "1");
            ReadSlot();
            Check(p.Plan is null && p.ShowsBoardPath && !p.UploadButton.IsEnabled && p.RecentNoteText.Contains("不能执行"), "a plan the current speeds do not allow was offered: " + p.RecentNoteText);
        }
        finally { w.Close(); Lang.Set(language, false); BoardLedger.PathOverride = null; if (File.Exists(ledger)) File.Delete(ledger); }
    }

    // The whole flow through the real window, the way the panel drives it: a figure that is not on the device is uploaded and then saved
    // (the pending-save marker must not block its own upload), a figure on the board is loaded instead of uploaded, and a refused load
    // falls back to the node-by-node upload.
    internal static void BoardSaveFlowSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Archive flow: " + reason); }
        static int Type(byte[] f) => ((f[1] >> 1) & 15) | (((f[2] >> 1) & 15) << 4);
        static int Field(byte[] f, int i) => ((f[1 + 2 * i] >> 1) & 15) | (((f[2 + 2 * i] >> 1) & 15) << 4);
        string ledger = Path.Combine(Path.GetTempPath(), "arm-board-flow-" + Guid.NewGuid().ToString("N") + ".json");
        BoardLedger.PathOverride = ledger;
        bool language = Lang.IsEn; Lang.Set(false, false);
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false };
        try
        {
            var p = w.xyPreview; p.SelectShapeForSelfTest(3); p.LoadExampleForSelfTest();
            w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            w.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=4 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.TestLine($"LINK fw=stage2-v7-absolute-q2 proto={DeviceContract.ProtocolVersion} motion_rates=1 joint_limits=1 traj_store=4");
            var plan = p.Plan ?? throw new InvalidOperationException("Archive flow: no plan");
            var start = plan.Nodes[0].Pose;
            w.TestLine($"TELEM t=10 epoch=4 seq=1 ref=1 source=2 p1={ExecutionTrajectory.Counts(0, start)} p2={ExecutionTrajectory.Counts(1, start)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            w.TestLine("INPUTS: J4-1/P0[4]=1 J4-2/P0[3]=1 NPN/P1[0]=1 nFAULT=1");
            Check(p.UploadButton.IsEnabled && w.CanPoint, "the fixture is not ready to upload");
            for (int slot = 0; slot < 4; slot++) w.TestLine($"TRAJ_SLOT slot={slot} valid=0");
            // The first LINK line already started the initial query (as it does on a real link); let it finish.
            if (w.storeJob is { } initial) { initial.Sequence = 30; w.TestLine("TRAJ_STORE state=idle slot=0 rows=0/0 seq=30 last=none last_slot=0 row_max_us=0"); }
            Check(w.storeJob is null, "the initial query did not finish");
            Check(w.StoreAvailable && w.boardKnown, $"the archive is not available (slots={w.storeSlotCount} framed={w.framedControl} epoch={w.reportedEpoch} known={w.boardKnown})");

            // One request on a figure that is not on the device: upload first, then save into the slot.
            w.RequestSaveToBoard(plan, 0);
            Check(w.uploading is not null && w.pendingBoardSave is { Slot: 0 } && w.deferredBoardSave is null && w.pointPacket.Length > 0 && Type(w.pointPacket) == 2, $"the save did not start its upload (uploading={w.uploading is not null} pending={w.pendingBoardSave} deferred={w.deferredBoardSave is not null} limitsReady={w.LimitsReady} busy={w.PointBusy} packet={w.pointPacket.Length}/{(w.pointPacket.Length > 2 ? Type(w.pointPacket) : -1)} message={w.boardMessage?.Invoke()})");
            for (int i = 0; i < plan.Nodes.Length; i++)
                w.TestLine($"TRAJ_STATE state={(i + 1 == plan.Nodes.Length ? 2 : 1)} count={i + 1} total={plan.Nodes.Length} index=0 seq={w.pendingPoint} epoch=4");
            Check(w.uploaded == plan && w.pendingBoardSave is null && w.storeJob is { Op: StoreOp.Save, Slot: 0 } && w.pendingControlType == 12, "the save did not follow the upload");
            w.storeJob!.Sequence = 40;   // the writer thread is not running in this fixture
            w.TestLine("TRAJ_STORE state=idle slot=0 rows=1/1 seq=40 last=saved last_slot=0 row_max_us=4000");
            Check(BoardLedger.Load().Any(e => e.Slot == 0 && e.Crc == TrajectoryArchive.PlanCrc(plan.Nodes)) && w.storeJob is { Op: StoreOp.Query } && !w.boardKnown, "the save was not recorded and read back");
            // Until the read-back arrives the old slot contents are not used: no load from a stale entry, no stars, no saves.
            w.TestLine("TRAJ_SLOT slot=0 valid=1 seq=9 nodes=" + plan.Nodes.Length + " duration_ms=" + plan.Nodes.Sum(n => (int)n.Milliseconds) + " q1=0 q2=0 crc=" + TrajectoryArchive.PlanCrc(plan.Nodes).ToString("X8"));
            Check(!w.boardKnown && (string)p.UploadButton.Content == "上传图形", "a half-read inventory was trusted: " + p.UploadButton.Content);
            // The read-back: the device reports the plan in slot 0, and the screen then offers to load it.
            int ms = plan.Nodes.Sum(n => (int)n.Milliseconds);
            w.TestLine($"TRAJ_SLOT slot=0 valid=1 seq=1 nodes={plan.Nodes.Length} duration_ms={ms} q1={(int)Math.Round(start.Q1 * 100)} q2={(int)Math.Round(start.Q2 * 100)} crc={TrajectoryArchive.PlanCrc(plan.Nodes):X8}");
            for (int slot = 1; slot < 4; slot++) w.TestLine($"TRAJ_SLOT slot={slot} valid=0");
            w.storeJob!.Sequence = 41;
            w.TestLine("TRAJ_STORE state=idle slot=0 rows=0/0 seq=41 last=saved last_slot=0 row_max_us=4000");
            Check(w.storeJob is null && TrajectoryArchive.SlotFor(w.boardSlots, plan.Nodes) is { Slot: 0 } && (string)p.UploadButton.Content == "从板上加载", "the plan on the board was not recognised: " + p.UploadButton.Content);

            // A figure on the board is loaded (type 13, slot 0) instead of uploaded, and nothing runs by itself.
            w.uploaded = null; w.uploadedRan = false;
            w.RequestTrajectory(plan);
            Check(w.loadingSlot == 0 && w.uploading is not null && Type(w.pointPacket) == 13 && (short)(Field(w.pointPacket, 9) | Field(w.pointPacket, 10) << 8) == 0 && !w.executing, "the stored plan was not loaded from its slot");
            w.TestLine($"TRAJ_LOAD slot=0 seq=1 nodes={plan.Nodes.Length} duration_ms={ms}");
            w.TestLine($"TRAJ_STATE state=2 count={plan.Nodes.Length} total={plan.Nodes.Length} index=0 seq={w.pendingPoint} epoch=4");
            Check(w.uploaded == plan && w.uploading is null && w.loadingSlot is null && !w.executing && w.pendingPoint == 0, "the load was not completed as a held plan");

            // The device refuses the stored plan (limits changed, slot damaged): the same figure is uploaded the normal way.
            w.uploaded = null; w.uploadedRan = false;
            w.RequestTrajectory(plan);
            Check(w.loadingSlot == 0, "second load did not start");
            w.TestLine("PTP refused: archived plan does not fit the current limits");
            Check(w.loadingSlot is null && w.uploading is not null && Type(w.pointPacket) == 2 && !w.boardKnown, "a refused load did not fall back to the upload");
            // The slot is rewritten between the preview and the load (another host): the device loads what is in it now.  Its load line names
            // that plan; unless it is the one that was previewed (same save counter and duration) the plan is uploaded instead.
            void FinishUpload()
            {
                for (int i = 0; i < plan.Nodes.Length; i++)
                    w.TestLine($"TRAJ_STATE state={(i + 1 == plan.Nodes.Length ? 2 : 1)} count={i + 1} total={plan.Nodes.Length} index=0 seq={w.pendingPoint} epoch=4");
                Check(w.uploaded == plan, "the upload did not complete");
            }
            FinishUpload();
            // The last case is a slot rewritten with a plan of another size: its status line is complete and consistent, only not ours.
            foreach (var (otherSeq, otherMs, lineLost, moreNodes) in new[] { (1u, ms, true, 0), (9u, ms, false, 0), (1u, ms + 50, false, 0), (1u, ms, false, 7) })
            {
                w.boardKnown = true; w.uploaded = null; w.uploadedRan = false;
                w.RequestTrajectory(plan);
                Check(w.loadingSlot == 0, "the load did not start");
                int heldNodes = plan.Nodes.Length + moreNodes;
                if (!lineLost) w.TestLine($"TRAJ_LOAD slot=0 seq={otherSeq} nodes={heldNodes} duration_ms={otherMs}");
                w.TestLine($"TRAJ_STATE state=2 count={heldNodes} total={heldNodes} index=0 seq={w.pendingPoint} epoch=4");
                string why = $"(seq {otherSeq}, {otherMs} ms, line lost {lineLost}, {moreNodes} more nodes)";
                Check(w.loadingSlot is null && w.uploading is not null && w.uploaded is null && Type(w.pointPacket) == 2 && !w.executing, "a load of another plan was accepted " + why);
                // The visible status names the real reason, not the refused-load one (limits or speeds changed, damaged data).
                Check(w.boardMessage?.Invoke() is { } shown && shown.Contains("被改写") && !shown.Contains("限制或速度"), "the fallback did not say that the slot changed " + why + ": " + w.boardMessage?.Invoke());
                FinishUpload();
            }
        }
        finally { w.Close(); Lang.Set(language, false); BoardLedger.PathOverride = null; if (File.Exists(ledger)) File.Delete(ledger); }
    }
}
