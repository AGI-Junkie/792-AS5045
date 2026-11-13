using System.Windows.Threading;

namespace TwoLinkCommissioning;

// Native traffic can starve Input-priority dispatcher jobs. Run below Render; a blocked UI still cannot renew its lease.
internal sealed class DispatcherHeartbeatTimer : IDisposable
{
    private readonly Dispatcher dispatcher;
    private readonly System.Threading.Timer timer;
    private TimeSpan interval = TimeSpan.FromMilliseconds(40);
    private volatile bool enabled;
    private int queued, generation;
    public event EventHandler? Tick;
    public bool IsEnabled => enabled;
    public TimeSpan Interval {
        get => interval;
        set { interval = value; if (enabled) timer.Change(value, value); }
    }

    public DispatcherHeartbeatTimer(Dispatcher dispatcher)
    {
        this.dispatcher = dispatcher;
        timer = new System.Threading.Timer(_ => QueueTick(), null, Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);
    }

    public void Start()
    {
        if (enabled) return;
        enabled = true;
        timer.Change(interval, interval);
    }

    public void Stop()
    {
        enabled = false;
        Interlocked.Increment(ref generation);
        timer.Change(Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);
    }

    private void QueueTick()
    {
        int version = Volatile.Read(ref generation);
        if (!enabled || dispatcher.HasShutdownStarted || Interlocked.CompareExchange(ref queued, 1, 0) != 0) return;
        dispatcher.BeginInvoke(DispatcherPriority.Loaded, () => {
            try { if (enabled && version == Volatile.Read(ref generation)) Tick?.Invoke(this, EventArgs.Empty); }
            finally { Volatile.Write(ref queued, 0); }
        });
    }

    public void Dispose() { Stop(); timer.Dispose(); }
}
