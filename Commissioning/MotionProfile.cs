namespace TwoLinkCommissioning;

// Integer pulse limits are the shared host/firmware settings. Mode caps remain
// those already commissioned: jog 800/600, home <= phase cap, PTP/plot 600/800.
public readonly record struct MotionProfile(int R1, int R2, int A1, int A2)
{
    public static double StepsPerDegree(int axis) => (axis == 0 ? DeviceContract.Arm1StepsPerRev : DeviceContract.Arm2StepsPerRev) / 360.0;
    public static MotionProfile From(PlanningRates r)
    {
        if (new[] { r.Feed, r.V1, r.V2, r.A1, r.A2 }.Any(x => !double.IsFinite(x) || x <= 0 || x > 1000))
            throw new ArgumentException("速度与加速度须为 0～1000 内的有限正数。");
        static int Limit(double x, int axis, int min, int max)
        {
            int pulses = (int)Math.Floor(x * StepsPerDegree(axis) + 1e-9);
            if (pulses < min) throw new ArgumentException($"arm{axis+1} 设置低于设备支持的下限 {min / StepsPerDegree(axis):F3}（速度 °/s 或加速度 °/s²）。");
            return Math.Min(pulses, max);
        }
        return new(Limit(r.V1,0,DeviceContract.RateMin,DeviceContract.Arm1RateMax), Limit(r.V2,1,DeviceContract.RateMin,DeviceContract.Arm2RateMax),
            Limit(r.A1,0,DeviceContract.AccelMin,DeviceContract.Arm1AccelMax), Limit(r.A2,1,DeviceContract.AccelMin,DeviceContract.Arm2AccelMax));
    }
    public PlanningRates DrawingRates(double feed) => new(feed, Math.Min(R1,DeviceContract.PlanRateMax)/StepsPerDegree(0),
        Math.Min(R2,DeviceContract.PlanRateMax)/StepsPerDegree(1), Math.Min(A1,DeviceContract.PlanAccelMax)/StepsPerDegree(0), Math.Min(A2,DeviceContract.PlanAccelMax)/StepsPerDegree(1));
    public JointPose Axis(int axis) => axis == 0 ? new(R1/100.0,A1/100.0) : new(R2/100.0,A2/100.0);
    public string Description => $"点动上限电机1 {R1/StepsPerDegree(0):F2}、电机2（q2−q1） {R2/StepsPerDegree(1):F2} °/s；绘图/定位另限每轴 {DeviceContract.PlanRateMax} 微步/s、{DeviceContract.PlanAccelMax} 微步/s²；回零慢碰另限 {DeviceContract.HomeSlowRate} 微步/s。";
}
