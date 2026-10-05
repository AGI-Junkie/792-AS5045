using System.Reflection;

namespace TwoLinkCommissioning;

// The numbers the host and the firmware must agree on.  Each constant has a twin in
// DRV8825.cydsn/main.c (named in the comment); build-release.ps1 runs
// `TwoLinkCommissioning.exe --print-contract <file>` and fails when a twin differs,
// so a change on one side cannot ship without the other.  Do not repeat these
// numbers elsewhere in the host: use the names.
public static class DeviceContract
{
    // Drive train.  Steps per output revolution: 200 full steps x 8 microsteps x gearbox.
    public const int FullStepsPerRev = 200;         // MOTOR_FULLSTEPS_REV, ARM2_MOTOR_FULLSTEPS_REV
    public const int Microsteps = 8;                // USTEP_DIV
    public const int Arm1GearRatio = 40;            // GEAR_RATIO (40:1)
    public const int Arm2GearRatioX100 = 5106;      // ARM2_GEAR_RATIO_X100 (51.06:1)
    public const int EncoderCounts = 4096;          // ENCODER_CPR (AS5045, 12 bit, on the arm1 motor's rear shaft)
    public const int Arm1StepsPerRev = FullStepsPerRev * Microsteps * Arm1GearRatio;             // 64000
    public const int Arm2StepsPerRev = FullStepsPerRev * Microsteps * Arm2GearRatioX100 / 100;   // 81696

    // Rates and accelerations the host may configure (microsteps/s and microsteps/s^2).
    public const int RateMin = 20;                  // CFG_RATE_MIN
    public const int Arm1RateMax = 4000;            // CFG_RATE_MAX_AXIS1
    public const int Arm2RateMax = 4000;            // CFG_RATE_MAX_AXIS2
    public const int AccelMin = 100;                // CFG_ACCEL_MIN
    public const int Arm1AccelMax = 20000;          // CFG_ACCEL_MAX_AXIS1
    public const int Arm2AccelMax = 12000;          // CFG_ACCEL_MAX_AXIS2

    // Caps for drawing and positioning moves, per axis.
    public const int PlanRateMax = 4000;            // PTP_RATE_MAX, TRAJ_RATE_MAX
    public const int PlanAccelMax = 3000;           // PTP_ACCEL, TRAJ_ACCEL_MAX
    public const int HomeSlowRate = 400;            // HOME_SLOW_RATE_USTEP_S

    // Trajectory upload.
    public const int TrajCapacity = 2048;           // TRAJ_CAPACITY
    public const int TrajNodeMinMs = 50;            // TRAJ_NODE_MIN_MS
    public const int TrajNodeMaxMs = 500;           // TRAJ_NODE_MAX_MS
    public const int TrajTotalMaxMs = 600000;       // TRAJ_TOTAL_MAX_MS

    // Streamed position telemetry (control type 11): 0 = only the 500 ms status block, otherwise this many TELEM lines a second.
    public const int TelemRateMin = 5;              // TELEM_HZ_MIN
    public const int TelemRateMax = 20;             // TELEM_HZ_MAX

    // Joint limits are 0.01 degree units within +/-180 degrees; the heartbeat deadline is the firmware's.
    public const int JointLimitCdMax = 18000;       // CFG_JOINT_CD_MAX
    // The elbow is motor 2's own angle q2 - q1.  Its range is a third entry of the joint limits (type 9, axis 2); this is
    // the firmware's boot value and the host's value for an empty field.  Provisional, NOT a measured mechanical stroke.
    public const int ElbowDefaultCd = 15500;        // PTP_ELBOW_DEFAULT_CD
    public const int DeadmanMs = 250;               // JOG_DEADMAN_MS

    // Version of the wire contract (docs/PROTOCOL.md).  The firmware reports it as proto=<n> in the LINK line and
    // the host refuses to move a machine whose number differs.  Bump both sides when a frame format, a field's
    // meaning, the sequence/epoch rules or the coordinate convention changes.
    public const int ProtocolVersion = 4;           // PROTO_VERSION

    // Every constant above as NAME=value lines, for the release gate.
    public static string[] Lines() => typeof(DeviceContract)
        .GetFields(BindingFlags.Public | BindingFlags.Static)
        .Where(f => f.IsLiteral)
        .Select(f => $"{f.Name}={f.GetRawConstantValue()}")
        .ToArray();
}
