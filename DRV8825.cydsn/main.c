/* ========================================================================
 * AS5045 + DRV8825: two-joint arm firmware (stage2-v8)
 * Target: CY8CKIT-059 (PSoC 5LP) @ 24 MHz BUS_CLK, GPIO + UDB host UART.
 *
 * MECHANICS: arm1 = 42 mm 1.8 deg stepper on DRV2 / stepper2 behind a 40:1
 * gearbox.  The AS5045 magnet sits on the MOTOR REAR SHAFT, so the encoder
 * measures the rotor, not the gearbox output.  arm2 = 28 mm stepper on DRV1 /
 * stepper1 (51.06:1), with a second motor-shaft AS5045 in the SSI chain.
 *
 *   1 output degree  = 40 motor degrees = 455.1 encoder LSB      (arm1)
 *   1 microstep(1/8) = 0.225 motor deg  = 2.56 LSB = 0.005625 output degree
 *   9 output degrees = one full motor turn -> the single-turn AS5045 WRAPS.
 *
 * Because of that 9-degree wrap, arm1 position is tracked MULTI-TURN: every
 * sample accumulates the shortest circular delta into a signed 32-bit counter.
 * The raw 12-bit angle is never used as a position by itself.
 *
 * WHAT RUNS: one foreground loop (jog_run); no interrupt drives the motors.
 *   - Every motion needs a live host lease (deadman); the watchdog is fed only
 *     by a complete loop; encoder tracking loss latches until an explicit enable.
 *   - Keyboard jog with velocity ramps, homing (a state machine), PTP moves and
 *     preloaded timed trajectories.  All step pulses go through jog_step(), which
 *     applies the fault, limit, sensor and joint-range checks.
 *   - arm1 keeps two tracks in encoder LSB: COMMANDED (2.56 LSB per issued
 *     microstep) and MEASURED (multi-turn AS5045).  slip = commanded - measured:
 *     SLIP_WARN_LSB halves the speed cap, SLIP_FAIL_LSB stops.
 *   - stage_stop() invalidates the coordinate reference.  Every stop needs an
 *     explicit re-enable; nothing recovers automatically.
 *
 * LIMITS: single-turn absolute means multi-turn position is lost at power
 * off; encoder on the rotor cannot see gearbox backlash, so output-side
 * repeatability is bounded by backlash, not by the 0.0022 deg/LSB resolution.
 * ======================================================================== */
#include <cytypes.h>
#include <cyfitter.h>
#include <cydevice_trm.h>
#include <cypins.h>
#include <CyLib.h>
#include "HostUart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ============================ USER CONFIGURATION ======================== */
#define USE_DEBUG_UART              (1u)     /* P12[7], 115200 8N1          */
#define PARITY_STRICT               (1u)
#define ENCODER_SELFTEST_MS         (5000u)

/* Physical directions confirmed on the bench: left DIR=0 for both axes.
 * Encoder polarity is learned only during manually requested homing.
 * This stage requires both motor encoders; there is no automatic fallback. */
#define ARM2_LEFT_DIR_LEVEL         (0u)
#define ARM1_LEFT_DIR_LEVEL         (0u)

#define JOG1_RATE_MAX_USTEP_S       (800u)   /* commissioning: 4.5 out-deg/s */
#define JOG1_RATE_START_USTEP_S     (200u)
#define JOG1_ACCEL_USTEP_S2         (20000u) /* 0.15 s to top speed          */
#define JOG1_DECEL_USTEP_S2         (40000u) /* 75 ms from top speed to stop */
#define JOG2_RATE_MAX_USTEP_S       (600u)   /* commissioning speed          */
#define JOG2_RATE_START_USTEP_S     (200u)
#define JOG2_ACCEL_USTEP_S2         (12000u)
#define JOG2_DECEL_USTEP_S2         (24000u)
#define JOG_DEADMAN_MS              (250u)   /* no key byte for this long -> stop */
#define JOG_STATUS_MS               (500u)   /* status line period while moving */
#define JOG_ENC_SAMPLE_US           (1000u)
#define JOG_TICK_US                 (1000u)  /* ramp / deadman / hold tick   */
#define JOG_HOLD_STEP_MS            (12u)    /* hold correction step spacing */
#define JOG_CMD_REPEAT_MS           (60u)
#define PTP_EPOCH_ANNOUNCE_MS       (250u)  /* re-announce a changed reference epoch this often */

/* Identifies this exact firmware build in every LINK line and in the boot banner.
 * The STATE line keeps the plain "stage2-v7" token because the host matches it to
 * select the framed transport.  Bump this on every firmware change so a field log
 * can answer "was this fix actually flashed?". */
#ifndef ARM2_ENCODER_FEEDBACK
#define ARM2_ENCODER_FEEDBACK       (0u) /* Enable only after the encoder/magnet is fixed and commissioned. */
#endif

#define FW_BUILD_ID                 "stage2-v9-optional-arm2-feedback"

/* Version of the wire contract in docs/PROTOCOL.md, reported as proto=<n> in the LINK line.  The host
 * refuses to move a machine whose proto differs from DeviceContract.ProtocolVersion, so the build id
 * above can change freely.  Bump it (and the host constant) whenever a frame format, a field's meaning,
 * the sequence/epoch rules or the coordinate convention changes.  2: PTP_STATE carries emin/emax and type 9
 * accepts axis 2, the elbow range (q2 - q1, i.e. motor 2's own travel). */
#define PROTO_VERSION               (4u)

/* The software UART only accepts odd bytes.  jog.ps1 maps physical R to this
 * otherwise-unused wire byte, because ASCII 'r' (0x72) is even. */
#define JOG_CMD_HOME                 (0x71u)
#define JOG_CMD_ENABLE               (0x63u) /* physical C, explicit enable */
#define JOG_CMD_STOP                 (0x79u) /* physical Space/Esc, latched */
#if ((JOG_CMD_HOME & 1u) == 0u)
    #error "JOG_CMD_HOME must be an odd software-UART byte"
#endif

/* `r` homes each axis with a fast touch, release/backoff, slow touch, then the move to straight.
 * The fast touch and the move to straight run at the operator's global speed and acceleration (g_motionRate/g_motionAccel).
 * Release/backoff (short moves clear of the sensor) and the slow touch (precision) keep the fixed rates and the
 * acceleration cap below; the global values can only lower them.
 * Directions are the real-world keyboard directions confirmed on the bench. */
#define ARM1_HOME_JOG_DIR            (1)     /* d: approach J4-2 / P0[3]    */
#define ARM2_HOME_JOG_DIR            (1)     /* Right: approach P1[0] NPN    */
#define HOME_RELEASE_RATE_USTEP_S    (JOG2_RATE_MAX_USTEP_S) /* release and backoff: the slower jog cap, as commissioned */
#define HOME_SLOW_RATE_USTEP_S       (400u)
#define HOME_BACKOFF_USTEPS          (64u)
#define HOME_RELEASE_MAX_USTEPS      (800u)
#define HOME_ACCEL_USTEP_S2         (4000u)
#define HOME_DEBOUNCE_MS             (2u)
#define HOME_PHASE_TIMEOUT_MS        (120000ul)
#define HOME_SETTLE_MS               (50u)
#define HOME_ZERO_SAMPLES            (12u)
#define HOME_ZERO_SAMPLE_MS          (1u)
#define ARM1_STRAIGHT_OUT_DEG10      (1260)   /* temporary visual correction: 125 + 1 deg */
#define ARM2_STRAIGHT_OUT_DEG10      (1615)   /* temporary visual correction: 160 + 1.5 deg */
#define ARM2_STRAIGHT_JOG_DIR        (-1)    /* user confirmed: Left */
#define ARM2_MOTOR_FULLSTEPS_REV     (200)   /* Anaheim 11YPG302S-LW4-R51, 1.8 deg */
#define ARM2_GEAR_RATIO_X100         (5106)  /* maker R51 table: 51.06:1, direct drive */

/* Numbers the host has to agree on.  Commissioning/DeviceContract.cs holds the same values and
 * build-release.ps1 compares the two, so change them together or the release build fails. */
#define CFG_RATE_MIN                 (20)     /* configurable jog/home/PTP rate range, microsteps/s    */
#define CFG_RATE_MAX_AXIS1           (4000)
#define CFG_RATE_MAX_AXIS2           (4000)
#define CFG_ACCEL_MIN                (100)    /* configurable acceleration range, microsteps/s^2       */
#define CFG_ACCEL_MAX_AXIS1          (20000)
#define CFG_ACCEL_MAX_AXIS2          (12000)
#define CFG_JOINT_CD_MAX             (18000)  /* joint limits stay within +/-180 deg (0.01 deg units)  */
#define TRAJ_NODE_MIN_MS             (50u)    /* one trajectory segment, milliseconds                  */
#define TRAJ_NODE_MAX_MS             (500u)
#define TRAJ_TOTAL_MAX_MS            (600000u)
_Static_assert(JOG1_RATE_MAX_USTEP_S <= CFG_RATE_MAX_AXIS1 && JOG2_RATE_MAX_USTEP_S <= CFG_RATE_MAX_AXIS2 &&
               JOG1_ACCEL_USTEP_S2 <= CFG_ACCEL_MAX_AXIS1 && JOG2_ACCEL_USTEP_S2 <= CFG_ACCEL_MAX_AXIS2,
               "default jog rate/acceleration exceeds the range the host may configure");

/* User-authorized low-speed trial. These are provisional software bounds,
 * NOT measured mechanical limits. Homing and all runtime protections still apply. */
#ifndef PTP_COMMISSIONED
#define PTP_COMMISSIONED             (1u)
#endif
#ifndef PTP_MIN1_CD
#define PTP_MIN1_CD                  (-3000)
#define PTP_MAX1_CD                  (3000)
#define PTP_MIN2_CD                  (-6000)
#define PTP_MAX2_CD                  (6000)
#endif
#if PTP_COMMISSIONED && ((PTP_MIN1_CD >= PTP_MAX1_CD) || (PTP_MIN2_CD >= PTP_MAX2_CD) || \
    (PTP_MIN1_CD > 0) || (PTP_MAX1_CD < 0) || (PTP_MIN2_CD > 0) || (PTP_MAX2_CD < 0) || \
    (PTP_MIN1_CD < -CFG_JOINT_CD_MAX) || (PTP_MAX1_CD > CFG_JOINT_CD_MAX) || (PTP_MIN2_CD < -CFG_JOINT_CD_MAX) || (PTP_MAX2_CD > CFG_JOINT_CD_MAX))
#error "PTP requires valid software bounds within +/-180 deg containing straight zero"
#endif
/* Elbow (motor 2, q2 - q1) range, 0.01 deg.  Like the boxes above this is a provisional software bound, NOT a
 * measured mechanical stroke; the host overwrites it (type 9, axis 2) before any motion is allowed.  The value
 * is the same as the host's DeviceContract.ElbowDefaultCd. */
#define PTP_ELBOW_DEFAULT_CD         (15500)
#define PTP_MINE_CD                  (-PTP_ELBOW_DEFAULT_CD)
#define PTP_MAXE_CD                  (PTP_ELBOW_DEFAULT_CD)
#if PTP_COMMISSIONED && (PTP_MINE_CD >= PTP_MAXE_CD || PTP_MINE_CD > 0 || PTP_MAXE_CD < 0 || \
    PTP_MINE_CD < -CFG_JOINT_CD_MAX || PTP_MAXE_CD > CFG_JOINT_CD_MAX)
#error "PTP requires an elbow bound within +/-180 deg containing straight zero"
#endif
#define PTP_RATE_MAX                 (4000u)
#define PTP_ACCEL                    (3000u)
#define PTP_FRAME_BYTES              (17u)
#define PTP_FRAME_START              (0xD1u)
#define PTP_FRAME_END                (0xD3u)
/* Compact heartbeat: D5 <lo(keys)> <hi(keys)> <lo(crc8)> <hi(crc8)> D7.  Six wire
 * bytes instead of 36, so one beat costs 5.5 ms of paced writes instead of 35.5 ms.
 * It carries no sequence and no epoch on purpose: a heartbeat answers "is the host
 * alive", while every command keeps using the full 17-byte frame with sequence,
 * epoch and CRC16.  A host that does not know this frame simply never sends it. */
#define PTP_SHORT_BYTES              (2u)
#define PTP_SHORT_START              (0xD5u)
#define PTP_SHORT_END                (0xD7u)
/* Confirmed by physical homing logs on 2026-09-24: encoder increases at DIR=1.
 * Manual referencing uses this bench calibration, with no automatic probe motion. */
#define MANUAL_ENC_INC_DIR           (1u)

/* ------------------------------ mechanics ------------------------------- */
#define GEAR_RATIO                  (40)     /* 42 motor + 40:1 gearbox      */
#define MOTOR_FULLSTEPS_REV         (200)
#define USTEP_DIV                   (8u)     /* DRV8825 M2/M1/M0 microstep   */

/* -------------------- encoder tracking bound, rotor settle -------------- */
#define MOVE_RATE_MAX_USTEP_S       (6000u)  /* 225 motor rpm = 33.8 out deg/s */
#define MOVE_SETTLE_MS              (30u)    /* let the rotor stop ringing   */

/* ------------------------- slip (lost step) limits ---------------------- */
/* 1 full step = 20.48 LSB. Pull-out is 2 full steps (90 deg electrical) =
 * 41 LSB, so the warning sits at 75% of the torque limit. */
#define SLIP_WARN_LSB               (31)     /* 1.5 full steps = 2.70 deg    */
#define SLIP_FAIL_LSB               (328)    /* 4 electrical cycles = 28.8   */
#define PULLOUT_LSB                 (41)     /* 2 full steps, for reporting  */
#define SLIP_DERATE_MAX             (3u)     /* speed cap can drop 8x        */

/* AS5045 output delay. MODE low/open = slow mode, 2.61 kHz / 384 us.
 * At 6000 ustep/s that delay alone looks like 6 LSB of slip, so it is
 * subtracted before the slip test. Set to 96 if MODE is tied to VDD5V. */
#define AS5045_DELAY_US             (384)
#define LAG_COMPENSATION            (1u)

/* ------------------ hold deadband, final-position check ----------------- */
#define CL_DEADBAND_LSB             (4)      /* 0.35 motor deg, 1.6 ustep    */
#define PTP_FINAL_ERROR_LSB         (228)    /* motion commissioning: about +/-0.50 output deg, not a precision claim */

/* ---------------------------- safety limits ----------------------------- */
#define SOFT_LIMIT_OUT_DEG10        (900)    /* +/- 90 output degrees        */
#define ENC_BAD_LIMIT               (5u)     /* consecutive unusable samples */

/* --------------------------- direction probe ---------------------------- */
#define DIR_PROBE_USTEPS            (16u)    /* 3.6 motor deg = 0.09 out deg */
#define DIR_PROBE_MIN_LSB           (16)
#define HOME_PROBE_MAX_USTEPS       (64u)
#define HOME_PROBE_SETTLE_US        (20000u)

#define STEP_LOW_US                 (10u)
#define DIR_SETUP_US                (20u)
#define ENCODER_READ_RETRIES        (3u)
/* ======================================================================== */

#if   (USTEP_DIV == 1u)
    #define M2_LVL 0u
    #define M1_LVL 0u
    #define M0_LVL 0u
#elif (USTEP_DIV == 2u)
    #define M2_LVL 0u
    #define M1_LVL 0u
    #define M0_LVL 1u
#elif (USTEP_DIV == 4u)
    #define M2_LVL 0u
    #define M1_LVL 1u
    #define M0_LVL 0u
#elif (USTEP_DIV == 8u)
    #define M2_LVL 0u
    #define M1_LVL 1u
    #define M0_LVL 1u
#elif (USTEP_DIV == 16u)
    #define M2_LVL 1u
    #define M1_LVL 0u
    #define M0_LVL 0u
#elif (USTEP_DIV == 32u)
    #define M2_LVL 1u
    #define M1_LVL 0u
    #define M0_LVL 1u
#else
    #error "USTEP_DIV must be 1, 2, 4, 8, 16 or 32"
#endif

#define ENCODER_CPR                 (4096)
#define ENCODER_MASK                (0x0FFFu)
#define FRAME_MASK                  (0x0003FFFFul)
#define USTEPS_PER_REV              (MOTOR_FULLSTEPS_REV * (int32)USTEP_DIV)

/* LSB per microstep in hundredths. 4096*100/(200*USTEP_DIV) = 2048/USTEP_DIV,
 * which is exact for every legal USTEP_DIV, so the commanded track can be
 * advanced with 32-bit integer adds and never drifts. */
#define LSB_PER_USTEP_X100          ((int32)((ENCODER_CPR * 100) / USTEPS_PER_REV))

/* -------------------------- register-level pins ------------------------- */
/* AS5045 serial interface, verified by the bring-up firmware. */
#define PIN_CSN                     CYREG_PRT12_PC1  /* P12[1] -> AS5045 CSn */
#define PIN_CLK                     CYREG_PRT12_PC0  /* P12[0] -> AS5045 CLK */
#define PIN_DO                      CYREG_PRT1_PC3   /* P1[3]  <- AS5045 DO  */

/* DRV2 / stepper2 carrier map verified on 2026-07-19. */
#define PIN_DIR2                    CYREG_PRT12_PC4
#define PIN_STEP2                   CYREG_PRT12_PC3
#define PIN_SLEEP2                  CYREG_PRT12_PC2
#define PIN_RST2                    CYREG_PRT1_PC2
#define PIN_M2_2                    CYREG_PRT1_PC4
#define PIN_M2_1                    CYREG_PRT1_PC5
#define PIN_M2_0                    CYREG_PRT1_PC6
#define PIN_EN2                     CYREG_PRT1_PC7
#define PIN_FAULT                   CYREG_PRT12_PC5  /* shared, active-low  */

/* DRV1 / stepper1 (arm2, 28 motor). STEP1 is also the board's blue LED, so
 * the LED must not be used as a status light once stepper1 is awake. */
#define PIN_DIR1                    CYREG_PRT2_PC0
#define PIN_STEP1                   CYREG_PRT2_PC1
#define PIN_SLEEP1                  CYREG_PRT2_PC2
#define PIN_RST1                    CYREG_PRT2_PC3
#define PIN_M1_2                    CYREG_PRT2_PC4
#define PIN_M1_1                    CYREG_PRT2_PC5
#define PIN_M1_0                    CYREG_PRT2_PC6
#define PIN_EN1                     CYREG_PRT2_PC7

#define PIN_LED                     CYREG_PRT2_PC1   /* board blue LED      */
#define PIN_DBG_TX                  CYREG_PRT12_PC7  /* KitProg USB serial  */
#define PIN_DBG_RX                  CYREG_PRT12_PC6  /* KitProg -> PSoC     */

/* External safety inputs. Both limits use their NC contacts to GND, so a
 * pressed switch (or broken wire) reads high. P0[4] is always global;
 * arm1 homes only on J4-2/P0[3]. */
#define PIN_LIMIT1                  CYREG_PRT0_PC3   /* J4-2, arm1 home     */
#define PIN_LIMIT2                  CYREG_PRT0_PC4   /* J4-1, global limit  */
#define PIN_ARM2_NPN                CYREG_PRT1_PC0   /* J12-5, NPN output   */
#define ARM1_HOME_ACTIVE()          (CyPins_ReadPin(PIN_LIMIT1) != 0u)
#define HARD_LIMIT_ACTIVE()         (CyPins_ReadPin(PIN_LIMIT2) != 0u)
#define ARM2_NPN_ACTIVE()           (CyPins_ReadPin(PIN_ARM2_NPN) == 0u)

#define FAULT_ACTIVE()              (CyPins_ReadPin(PIN_FAULT) == 0u)

/* ------------------------------- timebase ------------------------------- */
#define SU_NOW()                    (CY_GET_REG32(CYREG_NVIC_SYSTICK_CURRENT))
#define SU_ELAPSED(t0)              (((t0) - SU_NOW()) & 0x00FFFFFFu)
#define SU_CY_PER_US                (BCLK__BUS_CLK__HZ / 1000000u)

/* Extend the existing free-running 24-bit down-counter without changing pulse
 * timing. The ISR only counts wraps; foreground stalls do not slow deadlines. */
static volatile uint32 g_clockWraps;
CY_ISR(clock_wrap_isr) { g_clockWraps++; }
static uint32 wall_ms(void)
{
    uint32 wraps, current;
    uint8 ints = CyEnterCriticalSection();
    wraps = g_clockWraps;
    current = SU_NOW();
    if (CY_GET_REG32(CYREG_NVIC_INTR_CTRL_STATE) & 0x04000000u) {
        /* A wrap occurred while interrupts were masked; count it exactly once. */
        wraps++;
        current = SU_NOW();
    }
    CyExitCriticalSection(ints);
    return (uint32)((((uint64_t)wraps << 24) + (0x00FFFFFFu-current)) /
                    (BCLK__BUS_CLK__HZ / 1000u));
}

static void su_timer_start(void)
{
    g_clockWraps = 0u;
    (void)CyIntSetSysVector(CY_INT_SYSTICK_IRQN, clock_wrap_isr);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_RELOAD,  0x00FFFFFFu);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_CURRENT, 0u);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_CTL,     0x07u);
}

/* Keep each call below the approximately 0.69 s SysTick wrap at 24 MHz. */
static void delay_us(uint32 us)
{
    uint32 t0 = SU_NOW();
    uint32 target = us * SU_CY_PER_US;
    while (SU_ELAPSED(t0) < target) { }
}

/* ---------------------------- debug logging ----------------------------- */
/* A 60-character line costs 5 ms at 115200. That is fine between moves and
 * on slip events (a longer dwell can never cause a stepper to lose sync),
 * but never log once per microstep inside the fast loop. */
#if (USE_DEBUG_UART != 0u)
static void dbg_puts(const char *s)
{
    while (*s != '\0')
    {
        HostUart_PutChar((uint8)*s);
        s++;
    }
}

static char g_dbg[512];   /* Dual-encoder fault snapshots must retain reason and CRLF. */
#define DBG(s)       dbg_puts(s)
#define DBGF(...)    do { snprintf(g_dbg, sizeof(g_dbg), __VA_ARGS__); \
                          dbg_puts(g_dbg); } while (0)
#else
#define DBG(s)       do { } while (0)
#define DBGF(...)    do { } while (0)
#endif

/* ------------------------------ GPIO ------------------------------------ */
static void pin_out_init(uint32 pinPC, uint8 initialLevel)
{
    uint8 value = CY_PINS_DM_STRONG;
    if (initialLevel != 0u) { value |= CY_PINS_PC_DATAOUT; }
    CY_SET_REG8(pinPC, value);
}

static void pin_write(uint32 pinPC, uint8 level)
{
    if (level != 0u) { CyPins_SetPin(pinPC); }
    else             { CyPins_ClearPin(pinPC); }
}

/* ============================== unit helpers ============================ */
static int32 abs_i32(int32 v)          { return (v < 0) ? -v : v; }

/* Not on the hot path, so 64-bit intermediates are fine here. */
static int32 usteps_to_lsb(int32 usteps)
{
    return (int32)(((int64)usteps * ENCODER_CPR) / USTEPS_PER_REV);
}

static int32 lsb_to_usteps(int32 lsb)
{
    return (int32)(((int64)lsb * USTEPS_PER_REV) / ENCODER_CPR);
}

/* Output-shaft degrees (tenths) -> motor-side encoder LSB. */
static int32 out_deg10_to_lsb(int32 deg10)
{
    return (int32)(((int64)deg10 * ENCODER_CPR * GEAR_RATIO) / 3600);
}

/* Motor-side encoder LSB -> output-shaft degrees, hundredths. */
static int32 lsb_to_out_deg100(int32 lsb) __attribute__((unused));
static int32 lsb_to_out_deg100(int32 lsb)
{
    return (int32)(((int64)lsb * 36000) / ((int64)ENCODER_CPR * GEAR_RATIO));
}

/* snprintf has no float support here; print signed fixed point by parts. */
#define DEG_SGN(v)   (((v) < 0) ? "-" : "")
#define DEG_INT(v)   ((long)(abs_i32(v) / 100))
#define DEG_FRC(v)   ((long)(abs_i32(v) % 100))

/* ================================ AS5045 ================================ */
/* Slow bit-banged SSI: approximately 167 kHz, well below the 1 MHz limit.
 * One frame costs about 115 us, which fits inside a 167 us step window. */
#define SSI_HALF_US                 (3u)
#define SSI_SAMPLE_US               (2u)

typedef struct
{
    uint32 raw;
    uint16 angle;
    uint8 ocf;
    uint8 cof;
    uint8 lin;
    uint8 mag_inc;
    uint8 mag_dec;
    uint8 par;
    uint8 par_ok;
} as5045_t;

static uint32 g_encIrqMaxCy, g_encRetries, g_encRejected, g_encMaxGapMs, g_followPeakLsb;
static uint32 as5045_read_raw(void)
{
    uint32 value = 0u;
    uint8 i;
    uint8 ints;
    uint32 started, duration;

    ints = CyEnterCriticalSection();
    started = SU_NOW();
    CyPins_ClearPin(PIN_CSN);
    delay_us(2u);

    for (i = 0u; i < 18u; i++)
    {
        CyPins_ClearPin(PIN_CLK);
        delay_us(SSI_HALF_US);
        CyPins_SetPin(PIN_CLK);
        delay_us(SSI_SAMPLE_US);
        value <<= 1;
        if (CyPins_ReadPin(PIN_DO) != 0u) { value |= 1u; }
        delay_us(SSI_HALF_US - SSI_SAMPLE_US);
    }

    CyPins_SetPin(PIN_CSN);
    duration=SU_ELAPSED(started);
    CyExitCriticalSection(ints);
    if(duration>g_encIrqMaxCy) { g_encIrqMaxCy=duration; }
    delay_us(2u);
    return (value & FRAME_MASK);
}

/* Daisy chain diagnostic: 2 x 19 clocks (18 data + 1 dummy per device).
 * Board 1 (wired to the MCU) comes first, board 2 second. Not used by the
 * closed loop; that still reads 18 bits from board 1 only. */
static void as5045_read_chain(uint32 *first, uint32 *second)
{
    uint32 a = 0u, b = 0u;
    uint8 i, ints;
    uint32 started, cost;

    CyPins_ClearPin(PIN_CSN);
    delay_us(2u);
    for (i = 0u; i < 38u; i++)
    {
        uint32 bit;
        ints=CyEnterCriticalSection(); started=SU_NOW();
        CyPins_ClearPin(PIN_CLK);
        delay_us(SSI_HALF_US);
        CyPins_SetPin(PIN_CLK);
        delay_us(SSI_SAMPLE_US);
        bit = (CyPins_ReadPin(PIN_DO) != 0u) ? 1u : 0u;
        if (i < 18u)       { a = (a << 1) | bit; }
        else if (i >= 19u && i < 37u) { b = (b << 1) | bit; } /* skip dummy bits 18 and 37 */
        delay_us(SSI_HALF_US - SSI_SAMPLE_US);
        cost=SU_ELAPSED(started); if(cost>g_encIrqMaxCy) { g_encIrqMaxCy=cost; }
        CyExitCriticalSection(ints);
    }
    CyPins_SetPin(PIN_CSN);
    delay_us(2u);
    *first = a & FRAME_MASK;
    *second = b & FRAME_MASK;
}

static uint8 frame_parity_ok(uint32 frame)
{
    uint32 value = frame & FRAME_MASK;
    value ^= value >> 16;
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return ((value & 1u) == 0u) ? 1u : 0u;
}

/* Shortest signed distance on the 12-bit circle. This is what makes the
 * multi-turn accumulation work across the 4095 -> 0 wrap. */
static int16_t angle_delta(uint16 now, uint16 previous)
{
    int16_t delta = (int16_t)now - (int16_t)previous;
    if (delta >  2048) { delta = (int16_t)(delta - 4096); }
    if (delta < -2048) { delta = (int16_t)(delta + 4096); }
    return delta;
}

static void as5045_decode(as5045_t *sample, uint32 raw)
{
    sample->raw     = raw;
    sample->angle   = (uint16)((sample->raw >> 6) & ENCODER_MASK);
    sample->ocf     = (uint8)((sample->raw >> 5) & 1u);
    sample->cof     = (uint8)((sample->raw >> 4) & 1u);
    sample->lin     = (uint8)((sample->raw >> 3) & 1u);
    sample->mag_inc = (uint8)((sample->raw >> 2) & 1u);
    sample->mag_dec = (uint8)((sample->raw >> 1) & 1u);
    sample->par     = (uint8)(sample->raw & 1u);
    sample->par_ok  = frame_parity_ok(sample->raw);
}

static void as5045_read(as5045_t *sample) { as5045_decode(sample, as5045_read_raw()); }

#define ST_OK                       (0u)
#define ST_DO_HIGH                  (1u)
#define ST_DO_LOW                   (2u)
#define ST_PARITY                   (3u)
#define ST_OCF                      (4u)
#define ST_COF                      (5u)
#define ST_FIELD                    (6u)
#define ST_LIN                      (7u)

static uint8 classify(const as5045_t *sample)
{
    if (sample->raw == FRAME_MASK)                     { return ST_DO_HIGH; }
    if (sample->raw == 0ul)                            { return ST_DO_LOW; }
#if (PARITY_STRICT != 0u)
    if (sample->par_ok == 0u)                          { return ST_PARITY; }
#endif
    if (sample->ocf == 0u)                             { return ST_OCF; }
    if (sample->cof != 0u)                             { return ST_COF; }
    if (sample->lin != 0u)                             { return ST_LIN; }
    if ((sample->mag_inc != 0u) &&
        (sample->mag_dec != 0u))                       { return ST_FIELD; }
    return ST_OK;
}

static const char *state_name(uint8 code)
{
    switch (code)
    {
        case ST_OK:      return "OK";
        case ST_DO_HIGH: return "DO stuck HIGH";
        case ST_DO_LOW:  return "DO stuck LOW";
        case ST_PARITY:  return "parity FAIL";
        case ST_OCF:     return "OCF=0 not ready";
        case ST_COF:     return "COF=1 overflow/off-centre";
        case ST_FIELD:   return "magnetic field warning (angle valid)";
        case ST_LIN:     return "LIN=1 magnet alignment/linearity alarm";
        default:         return "unknown";
    }
}

/* The AS5045 status table marks INC=DEC=1 with OCF=1/COF=LIN=0 as valid
 * angle data (the yellow range when the red-yellow-green OTP option is set).
 * Keep reporting it, but do not force arm1 open loop solely for this warning. */
static uint8 encoder_state_valid(uint8 code)
{
    return ((code == ST_OK) || (code == ST_FIELD)) ? 1u : 0u;
}

/* ====================== multi-turn position tracking ==================== */
/* Command and measured rotor state are independent for each axis. */
#define ENC_MAX_GAP_MS (100u)
#define ENC_JUMP_MARGIN_LSB (SLIP_FAIL_LSB)
/* The rotor must never move half a turn (2048 LSB) between two accepted samples, or the wrap direction is
 * ambiguous: the fastest configured rate over the longest accepted gap, plus the jump margin, has to fit. */
_Static_assert((MOVE_RATE_MAX_USTEP_S * ENC_MAX_GAP_MS / 1000u) * LSB_PER_USTEP_X100 / 100 + ENC_JUMP_MARGIN_LSB < ENCODER_CPR / 2,
               "multi-turn wrap becomes ambiguous: lower MOVE_RATE_MAX_USTEP_S or ENC_MAX_GAP_MS");
typedef struct
{
    int32  pos;         /* accumulated motor-side position, encoder LSB     */
    uint16 lastAngle;
    uint8  primed;
    uint8  badStreak;
    uint8  trackingLost;
    uint32 lastMs;
    int32 lastCmd;
    int32 cmdUsteps, cmdLsbX100;
    uint8 incDir, directionKnown;
    int32 softCenter, origin;
} enc_t;

static enc_t g_enc, g_enc2;
#define g_cmdUsteps (g_enc.cmdUsteps)
#define g_cmdLsbX100 (g_enc.cmdLsbX100)

/* One measurement update. Returns 0 when the frame was unusable; the caller
 * decides whether the streak is long enough to be a fault. */
static uint8 enc_update(enc_t *e, const as5045_t *sample, uint8 valid)
{
    uint32 now, gap;
    int32 delta;
    int64 commanded, position;

    if(e->trackingLost) { e->badStreak=ENC_BAD_LIMIT; return 0u; }

    if (valid == 0u)
    {
        if (e->badStreak < 0xFFu) { e->badStreak++; }
        return 0u;
    }
    e->badStreak = 0u;

    if (e->primed == 0u)
    {
        e->lastAngle = sample->angle;
        e->primed = 1u;
    }
    else
    {
        now=wall_ms(); gap=now-e->lastMs;
        if(gap>g_encMaxGapMs) { g_encMaxGapMs=gap; }
        delta=(int32)angle_delta(sample->angle, e->lastAngle);
        commanded=(int64)e->cmdUsteps-e->lastCmd;
        if(commanded<0) { commanded=-commanded; }
        position=(int64)e->pos+delta;
        /* At the configured maximum 6000 usteps/s, 100 ms plus the margin
         * stays below half a motor turn. A lost turn cannot be reconstructed. */
        if(gap>ENC_MAX_GAP_MS ||
           abs_i32(delta)>(int32)((MOVE_RATE_MAX_USTEP_S*(uint32)LSB_PER_USTEP_X100*(gap+1u))/100000u)+ENC_JUMP_MARGIN_LSB ||
           abs_i32(delta)>commanded*LSB_PER_USTEP_X100/100+ENC_JUMP_MARGIN_LSB ||
           position>INT32_MAX/100 || position<INT32_MIN/100) {
            e->trackingLost=1u; e->badStreak=ENC_BAD_LIMIT; g_encRejected++;
            return 0u;
        }
        e->pos = (int32)position;
        e->lastAngle = sample->angle;
    }
    e->lastMs=wall_ms(); e->lastCmd=e->cmdUsteps;
    {
        int64 following=(int64)e->cmdLsbX100/100-e->pos;
        if(following<0) { following=-following; }
        if((uint64_t)following>g_followPeakLsb) { g_followPeakLsb=(uint32)following; }
    }
    return 1u;
}

/* One shared 38-clock transaction; a bad axis never advances its position. */
static uint8 enc_sample(void)
{
    as5045_t a, b;
    uint32 raw1, raw2;
    uint8 i, v1=0u, v2=0u, ok1, ok2;
    for(i=0u;i<ENCODER_READ_RETRIES;i++) {
        if(i) { g_encRetries++; }
        as5045_read_chain(&raw1,&raw2);
        as5045_decode(&a,raw1); as5045_decode(&b,raw2);
        v1=encoder_state_valid(classify(&a)); v2=encoder_state_valid(classify(&b));
        if(v1 && (!ARM2_ENCODER_FEEDBACK || v2)) { break; }
    }
    ok1=enc_update(&g_enc,&a,v1); ok2=enc_update(&g_enc2,&b,v2);
    return ok1 && (!ARM2_ENCODER_FEEDBACK || ok2);
}

/* Average 12 samples relative to the first angle so the average also works
 * when the wrap lies inside the group. Used once, to set the origin. */
static uint8 enc_set_origin(void)
{
    uint16 reference[2]={0u,0u};
    int32 sum[2]={0,0};
    uint8 i, axis;
    uint32 raw[2];
    as5045_t sample;
    for(i=0u;i<12u;i++) {
        as5045_read_chain(&raw[0],&raw[1]);
        for(axis=0u;axis<(ARM2_ENCODER_FEEDBACK ? 2u : 1u);axis++) {
            as5045_decode(&sample,raw[axis]);
            if(!encoder_state_valid(classify(&sample))) { return 0u; }
            if(!i) { reference[axis]=sample.angle; }
            sum[axis]+=angle_delta(sample.angle,reference[axis]);
        }
        CyDelay(1u);
    }
    for(axis=0u;axis<(ARM2_ENCODER_FEEDBACK ? 2u : 1u);axis++) {
        enc_t *e=axis ? &g_enc2 : &g_enc;
        e->lastAngle=(uint16)(((int32)reference[axis]+sum[axis]/12+ENCODER_CPR)%ENCODER_CPR);
        e->pos=0; e->primed=1u; e->badStreak=0u; e->trackingLost=0u;
        e->lastMs=wall_ms(); e->lastCmd=e->cmdUsteps;
    }
    return 1u;
}

/* ========================== commanded position ========================== */

static int32 cmd_lsb(void)      { return g_cmdLsbX100 / 100; }

static void cmd_reset_axis(enc_t *e)
{
    e->cmdLsbX100 = e->pos * 100;
    e->cmdUsteps  = lsb_to_usteps(e->pos);
    e->lastCmd=e->cmdUsteps;
}

static void cmd_reset_to_measured(void) { cmd_reset_axis(&g_enc); }

/* commanded - measured, with the AS5045 output delay taken out.
 * At 6000 ustep/s the 384 us slow-mode delay alone is 6 LSB, which would
 * otherwise eat 20% of the slip warning margin. */
static int32 slip_axis(enc_t *e, int8 sign, uint32 periodUs)
{
    int32 lag = 0;

#if (LAG_COMPENSATION != 0u)
    if (periodUs > 0u)
    {
        lag = ((int32)AS5045_DELAY_US * LSB_PER_USTEP_X100) /
              ((int32)periodUs * 100);
        if (sign < 0) { lag = -lag; }
    }
#else
    (void)sign; (void)periodUs;
#endif
    return e->cmdLsbX100/100 - (e->pos + lag);
}

/* ============================== LED status ============================== */
/* ============================= DRV8825 #2 =============================== */
static uint8 g_last_dir_level = 0xFFu;
#define g_encIncDir (g_enc.incDir)
static uint8 g_arm1HomeActive;
static uint8 g_arm2HomeActive;

static void stepper2_disable(void)
{
    pin_write(PIN_EN2, 1u);
    pin_write(PIN_SLEEP2, 0u);
    pin_write(PIN_RST2, 0u);
    pin_write(PIN_STEP2, 1u);
    g_last_dir_level = 0xFFu;
}

static void stepper2_enable(void)
{
    pin_write(PIN_RST2, 1u);
    pin_write(PIN_SLEEP2, 1u);
    CyDelay(2u);                      /* DRV8825 needs 1.7 ms wake-up       */
    pin_write(PIN_EN2, 0u);
    CyDelay(2u);
}

/* ============================= DRV8825 #1 =============================== */
/* STEP1 idles LOW and pulses HIGH (DRV8825 steps on the rising edge), so
 * the shared blue LED is dark when arm2 is at rest and glows while it moves. */
static void stepper1_disable(void)
{
    pin_write(PIN_EN1, 1u);
    pin_write(PIN_SLEEP1, 0u);
    pin_write(PIN_RST1, 0u);
    pin_write(PIN_STEP1, 0u);
}

static void stepper1_enable(void)
{
    pin_write(PIN_STEP1, 0u);
    pin_write(PIN_RST1, 1u);
    pin_write(PIN_SLEEP1, 1u);
    CyDelay(2u);                      /* DRV8825 needs 1.7 ms wake-up       */
    pin_write(PIN_EN1, 0u);
    CyDelay(2u);
}

static uint32 step_period_us(uint32 rate)
{
    uint32 period;
    if (rate == 0u) { rate = 1u; }
    period = 1000000ul / rate;
    if (period <= STEP_LOW_US) { period = STEP_LOW_US + 1u; }
    return period;
}

/* Pulse edges are scheduled against the free-running SysTick: start now, end
 * at an absolute deadline, so software overhead cannot stretch the width. */
static void pulse_hold_us(uint32 tEdge, uint32 widthUs)
{
    uint32 target = widthUs * SU_CY_PER_US;
    while (((tEdge - SU_NOW()) & 0x00FFFFFFu) < target) { }
}

static void pulse_step_pin(uint32 pinPC, uint8 idleHigh)
{
    uint32 t0 = SU_NOW();
    pin_write(pinPC, (uint8)((idleHigh != 0u) ? 0u : 1u));
    pulse_hold_us(t0, STEP_LOW_US);
    pin_write(pinPC, idleHigh);
}

/* ========================= trapezoidal ramp ============================= */
/* AVR446 integer recurrence: c0 = 0.676e6 * sqrt(2/a) us, then
 * c(n+1) = c(n) - (2*c(n) + rest)/(4n+1). One divide per microstep, no
 * floats.  The `rest` term is not optional: without it the truncated
 * division stalls the ramp at about 2800 ustep/s and it never reaches the
 * speed cap. */
typedef struct
{
    uint32 c;           /* current step period, us                          */
    uint32 c0;          /* first step period                                */
    uint32 cMin;        /* cruise period = speed cap                        */
    int32  n;           /* >0 accelerating, <0 decelerating                 */
    int32  rest;        /* carried division remainder                       */
    int32  total;       /* 0: accelerate and cruise until an external stop */
    int32  decelStart;
    uint8  cruising;
} ramp_t;

static uint32 isqrt32(uint32 v)
{
    uint32 rem = 0u;
    uint32 root = 0u;
    int8 i;

    for (i = 15; i >= 0; i--)
    {
        root <<= 1;
        rem = (rem << 2) | (v >> 30);
        v <<= 2;
        if (root < rem) { rem -= root | 1u; root += 2u; }
    }
    return root >> 1;
}

static void ramp_init(ramp_t *r, int32 total, uint32 rateMax, uint32 accel)
{
    uint32 sq = isqrt32(accel);

    r->total      = total;
    r->cMin       = step_period_us(rateMax);
    r->c0         = (sq < 2u) ? 60000ul : (956000ul / sq);
    if (r->c0 > 60000ul)  { r->c0 = 60000ul; }
    if (r->c0 < r->cMin)  { r->c0 = r->cMin; }
    r->c          = r->c0;
    r->n          = 0;
    r->rest       = 0;
    r->cruising   = 0u;
    r->decelStart = total / 2;
}

static void ramp_advance(ramp_t *r, int32 done)
{
    int32 denom;
    int32 num;

    if ((r->total == 0) && (r->cruising != 0u)) { return; }
    if ((r->total > 0) && (done >= r->decelStart))
    {
        if (r->n > 0) { r->n = -(r->total - done); r->rest = 0; }
        else          { r->n++; }
        if (r->n >= 0) { r->n = -1; }
    }
    else
    {
        r->n++;
    }

    denom = (4 * r->n) + 1;
    if (denom != 0)
    {
        num = (2 * (int32)r->c) + r->rest;
        r->c = (uint32)((int32)r->c - (num / denom));
        r->rest = num % denom;
    }

    if (r->c < r->cMin)
    {
        r->c = r->cMin;
        r->rest = 0;
        /* First time at cruise: the deceleration ramp must mirror the
         * acceleration ramp that was actually used, not total/2. */
        if ((r->cruising == 0u) && (r->n > 0))
        {
            r->cruising = 1u;
            if ((r->total - done) > done) { r->decelStart = r->total - done; }
        }
    }
    if (r->c > r->c0) { r->c = r->c0; r->rest = 0; }
}

/* --------------------------- hardware setup ----------------------------- */
static void hw_init(void)
{
    CyGlobalIntEnable;

    pin_out_init(PIN_CSN, 1u);
    pin_out_init(PIN_CLK, 1u);
    CY_SET_REG8(PIN_DO, CY_PINS_DM_DIG_HIZ);
    pin_out_init(PIN_LED, 0u);

    /* Driver 1 shares its STEP with the blue LED. Keep it asleep, reset and
     * disabled during start-up so the status LED can never move stepper1;
     * jog mode wakes it only after the LED is no longer used. */
    pin_out_init(PIN_EN1,    1u);
    pin_out_init(PIN_SLEEP1, 0u);
    pin_out_init(PIN_RST1,   0u);
    pin_out_init(PIN_DIR1,   0u);
    pin_out_init(PIN_M1_2,   M2_LVL);
    pin_out_init(PIN_M1_1,   M1_LVL);
    pin_out_init(PIN_M1_0,   M0_LVL);

    pin_out_init(PIN_EN2,    1u);
    pin_out_init(PIN_STEP2,  1u);
    pin_out_init(PIN_DIR2,   0u);
    pin_out_init(PIN_SLEEP2, 0u);
    pin_out_init(PIN_RST2,   0u);
    pin_out_init(PIN_M2_2,   M2_LVL);
    pin_out_init(PIN_M2_1,   M1_LVL);
    pin_out_init(PIN_M2_0,   M0_LVL);
    CY_SET_REG8(PIN_FAULT, CY_PINS_DM_RES_UP | CY_PINS_PC_DATAOUT);
    CY_SET_REG8(PIN_LIMIT1, CY_PINS_DM_RES_UP | CY_PINS_PC_DATAOUT);
    CY_SET_REG8(PIN_LIMIT2, CY_PINS_DM_RES_UP | CY_PINS_PC_DATAOUT);
    CY_SET_REG8(PIN_ARM2_NPN, CY_PINS_DM_RES_UP | CY_PINS_PC_DATAOUT);

#if (USE_DEBUG_UART != 0u)
    /* Creator owns the UART pin routing; never overwrite its PC registers. */
    HostUart_Start();
    CyGlobalIntEnable;
#endif
    su_timer_start();
}

/* ============================ keyboard jog ============================== */
/* Hold-to-move jogging of both joints from a PC keyboard.  The host script
 * (jog.ps1) samples the keys with GetAsyncKeyState and writes one state byte
 * every 40 ms to the KitProg USB-UART, which arrives here on P12[6].  That
 * byte is also the heartbeat: if none arrives for JOG_DEADMAN_MS every axis
 * is stopped and requires explicit enable, so a lost host cannot
 * never leave a joint running.
 *
 * Byte protocol.  Every byte is ODD (bit 0 set): the receiver polls the
 * line and may see the start bit late, so instead of trusting the falling
 * edge it waits for the RISING edge that ends the start bit (which exists
 * exactly one bit later only because d0 = 1) and samples d1..d7 from that.
 * A start bit is only accepted after >= 1.5 bit times of idle, so a data
 * bit's falling edge inside a frame can never be mistaken for a start bit.
 * The host therefore sends every byte three times as SEPARATE writes about
 * 1 ms apart, never back to back.
 *   0x41 | (mask << 1)   mask bit0 arm1 left, bit1 arm1 right,
 *                             bit2 arm2 left, bit3 arm2 right   (heartbeat)
 *   JOG_CMD_ENABLE explicitly recheck and enable; JOG_CMD_STOP latches stop.
 *   Legacy direction inversion commands are rejected in this stage.
 *   JOG_CMD_HOME   home arm1 at J4-2, then arm2 at the NPN sensor
 *   's'           positions + receive statistics
 *   '?'           help
 *   '!'           arm the RX edge capture (diagnostic)
 * Command bytes repeated within JOG_CMD_REPEAT_MS are the host's copies and
 * are ignored, so '1' cannot toggle three times.
 *
 * Motion: each axis has a linear velocity ramp updated every JOG_TICK_US
 * (accelerate while the key is held, decelerate to zero when released or
 * reversed).  Steps are scheduled on SysTick from a tight non-blocking loop
 * that drains the UART receive buffer. One encoder frame (about 115 us)
 * masks interrupts, less than the four-byte RX FIFO time at 115200 baud.
 * TX enqueues without waiting for a byte to finish on the wire.
 *
 * arm1 keeps the dual-track slip monitor: slip >= SLIP_WARN_LSB halves the
 * speed cap for the rest of that key press, slip >= SLIP_FAIL_LSB stops the
 * entire stage until explicitly re-enabled.  After every stop arm1 re-anchors the
 * commanded track to the measurement and holds that position closed-loop.
 * After polarity learning, soft limits use measured position; after home
 * their center is the verified straight position, while zero stays at P0[3].
 *
 * arm2 shares encoder validation, following protection and jog hold with arm1.
 */

/* Receive statistics: accepted heartbeats, overflow events, UART errors,
 * and bytes outside the command protocol. LINK reports transport counters. */
static uint32 g_rxGood;
static uint32 g_rxLate;
static uint32 g_rxFraming;
static uint32 g_rxRejected;
static uint8  g_rxLastBad[8];   /* last rejected values, ring          */
static uint8  g_rxLastBadIdx;

/* Raw byte rejections share telemetry; rejected decoded frames keep their own cause. */
static void jog_reject_byte(uint8 b)
{
    g_rxRejected++;
    g_rxLastBad[g_rxLastBadIdx] = b;
    g_rxLastBadIdx = (uint8)((g_rxLastBadIdx + 1u) % 8u);
}

/* --------------------- non-blocking debug output ------------------------ */
/* Queue complete messages; feed the interrupt-driven UART without blocking. */
static void txq_flush_blocking(void) __attribute__((unused));
#if (USE_DEBUG_UART != 0u)
#define TXQ_SIZE                    (2048u)
static char   g_txq[TXQ_SIZE];
static uint16 g_txqHead;
static uint16 g_txqTail;
static uint16 g_txqDropped;

static void txq_puts(const char *s)
{
    /* Drop a whole message, never a partial ACK that can corrupt the next line. */
    uint16 free = (uint16)((g_txqTail + TXQ_SIZE - g_txqHead - 1u) % TXQ_SIZE);
    if (strlen(s) > free) { g_txqDropped++; return; }
    while (*s != '\0')
    {
        uint16 next = (uint16)((g_txqHead + 1u) % TXQ_SIZE);
        g_txq[g_txqHead] = *s;
        g_txqHead = next;
        s++;
    }
}

static uint8 txq_pending(void)
{
    return (g_txqHead != g_txqTail) ? 1u : 0u;
}

static uint16 txq_free(void)
{
    return (uint16)((g_txqTail + TXQ_SIZE - g_txqHead - 1u) % TXQ_SIZE);
}

static void txq_send_one(void)
{
    if (g_txqHead != g_txqTail && HostUart_GetTxBufferSize() < HostUart_TX_BUFFER_SIZE - 1u)
    {
        HostUart_PutChar((uint8)g_txq[g_txqTail]);
        g_txqTail = (uint16)((g_txqTail + 1u) % TXQ_SIZE);
    }
}

static void txq_flush_blocking(void)
{
    uint32 start = wall_ms();
    while (txq_pending() != 0u) {
        if (wall_ms()-start >= 100u) {
            g_txqTail=g_txqHead; g_txqDropped++; return;
        }
        txq_send_one();
    }
}

#define JLOG(s)      txq_puts(s)
#define JLOGF(...)   do { snprintf(g_dbg, sizeof(g_dbg), __VA_ARGS__); \
                          txq_puts(g_dbg); } while (0)

/* UART component RX ISR fills its 256-byte ring. Parsing stays in the main
 * loop. Check both hardware errors and the separate software-overflow flag
 * atomically: GetByte() alone does NOT report software ring overflow. */
static int16 uart_rx_service(void)
{
    uint16 received = 0u;
    uint8 overflow, errors = 0u;
    int16 result = -1;
    uint8 ints = CyEnterCriticalSection();
    overflow = HostUart_rxBufferOverflow;
    if (HostUart_GetRxBufferSize() != 0u || HostUart_errorStatus != 0u || overflow != 0u)
    {
        received = HostUart_GetByte();
        errors = (uint8)(received >> 8);
        if (errors != 0u || overflow != 0u)
        {
            if (overflow || (errors & HostUart_RX_STS_OVERRUN)) { g_rxLate++; }
            if (errors & (HostUart_RX_STS_BREAK | HostUart_RX_STS_PAR_ERROR | HostUart_RX_STS_STOP_ERROR)) { g_rxFraming++; }
            HostUart_ClearRxBuffer();
            result = (!overflow && errors == HostUart_RX_STS_STOP_ERROR) ? -3 : -2;
        }
        else { result = (int16)(received & 0xffu); }
    }
    CyExitCriticalSection(ints);
    if (result <= -2)
    {
        JLOGF("UART RX fault: flags=0x%02X sw_overflow=%u break=%u parity=%u framing=%u overrun=%u\r\n",
              (unsigned)errors, (unsigned)overflow,
              (unsigned)((errors & HostUart_RX_STS_BREAK) != 0u),
              (unsigned)((errors & HostUart_RX_STS_PAR_ERROR) != 0u),
              (unsigned)((errors & HostUart_RX_STS_STOP_ERROR) != 0u),
              (unsigned)((errors & HostUart_RX_STS_OVERRUN) != 0u));
    }
    return result;
}

static void uart_discard_rx(void)
{
    uint8 ints = CyEnterCriticalSection();
    HostUart_ClearRxBuffer();
    HostUart_errorStatus = 0u;
    CyExitCriticalSection(ints);
}
#else
static uint8 txq_pending(void)       { return 0u; }
static uint16 txq_free(void)         { return 0u; }
static void  txq_send_one(void)      { }
static void  txq_flush_blocking(void){ }
static int16 uart_rx_service(void)     { return -1; }
static void uart_discard_rx(void)      { }
#define JLOG(s)      do { } while (0)
#define JLOGF(...)   do { } while (0)
#endif

/* ----------------------------- jog axes --------------------------------- */
#define JOG_TICK_CY                 (JOG_TICK_US * SU_CY_PER_US)
#define JOG_ENC_SAMPLE_CY           (JOG_ENC_SAMPLE_US * SU_CY_PER_US)
#define JOG_ENC_READ_US             (360u)   /* dual AS5045 frame + GPIO overhead margin */
#define JOG_TX_BYTE_US              (10u)    /* enqueue only; UART shifts independently */

typedef struct
{
    const char *name;
    uint32 pinStep;
    uint32 pinDir;
    uint8  stepIdleHigh;    /* 1: idle high / low pulse (stepper2 style)   */
    uint8  hasEnc;          /* both axes: closed track on AS5045            */
    uint8  leftDirLevel;    /* open-loop axes: DIR level that means "left" */
    uint8  lastDirLevel;    /* 0xFF = unknown, forces a DIR write          */
    uint8  invert;          /* run-time direction inversion ('1' / '2')    */
    int8   want;            /* from the keys: -1 left, +1 right, 0 stop    */
    int8   moving;          /* sign while stepping, 0 at rest              */
    uint8  latched;         /* stopped by slip: ignore keys until released */
    uint8  limitHit;        /* soft limit message already printed          */
    uint32 rate;            /* current ustep/s                             */
    uint32 rateCap;         /* cap for this press, halved on slip warnings */
    uint32 rateMax;
    uint32 rateStart;
    uint32 accel;           /* ustep/s^2                                   */
    uint32 decel;
    uint32 rampRemainder;   /* fractional rate increment, per 1 ms tick */
    uint32 periodCy;        /* BUS_CLK cycles between steps at `rate`      */
    uint32 tLastStep;
    int32  usteps;          /* net microsteps since power-up, right = +    */
    int32  peakSlip;        /* worst slip of the current press, LSB        */
    uint8  derates;
    uint16 settleMs;        /* >0: just stopped, rotor still ringing       */
    uint8  holding;         /* closed-loop hold at holdTarget        */
    int32  holdTarget;
    uint32 holdCooldownMs;
    uint32 statusMs;        /* ms since the last status line               */
} jog_axis_t;

static jog_axis_t g_arm1;
static jog_axis_t g_arm2;
static enc_t *axis_encoder(const jog_axis_t *a) { return a==&g_arm2 ? &g_enc2 : &g_enc; }
static int32 encoder_deg100(const jog_axis_t *a, int32 lsb) {
    return (int32)((int64)lsb*3600000/(ENCODER_CPR*(a==&g_arm2 ? ARM2_GEAR_RATIO_X100 : GEAR_RATIO*100)));
}
static uint8 dual_ready(void) {
    return g_arm1.hasEnc && g_enc.directionKnown && !g_enc.badStreak &&
        (!ARM2_ENCODER_FEEDBACK || (g_arm2.hasEnc && g_enc2.directionKnown && !g_enc2.badStreak));
}
static uint32 g_jogMs;          /* latest monotonic hardware time, not loop ticks */
static uint32 g_loopMaxMs, g_tickMaxLagMs;
static uint32 g_lastKeyMs;      /* last valid key byte                   */
static uint8  g_keysSeen;

/* Faults require an explicit C command after the cause is cleared. */
static uint8  g_jogFaulted;
static uint32 g_jogFaultCount;
static uint8  g_jogKeysIdle = 1u;  /* last key byte had no key down (or host gone) */
static uint8  g_limitStop;
static uint8  g_homeNeedKeyRelease;
static uint8  g_arm1HomeLatched;
static uint8  g_arm2NpnLatched;
static uint8  g_motionEnabled;
#define g_directionKnown (g_enc.directionKnown)
static uint8  g_homed1;
static uint8  g_homed2;
#define g_arm1SoftCenter (g_enc.softCenter)
/* Volatile operator settings: boot restores defaults; stop/enable retain them.  Index 0/1 bound the link angles
 * q1/q2; index 2 bounds the elbow q2 - q1, which is what motor 2 actually travels. */
#define JOINT_ELBOW                 (2u)
static int16 g_jointMinCd[3] = { PTP_MIN1_CD, PTP_MIN2_CD, PTP_MINE_CD };
static int16 g_jointMaxCd[3] = { PTP_MAX1_CD, PTP_MAX2_CD, PTP_MAXE_CD };
static uint8 g_limitsAccepted[15];
static uint16 g_motionRate[2] = { JOG1_RATE_MAX_USTEP_S, JOG2_RATE_MAX_USTEP_S };
static uint16 g_motionAccel[2] = { JOG1_ACCEL_USTEP_S2, JOG2_ACCEL_USTEP_S2 };
static uint8 g_ratesAccepted[15];
/* Position telemetry stream (control type 11).  0 = the TELEM line only rides in the 500 ms status block, as before;
 * 5..20 Hz = a standalone TELEM line at that rate while enabled.  It never takes the last half of the transmit queue:
 * when the queue is that full the line is skipped (counted), so ACKs and stop messages keep their room. */
#define TELEM_HZ_MIN                (5u)
#define TELEM_HZ_MAX                (20u)
#define TELEM_MIN_FREE_BYTES        (1024u)
static uint8  g_telemHz;
static uint8  g_telemAccepted[15];
static uint32 g_telemLastMs, g_telemSent, g_telemSkipped, g_telemMaxCy;
static uint32 motion_min(uint32 a, uint32 b) { return a < b ? a : b; }

static struct {
    uint8 active; /* 1 moving, 2 settling */
    uint8 reference;
    uint8 referenceSource; /* 1 switch homing, 2 operator-defined straight pose */
    int32 encoderOrigin; /* arm1 compatibility field */
    uint32 epoch;
    uint32 sequence;
    uint32 sinceMs, settleMs, lastStep, periodUs;
    int32 origin[2], target[2];
    uint32 distance[2], error[2], total, done;
    int8 sign[2];
    uint8 accepted[15];
    ramp_t ramp;
} g_ptp = { .epoch = 1u };
static struct {
    uint8 active, count, last;
    uint8 bytes[PTP_FRAME_BYTES];
    uint32 sinceMs, lastMs;
} g_ptpRx;
static struct {
    uint8 active, count, last;
    uint8 bytes[PTP_SHORT_BYTES];
    uint32 sinceMs, lastMs;
} g_shortRx;
static uint32 g_shortRxOk;      /* accepted compact heartbeats */
static uint32 g_ptpRxDropped;
static uint32 g_ptpRxCrc;       /* dropped: CRC mismatch                  */
static uint32 g_ptpRxShort;     /* dropped: frame ended with too few bytes */
static uint32 g_ptpRxTimeout;   /* dropped: no terminator within the frame window */
static uint32 g_ptpEpochRejected; /* control frames refused on a stale epoch */
static uint32 g_epochAnnounceMs;  /* throttle for the epoch re-announcement */
static uint8  g_epochAnnounceSeen;
static uint8 g_uploadRxQuarantine;

/* Heartbeat margin telemetry.  The deadman is only checked while motion is
 * enabled, so the worst interval is measured over exactly that window and is
 * reset by ENABLE: it reports "how close did this run come to 250 ms". */
static uint32 g_keyMaxGapMs;

/* Whole-path preload: no movement until every numbered node is accepted. */
#define TRAJ_CAPACITY 2048u
#define TRAJ_RATE_MAX 4000u
#define TRAJ_ACCEL_MAX 3000u
/* Never pulse a trajectory faster than its own rate cap, even when catching up. */
#define TRAJ_STEP_MIN_US (1000000u / TRAJ_RATE_MAX)
typedef struct { int16 q1, q2; uint16 ms; } traj_node_t;
static struct {
    uint8 state; /* 0 empty, 1 uploading, 2 ready, 3 running, 4 done */
    uint16 count, expected, index;
    uint32 durationMs, elapsedCy, clock;
    int32 velocity[2];
    traj_node_t nodes[TRAJ_CAPACITY];
} g_traj;

typedef enum { HOME_IDLE, HOME_RELEASE, HOME_BACKOFF, HOME_FAST,
               HOME_SLOW, HOME_SETTLE, HOME_STRAIGHT, HOME_STRAIGHT_SETTLE } home_phase_t;
static struct {
    home_phase_t phase;
    home_phase_t afterBackoff;
    uint8 arm;
    uint8 confirming;
    uint32 sinceMs;
    uint32 sensorMs;
    uint32 steps;
    uint32 lastStep;
    uint32 periodUs;
    int8 direction;
    int32 probePos;
    int32 probeSteps;
    uint32 probeNextSteps;
    uint8 zeroSamples;
    uint16 zeroReference;
    int32 zeroSum;
    uint32 zeroSampleMs;
    ramp_t ramp;
} g_home;

static void jog_print_inputs(void);
static void stage_status(void);

static uint8 host_lease_fresh(void)
{
    return g_keysSeen && g_jogMs-g_lastKeyMs<=JOG_DEADMAN_MS;
}

/* Renewing the lease is deliberately separate from applying a control frame.  The
 * lease answers "is the host alive", not "is the host on the current reference
 * epoch"; conflating the two is what made the epoch change at the start of homing
 * expire the lease and stop the machine. */
static void host_lease_renew(void)
{
    if ((g_keysSeen != 0u) && (g_motionEnabled != 0u))
    {
        uint32 gap = g_jogMs - g_lastKeyMs;
        if (gap > g_keyMaxGapMs) { g_keyMaxGapMs = gap; }
    }
    g_lastKeyMs = g_jogMs;
    g_keysSeen  = 1u;
}

/* Re-announce the reference epoch while a host is still sending frames for the
 * previous one.  Homing already publishes a full status, but that line can be
 * dropped when the transmit queue is full (tx_dropped is non-zero in the field), so
 * the host needs a cheap, repeated way back into sync.  Throttled, and harmless to
 * a host that does not know the line. */
static void ptp_announce_epoch(void)
{
    if ((g_epochAnnounceSeen != 0u) && ((g_jogMs - g_epochAnnounceMs) < PTP_EPOCH_ANNOUNCE_MS)) { return; }
    g_epochAnnounceSeen = 1u; g_epochAnnounceMs = g_jogMs;
    JLOGF("PTP_EPOCH %lu\r\n", (unsigned long)g_ptp.epoch);
}
static uint8 host_input_idle(void)
{
    return host_lease_fresh() && g_jogKeysIdle && !g_homeNeedKeyRelease;
}
static uint8 axes_idle(void)
{
    return !g_arm1.moving && !g_arm2.moving && !g_arm1.want && !g_arm2.want &&
           !g_arm1.settleMs && !g_arm2.settleMs;
}
static uint8 motion_inputs_clear(void)
{
    return !FAULT_ACTIVE() && !HARD_LIMIT_ACTIVE();
}
static uint8 home_inputs_clear(void)
{
    return !ARM1_HOME_ACTIVE() && !ARM2_NPN_ACTIVE();
}

static void jog_stop_now(jog_axis_t *a)
{
    a->want = 0;
    a->moving = 0;
    a->rate = 0u;
    a->settleMs = 0u;
    a->holding = 0u;
}

/* Forget the held trajectory (and any upload isolation) without touching the reference. */
static void traj_discard(void)
{
    if(g_uploadRxQuarantine) { uart_discard_rx(); }
    g_uploadRxQuarantine=0u;
    g_traj.state=0u; g_traj.count=g_traj.expected=g_traj.index=0u;
}

static void ptp_invalidate(void)
{
    traj_discard();
    g_ptp.active = 0u; g_ptp.reference = 0u; g_ptp.referenceSource = 0u; g_ptpRx.active = 0u;
    if (++g_ptp.epoch == 0u) { ++g_ptp.epoch; }
}

/* What the machine was doing when a stop began, as one key=value line printed BEFORE anything is reset.  It states
 * only what is known: the nFAULT pin cannot tell over-current from over-temperature or under-voltage, so a driver
 * stop says drv=nfault_undetermined and nothing more.  Positions are as in TELEM (relative to the reference). */
static const char *stop_phase(void)
{
    if (g_home.phase != HOME_IDLE) { return "home"; }
    if (g_ptp.active) { return g_traj.state == 3u ? "traj_run" : "ptp"; }
    if (g_traj.state == 1u) { return "traj_upload"; }
    if (g_arm1.moving || g_arm2.moving || g_arm1.want || g_arm2.want) { return "jog"; }
    return g_motionEnabled ? "idle" : "disabled";
}
#if ARM2_ENCODER_FEEDBACK
#define ENC2_LOG_FORMAT "e2=%ld err2=%ld "
#define ENC2_LOG_ARGS (long)((g_enc2.pos-g_enc2.origin)*(g_enc2.incDir==ARM2_LEFT_DIR_LEVEL?1:-1)), (long)(g_enc2.cmdLsbX100/100-g_enc2.pos),
#else
#define ENC2_LOG_FORMAT ""
#define ENC2_LOG_ARGS
#endif

static void fault_snapshot(const char *reason, uint8 driverFault)
{
    JLOGF("FAULT_SNAPSHOT t=%lu epoch=%lu seq=%lu phase=%s home=%u traj=%u idx=%u/%u ref=%u p1=%ld p2=%ld e1=%ld err=%ld "
        ENC2_LOG_FORMAT
        "inputs=%u hb_age_ms=%lu hb_max_gap_ms=%lu drv=%s reason=\"%s\"\r\n",
          (unsigned long)g_jogMs,(unsigned long)g_ptp.epoch,(unsigned long)g_ptp.sequence,stop_phase(),
          (unsigned)g_home.phase,(unsigned)g_traj.state,(unsigned)g_traj.index,(unsigned)g_traj.count,
          (unsigned)(g_ptp.reference && dual_ready()),
          (long)(g_ptp.origin[0]-g_arm1.usteps),(long)(g_ptp.origin[1]-g_arm2.usteps),
          (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),(long)(cmd_lsb()-g_enc.pos),
        ENC2_LOG_ARGS
          (unsigned)((HARD_LIMIT_ACTIVE()?1u:0u)|(ARM1_HOME_ACTIVE()?2u:0u)|(ARM2_NPN_ACTIVE()?4u:0u)|(FAULT_ACTIVE()?8u:0u)),
          (unsigned long)(g_keysSeen ? (g_jogMs - g_lastKeyMs) : 0u),(unsigned long)g_keyMaxGapMs,
          (driverFault != 0u || FAULT_ACTIVE()) ? "nfault_undetermined" : "ok", reason);
}

/* Protective stops retain holding current unless the driver itself faults.
 * The explicit operator STOP also disables both drivers at its command entry. */
static void stage_stop(const char *reason, uint8 driverFault)
{
    fault_snapshot(reason, driverFault);
    JLOGF("STOP context: home=%u arm=%u phase_steps=%lu key_age_ms=%lu key_max_gap_ms=%lu rx_good=%lu late=%lu rejected=%lu\r\n",
          (unsigned)g_home.phase, (unsigned)g_home.arm, (unsigned long)g_home.steps,
          (unsigned long)(g_jogMs - g_lastKeyMs), (unsigned long)g_keyMaxGapMs, (unsigned long)g_rxGood,
          (unsigned long)g_rxLate, (unsigned long)g_rxRejected);
    jog_stop_now(&g_arm1);
    jog_stop_now(&g_arm2);
    ptp_invalidate();
    g_home.phase = HOME_IDLE;
    g_arm1HomeActive = 0u;
    g_arm2HomeActive = 0u;
    g_motionEnabled = 0u;
    g_homed1 = 0u;
    g_homed2 = 0u;
    g_homeNeedKeyRelease = 1u;
    if (driverFault != 0u)
    {
        stepper1_disable();
        stepper2_disable();
        g_jogFaulted = 1u;
    }
    JLOGF("STOP: %s; position invalid, release keys then C to enable\r\n", reason);
    jog_print_inputs();
    stage_status();
}

static void jog_limit_stop(void)
{
    stage_stop("P0[4] global limit", 0u);
    g_limitStop = 1u;
    JLOG("LIMIT: clear input, release keys, then explicitly enable with C\r\n");
}

/* Called before either axis can issue another step. Each home input only
 * stops its own axis; J4-1 remains the global hard limit. */
static uint8 jog_safety_blocked(void)
{
    if (FAULT_ACTIVE() && (g_jogFaulted == 0u))
    {
        stage_stop("DRV8825 nFAULT; drivers OFF, arm is NOT held", 1u);
        g_jogFaultCount++;
        return 1u;
    }
    if (g_motionEnabled == 0u) { return 1u; }
    if (HARD_LIMIT_ACTIVE())
    {
        if (g_limitStop == 0u) { jog_limit_stop(); }
        return 1u;
    }
    if (g_limitStop != 0u) { return 1u; }
    if (g_home.phase != HOME_IDLE) { return 0u; }

    if ((ARM1_HOME_ACTIVE()) && (g_arm1HomeActive == 0u))
    {
        g_arm1.holding = 0u;
        if ((g_arm1HomeLatched == 0u) && (g_arm1.moving == ARM1_HOME_JOG_DIR))
        {
            g_arm1.moving = 0;
            g_arm1.rate = 0u;
            g_arm1.settleMs = 0u;
            g_arm1.latched = 1u;
            g_arm1HomeLatched = 1u;
            JLOG("ARM1 switch: home sensor detected; arm1 stopped, release key to back off\r\n");
        }
    }
    else
    {
        g_arm1HomeLatched = 0u;
    }

    if ((ARM2_NPN_ACTIVE()) && (g_arm2HomeActive == 0u))
    {
        if ((g_arm2NpnLatched == 0u) && (g_arm2.moving == ARM2_HOME_JOG_DIR))
        {
            g_arm2.moving = 0;
            g_arm2.rate = 0u;
            g_arm2.settleMs = 0u;
            g_arm2.latched = 1u;
            g_arm2NpnLatched = 1u;
            JLOG("ARM2 NPN: calibration sensor detected; arm2 stopped, release key to back off\r\n");
        }
    }
    else
    {
        g_arm2NpnLatched = 0u;
    }
    return 0u;
}

/* arm1 motion sign <-> encoder sign. k = +/-1 is an involution, so the same
 * helper maps a wanted encoder direction back to a motion sign. */
static int8 jog_enc_sign(const jog_axis_t *a, int8 s)
{
    enc_t *e=axis_encoder(a);
    int8 k = (a->leftDirLevel == e->incDir) ? -1 : 1;
    if (a->invert != 0u) { k = (int8)(-k); }
    return (int8)(s * k);
}

static uint8 jog_dir_level(const jog_axis_t *a, int8 s)
{
    /* Physical left/right never depends on an unverified encoder polarity.
     * Learn encoder polarity during the operator-requested home movement. */
    uint8 lvl = (s < 0) ? a->leftDirLevel : (uint8)(a->leftDirLevel == 0u);
    if (a->invert != 0u) { lvl = (uint8)(lvl == 0u); }
    return lvl;
}

static const char *jog_dir_name(int8 s)
{
    return (s < 0) ? "LEFT" : ((s > 0) ? "RIGHT" : "stop");
}

/* One microstep, s = +1 right / -1 left.  Never blocks beyond the pulse. */
static uint8 joint_step_allowed(const jog_axis_t *a, int8 s);
static void jog_step(jog_axis_t *a, int8 s)
{
    enc_t *e=axis_encoder(a);
    uint8 lvl = jog_dir_level(a, s);

    if ((g_motionEnabled == 0u) || FAULT_ACTIVE() || HARD_LIMIT_ACTIVE() ||
        ((a == &g_arm1) && (s == ARM1_HOME_JOG_DIR) &&
         ARM1_HOME_ACTIVE() && (g_arm1HomeActive == 0u)) ||
        ((a == &g_arm2) && (s == ARM2_HOME_JOG_DIR) &&
         ARM2_NPN_ACTIVE() && (g_arm2HomeActive == 0u)) || !joint_step_allowed(a,s)) { return; }
    if (lvl != a->lastDirLevel)
    {
        pin_write(a->pinDir, lvl);
        delay_us(DIR_SETUP_US);
        a->lastDirLevel = lvl;
    }
    pulse_step_pin(a->pinStep, a->stepIdleHigh);
    a->usteps += s;
    if (a->hasEnc != 0u)
    {
        int8 sign = jog_enc_sign(a, s);
        e->cmdLsbX100 += (int32)sign * LSB_PER_USTEP_X100;
        e->cmdUsteps  += sign;
    }
}

static uint8 home_sensor_active(uint8 arm)
{
    return (arm == 1u) ? ARM1_HOME_ACTIVE() : ARM2_NPN_ACTIVE();
}

static uint32 home_straight_steps(uint8 arm)
{
    if (arm == 1u)
    { return (uint32)abs_i32(lsb_to_usteps(out_deg10_to_lsb(ARM1_STRAIGHT_OUT_DEG10))); }
    /* Round to nearest pulse; 163 deg at 1/8 and 51.06:1 -> 36990 pulses. */
    return (uint32)(((int64)ARM2_STRAIGHT_OUT_DEG10 * ARM2_MOTOR_FULLSTEPS_REV *
                     USTEP_DIV * ARM2_GEAR_RATIO_X100 + 180000) / 360000);
}

/* Odd-byte framed absolute joint target. Data nibble includes alternating parity
 * so the three wire copies are idempotent. CRC protects missing/reordered data.
 * 15 payload bytes: version, seq:u32, epoch:u32, q1:i16, q2:i16, rate:u16 (LE).
 * Followed by CRC16-CCITT (init FFFF), little endian. Angles are centidegrees
 * relative to the verified straight reference; positive is physical LEFT/CCW. */
/* CRC-8/ATM over the compact heartbeat payload.  One data byte plus one check byte
 * detects every one- and two-bit error, which is what protects the jog mask. */
static uint8 ptp_crc8(const uint8 *p, uint8 n)
{
    uint8 crc = 0u, i;
    while (n-- != 0u) {
        crc ^= *p++;
        for (i = 0u; i < 8u; i++) {
            crc = (uint8)(((crc & 0x80u) != 0u) ? (((uint8)(crc << 1)) ^ 0x07u) : (uint8)(crc << 1));
        }
    }
    return crc;
}

static uint16 ptp_crc(const uint8 *p, uint8 n)
{
    uint16 crc = 0xFFFFu; uint8 i;
    while (n-- != 0u) {
        crc ^= (uint16)((uint16)*p++ << 8);
        for (i=0u; i<8u; i++) { crc = (uint16)((crc & 0x8000u) ? (crc << 1) ^ 0x1021u : crc << 1); }
    }
    return crc;
}
static uint16 ptp_u16(const uint8 *p) { return (uint16)((uint16)p[0] | ((uint16)p[1] << 8)); }
static uint32 ptp_u32(const uint8 *p) { return (uint32)p[0] | ((uint32)p[1]<<8) | ((uint32)p[2]<<16) | ((uint32)p[3]<<24); }
static int32 ptp_offset(uint8 axis, int32 cd)
{
    int64 n = (int64)cd * ((axis == 0u) ? (int32)(MOTOR_FULLSTEPS_REV * USTEP_DIV * GEAR_RATIO) :
        (int32)(ARM2_MOTOR_FULLSTEPS_REV * USTEP_DIV * ARM2_GEAR_RATIO_X100 / 100));
    return (int32)((n >= 0) ? (n + 18000) / 36000 : -((-n + 18000) / 36000));
}
static int32 ptp_axis_angle_cd(uint8 axis, int32 q1, int32 q2)
{
    return axis == 0u ? q1 : q2 - q1; /* q2 is link 2 relative to fixed +X; motor 2 is elbow-relative */
}
static int32 ptp_angle_from_counts(uint8 axis, int32 count)
{
    int32 steps=g_ptp.origin[axis]-count;
    int32 revolution=axis==0u ? MOTOR_FULLSTEPS_REV*USTEP_DIV*GEAR_RATIO :
        ARM2_MOTOR_FULLSTEPS_REV*USTEP_DIV*ARM2_GEAR_RATIO_X100/100;
    int64 scaled=(int64)steps*36000;
    return (int32)(scaled>=0 ? (scaled+revolution/2)/revolution : -((-scaled+revolution/2)/revolution));
}
static uint8 ptp_counts_in_range(uint8 axis, int32 count)
{
    int32 q1=ptp_angle_from_counts(0u,axis==0u?count:g_arm1.usteps);
    int32 q2=q1+ptp_angle_from_counts(1u,axis==1u?count:g_arm2.usteps);
    int32 value=axis==0u ? q1 : q2;
    int32 elbow=q2-q1;
    return value>=g_jointMinCd[axis] && value<=g_jointMaxCd[axis] &&
           elbow>=g_jointMinCd[JOINT_ELBOW] && elbow<=g_jointMaxCd[JOINT_ELBOW];
}
static uint8 joint_step_allowed(const jog_axis_t *a, int8 s)
{
    uint8 axis = (a == &g_arm1) ? 0u : 1u;
    if(g_home.phase!=HOME_IDLE || !g_directionKnown || !(axis ? g_homed2 : g_homed1)) { return 1u; }
    /* Homed/manual-reference zero survives switching to jog. Permit retreat from
     * outside a bound, but never another outward step (including deceleration). */
    int32 q1=ptp_angle_from_counts(0u,g_arm1.usteps);
    int32 elbow=ptp_angle_from_counts(1u,g_arm2.usteps);
    /* Quantized estimates can sit just inside a limit; compare actual counts
     * with the nearest legal pulse boundary before allowing an outward jog. */
    if(axis==0u && ((s<0 && a->usteps<=g_ptp.origin[0]-ptp_offset(0u,g_jointMaxCd[0])) ||
                    (s>0 && a->usteps>=g_ptp.origin[0]-ptp_offset(0u,g_jointMinCd[0])))) return 0u;
    if(axis==1u && ((s<0 && a->usteps<=g_ptp.origin[1]-ptp_offset(1u,g_jointMaxCd[1]-q1)) ||
                    (s>0 && a->usteps>=g_ptp.origin[1]-ptp_offset(1u,g_jointMinCd[1]-q1)))) return 0u;
    /* The elbow is motor 2's own angle, so its range bounds that motor directly. */
    if(axis==1u && ((s<0 && a->usteps<=g_ptp.origin[1]-ptp_offset(1u,g_jointMaxCd[JOINT_ELBOW])) ||
                    (s>0 && a->usteps>=g_ptp.origin[1]-ptp_offset(1u,g_jointMinCd[JOINT_ELBOW])))) return 0u;
    if(axis==0u && ((s<0 && g_arm1.usteps<=g_ptp.origin[0]-ptp_offset(0u,g_jointMaxCd[1]-elbow)) ||
                    (s>0 && g_arm1.usteps>=g_ptp.origin[0]-ptp_offset(0u,g_jointMinCd[1]-elbow)))) return 0u;
    if(dual_ready()) {
        int32 m1=(int32)(((int64)g_enc.pos-g_enc.softCenter)*(g_enc.incDir==ARM1_LEFT_DIR_LEVEL?1:-1)*36000/(ENCODER_CPR*GEAR_RATIO));
        int32 me=ARM2_ENCODER_FEEDBACK ? (int32)(((int64)g_enc2.pos-g_enc2.softCenter)*(g_enc2.incDir==ARM2_LEFT_DIR_LEVEL?1:-1)*3600000/(ENCODER_CPR*ARM2_GEAR_RATIO_X100)) : elbow;
        int32 m2=m1+me;
        if((s<0 && (m2>=g_jointMaxCd[1] || (axis ? me>=g_jointMaxCd[2] : m1>=g_jointMaxCd[0]))) ||
           (s>0 && (m2<=g_jointMinCd[1] || (axis ? me<=g_jointMinCd[2] : m1<=g_jointMinCd[0])))) { return 0u; }
    }
    return 1u;
}
static void traj_status(void)
{
    JLOGF("TRAJ_STATE state=%u count=%u total=%u index=%u seq=%lu epoch=%lu\r\n",
          (unsigned)g_traj.state,(unsigned)g_traj.count,(unsigned)g_traj.expected,
          (unsigned)g_traj.index,(unsigned long)g_ptp.sequence,(unsigned long)g_ptp.epoch);
}
static uint32 g_controlSequence;
static void jog_apply_keys(uint8 mask);
static void stage_enable(void);
static void jog_home_all(void);
static void motion_config_status(void);

/* Types 9 and 10 share the control sequence but never change the reference epoch. Exact retries only repeat
 * the receipt.  Ranges (type 9) redefine what "inside" means for a reference, so they need a stopped,
 * reference-free stage; ENABLE changes the epoch before any motion can start.  Speeds and accelerations
 * (type 10) only shape the next move, so they are also taken while enabled and referenced, as long as nothing
 * moves, homes or uploads (advertised as rates_live=1).  A plan the device holds was checked against the old
 * speeds, so a change drops it and the host must upload again. */
static void joint_limits_accept(const uint8 *p)
{
    uint32 seq=ptp_u32(p+1);
    uint16 axis=ptp_u16(p+13);
    int16 min=(int16)ptp_u16(p+9), max=(int16)ptp_u16(p+11);
    uint8 rates=(p[0]==10u);
    uint8 *accepted=rates ? g_ratesAccepted : g_limitsAccepted;
    uint8 busy;
    if(ptp_u32(p+5)!=g_ptp.epoch) {
        g_rxRejected++; g_ptpEpochRejected++; ptp_announce_epoch();
        JLOG("PTP_LIMITS refused: stale epoch\r\n"); return;
    }
    if(!seq || axis>(rates ? 1u : JOINT_ELBOW) || (rates ?
       (min<CFG_RATE_MIN || min>(axis==0u?CFG_RATE_MAX_AXIS1:CFG_RATE_MAX_AXIS2) ||
        max<CFG_ACCEL_MIN || max>(axis==0u?CFG_ACCEL_MAX_AXIS1:CFG_ACCEL_MAX_AXIS2)) :
       (min < -CFG_JOINT_CD_MAX || max > CFG_JOINT_CD_MAX || min>=max || min>0 || max<0))) {
        g_rxRejected++; JLOG("PTP_LIMITS refused: axis/range\r\n"); return;
    }
    busy=(uint8)(g_home.phase!=HOME_IDLE || !axes_idle() || g_ptp.active || g_traj.state==1u || g_traj.state==3u || g_uploadRxQuarantine);
    if(rates ? busy : (busy || g_motionEnabled || g_arm1.holding || g_arm2.holding || g_ptp.reference || g_traj.state)) {
        g_rxRejected++;
        JLOG(rates ? "PTP_LIMITS refused: moving, homing or uploading\r\n" : "PTP_LIMITS refused: stop and clear reference first\r\n");
        return;
    }
    if(memcmp(accepted,p,15u)!=0) {
        if(seq<=g_controlSequence) { g_rxRejected++; JLOG("PTP_LIMITS refused: stale sequence\r\n"); return; }
        if(rates) {
            jog_axis_t *a=axis==0u?&g_arm1:&g_arm2;
            uint8 changed=(uint8)(g_motionRate[axis]!=(uint16)min || g_motionAccel[axis]!=(uint16)max);
            g_motionRate[axis]=(uint16)min; g_motionAccel[axis]=(uint16)max;
            a->rateMax=(uint32)min; a->rateStart=20u;
            a->accel=a->decel=(uint32)max; a->rampRemainder=0u;
            if(changed && g_traj.state) { traj_discard(); JLOG("TRAJ held plan dropped: speed/accel changed\r\n"); traj_status(); }
        } else { g_jointMinCd[axis]=min; g_jointMaxCd[axis]=max; }
        g_controlSequence=seq; memcpy(accepted,p,15u);
    }
    JLOGF("%s epoch=%lu seq=%lu axis=%u min=%d max=%d\r\n",rates?"PTP_RATES":"PTP_LIMITS",
          (unsigned long)g_ptp.epoch,(unsigned long)seq,(unsigned)axis,(int)min,(int)max);
    motion_config_status(); /* configuration ACKs need no full diagnostic burst */
}

/* Type 11: how often position telemetry is streamed (0 = off, else 5..20 Hz).  It only changes what is printed, so
 * it is taken at any time (enabled, moving, uploading).  The epoch and sequence rules are the usual ones; an exact
 * retry only repeats the receipt. */
static void telem_rate_accept(const uint8 *p)
{
    uint32 seq=ptp_u32(p+1);
    uint16 hz=ptp_u16(p+13);
    if(ptp_u32(p+5)!=g_ptp.epoch) {
        g_rxRejected++; g_ptpEpochRejected++; ptp_announce_epoch();
        JLOG("TELEM_RATE refused: stale epoch\r\n"); return;
    }
    if(!seq || ptp_u16(p+9) || ptp_u16(p+11) || (hz!=0u && (hz<TELEM_HZ_MIN || hz>TELEM_HZ_MAX))) {
        g_rxRejected++; JLOG("TELEM_RATE refused: rate\r\n"); return;
    }
    if(memcmp(g_telemAccepted,p,15u)!=0) {
        if(seq<=g_controlSequence) { g_rxRejected++; JLOG("TELEM_RATE refused: stale sequence\r\n"); return; }
        g_telemHz=(uint8)hz; g_telemLastMs=g_jogMs;
        g_controlSequence=seq; memcpy(g_telemAccepted,p,15u);
    }
    JLOGF("TELEM_RATE epoch=%lu seq=%lu hz=%u\r\n",(unsigned long)g_ptp.epoch,(unsigned long)seq,(unsigned)g_telemHz);
}

/* A separate sequence lets periodic input leases coexist with contiguous
 * trajectory nodes, using the same CRC envelope and current reference epoch. */
static void control_accept(const uint8 *p)
{
    uint32 seq=ptp_u32(p+1);
    uint16 value=ptp_u16(p+13);
    if(!seq || ptp_u16(p+9) || ptp_u16(p+11) || value>(p[0]==6u?15u:0u)) {
        g_rxRejected++; return;
    }
    if(ptp_u32(p+5)!=g_ptp.epoch) {
        /* A heartbeat for a previous reference epoch still proves the host is alive,
         * so it renews the lease.  The jog mask is NOT applied, the sequence is NOT
         * accepted and no command runs, so a stale host can never start or keep
         * motion -- but it also can never cause a deadman stop by itself. */
        g_ptpEpochRejected++; g_rxRejected++;
        ptp_announce_epoch();
        if(p[0]==6u) { host_lease_renew(); g_rxGood++; }
        return;
    }
    if(seq<=g_controlSequence) { g_rxRejected++; return; }
    if(g_uploadRxQuarantine && (p[0]!=6u || value!=0u)) { g_rxRejected++; return; }
    g_controlSequence=seq;
    if(p[0]==6u) { jog_apply_keys((uint8)value); }
    else if(p[0]==7u) { stage_enable(); }
    else { jog_home_all(); }
}

static void traj_segment(void)
{
    uint8 i; jog_axis_t *axes[2]; traj_node_t *n=&g_traj.nodes[g_traj.index];
    axes[0]=&g_arm1; axes[1]=&g_arm2;
    g_ptp.target[0]=g_ptp.origin[0]-ptp_offset(0u,n->q1);
    g_ptp.target[1]=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,n->q1,n->q2));
    for(i=0u;i<2u;i++) {
        int32 d=g_ptp.target[i]-axes[i]->usteps;
        g_ptp.distance[i]=(uint32)abs_i32(d); g_ptp.error[i]=0u;
        g_ptp.sign[i]=(d<0)?-1:1;
    }
    g_ptp.total=(g_ptp.distance[0]>g_ptp.distance[1])?g_ptp.distance[0]:g_ptp.distance[1];
    g_ptp.done=0u;
    g_ptp.periodUs=g_ptp.total ? (uint32)n->ms*1000u/g_ptp.total : 1000u;
}
static uint8 traj_accept(const uint8 *p)
{
    uint8 i; uint16 value=ptp_u16(p+13);
    int16 q[2]; int32 velocity[2]; uint16 previousMs;
    q[0]=(int16)ptp_u16(p+9); q[1]=(int16)ptp_u16(p+11);
    if (p[0]==2u) {
        if(value<2u || value>TRAJ_CAPACITY) { JLOG("PTP refused: trajectory capacity\r\n"); return 0u; }
        if(g_arm1.usteps!=g_ptp.origin[0]-ptp_offset(0u,q[0]) || g_arm2.usteps!=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,q[0],q[1]))) {
            JLOG("PTP refused: move to trajectory start first\r\n"); return 0u;
        }
        g_traj.nodes[0].q1=q[0]; g_traj.nodes[0].q2=q[1]; g_traj.nodes[0].ms=0u;
        g_traj.expected=value; g_traj.count=1u; g_traj.index=0u; g_traj.durationMs=0u;
        g_traj.velocity[0]=g_traj.velocity[1]=0; g_traj.state=1u;
    } else if(p[0]==3u) {
        traj_node_t *last=&g_traj.nodes[g_traj.count ? g_traj.count-1u : 0u];
        if(g_traj.state!=1u || g_traj.count>=g_traj.expected || ptp_u32(p+1)!=g_ptp.sequence+1u ||
           value<TRAJ_NODE_MIN_MS || value>TRAJ_NODE_MAX_MS || g_traj.durationMs+value>TRAJ_TOTAL_MAX_MS) {
            JLOG("PTP refused: trajectory order/time\r\n"); return 0u;
        }
        previousMs=last->ms ? last->ms : value;
        for(i=0u;i<2u;i++) {
            int32 d=ptp_offset(i,ptp_axis_angle_cd(i,q[0],q[1]))-
                ptp_offset(i,ptp_axis_angle_cd(i,last->q1,last->q2));
            velocity[i]=d*1000/(int32)value;
            if((uint32)abs_i32(d)*1000u>motion_min(TRAJ_RATE_MAX,g_motionRate[i])*value ||
               (uint32)abs_i32(velocity[i]-g_traj.velocity[i])*2000u>motion_min(TRAJ_ACCEL_MAX,g_motionAccel[i])*((uint32)value+previousMs) ||
               (g_traj.count+1u==g_traj.expected && (uint32)abs_i32(velocity[i])*2000u>motion_min(TRAJ_ACCEL_MAX,g_motionAccel[i])*value)) {
                JLOG("PTP refused: trajectory rate/velocity change\r\n"); return 0u;
            }
        }
        g_traj.nodes[g_traj.count].q1=q[0]; g_traj.nodes[g_traj.count].q2=q[1];
        g_traj.nodes[g_traj.count++].ms=value; g_traj.durationMs+=value;
        g_traj.velocity[0]=velocity[0]; g_traj.velocity[1]=velocity[1];
        if(g_traj.count==g_traj.expected) { g_traj.state=2u; }
    } else {
        /* state 4 = the held plan already ran once; the nodes are intact, so a fresh RUN (new sequence) repeats it */
        if(p[0]!=4u || q[0]!=0 || q[1]!=0 || value!=0u || (g_traj.state!=2u && g_traj.state!=4u) ||
           g_traj.count!=g_traj.expected || ptp_u32(p+1)!=g_ptp.sequence+1u ||
           g_arm1.usteps!=g_ptp.origin[0]-ptp_offset(0u,g_traj.nodes[0].q1) ||
           g_arm2.usteps!=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,g_traj.nodes[0].q1,g_traj.nodes[0].q2))) {
            JLOG("PTP refused: trajectory incomplete/start position\r\n"); return 0u;
        }
        g_traj.state=3u; g_traj.index=1u; g_traj.elapsedCy=0u; g_traj.clock=SU_NOW();
        g_ptp.sinceMs=g_jogMs; g_ptp.lastStep=SU_NOW(); g_ptp.active=1u; traj_segment();
    }
    return 1u;
}
static void ptp_accept(void)
{
    const uint8 *p = g_ptpRx.bytes;
    uint32 seq = ptp_u32(p+1);
    int32 q1 = (int16)ptp_u16(p+9), q2 = (int16)ptp_u16(p+11);
    uint16 rate = ptp_u16(p+13); uint8 i;
    jog_axis_t *axes[2]; axes[0]=&g_arm1; axes[1]=&g_arm2;
    if (ptp_crc(p,15u) != ptp_u16(p+15)) { g_ptpRxDropped++; g_ptpRxCrc++; JLOG("PTP RX dropped: CRC\r\n"); return; }
    if (p[0]==9u || p[0]==10u) { joint_limits_accept(p); return; }
    if (p[0]==11u) { telem_rate_accept(p); return; }
    if (p[0]>=6u && p[0]<=8u) { control_accept(p); return; }
    if (p[0] < 1u || p[0] > 5u) { JLOG("PTP refused: version\r\n"); return; }
    if (!PTP_COMMISSIONED) { JLOG("PTP refused: commissioning lock\r\n"); return; }
    if(p[0]==5u && g_ptp.reference && seq==g_ptp.sequence && memcmp(g_ptp.accepted,p,15u)==0) {
        stage_status(); return; /* retry confirms the same reference, never re-zero */
    }
    if (ptp_u32(p+5) != g_ptp.epoch || (!g_ptp.reference && p[0]!=5u)) { JLOG("PTP refused: stale reference; establish reference first\r\n"); return; }
    if (seq == g_ptp.sequence && seq != 0u && memcmp(g_ptp.accepted,p,15u)==0) {
        JLOGF("PTP duplicate seq=%lu; not restarted\r\n",(unsigned long)seq); stage_status(); return;
    }
    if (seq == 0u || seq <= g_ptp.sequence) { JLOG("PTP refused: stale sequence\r\n"); return; }
    if (!g_motionEnabled || (p[0]!=5u && (!g_homed1 || !g_homed2 || !dual_ready())) || (!g_arm1.hasEnc || (ARM2_ENCODER_FEEDBACK && !g_arm2.hasEnc)) ||
        g_home.phase != HOME_IDLE || g_ptp.active || (p[0]==1u && (g_traj.state==1u || g_traj.state==2u)) ||
        !host_input_idle() || !axes_idle() || !motion_inputs_clear() || !home_inputs_clear()) {
        JLOG("PTP refused: not homed/idle/healthy\r\n"); return;
    }
    if(p[0]==5u) {
        if(q1!=0 || q2!=0 || rate!=0u || g_traj.state==1u || g_traj.state==2u) {
            JLOG("PTP refused: manual reference payload/busy\r\n"); return;
        }
        if(ARM2_ENCODER_FEEDBACK && !g_enc2.directionKnown) { JLOG("MANUAL REF refused: home first to learn arm2 encoder polarity\r\n"); return; }
        if(!enc_sample()) { stage_stop("manual reference encoder invalid",0u); return; }
        jog_stop_now(&g_arm1); jog_stop_now(&g_arm2);
        ptp_invalidate(); /* discard old targets and increment reference epoch */
        g_encIncDir=MANUAL_ENC_INC_DIR; g_directionKnown=1u;
        cmd_reset_to_measured(); cmd_reset_axis(&g_enc2); g_arm1SoftCenter=g_enc.pos; g_enc2.softCenter=g_enc2.pos;
        g_ptp.origin[0]=g_arm1.usteps; g_ptp.origin[1]=g_arm2.usteps;
        g_ptp.encoderOrigin=g_enc.pos; g_enc.origin=g_enc.pos; g_enc2.origin=g_enc2.pos; g_ptp.referenceSource=2u;
        g_homed1=g_homed2=g_ptp.reference=1u; /* operator-defined reference, NOT sensor homing */
        g_ptp.sequence=seq; memcpy(g_ptp.accepted,p,15u);
        JLOG("MANUAL REF OK: operator declares current pose q1=q2=0 straight; no motion; NOT sensor homed\r\n");
        stage_status(); return;
    }
    if (q1 < g_jointMinCd[0] || q1 > g_jointMaxCd[0] || q2 < g_jointMinCd[1] || q2 > g_jointMaxCd[1] ||
        q2-q1 < g_jointMinCd[JOINT_ELBOW] || q2-q1 > g_jointMaxCd[JOINT_ELBOW] || (p[0]==1u && (rate < (uint32)CFG_RATE_MIN || rate > PTP_RATE_MAX)) ||
        !ptp_counts_in_range(0u,g_arm1.usteps) || !ptp_counts_in_range(1u,g_arm2.usteps)) {
        JLOG("PTP refused: joint range/rate\r\n"); return;
    }
    if (!enc_sample() || (abs_i32(cmd_lsb()-g_enc.pos) >= SLIP_FAIL_LSB || (ARM2_ENCODER_FEEDBACK && abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)>=SLIP_FAIL_LSB))) { stage_stop("PTP start encoder mismatch",0u); return; }
    jog_stop_now(&g_arm1); jog_stop_now(&g_arm2);
    if(p[0]!=1u) {
        if(!traj_accept(p)) { return; }
        if(p[0]==4u) { g_uploadRxQuarantine=0u; } /* only a validated RUN ends upload isolation */
        g_ptp.sequence=seq; memcpy(g_ptp.accepted,p,15u); traj_status(); return;
    }
    if(g_traj.state!=4u) { g_traj.state=0u; g_traj.count=g_traj.expected=g_traj.index=0u; } /* a finished plan survives the move back to its start */
    g_ptp.target[0]=g_ptp.origin[0]-ptp_offset(0u,q1);
    g_ptp.target[1]=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,q1,q2));
    for(i=0u;i<2u;i++) {
        int32 delta=g_ptp.target[i]-axes[i]->usteps;
        g_ptp.distance[i]=(uint32)abs_i32(delta); g_ptp.error[i]=0u;
        g_ptp.sign[i]=(delta < 0) ? -1 : 1;
    }
    g_ptp.total=(g_ptp.distance[0]>g_ptp.distance[1])?g_ptp.distance[0]:g_ptp.distance[1];
    g_ptp.done=0u; g_ptp.sequence=seq; memcpy(g_ptp.accepted,p,15u);
    g_ptp.sinceMs=g_jogMs; g_ptp.lastStep=SU_NOW();
    {
        uint32 accel=PTP_ACCEL;
        for(i=0u;i<2u;i++) if(g_ptp.distance[i]) {
            rate=(uint16)motion_min(rate,g_motionRate[i]);
            accel=motion_min(accel,g_motionAccel[i]);
        }
        ramp_init(&g_ptp.ramp,(int32)g_ptp.total,rate,accel);
        /* Configured low accelerations need a longer first-step interval than
         * the legacy 60 ms floor; stays below one SysTick wrap. */
        g_ptp.ramp.c0=motion_min(600000u,956000u/isqrt32(accel));
        if(g_ptp.ramp.c0<g_ptp.ramp.cMin)g_ptp.ramp.c0=g_ptp.ramp.cMin;
        g_ptp.ramp.c=g_ptp.ramp.c0;
    }
    g_ptp.periodUs=g_ptp.ramp.c; g_ptp.active=1u;
    JLOGF("PTP accepted seq=%lu target1=%ld target2=%ld rate=%u\r\n",(unsigned long)seq,(long)g_ptp.target[0],(long)g_ptp.target[1],(unsigned)rate);
    stage_status();
}
/* A compact heartbeat renews the motion lease with the keys it carries.  It has no
 * sequence and no epoch, so it can never start a command; the CRC8 only has to
 * protect the jog mask. */
static void ptp_accept_short(void)
{
    uint8 keys = g_shortRx.bytes[0];
    if (ptp_crc8(g_shortRx.bytes, 1u) != g_shortRx.bytes[1]) {
        g_ptpRxDropped++; g_ptpRxCrc++; JLOG("PTP RX dropped: short CRC\r\n"); return;
    }
    if (keys > 15u) { g_rxRejected++; return; }
    if (g_uploadRxQuarantine && (keys != 0u)) { g_rxRejected++; return; }
    g_shortRxOk++;
    jog_apply_keys(keys);
}

static uint8 ptp_receive(uint8 b)
{
    uint8 nibble, expected;
    if (b == PTP_FRAME_START) {
        g_shortRx.active=0u;
        g_ptpRx.active=1u; g_ptpRx.count=0u; g_ptpRx.last=0u;
        g_ptpRx.sinceMs=g_ptpRx.lastMs=g_jogMs; return 1u;
    }
    if (b == PTP_SHORT_START) {
        g_ptpRx.active=0u;
        g_shortRx.active=1u; g_shortRx.count=0u; g_shortRx.last=0u;
        g_shortRx.sinceMs=g_shortRx.lastMs=g_jogMs; return 1u;
    }
    if (b == PTP_SHORT_END) {
        if (g_shortRx.active==1u && g_shortRx.count==PTP_SHORT_BYTES*2u) {
            g_shortRx.active=0u; ptp_accept_short();
        } else {
            if (g_shortRx.active) { g_ptpRxDropped++; g_ptpRxShort++; JLOGF("PTP RX dropped: short nibbles=%u/4\r\n",(unsigned)g_shortRx.count); }
            g_shortRx.active=0u;
        }
        return 1u;
    }
    if (b == PTP_FRAME_END) {
        g_shortRx.active=0u;
        if (g_ptpRx.active==1u && g_ptpRx.count==PTP_FRAME_BYTES*2u) { g_ptpRx.active=0u; ptp_accept(); }
        else {
            if(g_ptpRx.active) { g_ptpRxDropped++; g_ptpRxShort++; JLOGF("PTP RX dropped: incomplete nibbles=%u/34\r\n",(unsigned)g_ptpRx.count); }
            g_ptpRx.active=0u;
        }
        return 1u;
    }
    if ((b & 0xC1u)==0x81u) {
        /* Both frames use the same alternating low/high nibble ranges, so the
         * duplicate-copy rule stays sound for either: adjacent wire bytes can only
         * be equal when they are the two copies of one byte. */
        if (g_shortRx.active==1u) {
            if (b == g_shortRx.last) { g_shortRx.lastMs=g_jogMs; return 1u; }
            expected=(uint8)((g_shortRx.count & 1u)<<5);
            if (g_shortRx.count>=PTP_SHORT_BYTES*2u) { g_shortRx.active=2u; return 1u; }
            if ((b & 0x20u)!=expected) { return 1u; }
            nibble=(uint8)((b>>1)&15u);
            if ((g_shortRx.count & 1u)==0u) { g_shortRx.bytes[g_shortRx.count/2u]=nibble; }
            else { g_shortRx.bytes[g_shortRx.count/2u] |= (uint8)(nibble<<4); }
            g_shortRx.last=b; g_shortRx.count++; g_shortRx.lastMs=g_jogMs; return 1u;
        }
        if (g_ptpRx.active!=1u) { return 1u; }
        if (b == g_ptpRx.last) { g_ptpRx.lastMs=g_jogMs; return 1u; }
        expected=(uint8)((g_ptpRx.count & 1u)<<5);
        if (g_ptpRx.count>=PTP_FRAME_BYTES*2u) { g_ptpRx.active=2u; return 1u; }
        if ((b & 0x20u)!=expected) { return 1u; } /* late/noisy copy; full length + CRC still required */
        nibble=(uint8)((b>>1)&15u);
        if ((g_ptpRx.count & 1u)==0u) { g_ptpRx.bytes[g_ptpRx.count/2u]=nibble; }
        else { g_ptpRx.bytes[g_ptpRx.count/2u] |= (uint8)(nibble<<4); }
        g_ptpRx.last=b; g_ptpRx.count++; g_ptpRx.lastMs=g_jogMs; return 1u;
    }
    if (g_ptpRx.active || g_shortRx.active) { return 1u; } /* ignore noise until delimiter/timeout; never decode payload as C/R */
    return 0u;
}
/* Piecewise joint-linear timed DDA, no rest/restart at ordinary nodes.
 * ponytail: bounded velocity changes between chords, not jerk-continuous motion;
 * validate low-speed tracking on hardware before increasing these caps. */
static uint8 coordinated_step(const char *failure)
{
    uint8 i;
    jog_axis_t *axes[2]; axes[0]=&g_arm1; axes[1]=&g_arm2;
    for(i=0u;i<2u;i++) {
        g_ptp.error[i]+=g_ptp.distance[i];
        if(g_ptp.error[i]>=g_ptp.total) {
            int32 before=axes[i]->usteps;
            g_ptp.error[i]-=g_ptp.total; jog_step(axes[i],g_ptp.sign[i]);
            if(axes[i]->usteps!=before+g_ptp.sign[i]) { stage_stop(failure,FAULT_ACTIVE()); return 0u; }
        }
    }
    g_ptp.done++;
    return 1u;
}
static void traj_service(void)
{
    uint32 now=SU_NOW();
    uint32 duration=(uint32)g_traj.nodes[g_traj.index].ms*1000u*SU_CY_PER_US;
    g_traj.elapsedCy+=(g_traj.clock-now)&0x00FFFFFFu; g_traj.clock=now;
    if(g_traj.elapsedCy>duration+20000u*SU_CY_PER_US) { stage_stop("trajectory timing overrun",0u); return; }
    if(g_ptp.done<g_ptp.total) {
        uint32 due=(uint32)((int64)(g_ptp.done+1u)*duration/g_ptp.total);
        if(g_traj.elapsedCy<due || SU_ELAPSED(g_ptp.lastStep)<TRAJ_STEP_MIN_US*SU_CY_PER_US) { return; }
        g_ptp.lastStep=SU_NOW();
        if(!coordinated_step("trajectory pulse blocked")) { return; }
    }
    if(g_ptp.done==g_ptp.total && g_traj.elapsedCy>=duration) {
        g_traj.elapsedCy-=duration;
        if(++g_traj.index==g_traj.count) { g_ptp.active=2u; g_ptp.settleMs=g_jogMs; }
        else { traj_segment(); }
    }
}
static void ptp_service(void)
{
    if (!g_ptp.active) { return; }
    if (!g_motionEnabled || !g_ptp.reference || !g_keysSeen || g_jogMs-g_lastKeyMs>JOG_DEADMAN_MS) { stage_stop("PTP heartbeat/reference lost",0u); return; }
    if (FAULT_ACTIVE()) { stage_stop("PTP nFAULT",1u); return; }
    if (HARD_LIMIT_ACTIVE() || ARM1_HOME_ACTIVE() || ARM2_NPN_ACTIVE()) { stage_stop("PTP limit/home sensor",0u); return; }
    if (g_jogMs-g_ptp.sinceMs>600000u || !ptp_counts_in_range(0u,g_arm1.usteps) || !ptp_counts_in_range(1u,g_arm2.usteps)) { stage_stop("PTP time/joint limit",0u); return; }
    if (!g_arm1.hasEnc || g_enc.badStreak || (ARM2_ENCODER_FEEDBACK && (!g_arm2.hasEnc || g_enc2.badStreak))) { return; } /* shared encoder watchdog will latch stop */
    if ((abs_i32(cmd_lsb()-g_enc.pos)>=SLIP_FAIL_LSB || (ARM2_ENCODER_FEEDBACK && abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)>=SLIP_FAIL_LSB))) { stage_stop("PTP encoder following error",0u); return; }
    if (g_ptp.active==2u) {
        if (g_jogMs-g_ptp.settleMs<HOME_SETTLE_MS) { return; }
        if (!enc_sample() || abs_i32(cmd_lsb()-g_enc.pos)>PTP_FINAL_ERROR_LSB || (ARM2_ENCODER_FEEDBACK && abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)>PTP_FINAL_ERROR_LSB) ||
            g_arm1.usteps!=g_ptp.target[0] || g_arm2.usteps!=g_ptp.target[1]) { stage_stop("PTP final position mismatch",0u); return; }
        g_ptp.active=0u;
        if(g_traj.state==3u) { g_traj.state=4u; }
#if ARM2_ENCODER_FEEDBACK
        JLOGF("PTP done seq=%lu; enabled motor encoders checked; error1_lsb=%ld error2_lsb=%ld final_limit_lsb=%d\r\n",
              (unsigned long)g_ptp.sequence,(long)(cmd_lsb()-g_enc.pos),
        (long)(g_enc2.cmdLsbX100/100-g_enc2.pos),PTP_FINAL_ERROR_LSB);
#else
        JLOGF("PTP done seq=%lu; arm1 encoder checked; arm2 commanded steps without feedback; error1_lsb=%ld final_limit_lsb=%d\r\n",
              (unsigned long)g_ptp.sequence,(long)(cmd_lsb()-g_enc.pos),PTP_FINAL_ERROR_LSB);
#endif
        stage_status(); return;
    }
    if(g_traj.state==3u) { traj_service(); return; }
    if (g_ptp.done>=g_ptp.total) { g_ptp.active=2u; g_ptp.settleMs=g_jogMs; return; }
    if (SU_ELAPSED(g_ptp.lastStep)<g_ptp.periodUs*SU_CY_PER_US) { return; }
    g_ptp.lastStep=SU_NOW(); /* no catch-up bursts after a late service */
    if(!coordinated_step("PTP pulse blocked")) { return; }
    ramp_advance(&g_ptp.ramp,(int32)g_ptp.done); g_ptp.periodUs=g_ptp.ramp.c;
}

/* Non-blocking home: one phase transition or pulse per service call.
 * RX, encoder sampling and the deadman keep running through every phase. */
static void home_enter(home_phase_t phase)
{
    jog_axis_t *a = (g_home.arm == 1u) ? &g_arm1 : &g_arm2;
    int8 toward = (g_home.arm == 1u) ? ARM1_HOME_JOG_DIR : ARM2_HOME_JOG_DIR;
    uint32 rate;
    uint32 accel;
    uint32 distance;
    g_home.phase = phase;
    g_home.sinceMs = g_jogMs;
    g_home.steps = 0u;
    g_home.confirming = 0u;
    g_home.zeroSamples = 0u;
    g_home.zeroSum = 0;
    g_home.lastStep = SU_NOW();
    g_home.probePos = axis_encoder(a)->pos;
    g_home.probeSteps = a->usteps;
    g_home.probeNextSteps = DIR_PROBE_USTEPS;
    g_home.direction = ((phase == HOME_RELEASE) || (phase == HOME_BACKOFF) ||
                        (phase == HOME_STRAIGHT)) ? (int8)-toward : toward;
    if ((phase == HOME_STRAIGHT) && (g_home.arm == 2u))
    { g_home.direction = ARM2_STRAIGHT_JOG_DIR; }
    /* The fast touch and the move to straight run at the global speed and acceleration.  Everything else (release,
     * backoff, slow touch, the settle phases that do not step) keeps a fixed rate and the acceleration cap, which the
     * global values can only lower. */
    rate = g_motionRate[g_home.arm-1u];
    accel = g_motionAccel[g_home.arm-1u];
    if ((phase != HOME_FAST) && (phase != HOME_STRAIGHT))
    {
        rate = motion_min(rate, (phase == HOME_SLOW) ? HOME_SLOW_RATE_USTEP_S : HOME_RELEASE_RATE_USTEP_S);
        accel = motion_min(accel, HOME_ACCEL_USTEP_S2);
    }
    distance = (phase == HOME_RELEASE) ? HOME_RELEASE_MAX_USTEPS :
               ((phase == HOME_BACKOFF) ? HOME_BACKOFF_USTEPS :
                ((phase == HOME_STRAIGHT) ? home_straight_steps(g_home.arm) : 0u));
    {
        ramp_init(&g_home.ramp, (int32)distance, rate, accel);
        g_home.ramp.c0=motion_min(600000u,956000u/isqrt32(accel));
        if(g_home.ramp.c0<g_home.ramp.cMin)g_home.ramp.c0=g_home.ramp.cMin;
        g_home.ramp.c=g_home.ramp.c0;
    }
    g_home.periodUs = g_home.ramp.c;
    JLOGF("HOME arm%u phase=%u %s rate=%lu usteps/s\r\n", (unsigned)g_home.arm, (unsigned)phase,
          (phase == HOME_RELEASE) ? "release sensor" :
          (phase == HOME_BACKOFF) ? "extra backoff" :
          (phase == HOME_FAST) ? "fast touch" :
          (phase == HOME_SLOW) ? "slow touch" :
          (phase == HOME_STRAIGHT) ? "move away from home to straight" : "settle and verify", (unsigned long)rate);
    if (phase == HOME_STRAIGHT)
    { JLOGF("HOME arm%u straight target: dir=%d pulses=%lu rate=%lu usteps/s (commanded)\r\n",
            (unsigned)g_home.arm, (int)g_home.direction, (unsigned long)distance,
            (unsigned long)rate); }
}

static void home_start_axis(uint8 arm)
{
    g_home.arm = arm;
    g_arm1HomeActive = (arm == 1u);
    g_arm2HomeActive = (arm == 2u);
    g_home.afterBackoff = HOME_FAST;
    home_enter(home_sensor_active(arm) ? HOME_RELEASE : HOME_FAST);
}

static uint8 home_set_zero(jog_axis_t *a)
{
    enc_t *e=axis_encoder(a);
    if (a->hasEnc != 0u)
    {
        uint16 mean;
        /* One fresh sample per interval; keep RX/STOP/deadman serviceable. */
        if ((g_home.zeroSamples != 0u) &&
            ((g_jogMs - g_home.zeroSampleMs) < HOME_ZERO_SAMPLE_MS)) { return 0u; }
        if (enc_sample() == 0u) { stage_stop("HOME zero: encoder invalid", 0u); return 0u; }
        g_home.zeroSampleMs = g_jogMs;
        if (g_home.zeroSamples == 0u) { g_home.zeroReference = e->lastAngle; }
        g_home.zeroSum += angle_delta(e->lastAngle, g_home.zeroReference);
        if (++g_home.zeroSamples < HOME_ZERO_SAMPLES) { return 0u; }
        mean = (uint16)(((int32)g_home.zeroReference +
                        g_home.zeroSum / (int32)HOME_ZERO_SAMPLES + ENCODER_CPR) % ENCODER_CPR);
        /* Preserve the latest reading relative to the averaged origin. */
        e->pos = angle_delta(e->lastAngle, mean);
        cmd_reset_axis(e);
        JLOGF("HOME arm1 averaged origin: samples=%u raw=%u lsb (motor single-turn)\r\n",
              (unsigned)g_home.zeroSamples, (unsigned)mean);
    }
    a->usteps = 0;
    a->holding = 0u;
    a->holdTarget = 0;
    return 1u;
}

static void jog_home_all(void)
{
    if ((g_motionEnabled == 0u) || ((g_arm1.hasEnc == 0u) || (ARM2_ENCODER_FEEDBACK && g_arm2.hasEnc == 0u)) ||
        !host_input_idle() || !axes_idle() ||
        (g_home.phase != HOME_IDLE) || g_ptp.active || !motion_inputs_clear())
    {
        JLOG("HOME refused: enable with C, keep host alive and release all motion keys\r\n");
        return;
    }
    jog_stop_now(&g_arm1);
    jog_stop_now(&g_arm2);
    ptp_invalidate();
    if (enc_sample() == 0u) { stage_stop("HOME start: encoder invalid", 0u); return; }
    cmd_reset_to_measured(); cmd_reset_axis(&g_enc2);
    g_homed1 = 0u;
    g_homed2 = 0u;
    g_homeNeedKeyRelease = 1u;
    home_start_axis(1u);
    stage_status(); /* Publish the new epoch before the 250 ms heartbeat deadline. */
}

static void home_service(void)
{
    jog_axis_t *a;
    enc_t *e;
    uint8 sensor;
    uint8 reached;
    uint32 maxSteps;
    int32 delta;
    int32 commanded;

    if ((g_home.phase == HOME_IDLE) || (g_motionEnabled == 0u)) { return; }
    if (FAULT_ACTIVE()) { stage_stop("HOME nFAULT; drivers OFF", 1u); return; }
    if (HARD_LIMIT_ACTIVE()) { jog_limit_stop(); return; }
    if ((g_keysSeen == 0u) || ((g_jogMs - g_lastKeyMs) > JOG_DEADMAN_MS))
    { stage_stop("HOME host heartbeat timeout", 0u); return; }
    if ((g_home.phase != HOME_FAST) && (g_home.phase != HOME_SLOW) &&
        ((g_jogMs - g_home.sinceMs) >= HOME_PHASE_TIMEOUT_MS))
    { stage_stop("HOME phase time limit", 0u); return; }

    a = (g_home.arm == 1u) ? &g_arm1 : &g_arm2;
    e=axis_encoder(a);
    sensor = home_sensor_active(g_home.arm);

    /* Learn polarity from real motion in the already requested direction.
     * Do not move at boot just to discover encoder polarity. */
    if ((a->hasEnc != 0u) && (e->directionKnown == 0u) &&
        ((uint32)abs_i32(a->usteps - g_home.probeSteps) >= g_home.probeNextSteps))
    {
        /* Wait for a fresh, settled-enough sample before issuing more steps. */
        if (SU_ELAPSED(g_home.lastStep) < (HOME_PROBE_SETTLE_US * SU_CY_PER_US)) { return; }
        if (enc_sample() == 0u) { stage_stop("HOME direction: encoder invalid", 0u); return; }
        delta = e->pos - g_home.probePos;
        JLOGF("HOME direction sample: steps=%lu delta=%ld lsb\r\n",
              (unsigned long)g_home.probeNextSteps, (long)delta);
        if ((abs_i32(delta) < DIR_PROBE_MIN_LSB) &&
            (g_home.probeNextSteps < HOME_PROBE_MAX_USTEPS))
        {
            g_home.probeNextSteps += DIR_PROBE_USTEPS;
            return; /* bounded extra motion, never bypass the encoder check */
        }
        if ((abs_i32(delta) < DIR_PROBE_MIN_LSB) || (abs_i32(delta) >= SLIP_FAIL_LSB))
        { stage_stop("HOME direction: no credible rotor motion; check VMOT/motor/encoder", 0u); return; }
        e->incDir = (delta > 0) ? jog_dir_level(a, g_home.direction) :
                         (uint8)!jog_dir_level(a, g_home.direction);
        e->directionKnown = 1u;
        cmd_reset_axis(e);
        JLOGF("HOME encoder polarity learned: increase DIR=%u\r\n", (unsigned)e->incDir);
    }
    if ((a->hasEnc != 0u) && (e->directionKnown != 0u) &&
        (abs_i32((e->cmdLsbX100/100) - e->pos) >= SLIP_FAIL_LSB))
    { stage_stop("HOME encoder following error", 0u); return; }

    /* The already homed axis must also stay credible while the other homes. */
    if ((g_enc.directionKnown && abs_i32(cmd_lsb()-g_enc.pos)>=SLIP_FAIL_LSB) ||
        (g_enc2.directionKnown && (ARM2_ENCODER_FEEDBACK && abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)>=SLIP_FAIL_LSB)))
    { stage_stop("HOME stationary/moving encoder following error",0u); return; }

    if ((g_home.phase == HOME_SETTLE) || (g_home.phase == HOME_STRAIGHT_SETTLE))
    {
        if ((g_jogMs - g_home.sinceMs) < HOME_SETTLE_MS) { return; }
        if (g_home.phase == HOME_STRAIGHT_SETTLE)
        {
            if (sensor != 0u) { stage_stop("HOME straight: sensor still active", 0u); return; }
            if (g_home.arm == 2u)
            {
                if (g_arm2.usteps != ARM2_STRAIGHT_JOG_DIR * (int32)home_straight_steps(2u))
                { stage_stop("HOME arm2 straight: commanded count mismatch", 0u); return; }
                if(!enc_sample() || !dual_ready() || abs_i32(cmd_lsb()-g_enc.pos)>PTP_FINAL_ERROR_LSB || (a->hasEnc && abs_i32(e->cmdLsbX100/100-e->pos)>PTP_FINAL_ERROR_LSB)) {
                    stage_stop("HOME arm2 straight: encoder mismatch",0u); return;
                }
                g_enc.softCenter=g_enc.pos; e->softCenter=e->pos; cmd_reset_to_measured(); cmd_reset_axis(e);
                g_homed2 = 1u;
                g_ptp.origin[0]=g_arm1.usteps; g_ptp.origin[1]=g_arm2.usteps;
                g_ptp.encoderOrigin=g_enc.pos; g_enc.origin=g_enc.pos; g_enc2.origin=g_enc2.pos; g_ptp.referenceSource=1u;
                g_ptp.reference=1u;
                g_home.phase = HOME_IDLE;
                g_arm1HomeActive = 0u;
                g_arm2HomeActive = 0u;
                g_homeNeedKeyRelease = 1u;
                JLOGF("HOME arm2 STRAIGHT commanded: %ld usteps from NPN origin (enabled feedback checked; arm2 may use commanded steps)\r\n",
                      (long)g_arm2.usteps);
                JLOG("HOME OK: both straight moves completed; enabled motor encoders checked; verify physical alignment\r\n");
                stage_status();
                return;
            }
            if (enc_sample() == 0u) { stage_stop("HOME straight: encoder invalid", 0u); return; }
            commanded = jog_enc_sign(&g_arm1, (int8)-ARM1_HOME_JOG_DIR) *
                        usteps_to_lsb(abs_i32(g_arm1.usteps));
            if ((e->directionKnown == 0u) ||
                (abs_i32(e->pos - commanded) >= SLIP_FAIL_LSB))
            { stage_stop("HOME straight: measured position differs from commanded", 0u); return; }
            g_arm1SoftCenter = e->pos;
            cmd_reset_axis(e);
            JLOG("HOME arm1 STRAIGHT OK: switch origin retained, not reset to zero\r\n");
            home_start_axis(2u);
            return;
        }
        if (sensor == 0u) { stage_stop("HOME slow contact did not stay active", 0u); return; }
        if (home_set_zero(a) == 0u) { return; }
        if (g_home.arm == 1u)
        {
            g_homed1 = 1u;
            JLOG("HOME arm1 ZERO OK: P0[3] is the angle origin\r\n");
        }
        else
        {
            JLOG("HOME arm2 ZERO OK: NPN is the commanded origin; moving left to straight\r\n");
        }
        home_enter(HOME_STRAIGHT);
        return;
    }

    if ((g_home.phase == HOME_RELEASE) || (g_home.phase == HOME_FAST) ||
        (g_home.phase == HOME_SLOW))
    {
        reached = (g_home.phase == HOME_RELEASE) ? (uint8)!sensor : sensor;
        if (reached != 0u)
        {
            if (g_home.confirming == 0u)
            { g_home.confirming = 1u; g_home.sensorMs = g_jogMs; }
            if ((g_jogMs - g_home.sensorMs) >= HOME_DEBOUNCE_MS)
            {
                if (g_home.phase == HOME_RELEASE) { home_enter(HOME_BACKOFF); }
                else if (g_home.phase == HOME_FAST)
                { g_home.afterBackoff = HOME_SLOW; home_enter(HOME_RELEASE); }
                else { home_enter(HOME_SETTLE); }
            }
            return; /* no steps into a sensor while confirming contact */
        }
        g_home.confirming = 0u;
    }

    if (g_home.phase == HOME_BACKOFF)
    {
        if (g_home.steps >= HOME_BACKOFF_USTEPS)
        {
            if (sensor != 0u) { stage_stop("HOME sensor reactivated during backoff", 0u); return; }
            home_enter(g_home.afterBackoff);
            return;
        }
        maxSteps = HOME_BACKOFF_USTEPS;
    }
    else if (g_home.phase == HOME_STRAIGHT)
    {
        maxSteps = home_straight_steps(g_home.arm);
        if ((g_home.steps >= HOME_RELEASE_MAX_USTEPS) && (sensor != 0u))
        { stage_stop("HOME straight: home sensor did not release", 0u); return; }
        if (g_home.steps >= maxSteps) { home_enter(HOME_STRAIGHT_SETTLE); return; }
    }
    else
    {
        maxSteps = (g_home.phase == HOME_RELEASE) ? HOME_RELEASE_MAX_USTEPS : 0u;
    }
    if ((maxSteps != 0u) && (g_home.steps >= maxSteps))
    { stage_stop("HOME travel limit: sensor missing or stuck", 0u); return; }
    if (SU_ELAPSED(g_home.lastStep) >= (g_home.periodUs * SU_CY_PER_US))
    {
        g_home.lastStep = SU_NOW();
        jog_step(a, g_home.direction);
        g_home.steps++;
        ramp_advance(&g_home.ramp, (int32)g_home.steps);
        g_home.periodUs = g_home.ramp.c;
    }
}

static void motion_config_status(void)
{
    JLOGF("PTP_STATE lock=%u ref=%u active=%u epoch=%lu seq=%lu min1=%d max1=%d min2=%d max2=%d emin=%d emax=%d\r\n",
          (unsigned)!PTP_COMMISSIONED,(unsigned)g_ptp.reference,(unsigned)g_ptp.active,
          (unsigned long)g_ptp.epoch,(unsigned long)g_ptp.sequence,
          (int)g_jointMinCd[0],(int)g_jointMaxCd[0],(int)g_jointMinCd[1],(int)g_jointMaxCd[1],
          (int)g_jointMinCd[JOINT_ELBOW],(int)g_jointMaxCd[JOINT_ELBOW]);
    JLOGF("MOTION_RATES epoch=%lu r1=%u r2=%u a1=%u a2=%u\r\n",(unsigned long)g_ptp.epoch,
          (unsigned)g_motionRate[0],(unsigned)g_motionRate[1],(unsigned)g_motionAccel[0],(unsigned)g_motionAccel[1]);
}

/* One coherent, low-rate sample, no extra encoder transactions. Positive = CCW.
 * e1 is motor encoder LSB relative to the reference; e2 measures the motor-2 elbow; q2=q1+elbow. */
static void telem_line(void)
{
    JLOGF("TELEM t=%lu epoch=%lu seq=%lu ref=%u source=%u p1=%ld p2=%ld e1=%ld err=%ld "
        ENC2_LOG_FORMAT
        "inputs=%u drops=%lu\r\n",
        (unsigned long)g_jogMs,(unsigned long)g_ptp.epoch,(unsigned long)g_ptp.sequence,
        (unsigned)(g_ptp.reference && dual_ready()),
        (unsigned)g_ptp.referenceSource,(long)(g_ptp.origin[0]-g_arm1.usteps),(long)(g_ptp.origin[1]-g_arm2.usteps),
        (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),
        (long)(cmd_lsb()-g_enc.pos),
        ENC2_LOG_ARGS
        (unsigned)((HARD_LIMIT_ACTIVE()?1u:0u)|(ARM1_HOME_ACTIVE()?2u:0u)|(ARM2_NPN_ACTIVE()?4u:0u)|(FAULT_ACTIVE()?8u:0u)),
        (unsigned long)g_ptpRxDropped);
}

/* The streamed TELEM line, between status blocks.  Its cost is measured (telem_max_us in TELEM_DIAG) because the
 * formatting runs in the same loop that times the step pulses. */
static void telem_service(void)
{
    uint32 started, cost;
    if (g_telemHz == 0u || g_motionEnabled == 0u || (g_jogMs - g_telemLastMs) < (1000u / g_telemHz)) { return; }
    g_telemLastMs = g_jogMs;
    if (txq_free() < TELEM_MIN_FREE_BYTES) { g_telemSkipped++; return; }
    started = SU_NOW();
    telem_line();
    cost = SU_ELAPSED(started);
    if (cost > g_telemMaxCy) { g_telemMaxCy = cost; }
    g_telemSent++;
}

static void stage_status(void)
{
    JLOGF("STATE fw=stage2-v9 enabled=%u home=%u homed1=%u homed2=%u fault=%u enc=%u dir=%u enc2=%u dir2=%u arm2fb=%u\r\n",
          (unsigned)g_motionEnabled, (unsigned)g_home.phase,
          (unsigned)g_homed1, (unsigned)g_homed2, (unsigned)g_jogFaulted,
          (unsigned)(g_arm1.hasEnc && (g_enc.badStreak == 0u)), (unsigned)g_directionKnown, (unsigned)(g_arm2.hasEnc && !g_enc2.badStreak), (unsigned)g_enc2.directionKnown, (unsigned)ARM2_ENCODER_FEEDBACK);
    JLOGF("CONTROL seq=%lu\r\n",(unsigned long)g_controlSequence);
    motion_config_status();
    JLOGF("PTP_CHECK final_lsb=%d follow_lsb=%d\r\n",PTP_FINAL_ERROR_LSB,SLIP_FAIL_LSB);
    traj_status();
    /* traj_repeat: a held plan that already ran accepts another RUN.  fault_snapshot: every stop prints a FAULT_SNAPSHOT
     * line first.  telem_hz: the streamed TELEM rate in force (0 = status block only); its presence means type 11 exists. */
    JLOGF("LINK uart=hw baud=115200 rx_overflow=%lu rx_error=%lu tx_dropped=%u fw=%s short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 dual_encoder=1 traj_repeat=1 fault_snapshot=1 telem_hz=%u proto=%u\r\n",
          (unsigned long)g_rxLate, (unsigned long)g_rxFraming, (unsigned)g_txqDropped, FW_BUILD_ID, (unsigned)g_telemHz, (unsigned)PROTO_VERSION);
    /* Frame loss by cause: rx_error only sees UART-level faults, so a frame that
     * simply arrives short must be counted here or it stays invisible. */
    JLOGF("PTP_RX short_ok=%lu drop_crc=%lu drop_short=%lu drop_timeout=%lu rejected=%lu epoch_rejected=%lu last_rejected_byte=0x%02X\r\n",
          (unsigned long)g_shortRxOk, (unsigned long)g_ptpRxCrc, (unsigned long)g_ptpRxShort, (unsigned long)g_ptpRxTimeout,
          (unsigned long)g_rxRejected, (unsigned long)g_ptpEpochRejected,
          (unsigned)g_rxLastBad[(g_rxLastBadIdx + 7u) % 8u]);
    JLOGF("TIMING loop_max_ms=%lu tick_lag_max_ms=%lu clock=hardware wdt=256-384ms key_age_ms=%lu key_max_gap_ms=%lu\r\n",
          (unsigned long)g_loopMaxMs,(unsigned long)g_tickMaxLagMs,
          (unsigned long)(g_keysSeen ? (g_jogMs - g_lastKeyMs) : 0u),(unsigned long)g_keyMaxGapMs);
    JLOGF("ENC_DIAG irq_max_us=%lu retries=%lu rejected=%lu gap_max_ms=%lu follow_peak_lsb=%lu\r\n",
          (unsigned long)(g_encIrqMaxCy/SU_CY_PER_US),(unsigned long)g_encRetries,
          (unsigned long)g_encRejected,(unsigned long)g_encMaxGapMs,(unsigned long)g_followPeakLsb);
    JLOGF("TELEM_DIAG hz=%u sent=%lu skipped=%lu max_us=%lu\r\n",
          (unsigned)g_telemHz,(unsigned long)g_telemSent,(unsigned long)g_telemSkipped,(unsigned long)(g_telemMaxCy/SU_CY_PER_US));
    telem_line();
}

static void stage_enable(void)
{
    if (g_motionEnabled != 0u) { JLOG("ENABLE: already enabled\r\n"); return; }
    ptp_invalidate();
    if (!host_input_idle() || !motion_inputs_clear())
    { JLOG("ENABLE refused: fresh idle heartbeat and clear nFAULT/P0[4] required\r\n"); return; }
    if (enc_set_origin() == 0u)
    { g_arm1.hasEnc = g_arm2.hasEnc = 0u; JLOG("ENABLE refused: encoder invalid; inspect E diagnostics\r\n"); return; }
    stepper2_enable();
    stepper1_enable();
    if (FAULT_ACTIVE() || HARD_LIMIT_ACTIVE())
    { stage_stop("ENABLE hardware fault/limit", 1u); return; }
    g_arm1.hasEnc = 1u; g_arm2.hasEnc = ARM2_ENCODER_FEEDBACK;
    g_arm1.usteps = 0;
    g_arm2.usteps = 0;
    g_arm1SoftCenter = g_enc2.softCenter = 0;
    g_directionKnown = g_enc2.directionKnown = 0u;
    g_homed1 = 0u;
    g_homed2 = 0u;
    g_jogFaulted = 0u;
    g_limitStop = 0u;
    g_motionEnabled = 1u;
    g_keyMaxGapMs = 0u;
    cmd_reset_to_measured(); cmd_reset_axis(&g_enc2);
    JLOG("ENABLE OK: no automatic movement; low-speed jog or R for home; Space stops\r\n");
    stage_status();
}

static void jog_axis_init(jog_axis_t *a, const char *name, uint32 pinStep,
                          uint32 pinDir, uint8 stepIdleHigh, uint8 hasEnc,
                          uint8 leftDirLevel, uint32 rateMax, uint32 rateStart,
                          uint32 accel, uint32 decel)
{
    a->name         = name;
    a->pinStep      = pinStep;
    a->pinDir       = pinDir;
    a->stepIdleHigh = stepIdleHigh;
    a->hasEnc       = hasEnc;
    a->leftDirLevel = leftDirLevel;
    a->lastDirLevel = 0xFFu;
    a->invert       = 0u;
    a->want         = 0;
    a->moving       = 0;
    a->latched      = 0u;
    a->limitHit     = 0u;
    a->rate         = 0u;
    a->rateCap      = rateMax;
    a->rateMax      = rateMax;
    a->rateStart    = (rateStart == 0u) ? 1u : rateStart;
    a->accel        = accel;
    a->decel        = decel;
    a->periodCy     = 0u;
    a->tLastStep    = SU_NOW();
    a->usteps       = 0;
    a->peakSlip     = 0;
    a->derates      = 0u;
    a->settleMs     = 0u;
    a->holding      = 0u;
    a->holdTarget   = 0;
    a->holdCooldownMs = 0u;
    a->statusMs     = 0u;
}

/* Is there at least `us` microseconds before any moving axis needs a step? */
static uint8 jog_slot_free(uint32 us)
{
    const jog_axis_t *axes[2];
    uint32 need = us * SU_CY_PER_US;
    uint8 i;

    if (g_ptp.active==1u) {
        uint32 period=g_ptp.periodUs*SU_CY_PER_US, since=SU_ELAPSED(g_ptp.lastStep);
        if(g_traj.state==3u) {
            uint32 duration=(uint32)g_traj.nodes[g_traj.index].ms*1000u*SU_CY_PER_US;
            since=g_traj.elapsedCy+SU_ELAPSED(g_traj.clock);
            period=(g_ptp.done<g_ptp.total) ? (uint32)((int64)(g_ptp.done+1u)*duration/g_ptp.total) : duration;
        }
        if (since>=period || period-since<need) { return 0u; }
    }

    if (g_home.phase != HOME_IDLE)
    {
        uint32 period = g_home.periodUs * SU_CY_PER_US;
        uint32 since = SU_ELAPSED(g_home.lastStep);
        /* Home pulses are not represented by jog_axis_t.moving. */
        if ((since < period) && ((period - since) < need)) { return 0u; }
    }

    axes[0] = &g_arm1;
    axes[1] = &g_arm2;
    for (i = 0u; i < 2u; i++)
    {
        if (axes[i]->moving != 0)
        {
            uint32 since = SU_ELAPSED(axes[i]->tLastStep);
            if (since >= axes[i]->periodCy)          { return 0u; }
            if ((axes[i]->periodCy - since) < need)  { return 0u; }
        }
    }
    return 1u;
}

static void jog_print_position(const jog_axis_t *a)
{
    enc_t *e=axis_encoder(a);
    if (a->hasEnc != 0u)
    {
        int32 d = encoder_deg100(a,e->pos);
        JLOGF("%s: encoder %s%ld.%02ld relative-deg (%ld lsb, %ld commanded usteps)%s\r\n",
              a->name, DEG_SGN(d), DEG_INT(d), DEG_FRC(d),
              (long)e->pos, (long)a->usteps,
              (a->invert != 0u) ? " [inverted]" : "");
    }
    else
    {
        JLOGF("%s: %ld commanded usteps since enable/home (NOT measured)%s\r\n",
              a->name, (long)a->usteps,
              (a->invert != 0u) ? " [inverted]" : "");
    }
}

/* Called when the rotor has had MOVE_SETTLE_MS after the last step. */
static void jog_axis_settled(jog_axis_t *a)
{
    enc_t *e=axis_encoder(a);
    if (a->hasEnc != 0u)
    {
        if(!enc_sample()) { stage_stop("jog settle: encoder invalid",0u); return; }
        cmd_reset_axis(e);
        a->holdTarget = e->pos;
        a->holding    = e->directionKnown;
        {
            int32 d = encoder_deg100(a,e->pos);
            JLOGF("%s stop: pos %s%ld.%02ld out-deg, peak slip %ld lsb (%ld%% of pull-out)\r\n",
                  a->name, DEG_SGN(d), DEG_INT(d), DEG_FRC(d),
                  (long)a->peakSlip, (long)((a->peakSlip * 100) / PULLOUT_LSB));
        }
    }
    else
    {
        JLOGF("%s stop: %ld usteps\r\n", a->name, (long)a->usteps);
    }
}

/* Velocity ramp, once per JOG_TICK_US. */
static void jog_axis_tick(jog_axis_t *a)
{
    enc_t *e=axis_encoder(a);
    int8 want = a->want;

    if (a->latched != 0u)
    {
        if (want == 0) { a->latched = 0u; }
        want = 0;
    }

    /* Calibrated jog shares the PTP bounds and per-step guard. Before a straight
     * reference exists, retain the old encoder-relative jog guard. */
    if (e->directionKnown && (a==&g_arm1 ? g_homed1 : g_homed2) && want!=0)
    {
        if(!joint_step_allowed(a,want)) {
            if(!a->limitHit) { JLOGF("%s: configured joint limit, %s refused\r\n",a->name,jog_dir_name(want)); }
            a->limitHit=1u; want=0;
        } else { a->limitHit=0u; }
    }
    else if ((a->hasEnc != 0u) && (e->directionKnown != 0u) && (want != 0))
    {
        int8  sign  = jog_enc_sign(a, want);
        int32 limit = (int32)((int64)SOFT_LIMIT_OUT_DEG10*ENCODER_CPR*(a==&g_arm2 ? ARM2_GEAR_RATIO_X100 : GEAR_RATIO*100)/360000);
        if (((sign > 0) && (e->pos >= e->softCenter + limit)) ||
            ((sign < 0) && (e->pos <= e->softCenter - limit)))
        {
            if (a->limitHit == 0u)
            {
                JLOGF("%s: soft limit %d.%d out-deg reached, %s refused\r\n",
                      a->name, SOFT_LIMIT_OUT_DEG10 / 10,
                      SOFT_LIMIT_OUT_DEG10 % 10, jog_dir_name(want));
                a->limitHit = 1u;
            }
            want = 0;
        }
        else
        {
            a->limitHit = 0u;
        }
    }

    if (a->moving == 0)
    {
        if ((want != 0) && (a->settleMs == 0u))
        {
            a->moving   = want;
            a->rate     = a->rateStart;
            a->rateCap  = a->rateMax;
            a->rampRemainder=0u;
            a->peakSlip = 0;
            a->derates  = 0u;
            a->holding  = 0u;
            a->statusMs = 0u;
            if (a->hasEnc != 0u)
            {
                if(!enc_sample()) { stage_stop("jog start: encoder invalid",0u); return; }
                cmd_reset_axis(e);
            }
            /* First step is due immediately. */
            a->periodCy  = BCLK__BUS_CLK__HZ / a->rate;
            a->tLastStep = (SU_NOW() + a->periodCy) & 0x00FFFFFFu;
            JLOGF("%s %s\r\n", a->name, jog_dir_name(want));
        }
    }
    else if (want == a->moving)
    {
        a->rampRemainder+=a->accel;
        a->rate += a->rampRemainder / (1000000u / JOG_TICK_US);
        a->rampRemainder %= (1000000u / JOG_TICK_US);
        if (a->rate > a->rateCap) { a->rate = a->rateCap; }
    }
    else
    {
        uint32 dec;
        a->rampRemainder+=a->decel;
        dec=a->rampRemainder / (1000000u / JOG_TICK_US);
        a->rampRemainder %= (1000000u / JOG_TICK_US);
        if (a->rate > (a->rateStart + dec))
        {
            a->rate -= dec;
        }
        else
        {
            a->rate     = 0u;
            a->moving   = 0;
            a->settleMs = MOVE_SETTLE_MS;
        }
    }

    if (a->moving != 0)
    {
        a->periodCy = BCLK__BUS_CLK__HZ / a->rate;
        a->statusMs += JOG_TICK_US / 1000u;
        if (a->statusMs >= JOG_STATUS_MS)
        {
            a->statusMs = 0u;
            if (a->hasEnc != 0u)
            {
                int32 d = encoder_deg100(a,e->pos);
                JLOGF("%s %s %lu ustep/s pos %s%ld.%02ld out-deg slip %ld lsb\r\n",
                      a->name, jog_dir_name(a->moving), (unsigned long)a->rate,
                      DEG_SGN(d), DEG_INT(d), DEG_FRC(d), (long)a->peakSlip);
            }
            else
            {
                JLOGF("%s %s %lu ustep/s n=%ld usteps\r\n",
                      a->name, jog_dir_name(a->moving), (unsigned long)a->rate,
                      (long)a->usteps);
            }
        }
    }
    else if (a->settleMs != 0u)
    {
        a->settleMs--;
        if (a->settleMs == 0u) { jog_axis_settled(a); }
    }
    else if ((a->hasEnc != 0u) && (a->holding != 0u))
    {
        /* Closed-loop hold: nudge the rotor back one microstep at a time.
         * A huge error invalidates the position and stops the whole stage. */
        if (a->holdCooldownMs != 0u) { a->holdCooldownMs--; }
        else
        {
            int32 err = a->holdTarget - e->pos;
            if (abs_i32(err) >= SLIP_FAIL_LSB)
            {
                stage_stop("hold position lost", 0u);
            }
            else if (abs_i32(err) > CL_DEADBAND_LSB)
            {
                int8 e = (err > 0) ? (int8)1 : (int8)-1;
                jog_step(a, jog_enc_sign(a, e));
                a->holdCooldownMs = JOG_HOLD_STEP_MS;
            }
        }
    }
}

/* Fast loop: emit a step when it is due.  No catch-up after a late step. */
static void jog_axis_service(jog_axis_t *a)
{
    if ((a->moving != 0) && (SU_ELAPSED(a->tLastStep) >= a->periodCy))
    {
        a->tLastStep = SU_NOW();
        jog_step(a, a->moving);
    }
}

/* Both axes share the slip monitor, after every encoder sample while moving. */
static void jog_slip_check(jog_axis_t *a)
{
    enc_t *e=axis_encoder(a);
    int32 slip;
    uint32 periodUs;

    if ((a->hasEnc == 0u) || (e->directionKnown == 0u) || (a->moving == 0)) { return; }

    periodUs = a->periodCy / SU_CY_PER_US;
    slip = abs_i32(slip_axis(e,jog_enc_sign(a, a->moving), periodUs));
    if (slip > a->peakSlip) { a->peakSlip = slip; }

    if (slip >= SLIP_FAIL_LSB)
    {
        stage_stop("jog encoder following error", 0u);
    }
    else if ((slip >= SLIP_WARN_LSB) && (a->derates < SLIP_DERATE_MAX))
    {
        a->rateCap >>= 1;
        if (a->rateCap < a->rateStart) { a->rateCap = a->rateStart; }
        if (a->rate > a->rateCap)      { a->rate = a->rateCap; }
        a->derates++;
        JLOGF("%s SLIP WARN: %ld lsb, cap -> %lu ustep/s\r\n",
              a->name, (long)slip, (unsigned long)a->rateCap);
    }
}

static int8 jog_want_from_bits(uint8 left, uint8 right)
{
    if ((left != 0u) && (right == 0u)) { return -1; }
    if ((right != 0u) && (left == 0u)) { return  1; }
    return 0;
}

static void jog_print_help(void)
{
    JLOG("keys: a/d = arm1 left/right, Left/Right = arm2, hold to move, release to stop\r\n");
    JLOG("      C = enable, R = home, Space/Esc = STOP; I inputs, P status, E encoder\r\n");
}

static void jog_print_inputs(void)
{
    JLOGF("INPUTS: J4-1/P0[4]=%u J4-2/P0[3]=%u NPN/P1[0]=%u nFAULT=%u\r\n",
          (unsigned)(CyPins_ReadPin(PIN_LIMIT2) != 0u),
          (unsigned)(CyPins_ReadPin(PIN_LIMIT1) != 0u),
          (unsigned)(CyPins_ReadPin(PIN_ARM2_NPN) != 0u),
          (unsigned)(CyPins_ReadPin(PIN_FAULT) != 0u));
}

static uint8  g_lastCmd;
static uint32 g_lastCmdMs;

static void jog_apply_keys(uint8 mask)
{
        if (mask != 0u) {
            if (g_ptp.active) { stage_stop("jog key pressed during PTP",0u); }
            else if (g_ptp.reference) { ptp_invalidate(); }
        }
        g_jogKeysIdle = (mask == 0u) ? 1u : 0u;
        if ((g_home.phase != HOME_IDLE) && (mask != 0u))
        { stage_stop("jog key pressed during HOME", 0u); }
        if ((g_home.phase != HOME_IDLE) || g_ptp.active || (g_motionEnabled == 0u) || (g_homeNeedKeyRelease != 0u))
        {
            if (mask == 0u) { g_homeNeedKeyRelease = 0u; }
            g_arm1.want = 0;
            g_arm2.want = 0;
        }
        else
        {
            g_arm1.want = jog_want_from_bits(mask & 0x01u, mask & 0x02u);
            g_arm2.want = jog_want_from_bits(mask & 0x04u, mask & 0x08u);
        }
        host_lease_renew();
        g_rxGood++;
}

static void jog_handle_byte(uint8 b)
{
    if (b == JOG_CMD_STOP)
    {
        g_ptpRx.active=g_shortRx.active=0u;
        /* Also release after a protective stop, when enabled is already zero.
         * Do not latch a driver fault: a fresh idle lease can enable again. */
        stepper1_disable();
        stepper2_disable();
        if ((g_motionEnabled != 0u) || (g_home.phase != HOME_IDLE))
        { stage_stop("operator stop; drivers OFF", 0u); }
        else { ptp_invalidate(); g_keysSeen=0u; stage_status(); }
        return;
    }
    /* Legacy neutral can release inputs, but never renew the motion lease. */
    if (b==0x41u) { g_arm1.want=g_arm2.want=0; return; }
    if (ptp_receive(b)) { return; }
    if(g_uploadRxQuarantine) { jog_reject_byte(b); return; }
    /* The host sends each legacy command three times; act on the first copy. */
    if ((b == g_lastCmd) && ((g_jogMs - g_lastCmdMs) < JOG_CMD_REPEAT_MS))
    {
        g_lastCmdMs = g_jogMs;
        return;
    }
    g_lastCmd   = b;
    g_lastCmdMs = g_jogMs;

    switch (b)
    {
        case JOG_CMD_ENABLE:
        case JOG_CMD_HOME:
            jog_reject_byte(b);
            JLOG("CONTROL refused: v7 requires framed enable/home/jog\r\n");
            break;
        case '1':
        case '3':
            JLOG("Direction inversion disabled in stage1; use calibrated physical DIR settings\r\n");
            break;
        case 'i':
            jog_print_inputs();
            break;
        case 's':
            stage_status();
            jog_print_position(&g_arm1);
            jog_print_position(&g_arm2);
            JLOGF("rx: %lu good, %lu late, %lu framing, %lu rejected\r\n",
                  (unsigned long)g_rxGood, (unsigned long)g_rxLate,
                  (unsigned long)g_rxFraming, (unsigned long)g_rxRejected);
            JLOGF("rx last rejected: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                  g_rxLastBad[0], g_rxLastBad[1], g_rxLastBad[2], g_rxLastBad[3],
                  g_rxLastBad[4], g_rxLastBad[5], g_rxLastBad[6], g_rxLastBad[7]);
            break;
        case '?':
            jog_print_help();
            break;
        case 'e':
        {
            /* One raw AS5045 frame, for adjusting the magnet: keep pressing
             * until it says OK, then power-cycle to enter closed loop. */
            as5045_t s;
            uint8 code;
            as5045_read(&s);
            code = classify(&s);
            JLOGF("ENCODER %s raw=0x%05lX angle=%u OCF=%u COF=%u LIN=%u INC=%u DEC=%u PAR=%u(%s)%s\r\n",
                  state_name(code), (unsigned long)s.raw, (unsigned)s.angle,
                  (unsigned)s.ocf, (unsigned)s.cof, (unsigned)s.lin,
                  (unsigned)s.mag_inc, (unsigned)s.mag_dec, (unsigned)s.par,
                  (s.par_ok != 0u) ? "ok" : "BAD",
                  (g_arm1.hasEnc != 0u) ? " [angle feedback]" : " [motion blocked; C rechecks]");
            break;
        }
        case 'f':
        {
            /* Daisy chain check: board 1 and board 2 frames from one 38-clock read. */
            uint32 f1, f2;
            as5045_read_chain(&f1, &f2);
            JLOGF("CHAIN board1 raw=0x%05lX angle=%u OCF=%u COF=%u LIN=%u par=%s | board2 raw=0x%05lX angle=%u OCF=%u COF=%u LIN=%u par=%s\r\n",
                  (unsigned long)f1, (unsigned)((f1 >> 6) & ENCODER_MASK),
                  (unsigned)((f1 >> 5) & 1u), (unsigned)((f1 >> 4) & 1u), (unsigned)((f1 >> 3) & 1u),
                  (frame_parity_ok(f1) != 0u) ? "ok" : "BAD",
                  (unsigned long)f2, (unsigned)((f2 >> 6) & ENCODER_MASK),
                  (unsigned)((f2 >> 5) & 1u), (unsigned)((f2 >> 4) & 1u), (unsigned)((f2 >> 3) & 1u),
                  (frame_parity_ok(f2) != 0u) ? "ok" : "BAD");
            break;
        }
        case '!':
            if (g_motionEnabled != 0u) { JLOG("rx capture refused while enabled\r\n"); break; }
            JLOG("rxcap unavailable: hardware UART; use LINK counters\r\n");
            break;
        default:
            g_lastCmd = 0u;
            jog_reject_byte(b);
            break;
    }
}

static void jog_update_time(void)
{
    g_jogMs=wall_ms();
    if (g_ptpRx.active && (g_jogMs-g_ptpRx.lastMs>250u || g_jogMs-g_ptpRx.sinceMs>2000u)) {
        g_ptpRx.active=0u; g_ptpRxDropped++; g_ptpRxTimeout++; JLOG("PTP RX dropped: frame timeout\r\n");
    }
    if (g_shortRx.active && (g_jogMs-g_shortRx.lastMs>250u || g_jogMs-g_shortRx.sinceMs>2000u)) {
        g_shortRx.active=0u; g_ptpRxDropped++; g_ptpRxTimeout++;
    }
    if (g_motionEnabled == 0u) { return; }
    if ((g_keysSeen == 0u) || ((g_jogMs - g_lastKeyMs) > JOG_DEADMAN_MS))
    { stage_stop("DEADMAN host heartbeat timeout", 0u); return; }
}

static void jog_tick(void)
{
    static uint32 lastStatus;
    jog_update_time();
    if ((g_jogMs - lastStatus) >= JOG_STATUS_MS)
    { lastStatus = g_jogMs; stage_status(); }
    telem_service();
    if (g_motionEnabled == 0u) { return; }
    if (jog_safety_blocked() != 0u) { return; }
    if (g_home.phase != HOME_IDLE || g_ptp.active) { return; }
    jog_axis_tick(&g_arm1);
    jog_axis_tick(&g_arm2);
}

/* A pure framing error can retry a healthy, stationary upload. Other receive
 * faults and every error during motion still invalidate the coordinate reference. */
static void jog_rx_service(void)
{
    uint8 i;
    for (i=0u; i<8u; i++)
    {
        int16 b = uart_rx_service();
        if (b == -1) { break; }
        if (b <= -2)
        {
            g_ptpRxDropped++;
            if(b==-3 && (g_traj.state==1u || g_traj.state==2u) && !g_ptp.active &&
               g_home.phase==HOME_IDLE && g_motionEnabled && g_ptp.reference &&
               g_homed1 && g_homed2 && dual_ready() && !g_jogFaulted && !g_limitStop &&
               host_input_idle() && axes_idle() && !g_arm1.holding && !g_arm2.holding &&
               dual_ready() && abs_i32(cmd_lsb()-g_enc.pos)<SLIP_FAIL_LSB && (!ARM2_ENCODER_FEEDBACK || abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)<SLIP_FAIL_LSB) &&
               motion_inputs_clear() && home_inputs_clear() &&
               ptp_counts_in_range(0u,g_arm1.usteps) && ptp_counts_in_range(1u,g_arm2.usteps)) {
                g_uploadRxQuarantine=1u; g_ptpRx.active=2u; g_ptpRx.count=0u;
                g_ptpRx.sinceMs=g_ptpRx.lastMs=g_jogMs;
                JLOG("TRAJ RX framing discarded; awaiting retry; reference and committed nodes retained\r\n");
                break;
            }
            g_keysSeen = 0u;
            stage_stop("UART receive error/overflow; buffered commands discarded", 0u);
            break;
        }
        jog_handle_byte((uint8)b);
    }
}

/* Foreground loop, split into pieces so the offline tests run this exact code
 * instead of a hand-copied loop: app_init() axes + banner, app_start() timers +
 * watchdog, app_poll() one pass. service_motion() and service_encoder() are the
 * parts the tests also call between their own simulated-time steps. */
static uint32 g_loopTick;   /* SU_NOW() at the last 1 ms housekeeping tick */
static uint32 g_loopEnc;    /* SU_NOW() at the last encoder sample         */
static uint32 g_loopAt;     /* wall_ms() at the start of the previous pass */

static void service_motion(void)
{
    if (jog_safety_blocked() != 0u) { return; }
    if (g_home.phase != HOME_IDLE) { home_service(); }
    else if (g_ptp.active) { ptp_service(); }
    else { jog_axis_service(&g_arm1); jog_axis_service(&g_arm2); }
}

static void service_encoder(void)
{
    if (enc_sample() == 0u)
    {
        if (g_enc.badStreak >= ENC_BAD_LIMIT || g_enc2.badStreak >= ENC_BAD_LIMIT)
        {
            uint8 newlyLost=(g_enc.badStreak>=ENC_BAD_LIMIT && g_arm1.hasEnc) ||
                            (g_enc2.badStreak>=ENC_BAD_LIMIT && g_arm2.hasEnc);
            if(g_enc.badStreak>=ENC_BAD_LIMIT) { g_arm1.hasEnc=0u; }
            if(g_enc2.badStreak>=ENC_BAD_LIMIT) { g_arm2.hasEnc=0u; }
            if(newlyLost) {
                stage_stop("encoder lost; automatic open-loop fallback disabled", 0u);
            }
        }
    }
    if ((g_motionEnabled != 0u) && (g_home.phase == HOME_IDLE))
    {
        if(!g_enc.badStreak) { jog_slip_check(&g_arm1); }
        if(g_motionEnabled && !g_enc2.badStreak) { jog_slip_check(&g_arm2); }
    }
}

static void app_init(uint8 encOk)
{
    jog_axis_init(&g_arm1, "arm1", PIN_STEP2, PIN_DIR2, 1u, encOk,
                  ARM1_LEFT_DIR_LEVEL,
                  JOG1_RATE_MAX_USTEP_S, JOG1_RATE_START_USTEP_S,
                  JOG1_ACCEL_USTEP_S2, JOG1_DECEL_USTEP_S2);
    jog_axis_init(&g_arm2, "arm2", PIN_STEP1, PIN_DIR1, 0u, encOk && ARM2_ENCODER_FEEDBACK,
                  ARM2_LEFT_DIR_LEVEL,
                  JOG2_RATE_MAX_USTEP_S, JOG2_RATE_START_USTEP_S,
                  JOG2_ACCEL_USTEP_S2, JOG2_DECEL_USTEP_S2);

    /* No startup enable, direction probe, return-to-origin, or holding steps. */
    JLOG("\r\nSTAGE2 v9 optional arm2 feedback framed control: both drivers disabled, reference unset; use home or explicit manual straight reference\r\n");
    JLOG("INPUT MAP: HOME1=J4-2/P0[3], GLOBAL=J4-1/P0[4], HOME2=NPN/P1[0]\r\n");
    JLOG("C = explicit enable/recheck; R = home; Space/Esc = latched stop\r\n");
    jog_print_inputs();
    stage_status();
    jog_print_help();
    /* Drain startup output in the foreground loop so encoder sampling starts now. */

    /* Never execute commands buffered during the power-on self-test. */
    uart_discard_rx();
}

static void app_start(void)
{
    g_loopTick = SU_NOW();
    g_loopEnc  = SU_NOW();
    g_jogMs    = wall_ms();
    g_loopAt   = g_jogMs;
    CyILO_Start1K();
    CyWdtStart(CYWDT_128_TICKS, CYWDT_LPMODE_NOCHANGE);
}

static void app_poll(void)
{
    uint32 loopNow = wall_ms();
    if (loopNow - g_loopAt > g_loopMaxMs) { g_loopMaxMs = loopNow - g_loopAt; }
    g_loopAt = loopNow;
    /* Check the elapsed deadline BEFORE consuming queued heartbeats or
     * emitting another pulse. No motion catch-up after a stalled loop. */
    jog_update_time();
    /* Bound RX work so a host flood cannot starve motion or deadman. */
    jog_rx_service();

    service_motion();

    jog_rx_service();

    if (SU_ELAPSED(g_loopTick) >= JOG_TICK_CY)
    {
        uint32 lag = SU_ELAPSED(g_loopTick) / (1000u * SU_CY_PER_US);
        if (lag > g_tickMaxLagMs) { g_tickMaxLagMs = lag; }
        g_loopTick = SU_NOW();
        jog_tick();
    }

    /* Encoder: about once per ms, in a gap between steps.  If no gap
     * appears for 5 ms take the sample anyway (one late step). */
    if (((g_arm1.hasEnc != 0u) || (g_arm2.hasEnc != 0u)) && (SU_ELAPSED(g_loopEnc) >= JOG_ENC_SAMPLE_CY))
    {
        if ((jog_slot_free(JOG_ENC_READ_US) != 0u) ||
            (SU_ELAPSED(g_loopEnc) >= (5u * JOG_ENC_SAMPLE_CY)))
        {
            g_loopEnc = SU_NOW();
            service_encoder();
        }
    }

    if ((txq_pending() != 0u) && (jog_slot_free(JOG_TX_BYTE_US) != 0u))
    {
        txq_send_one();
    }
    CyWdtClear(); /* Only a completed foreground iteration feeds the watchdog. */
}

static void jog_run(uint8 encOk) __attribute__((noreturn));
static void jog_run(uint8 encOk)
{
    app_init(encOk);
    app_start();
    for (;;) { app_poll(); }
}

/* ================================= main ================================= */
static uint8 app_boot(void)
{
    as5045_t sample, second;
    uint32 raw1, raw2;
    uint8 code = ST_DO_LOW;
    uint8 lastCode = 0xFFu;
    uint16 validStreak = 0u;
    uint16 validCount = 0u;
    uint16 count;
    uint8 encOk;

    hw_init();
    CyDelay(200u);  /* MODE floating/LOW needs at most 80 ms until OCF=1. */

    DBG("\r\nSELFTEST START: static inputs and AS5045, both drivers disabled\r\n");
    for (count = 0u; count < ENCODER_SELFTEST_MS / 50u; count++)
    {
        as5045_read_chain(&raw1,&raw2);
        as5045_decode(&sample,raw1); as5045_decode(&second,raw2);
        code = classify(&sample);
        if(ARM2_ENCODER_FEEDBACK && !encoder_state_valid(classify(&second))) { code=classify(&second); }
        if (encoder_state_valid(code)) { validStreak++; validCount++; }
        else { validStreak = 0u; }
        if (code != lastCode)
        {
            DBGF("SELFTEST encoder=%s raw=0x%05lX angle=%u\r\n",
                 state_name(code), (unsigned long)sample.raw, (unsigned)sample.angle);
            lastCode = code;
        }
        CyDelay(50u);
    }
    encOk = (validStreak >= 20u) ? enc_set_origin() : 0u;
    DBGF("SELFTEST %s: valid=%u/%u recent=%u; nFAULT=%u P0[4]=%u P0[3]=%u NPN=%u\r\n",
         (encOk && !FAULT_ACTIVE() && !HARD_LIMIT_ACTIVE()) ? "PASS" : "BLOCKED",
         (unsigned)validCount, (unsigned)count, (unsigned)validStreak,
         (unsigned)(CyPins_ReadPin(PIN_FAULT) != 0u), (unsigned)(CyPins_ReadPin(PIN_LIMIT2) != 0u),
         (unsigned)(CyPins_ReadPin(PIN_LIMIT1) != 0u), (unsigned)(CyPins_ReadPin(PIN_ARM2_NPN) != 0u));
    DBG("SELFTEST is electrical status only: motor power/motion and switch wiring need bench verification\r\n");
    cmd_reset_to_measured(); cmd_reset_axis(&g_enc2);
    return encOk;
}

int main(void)
{
    jog_run(app_boot());
    return 0;
}

/* [] END OF FILE */
