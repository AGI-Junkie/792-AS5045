/* ========================================================================
 * AS5045 + DRV8825: two-joint arm firmware (stage2-v11, see FW_BUILD_ID)
 * Target: CY8CKIT-059 (PSoC 5LP), BUS_CLK as set in the Clocks tab (24 MHz until the clock step, then 48 MHz), GPIO + UDB host UART.
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
/* Hardware offload switches (docs/HARDWARE_OFFLOAD_PLAN.md).  A switch is 1 only while the matching component is in
 * TopDesign; 0 keeps the software path, which the offline fixture also builds. */
#ifndef HW_SSI
#define HW_SSI                      (0u) /* AS5045 read-out by the UDB SPI Master "Ssi" instead of bit-banging */
#endif
#ifndef HW_STEP
#define HW_STEP                     (0u) /* STEP pulses from Control_Regs "StepArm1Req"/"StepArm2Req" + UDB PWMs, no busy-wait */
#endif
#ifndef HW_EVT
#define HW_EVT                      (0u) /* trajectory steps from the EvtTimer interrupt instead of the foreground poll */
#endif
#ifndef HW_PINIRQ
#define HW_PINIRQ                   (0u) /* nFAULT / global limit pin interrupts kill the STEP PWMs at once (needs HW_STEP) */
#endif
#ifndef HW_TXDMA
#define HW_TXDMA                    (0u) /* debug-UART transmit by DMA from the text queue into the UART's TX FIFO */
#endif
#ifndef HW_DEAD
#define HW_DEAD                     (0u) /* hardware deadman: DeadTimer + AliveReq cut the STEP PWMs if the foreground stalls */
#endif
#if (HW_EVT != 0u || HW_DEAD != 0u || HW_PINIRQ != 0u) && (HW_STEP == 0u)
#error "HW_EVT, HW_DEAD and HW_PINIRQ need HW_STEP: the interrupt must not busy-wait for pulses, the kills act on the STEP PWMs"
#endif

#include <cytypes.h>
#include <cyfitter.h>
#include <cydevice_trm.h>
#include <cypins.h>
#include <CyLib.h>
#include "HostUart.h"
#if (HW_SSI != 0u)
#include "Ssi.h"
#endif
#if (HW_STEP != 0u)
#include "StepArm1Req.h"
#include "StepArm2Req.h"
#include "StepArm1.h"
#include "StepArm2.h"
#endif
#if (HW_EVT != 0u)
#include "EvtTimer.h"
#include "EvtRestart.h"
#include "EvtIsr.h"
#endif
#if (HW_DEAD != 0u)
#include "AliveReq.h"
#include "DeadTimer.h"
#endif
#if (HW_PINIRQ != 0u)
#include "FAULT.h"
#include "LIMIT2.h"
#include "StepKill.h"
#include "PinIsr.h"
#endif
#if (HW_TXDMA != 0u)
#include "TxDma_dma.h"
#ifndef TXDMA_ADDR
#define TXDMA_ADDR(p)               LO16((uint32)(p))   /* low half of a bus address; the high half is the channel's */
#endif
#endif
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ============================ USER CONFIGURATION ======================== */
#define USE_DEBUG_UART              (1u)     /* P12[7], 115200 8N1          */
#define PARITY_STRICT               (1u)
#define ENCODER_SELFTEST_MS         (5000u)

/* Physical directions confirmed on the bench: left DIR=0 for both axes.
 * Encoder polarity is learned only during manually requested homing.
 * arm1 always needs its motor encoder.  arm2's is optional: with ARM2_ENCODER_FEEDBACK=0 (the delivery default)
 * arm2 runs on commanded steps without feedback; with 1 both encoders are required.  Either way a lost encoder
 * that is required stops the machine; there is no automatic open-loop fallback. */
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
 * The STATE line keeps the plain "stage2-v11" token because the host matches it to
 * select the framed transport (StatePattern in App.cs).  Bump this on every firmware change so a field log
 * can answer "was this fix actually flashed?". */
#ifndef ARM2_ENCODER_FEEDBACK
#define ARM2_ENCODER_FEEDBACK       (1u) /* Enable only after the encoder/magnet is fixed and commissioned. */
#endif

#define FW_BUILD_ID                 "stage2-v11-r2500-slip-mref2"

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
#define ARM2_STRAIGHT_OUT_DEG10      (1620)   /* temporary visual correction: 160 + 2 deg */
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
#define PTP_RATE_MAX                 (2500u)
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
/* arm2 counterpart, used by a manual reference only while homing has not learned arm2's polarity (power-up).
 * Bench evidence is one homing log (2026-10-07: "increase DIR=1", same as arm1).  A wrong value cannot move the arm
 * silently: the first move ends in an encoder following-error stop, and a homing run learns the real polarity. */
#define MANUAL_ENC2_INC_DIR          (1u)

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
#if (HW_STEP != 0u)
#define DIR_SETUP_US                (2u)     /* DRV8825 needs 650 ns; the pulse also starts >= 1 us after the request */
#else
#define DIR_SETUP_US                (20u)
#endif
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

/* Extend the existing free-running 24-bit down-counter without changing pulse timing. The ISR only accounts for
 * wraps; foreground stalls do not slow deadlines.  One wrap is 2^24 cycles = SU_WRAP_MS whole milliseconds plus
 * SU_WRAP_REM cycles at any whole-MHz bus clock, so the ISR keeps (milliseconds, remainder cycles) exactly and
 * wall_ms() needs one divide by a constant: ms = g_wallMs + (g_wallRemCy + cycles since the last wrap) / SU_CY_PER_MS.
 * (24 MHz: 699 ms + 1216 cycles per wrap; 48 MHz: 349 ms + 25216 cycles.) */
#define SU_CY_PER_MS                (BCLK__BUS_CLK__HZ / 1000u)
#define SU_WRAP_MS                  (0x01000000u / SU_CY_PER_MS)
#define SU_WRAP_REM                 (0x01000000u % SU_CY_PER_MS)
_Static_assert(BCLK__BUS_CLK__HZ % 1000000u == 0u, "SU_CY_PER_US needs a whole-MHz bus clock");
static volatile uint32 g_clockWraps, g_wallMs, g_wallRemCy;
static void clock_wrap_account(volatile uint32 *ms, volatile uint32 *rem)
{
    *ms += SU_WRAP_MS; *rem += SU_WRAP_REM;
    if (*rem >= SU_CY_PER_MS) { *rem -= SU_CY_PER_MS; (*ms)++; }
}
CY_ISR(clock_wrap_isr) { g_clockWraps++; clock_wrap_account(&g_wallMs, &g_wallRemCy); }
static uint32 wall_ms(void)
{
    uint32 ms, rem, current;
    uint8 ints = CyEnterCriticalSection();
    ms = g_wallMs; rem = g_wallRemCy;
    current = SU_NOW();
    if (CY_GET_REG32(CYREG_NVIC_INTR_CTRL_STATE) & 0x04000000u) {
        /* A wrap occurred while interrupts were masked; count it exactly once. */
        clock_wrap_account(&ms, &rem);
        current = SU_NOW();
    }
    CyExitCriticalSection(ints);
    return ms + (rem + (0x00FFFFFFu - current)) / SU_CY_PER_MS;
}

static void su_timer_start(void)
{
    g_clockWraps = 0u; g_wallMs = 0u; g_wallRemCy = 0u;
    (void)CyIntSetSysVector(CY_INT_SYSTICK_IRQN, clock_wrap_isr);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_RELOAD,  0x00FFFFFFu);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_CURRENT, 0u);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_CTL,     0x07u);
}

/* Keep each call far below one SysTick wrap (SU_WRAP_MS: 699 ms at 24 MHz, 349 ms at 48 MHz). */
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

static char g_dbg[768];   /* Stop evidence includes trajectory timing; keep reason and CRLF. */
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
/* The serial read-out (bit-banged or UDB SPI Master, see hw_ssi.h) gives as5045_read_clocks() and the
 * ssi_begin()/ssi_done()/ssi_take() trio. */
#include "hw_ssi.h"
static void as5045_read_chain(uint32 *first, uint32 *second) { as5045_read_clocks(first, second, 38u); }
static uint32 as5045_read_raw(void) { uint32 first, second; as5045_read_clocks(&first, &second, 18u); return first; }

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
    int32 snapUsteps, snapLsbX100;   /* the command as it stood when the frame now being decoded was started */
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
    int64 commanded, commandedLsb, position;

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
        commanded=(int64)e->snapUsteps-e->lastCmd;
        if(commanded<0) { commanded=-commanded; }
        /* commanded steps as encoder LSB; 32-bit hardware divide unless the product could overflow */
        commandedLsb=(commanded<=INT32_MAX/LSB_PER_USTEP_X100) ? (int64)((int32)commanded*LSB_PER_USTEP_X100/100)
                                                               : commanded*LSB_PER_USTEP_X100/100;
        position=(int64)e->pos+delta;
        /* At the configured maximum 6000 usteps/s, 100 ms plus the margin
         * stays below half a motor turn. A lost turn cannot be reconstructed. */
        if(gap>ENC_MAX_GAP_MS ||
           abs_i32(delta)>(int32)((MOVE_RATE_MAX_USTEP_S*(uint32)LSB_PER_USTEP_X100*(gap+1u))/100000u)+ENC_JUMP_MARGIN_LSB ||
           abs_i32(delta)>commandedLsb+ENC_JUMP_MARGIN_LSB ||
           position>INT32_MAX/100 || position<INT32_MIN/100) {
            e->trackingLost=1u; e->badStreak=ENC_BAD_LIMIT; g_encRejected++;
            return 0u;
        }
        e->pos = (int32)position;
        e->lastAngle = sample->angle;
    }
    e->lastMs=wall_ms(); e->lastCmd=e->snapUsteps;
    {
        int64 following=(int64)(e->snapLsbX100/100)-e->pos; /* int32 divide: same quotient, no 64-bit libcall */
        if(following<0) { following=-following; }
        if((uint64_t)following>g_followPeakLsb) { g_followPeakLsb=(uint32)following; }
    }
    return 1u;
}

/* The command is captured when a frame is started, not when it is decoded: with the hardware read-out the steps keep
 * coming while the frame is on the wire. */
static void enc_snapshot(void)
{
    g_enc.snapUsteps=g_enc.cmdUsteps;   g_enc.snapLsbX100=g_enc.cmdLsbX100;
    g_enc2.snapUsteps=g_enc2.cmdUsteps; g_enc2.snapLsbX100=g_enc2.cmdLsbX100;
}
/* Decode one raw frame pair; true when every frame this configuration needs is usable. */
static uint8 enc_decode_pair(uint32 raw1, uint32 raw2, as5045_t *a, as5045_t *b)
{
    uint8 v1, v2=0u;
    as5045_decode(a,raw1);
    v1=encoder_state_valid(classify(a));
    if(ARM2_ENCODER_FEEDBACK) { as5045_decode(b,raw2); v2=encoder_state_valid(classify(b)); }
    return (uint8)(v1 && (!ARM2_ENCODER_FEEDBACK || v2));
}
/* One shared transaction; a bad axis never advances its position.  With arm2 feedback off (the default) it is the
 * 18-clock read of board 1 alone and g_enc2 is left untouched: nothing uses it, and updating it from a floating
 * second frame only polluted ENC_DIAG (follow_peak_lsb, gap, rejected) with arm2's unmeasured "following error". */
static uint8 enc_apply(const as5045_t *a, const as5045_t *b)
{
    uint8 ok1=enc_update(&g_enc,a,encoder_state_valid(classify(a))), ok2;
    if(!ARM2_ENCODER_FEEDBACK) { return ok1; }
    ok2=enc_update(&g_enc2,b,encoder_state_valid(classify(b)));
    return (uint8)(ok1 && ok2);
}
#if (HW_SSI != 0u)
static uint8 g_encFlight, g_encTry;   /* a frame is on the wire / attempts made for the current sample */
static uint32 g_encFlightAt;
#endif
static uint8 enc_sample(void)
{
    as5045_t a, b;
    uint32 raw1, raw2;
    uint8 i;
#if (HW_SSI != 0u)
    if(g_encFlight) { g_encFlight=0u; ssi_abort(); }   /* a synchronous read takes the interface over */
#endif
    for(i=0u;i<ENCODER_READ_RETRIES;i++) {
        if(i) { g_encRetries++; }
        enc_snapshot();
        as5045_read_clocks(&raw1,&raw2,ARM2_ENCODER_FEEDBACK ? 38u : 18u);
        if(enc_decode_pair(raw1,raw2,&a,&b)) { break; }
    }
    return enc_apply(&a,&b);
}
#if (HW_SSI != 0u)
/* The same sample without waiting: enc_flight_start() queues a frame, enc_flight_poll() finishes it (with the same
 * retries) once the hardware has it.  Returns true when a sample is complete; *ok is what enc_sample() would return. */
static void enc_flight_start(uint8 retry)
{
    if(retry) { g_encRetries++; }
    enc_snapshot();
    ssi_begin(ARM2_ENCODER_FEEDBACK ? 38u : 18u);
    g_encFlightAt=SU_NOW(); g_encFlight=1u;
}
static uint8 enc_flight_poll(uint8 *ok)
{
    as5045_t a, b;
    uint32 raw1=0u, raw2=0u;
    if(!g_encFlight) { return 0u; }
    if(ssi_done()) { ssi_take(&raw1,&raw2); }
    else if(SU_ELAPSED(g_encFlightAt) >= SSI_TIMEOUT_US*SU_CY_PER_US) { ssi_abort(); }   /* a dead frame reads as 0 */
    else { return 0u; }
    if(!enc_decode_pair(raw1,raw2,&a,&b) && (uint8)(g_encTry+1u)<ENCODER_READ_RETRIES) { g_encTry++; enc_flight_start(1u); return 0u; }
    g_encFlight=0u;
    *ok=enc_apply(&a,&b);
    return 1u;
}
#endif

/* Average 12 samples relative to the first angle so the average also works
 * when the wrap lies inside the group. Used once, to set the origin. */
static uint8 enc_set_origin(void)
{
#if (HW_SSI != 0u)
    if(g_encFlight) { g_encFlight=0u; ssi_abort(); }
#endif
    uint16 reference[2]={0u,0u};
    int32 sum[2]={0,0};
    uint8 i, axis;
    uint32 raw[2];
    as5045_t sample;
    for(i=0u;i<12u;i++) {
        uint8 attempt, good=0u;
        /* ENABLE already blocks for these 12 ms, so one bad frame gets the same retries enc_sample() allows (1 ms apart)
         * instead of refusing the whole enable. */
        for(attempt=0u;attempt<ENCODER_READ_RETRIES && !good;attempt++) {
            if(attempt) { g_encRetries++; CyDelay(1u); }
            as5045_read_clocks(&raw[0],&raw[1],ARM2_ENCODER_FEEDBACK ? 38u : 18u);
            good=1u;
            for(axis=0u;axis<(ARM2_ENCODER_FEEDBACK ? 2u : 1u);axis++) {
                as5045_decode(&sample,raw[axis]);
                if(!encoder_state_valid(classify(&sample))) { good=0u; }
            }
        }
        if(!good) { return 0u; }
        for(axis=0u;axis<(ARM2_ENCODER_FEEDBACK ? 2u : 1u);axis++) {
            as5045_decode(&sample,raw[axis]);
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
#if (HW_SSI != 0u)
    if(g_encFlight) { g_encFlight=0u; ssi_abort(); }   /* its snapshot predates this re-anchoring */
#endif
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
#if (HW_STEP == 0u)
    pin_write(PIN_STEP2, 1u);
#endif
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
#if (HW_STEP == 0u)
    pin_write(PIN_STEP1, 0u);
#endif
}

static void stepper1_enable(void)
{
#if (HW_STEP == 0u)
    pin_write(PIN_STEP1, 0u);
#endif
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

#if (HW_STEP == 0u)
static void pulse_step_pin(uint32 pinPC, uint8 idleHigh)
{
    uint32 t0 = SU_NOW();
    pin_write(pinPC, (uint8)((idleHigh != 0u) ? 0u : 1u));
    pulse_hold_us(t0, STEP_LOW_US);
    pin_write(pinPC, idleHigh);
}
#else
/* One register write per axis: "StepArm1Req" / "StepArm2Req" are 1-bit PulseMode Control_Regs on the 12 MHz step clock,
 * each triggering a one-shot PWM ("StepArm1": STEP2, low pulse, idle high; "StepArm2": STEP1, high pulse, idle low)
 * whose period is the pulse width (STEP_LOW_US).  The two axes have separate registers on purpose: a write to a shared
 * register would clear the other axis' request before the 12 MHz clock had sampled it. */
#define STEP_ARM1                   (1u)
#define STEP_ARM2                   (2u)
static void step_fire(uint8 mask)
{
    if ((mask & STEP_ARM1) != 0u) { StepArm1Req_Write(1u); }
    if ((mask & STEP_ARM2) != 0u) { StepArm2Req_Write(1u); }
}
#endif

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
#if (HW_EVT != 0u)
static void evt_init(void);
#endif
#if (HW_TXDMA != 0u)
static void txdma_init(void);
#endif
#if (HW_PINIRQ != 0u)
static void pinirq_init(void);
#endif
static void hw_init(void)
{
    CyGlobalIntEnable;

#if (HW_SSI != 0u)
    Ssi_Start();   /* Creator owns CLK, CSn and DO (the SPI Master's pins); never overwrite their PC registers */
#else
    pin_out_init(PIN_CSN, 1u);
    pin_out_init(PIN_CLK, 1u);
    CY_SET_REG8(PIN_DO, CY_PINS_DM_DIG_HIZ);
#endif
#if (HW_STEP == 0u)
    pin_out_init(PIN_LED, 0u);
#else
    StepArm1_Start();   /* Creator owns STEP1 (it is the LED pin) and STEP2: both are PWM outputs, idle until a request */
    StepArm2_Start();
#endif

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
#if (HW_STEP == 0u)
    pin_out_init(PIN_STEP2,  1u);
#endif
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

#if (HW_EVT != 0u)
    evt_init();
#endif
#if (HW_PINIRQ != 0u)
    pinirq_init();
#endif
#if (USE_DEBUG_UART != 0u)
    /* Creator owns the UART pin routing; never overwrite its PC registers. */
    HostUart_Start();
#if (HW_TXDMA != 0u)
    txdma_init();
#endif
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

#if (HW_TXDMA != 0u)
/* HostUart's TX buffer is just its 4-byte FIFO (Creator: TX buffer size 4): the DMA channel "TxDma" (1-byte bursts, one
 * request per burst, request = TX FIFO not full) copies g_txq[tail .. tail+len) into HostUart_TXDATA, so the CPU
 * only queues text.  g_txqTail moves when a transfer has finished, never earlier, so the space the DMA still reads
 * is not offered to txq_puts(). */
static uint8 g_txDmaCh, g_txDmaTd;
static uint16 g_txDmaLen;            /* bytes of the running transfer, 0 = idle */
static void txdma_init(void)
{
    g_txDmaCh = TxDma_DmaInitialize(1u, 1u, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_PERIPH_BASE));
    g_txDmaTd = CyDmaTdAllocate();
}
#endif
static uint8 txq_pending(void)
{
#if (HW_TXDMA != 0u)
    if (g_txDmaLen != 0u) { return 1u; }
#endif
    return (g_txqHead != g_txqTail) ? 1u : 0u;
}

static uint16 txq_free(void)
{
    return (uint16)((g_txqTail + TXQ_SIZE - g_txqHead - 1u) % TXQ_SIZE);
}

#if (HW_TXDMA != 0u)
/* Keeps the DMA fed: retires a finished transfer, starts the next contiguous run of the queue. */
static void txq_send_one(void)
{
    if (g_txDmaLen != 0u)
    {
        uint8 td, state;
        (void)CyDmaChStatus(g_txDmaCh, &td, &state);
        if ((state & (CY_DMA_STATUS_CHAIN_ACTIVE | CY_DMA_STATUS_TD_ACTIVE)) != 0u) { return; }
        g_txqTail = (uint16)((g_txqTail + g_txDmaLen) % TXQ_SIZE);
        g_txDmaLen = 0u;
    }
    if (g_txqHead != g_txqTail)
    {
        uint16 len = (g_txqHead > g_txqTail) ? (uint16)(g_txqHead - g_txqTail) : (uint16)(TXQ_SIZE - g_txqTail);
        (void)CyDmaTdSetConfiguration(g_txDmaTd, len, CY_DMA_DISABLE_TD, CY_DMA_TD_INC_SRC_ADR);
        (void)CyDmaTdSetAddress(g_txDmaTd, TXDMA_ADDR(&g_txq[g_txqTail]), TXDMA_ADDR(HostUart_TXDATA_PTR));
        (void)CyDmaChSetInitialTd(g_txDmaCh, g_txDmaTd);
        (void)CyDmaChEnable(g_txDmaCh, 1u);
        g_txDmaLen = len;
    }
}
#else
static void txq_send_one(void)
{
    if (g_txqHead != g_txqTail && HostUart_GetTxBufferSize() < HostUart_TX_BUFFER_SIZE - 1u)
    {
        HostUart_PutChar((uint8)g_txq[g_txqTail]);
        g_txqTail = (uint16)((g_txqTail + 1u) % TXQ_SIZE);
    }
}
#endif

static void txq_flush_blocking(void)
{
    uint32 start = wall_ms();
    while (txq_pending() != 0u) {
        if (wall_ms()-start >= 100u) {
#if (HW_TXDMA != 0u)
            (void)CyDmaChDisable(g_txDmaCh); g_txDmaLen=0u;   /* abandon the transfer along with the queue */
#endif
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
#define JOG_ENC_READ_US             (ARM2_ENCODER_FEEDBACK ? 360u : 200u) /* 38-clock dual / 18-clock single AS5045 frame + GPIO overhead margin */
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

/* Whole-path preload: no movement until every numbered node is accepted.  The plan's data and rules are in traj.h; the
 * numbers it is built on stay here, where the release gate reads them against DeviceContract.cs. */
#define TRAJ_CAPACITY 2048u
#define TRAJ_RATE_MAX 2500u
#define TRAJ_ACCEL_MAX 3000u
/* Never pulse a trajectory faster than this, even when catching up.  Deliberately NOT tied to TRAJ_RATE_MAX: the plan stays
 * under 2500 steps/s, but a loop that fell a little behind must still be able to close the gap (250 us is the spacing the
 * hardware-tested builds used). */
#define TRAJ_BURST_RATE_MAX 4000u
#define TRAJ_STEP_MIN_US (1000000u / TRAJ_BURST_RATE_MAX)
/* Ride the limit instead of stopping: a step more than TRAJ_LATE_SOFT_US behind its due time means the loop cannot keep this
 * plan's pace right now, so the plan clock is held back by the excess (the segment simply takes longer; every segment still
 * gets exactly its step count, so the drawn shape is unchanged).  Only a stall that outlasts the 20 ms segment margin, or a
 * total stretch beyond TRAJ_SLIP_MAX_MS (the run is not sustainable at all), still stops. */
#define TRAJ_LATE_SOFT_US 1000u
#define TRAJ_SLIP_MAX_MS 5000u
#include "traj.h"

static struct {
    uint8 active; /* 1 moving, 2 settling */
    uint8 reference;
    uint8 referenceSource; /* 1 switch homing, 2 operator-defined straight pose */
    int32 encoderOrigin; /* arm1 compatibility field */
    uint32 epoch;
    uint32 sequence;
    uint32 sinceMs, settleMs, lastStep, periodUs;
    int32 origin[2];
    dda_t seg; /* the segment being stepped: one point-to-point move, or one trajectory node */
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

static traj_t g_traj;

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
static void stage_status_service(void);
static uint8 jog_slot_free(uint32 us);

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

/* Event timer (HW_EVT): evt_cancel() must run before anything rewrites the trajectory state, so the interrupt can no
 * longer be in the middle of it. */
#if (HW_EVT != 0u)
static void evt_cancel(void);
static void evt_start(void);
#else
#define evt_cancel() ((void)0)
#endif

/* Hardware deadman (HW_DEAD): the foreground writes AliveReq once per pass; DeadTimer is reset by it and, if 100 ms
 * pass without a write, its terminal count latches a kill on both STEP PWMs, with or without any software running. */
#if (HW_DEAD != 0u)
static void alive_kick(void) { AliveReq_Write(1u); }
static void deadman_arm(void)   /* (re)start the PWMs, which clears a latched kill, and the timer */
{
    StepArm1_Stop(); StepArm1_Start(); StepArm2_Stop(); StepArm2_Start();
    DeadTimer_Start();
    (void)DeadTimer_ReadStatusRegister();
    alive_kick();
}
static uint8 deadman_tripped(void) { return ((DeadTimer_ReadStatusRegister() & DeadTimer_STATUS_TC) != 0u) ? 1u : 0u; }
#else
#define alive_kick()     ((void)0)
#define deadman_arm()    ((void)0)
#define deadman_tripped() (0u)
#endif

/* Pin interrupts (HW_PINIRQ): nFAULT falling / P0[4] rising kill both STEP PWMs from priority 0 through StepKill (a
 * Control_Reg wired into their kill input) before any foreground code has looked at the pins; the foreground then stops
 * the stage, whether or not the pin is still active (a pulse is a stop too).  Only an operator enable clears the kill. */
#if (HW_PINIRQ != 0u)
static volatile uint8 g_pinStop;
CY_ISR(pin_stop_isr)
{
    (void)FAULT_ClearInterrupt();
    (void)LIMIT2_ClearInterrupt();
    StepKill_Write(1u);
    g_pinStop = 1u;
}
static void pinirq_init(void)
{
    PinIsr_StartEx(pin_stop_isr);
    PinIsr_SetPriority(0u);
}
static void step_unkill(void) { g_pinStop = 0u; StepKill_Write(0u); }
#else
#define step_unkill()   ((void)0)
#endif

/* Forget the held trajectory (and any upload isolation) without touching the reference. */
static void traj_discard(void)
{
    evt_cancel();
    if(g_uploadRxQuarantine) { uart_discard_rx(); }
    g_uploadRxQuarantine=0u;
    traj_reset(&g_traj);
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
    if (g_ptp.active) { return g_traj.state == TRAJ_RUNNING ? "traj_run" : "ptp"; }
    if (g_traj.state == TRAJ_UPLOADING) { return "traj_upload"; }
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

/* RAM only: repeat evidence while stopped, even if the first TX attempt was dropped.
 * A later operator STOP while already stopped must not overwrite the initiating fault. */
static char g_lastStop[800];
static uint32 g_lastStopId;
static void fault_snapshot(const char *reason, uint8 driverFault)
{
    uint32 elapsedUs=0u, plannedUs=0u;
    if(g_traj.state==TRAJ_RUNNING && g_traj.index<g_traj.count) {
        elapsedUs=traj_elapsed_cy(&g_traj,SU_NOW())/SU_CY_PER_US;
        plannedUs=(uint32)g_traj.nodes[g_traj.index].ms*1000u;
    }
    JLOGF("FAULT_SNAPSHOT t=%lu epoch=%lu seq=%lu phase=%s home=%u traj=%u idx=%u/%u ref=%u p1=%ld p2=%ld e1=%ld err=%ld "
        ENC2_LOG_FORMAT
        "inputs=%u hb_age_ms=%lu hb_max_gap_ms=%lu elapsed_us=%lu plan_us=%lu late_us=%lu done=%lu total=%lu drv=%s reason=\"%s\"\r\n",
          (unsigned long)g_jogMs,(unsigned long)g_ptp.epoch,(unsigned long)g_ptp.sequence,stop_phase(),
          (unsigned)g_home.phase,(unsigned)g_traj.state,(unsigned)g_traj.index,(unsigned)g_traj.count,
          (unsigned)(g_ptp.reference && dual_ready()),
          (long)(g_ptp.origin[0]-g_arm1.usteps),(long)(g_ptp.origin[1]-g_arm2.usteps),
          (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),(long)(cmd_lsb()-g_enc.pos),
        ENC2_LOG_ARGS
          (unsigned)((HARD_LIMIT_ACTIVE()?1u:0u)|(ARM1_HOME_ACTIVE()?2u:0u)|(ARM2_NPN_ACTIVE()?4u:0u)|(FAULT_ACTIVE()?8u:0u)),
          (unsigned long)(g_keysSeen ? (g_jogMs - g_lastKeyMs) : 0u),(unsigned long)g_keyMaxGapMs,
          (unsigned long)elapsedUs,(unsigned long)plannedUs,(unsigned long)(elapsedUs>plannedUs ? elapsedUs-plannedUs : 0u),
          (unsigned long)g_ptp.seg.done,(unsigned long)g_ptp.seg.total,
          (driverFault != 0u || FAULT_ACTIVE()) ? "nfault_undetermined" : "ok", reason);
}

/* Protective stops retain holding current unless the driver itself faults.
 * The explicit operator STOP also disables both drivers at its command entry. */
static void stage_stop(const char *reason, uint8 driverFault)
{
    evt_cancel();   /* the evidence below must describe a frozen plan */
    fault_snapshot(reason, driverFault);
#if (USE_DEBUG_UART != 0u)
    if(g_motionEnabled || g_home.phase!=HOME_IDLE || !g_lastStop[0]) {
        snprintf(g_lastStop,sizeof(g_lastStop),"LAST_STOP id=%lu %s",(unsigned long)++g_lastStopId,g_dbg+15);
    }
#endif
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
#if (HW_STEP != 0u)
    step_fire((a == &g_arm1) ? STEP_ARM1 : STEP_ARM2);
#else
    pulse_step_pin(a->pinStep, a->stepIdleHigh);
#endif
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
/* The two angle<->count conversions below sit under every pulse and every pass of the trajectory loop.  Inside these
 * bounds the products fit 32 bits, so they use the hardware divider instead of a libgcc 64-bit divide (several us at
 * 24 MHz); outside them the original 64-bit arithmetic runs, so the results never differ. */
#define PTP_CD_FAST    (21474)  /* |cd| * 100000 + 18000 < 2^31 */
#define PTP_STEPS_FAST (58000)  /* |steps| * 36000 + 50000 < 2^31 */
_Static_assert(MOTOR_FULLSTEPS_REV * USTEP_DIV * GEAR_RATIO <= 100000 &&
               ARM2_MOTOR_FULLSTEPS_REV * USTEP_DIV * ARM2_GEAR_RATIO_X100 / 100 <= 100000,
               "ptp_offset/ptp_angle_from_counts 32-bit fast path assumes at most 100000 counts per revolution");
static int32 ptp_offset(uint8 axis, int32 cd)
{
    int32 perRev = (axis == 0u) ? (int32)(MOTOR_FULLSTEPS_REV * USTEP_DIV * GEAR_RATIO) :
        (int32)(ARM2_MOTOR_FULLSTEPS_REV * USTEP_DIV * ARM2_GEAR_RATIO_X100 / 100);
    int64 n;
    if (cd >= -PTP_CD_FAST && cd <= PTP_CD_FAST) {
        int32 m = cd * perRev;
        return (m >= 0) ? (m + 18000) / 36000 : -((-m + 18000) / 36000);
    }
    n = (int64)cd * perRev;
    return (int32)((n >= 0) ? (n + 18000) / 36000 : -((-n + 18000) / 36000));
}
static int32 ptp_axis_angle_cd(uint8 axis, int32 q1, int32 q2)
{
    return axis == 0u ? q1 : q2 - q1; /* q2 is link 2 relative to fixed +X; motor 2 is elbow-relative */
}
static int32 traj_joint_steps(uint8 axis, int16 q1, int16 q2)
{
    return ptp_offset(axis, ptp_axis_angle_cd(axis, q1, q2));
}
static int32 ptp_angle_from_counts(uint8 axis, int32 count)
{
    int32 steps=g_ptp.origin[axis]-count;
    int32 revolution=axis==0u ? MOTOR_FULLSTEPS_REV*USTEP_DIV*GEAR_RATIO :
        ARM2_MOTOR_FULLSTEPS_REV*USTEP_DIV*ARM2_GEAR_RATIO_X100/100;
    int64 scaled;
    if(steps>=-PTP_STEPS_FAST && steps<=PTP_STEPS_FAST) {
        int32 s32=steps*36000;
        return s32>=0 ? (s32+revolution/2)/revolution : -((-s32+revolution/2)/revolution);
    }
    scaled=(int64)steps*36000;
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
_Static_assert(ENCODER_CPR * GEAR_RATIO * 225 == 36000 * 1024, "joint_step_allowed: 225/1024 must equal 36000/(ENCODER_CPR*GEAR_RATIO)");
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
        /* 36000 / (ENCODER_CPR * GEAR_RATIO) = 225 / 1024: same truncated quotient, but a shift instead of a 64-bit divide. */
        int32 m1=(int32)(((int64)g_enc.pos-g_enc.softCenter)*(g_enc.incDir==ARM1_LEFT_DIR_LEVEL?1:-1)*225/1024);
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
    busy=(uint8)(g_home.phase!=HOME_IDLE || !axes_idle() || g_ptp.active || g_traj.state==TRAJ_UPLOADING || g_traj.state==TRAJ_RUNNING || g_uploadRxQuarantine);
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

static uint8 traj_accept(const uint8 *p)
{
    uint16 value=ptp_u16(p+13);
    int16 q[2];
    q[0]=(int16)ptp_u16(p+9); q[1]=(int16)ptp_u16(p+11);
    if (p[0]==2u) {
        if(value<2u || value>TRAJ_CAPACITY) { JLOG("PTP refused: trajectory capacity\r\n"); return 0u; }
        if(g_arm1.usteps!=g_ptp.origin[0]-ptp_offset(0u,q[0]) || g_arm2.usteps!=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,q[0],q[1]))) {
            JLOG("PTP refused: move to trajectory start first\r\n"); return 0u;
        }
        traj_begin(&g_traj,q[0],q[1],value);
    } else if(p[0]==3u) {
        traj_limits_t lim; uint8 i;
        for(i=0u;i<2u;i++) {
            lim.rate[i]=motion_min(TRAJ_RATE_MAX,g_motionRate[i]);
            lim.accel[i]=motion_min(TRAJ_ACCEL_MAX,g_motionAccel[i]);
        }
        switch(traj_push(&g_traj,&lim,(uint8)(ptp_u32(p+1)==g_ptp.sequence+1u),q[0],q[1],value)) {
        case TRAJ_PUSH_ORDER: JLOG("PTP refused: trajectory order/time\r\n"); return 0u;
        case TRAJ_PUSH_RATE: JLOG("PTP refused: trajectory rate/velocity change\r\n"); return 0u;
        default: break;
        }
    } else {
        /* TRAJ_DONE = the held plan already ran once; the nodes are intact, so a fresh RUN (new sequence) repeats it */
        if(p[0]!=4u || q[0]!=0 || q[1]!=0 || value!=0u || !traj_runnable(&g_traj) ||
           ptp_u32(p+1)!=g_ptp.sequence+1u ||
           g_arm1.usteps!=g_ptp.origin[0]-ptp_offset(0u,g_traj.nodes[0].q1) ||
           g_arm2.usteps!=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,g_traj.nodes[0].q1,g_traj.nodes[0].q2))) {
            JLOG("PTP refused: trajectory incomplete/start position\r\n"); return 0u;
        }
        {
            int32 pos[2];
            pos[0]=g_arm1.usteps; pos[1]=g_arm2.usteps;
            traj_start(&g_traj,&g_ptp.seg,g_ptp.origin,pos,SU_NOW());
        }
        g_ptp.sinceMs=g_jogMs; g_ptp.active=1u;
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
        g_home.phase != HOME_IDLE || g_ptp.active || (p[0]==1u && (g_traj.state==TRAJ_UPLOADING || g_traj.state==TRAJ_READY)) ||
        !host_input_idle() || !axes_idle() || !motion_inputs_clear() || !home_inputs_clear()) {
        JLOG("PTP refused: not homed/idle/healthy\r\n"); return;
    }
    if(p[0]==5u) {
        if(q1!=0 || q2!=0 || rate!=0u || g_traj.state==TRAJ_UPLOADING || g_traj.state==TRAJ_READY) {
            JLOG("PTP refused: manual reference payload/busy\r\n"); return;
        }
        if(!enc_sample()) { stage_stop("manual reference encoder invalid",0u); return; }
        jog_stop_now(&g_arm1); jog_stop_now(&g_arm2);
        ptp_invalidate(); /* discard old targets and increment reference epoch */
        g_encIncDir=MANUAL_ENC_INC_DIR; g_directionKnown=1u;
        if(ARM2_ENCODER_FEEDBACK && !g_enc2.directionKnown) { g_enc2.incDir=MANUAL_ENC2_INC_DIR; g_enc2.directionKnown=1u; JLOG("MANUAL REF: arm2 encoder polarity from bench calibration (not learned by homing)\r\n"); }
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
        g_ptp.sequence=seq; memcpy(g_ptp.accepted,p,15u); traj_status();
#if (HW_EVT != 0u)
        if(p[0]==4u) { evt_start(); }
#endif
        return;
    }
    if(g_traj.state!=TRAJ_DONE) { traj_reset(&g_traj); } /* a finished plan survives the move back to its start */
    {
        int32 to[2], from[2];
        to[0]=g_ptp.origin[0]-ptp_offset(0u,q1);
        to[1]=g_ptp.origin[1]-ptp_offset(1u,ptp_axis_angle_cd(1u,q1,q2));
        from[0]=axes[0]->usteps; from[1]=axes[1]->usteps;
        dda_start(&g_ptp.seg,to,from);
    }
    g_ptp.sequence=seq; memcpy(g_ptp.accepted,p,15u);
    g_ptp.sinceMs=g_jogMs; g_ptp.lastStep=SU_NOW();
    {
        uint32 accel=PTP_ACCEL;
        for(i=0u;i<2u;i++) if(g_ptp.seg.distance[i]) {
            rate=(uint16)motion_min(rate,g_motionRate[i]);
            accel=motion_min(accel,g_motionAccel[i]);
        }
        ramp_init(&g_ptp.ramp,(int32)g_ptp.seg.total,rate,accel);
        /* Configured low accelerations need a longer first-step interval than
         * the legacy 60 ms floor; stays below one SysTick wrap. */
        g_ptp.ramp.c0=motion_min(600000u,956000u/isqrt32(accel));
        if(g_ptp.ramp.c0<g_ptp.ramp.cMin)g_ptp.ramp.c0=g_ptp.ramp.cMin;
        g_ptp.ramp.c=g_ptp.ramp.c0;
    }
    g_ptp.periodUs=g_ptp.ramp.c; g_ptp.active=1u;
    JLOGF("PTP accepted seq=%lu target1=%ld target2=%ld rate=%u\r\n",(unsigned long)seq,(long)g_ptp.seg.target[0],(long)g_ptp.seg.target[1],(unsigned)rate);
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
static uint8 pulse_axes_try(const int8 step[2])   /* false when a step was refused; stops nothing (interrupt safe) */
{
    uint8 i;
    jog_axis_t *axes[2]; axes[0]=&g_arm1; axes[1]=&g_arm2;
    for(i=0u;i<2u;i++) {
        if(step[i]) {
            int32 before=axes[i]->usteps;
            jog_step(axes[i],step[i]);
            if(axes[i]->usteps!=before+step[i]) { return 0u; }
        }
    }
    return 1u;
}
static uint8 pulse_axes(const int8 step[2], const char *failure)
{
    if(!pulse_axes_try(step)) { stage_stop(failure,FAULT_ACTIVE()); return 0u; }
    return 1u;
}
static uint8 coordinated_step(const char *failure)
{
    int8 step[2];
    dda_step(&g_ptp.seg,step);
    return pulse_axes(step,failure);
}
static void traj_service(void)
{
    int8 step[2];
    switch(traj_poll(&g_traj,&g_ptp.seg,g_ptp.origin,SU_NOW(),step)) {
    case TRAJ_POLL_OVERRUN: stage_stop("trajectory timing overrun",0u); break;
    case TRAJ_POLL_UNSUSTAINED: stage_stop("trajectory rate not sustained",0u); break;
    case TRAJ_POLL_STEP: (void)pulse_axes(step,"trajectory pulse blocked"); break;
    case TRAJ_POLL_DONE: g_ptp.active=2u; g_ptp.settleMs=g_jogMs; break;
    default: break;
    }
}
#if (HW_EVT != 0u)
/* EvtTimer is a fixed-function one-shot restarted from EvtRestart (a PulseMode Control_Reg on its 1 MHz clock): each
 * event writes the next delay into Period and pulses the restart, which reloads the counter from Period.  The restart
 * costs EVT_RESTART_LAG_US between the write and the first count (two timer clocks plus the register sync); it is
 * taken off every delay and is the one number to calibrate on the bench.  The interrupt does the plan's steps and
 * nothing else: stops are only requested (g_evtStop) and logged by the foreground, which also keeps all supervision. */
#define EVT_RESTART_LAG_US          (3u)
#define EVT_MIN_US                  (EVT_RESTART_LAG_US + 2u)
#define EVT_MAX_US                  (60000u)    /* the 16-bit timer at 1 MHz; longer waits are re-armed on the way */
#define TRAJ_FOREGROUND_MAX_MS      (40u)       /* no foreground pass for this long: the plan is stopped */
static volatile uint8 g_evtRunning;             /* the timer owns the trajectory's pulses */
static const char * volatile g_evtStop;         /* why the interrupt ended a run (static text) */
static uint32 g_fgPassAt;                       /* SU_NOW() at the start of the latest foreground pass */
static uint32 g_evtMaxCy, g_evtBusyCy, g_evtEvents, g_evtRunMs;

static void evt_arm_cy(uint32 cy)
{
    uint32 us = (cy + SU_CY_PER_US - 1u) / SU_CY_PER_US;
    if (us < EVT_MIN_US) { us = EVT_MIN_US; }
    if (us > EVT_MAX_US) { us = EVT_MAX_US; }
    EvtTimer_WritePeriod((uint16)(us - EVT_RESTART_LAG_US - 1u));
    EvtRestart_Write(1u);
}
static void evt_fail(const char *reason) { g_evtRunning = 0u; g_evtStop = reason; }
CY_ISR(evt_isr)
{
    uint32 started = SU_NOW(), cost;
    int8 step[2];
    (void)EvtTimer_ReadStatusRegister();        /* clears the interrupt */
    if (g_evtRunning == 0u) { return; }
    if (SU_ELAPSED(g_fgPassAt) > TRAJ_FOREGROUND_MAX_MS * 1000u * SU_CY_PER_US) { evt_fail("trajectory foreground stalled"); return; }
    switch (traj_poll(&g_traj, &g_ptp.seg, g_ptp.origin, started, step)) {
    case TRAJ_POLL_OVERRUN:     evt_fail("trajectory timing overrun"); return;
    case TRAJ_POLL_UNSUSTAINED: evt_fail("trajectory rate not sustained"); return;
    case TRAJ_POLL_STEP:        if (pulse_axes_try(step) == 0u) { evt_fail("trajectory pulse blocked"); return; } break;
    case TRAJ_POLL_DONE:        g_ptp.active = 2u; g_ptp.settleMs = g_jogMs; g_evtRunning = 0u; return;
    default: break;
    }
    evt_arm_cy(traj_wait_cy(&g_traj, &g_ptp.seg, SU_NOW()));
    cost = SU_ELAPSED(started);
    if (cost > g_evtMaxCy) { g_evtMaxCy = cost; }
    g_evtBusyCy += cost; g_evtEvents++;
}
static void evt_init(void)
{
    EvtIsr_StartEx(evt_isr);
    EvtIsr_SetPriority(1u);
    EvtIsr_Disable();
    EvtTimer_Start();
}
static void evt_start(void)    /* a trajectory RUN was accepted: step it from the timer */
{
    uint8 ints = CyEnterCriticalSection();
    g_evtStop = 0; g_fgPassAt = SU_NOW();
    g_evtMaxCy = 0u; g_evtBusyCy = 0u; g_evtEvents = 0u; g_evtRunMs = g_jogMs;
    (void)EvtTimer_ReadStatusRegister();
    EvtIsr_ClearPending();
    EvtIsr_Enable();
    g_evtRunning = 1u;
    evt_arm_cy(0u);
    CyExitCriticalSection(ints);
}
static void evt_cancel(void)
{
    uint8 ints = CyEnterCriticalSection();
    g_evtRunning = 0u;
    EvtIsr_Disable();
    EvtIsr_ClearPending();
    (void)EvtTimer_ReadStatusRegister();
    CyExitCriticalSection(ints);
}
#define FG_PASS_MARK()              (g_fgPassAt = SU_NOW())
#else
#define FG_PASS_MARK()              ((void)0)
#endif

static void ptp_service(void)
{
    if (!g_ptp.active) { return; }
#if (HW_EVT != 0u)
    if (g_evtStop != 0) { const char *why = g_evtStop; g_evtStop = 0; stage_stop(why, FAULT_ACTIVE()); return; }
#endif
    if (!g_motionEnabled || !g_ptp.reference || !g_keysSeen || g_jogMs-g_lastKeyMs>JOG_DEADMAN_MS) { stage_stop("PTP heartbeat/reference lost",0u); return; }
    if (FAULT_ACTIVE()) { stage_stop("PTP nFAULT",1u); return; }
    if (HARD_LIMIT_ACTIVE() || ARM1_HOME_ACTIVE() || ARM2_NPN_ACTIVE()) { stage_stop("PTP limit/home sensor",0u); return; }
    if (g_jogMs-g_ptp.sinceMs>600000u || !ptp_counts_in_range(0u,g_arm1.usteps) || !ptp_counts_in_range(1u,g_arm2.usteps)) { stage_stop("PTP time/joint limit",0u); return; }
    if (!g_arm1.hasEnc || g_enc.badStreak || (ARM2_ENCODER_FEEDBACK && (!g_arm2.hasEnc || g_enc2.badStreak))) { return; } /* shared encoder watchdog will latch stop */
    if ((abs_i32(cmd_lsb()-g_enc.pos)>=SLIP_FAIL_LSB || (ARM2_ENCODER_FEEDBACK && abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)>=SLIP_FAIL_LSB))) { stage_stop("PTP encoder following error",0u); return; }
    if (g_ptp.active==2u) {
        if (g_jogMs-g_ptp.settleMs<HOME_SETTLE_MS) { return; }
        if (!enc_sample() || abs_i32(cmd_lsb()-g_enc.pos)>PTP_FINAL_ERROR_LSB || (ARM2_ENCODER_FEEDBACK && abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)>PTP_FINAL_ERROR_LSB) ||
            g_arm1.usteps!=g_ptp.seg.target[0] || g_arm2.usteps!=g_ptp.seg.target[1]) { stage_stop("PTP final position mismatch",0u); return; }
        g_ptp.active=0u;
        if(g_traj.state==TRAJ_RUNNING) { g_traj.state=TRAJ_DONE; }
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
    if(g_traj.state==TRAJ_RUNNING) {
#if (HW_EVT != 0u)
        if(g_evtRunning) { return; }   /* the event timer steps the plan */
#endif
        traj_service(); return;
    }
    if (g_ptp.seg.done>=g_ptp.seg.total) { g_ptp.active=2u; g_ptp.settleMs=g_jogMs; return; }
    if (SU_ELAPSED(g_ptp.lastStep)<g_ptp.periodUs*SU_CY_PER_US) { return; }
    g_ptp.lastStep=SU_NOW(); /* no catch-up bursts after a late service */
    if(!coordinated_step("PTP pulse blocked")) { return; }
    ramp_advance(&g_ptp.ramp,(int32)g_ptp.seg.done); g_ptp.periodUs=g_ptp.ramp.c;
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
        JLOGF("HOME %s averaged origin: samples=%u raw=%u lsb (motor single-turn)\r\n",
              a->name, (unsigned)g_home.zeroSamples, (unsigned)mean);
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

static void ptp_state_line(void)
{
    JLOGF("PTP_STATE lock=%u ref=%u active=%u epoch=%lu seq=%lu min1=%d max1=%d min2=%d max2=%d emin=%d emax=%d\r\n",
          (unsigned)!PTP_COMMISSIONED,(unsigned)g_ptp.reference,(unsigned)g_ptp.active,
          (unsigned long)g_ptp.epoch,(unsigned long)g_ptp.sequence,
          (int)g_jointMinCd[0],(int)g_jointMaxCd[0],(int)g_jointMinCd[1],(int)g_jointMaxCd[1],
          (int)g_jointMinCd[JOINT_ELBOW],(int)g_jointMaxCd[JOINT_ELBOW]);
}

static void motion_rates_line(void)
{
    JLOGF("MOTION_RATES epoch=%lu r1=%u r2=%u a1=%u a2=%u\r\n",(unsigned long)g_ptp.epoch,
          (unsigned)g_motionRate[0],(unsigned)g_motionRate[1],(unsigned)g_motionAccel[0],(unsigned)g_motionAccel[1]);
}

static void motion_config_status(void)
{
    ptp_state_line(); motion_rates_line();
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

/* Format only one status line per foreground slice during motion. A queued UART
 * does not make the preceding snprintf burst nonblocking. Keep safety deadlines. */
static uint8 g_statusNext;
static uint32 g_statusAt, g_statusMaxCy, g_statusEpoch;
static uint8 status_moving(void)
{
    return g_motionEnabled && (g_ptp.active || g_home.phase!=HOME_IDLE || g_arm1.moving || g_arm2.moving);
}
#if (HW_SSI != 0u || HW_STEP != 0u || HW_EVT != 0u || HW_DEAD != 0u || HW_PINIRQ != 0u || HW_TXDMA != 0u)
#define STATUS_LINES                (13u)    /* one more line when any hardware offload is on */
#else
#define STATUS_LINES                (12u)
#endif
static void stage_status_line(void)
{
    uint32 start=SU_NOW(), cost;
    switch(g_statusNext++) {
    case 1u:
    JLOGF("STATE fw=stage2-v11 enabled=%u home=%u homed1=%u homed2=%u fault=%u enc=%u dir=%u enc2=%u dir2=%u arm2fb=%u\r\n",
          (unsigned)g_motionEnabled, (unsigned)g_home.phase,
          (unsigned)g_homed1, (unsigned)g_homed2, (unsigned)g_jogFaulted,
          (unsigned)(g_arm1.hasEnc && (g_enc.badStreak == 0u)), (unsigned)g_directionKnown, (unsigned)(g_arm2.hasEnc && !g_enc2.badStreak), (unsigned)g_enc2.directionKnown, (unsigned)ARM2_ENCODER_FEEDBACK); break;
    case 2u:
    JLOGF("CONTROL seq=%lu\r\n",(unsigned long)g_controlSequence); break;
    case 3u:
    ptp_state_line(); break;
    case 4u:
    motion_rates_line(); break;
    case 5u:
    JLOGF("PTP_CHECK final_lsb=%d follow_lsb=%d\r\n",PTP_FINAL_ERROR_LSB,SLIP_FAIL_LSB); break;
    case 6u:
    traj_status(); break;
    case 7u:
    JLOGF("LINK uart=hw baud=115200 rx_overflow=%lu rx_error=%lu tx_dropped=%u fw=%s short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 dual_encoder=%u traj_repeat=1 fault_snapshot=1 telem_hz=%u proto=%u\r\n",
          (unsigned long)g_rxLate, (unsigned long)g_rxFraming, (unsigned)g_txqDropped, FW_BUILD_ID, (unsigned)ARM2_ENCODER_FEEDBACK, (unsigned)g_telemHz, (unsigned)PROTO_VERSION); break;
    case 8u:
    JLOGF("PTP_RX short_ok=%lu drop_crc=%lu drop_short=%lu drop_timeout=%lu rejected=%lu epoch_rejected=%lu last_rejected_byte=0x%02X\r\n",
          (unsigned long)g_shortRxOk, (unsigned long)g_ptpRxCrc, (unsigned long)g_ptpRxShort, (unsigned long)g_ptpRxTimeout,
          (unsigned long)g_rxRejected, (unsigned long)g_ptpEpochRejected,
          (unsigned)g_rxLastBad[(g_rxLastBadIdx + 7u) % 8u]); break;
    case 9u:
    JLOGF("TIMING loop_max_ms=%lu tick_lag_max_ms=%lu clock=hardware wdt=256-384ms key_age_ms=%lu key_max_gap_ms=%lu status_max_us=%lu traj_lag_max_us=%lu traj_slip_us=%lu\r\n",
          (unsigned long)g_loopMaxMs,(unsigned long)g_tickMaxLagMs,
          (unsigned long)(g_keysSeen ? (g_jogMs - g_lastKeyMs) : 0u),(unsigned long)g_keyMaxGapMs,(unsigned long)(g_statusMaxCy/SU_CY_PER_US),
          (unsigned long)(g_traj.lagMaxCy/SU_CY_PER_US),(unsigned long)(g_traj.slipCy/SU_CY_PER_US)); break;
    case 10u:
    JLOGF("ENC_DIAG irq_max_us=%lu retries=%lu rejected=%lu gap_max_ms=%lu follow_peak_lsb=%lu\r\n",
          (unsigned long)(g_encIrqMaxCy/SU_CY_PER_US),(unsigned long)g_encRetries,
          (unsigned long)g_encRejected,(unsigned long)g_encMaxGapMs,(unsigned long)g_followPeakLsb); break;
    case 11u:
    JLOGF("TELEM_DIAG hz=%u sent=%lu skipped=%lu max_us=%lu\r\n",
          (unsigned)g_telemHz,(unsigned long)g_telemSent,(unsigned long)g_telemSkipped,(unsigned long)(g_telemMaxCy/SU_CY_PER_US)); break;
    case 12u:
    telem_line(); break;
#if (HW_SSI != 0u || HW_STEP != 0u || HW_EVT != 0u || HW_DEAD != 0u || HW_PINIRQ != 0u || HW_TXDMA != 0u)
    case 13u:
    {
#if (HW_EVT != 0u)
        unsigned long isrMaxUs=g_evtMaxCy/SU_CY_PER_US, events=g_evtEvents;
        unsigned long busyPermille=(g_jogMs>g_evtRunMs) ? (g_evtBusyCy/SU_CY_PER_US)/(g_jogMs-g_evtRunMs) : 0ul;   /* us per ms = permille */
#else
        unsigned long isrMaxUs=0ul, events=0ul, busyPermille=0ul;
#endif
        JLOGF("HWOFFLOAD bus_mhz=%u ssi=%u step=%u evt=%u dead=%u pinirq=%u txdma=%u evt_isr_max_us=%lu evt_isr_busy_permille=%lu evt_events=%lu\r\n",
              (unsigned)(BCLK__BUS_CLK__HZ/1000000u),(unsigned)HW_SSI,(unsigned)HW_STEP,(unsigned)HW_EVT,(unsigned)HW_DEAD,
              (unsigned)HW_PINIRQ,(unsigned)HW_TXDMA,
              isrMaxUs,busyPermille,events);
    } break;
#endif
    }
    if(g_statusNext>STATUS_LINES) g_statusNext=0u;
    cost=SU_ELAPSED(start);if(cost>g_statusMaxCy)g_statusMaxCy=cost;
}
static void stage_status_service(void)
{
    if(!g_statusNext) return;
    if(status_moving() && (SU_ELAPSED(g_statusAt)<5000u*SU_CY_PER_US || !jog_slot_free(50u))) return;
    g_statusAt=SU_NOW();stage_status_line();
}
static void stage_status(void)
{
    if(status_moving()) {
        /* A new reference epoch must restart at STATE/PTP_STATE, even in a partial block. */
        if(!g_statusNext || g_statusEpoch!=g_ptp.epoch)g_statusNext=1u;
        g_statusEpoch=g_ptp.epoch;return;
    }
    /* Stops publish the retained cause first, and supersede any partial running block. */
    if(!g_motionEnabled && g_lastStop[0]) JLOG(g_lastStop);
    g_statusEpoch=g_ptp.epoch;g_statusNext=1u;while(g_statusNext)stage_status_line();
}

static void stage_enable(void)
{
    if (g_motionEnabled != 0u) { JLOG("ENABLE: already enabled\r\n"); return; }
    ptp_invalidate();
    deadman_arm();   /* an operator enable is what clears a latched hardware kill */
    step_unkill();
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
    g_traj.lagMaxCy = 0u;
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
        uint32 period, since;
        if(g_traj.state==TRAJ_RUNNING) { traj_next_due(&g_traj,&g_ptp.seg,SU_NOW(),&since,&period); }
        else { period=g_ptp.periodUs*SU_CY_PER_US; since=SU_ELAPSED(g_ptp.lastStep); }
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
    /* Without arm2 feedback g_enc2 never learns a direction; arm2's calibrated zero then hangs on the same
     * arm1 polarity that joint_step_allowed() itself checks. */
    if (((a==&g_arm2 && !ARM2_ENCODER_FEEDBACK) ? g_directionKnown : e->directionKnown) && (a==&g_arm1 ? g_homed1 : g_homed2) && want!=0)
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
            if(b==-3 && (g_traj.state==TRAJ_UPLOADING || g_traj.state==TRAJ_READY) && !g_ptp.active &&
               g_home.phase==HOME_IDLE && g_motionEnabled && g_ptp.reference &&
               g_homed1 && g_homed2 && dual_ready() && !g_jogFaulted && !g_limitStop &&
               host_input_idle() && axes_idle() && !g_arm1.holding && !g_arm2.holding &&
               abs_i32(cmd_lsb()-g_enc.pos)<SLIP_FAIL_LSB && (!ARM2_ENCODER_FEEDBACK || abs_i32(g_enc2.cmdLsbX100/100-g_enc2.pos)<SLIP_FAIL_LSB) &&
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
#if (HW_PINIRQ != 0u)
    if (g_pinStop != 0u)
    {
        g_pinStop = 0u;
        /* A pin that is still active is handled, with its own reason, by jog_safety_blocked() below; one that has
         * already gone again killed the PWMs all the same and stops the stage here. */
        if ((g_motionEnabled != 0u) && !FAULT_ACTIVE() && !HARD_LIMIT_ACTIVE())
        { stage_stop("nFAULT or P0[4] pulse (pin interrupt); PWMs killed", 0u); return; }
    }
#endif
    if (deadman_tripped() != 0u) { stage_stop("hardware deadman: foreground stalled", 0u); return; }
    if (jog_safety_blocked() != 0u) { return; }
    if (g_home.phase != HOME_IDLE) { home_service(); }
    else if (g_ptp.active) { ptp_service(); }
    else { jog_axis_service(&g_arm1); jog_axis_service(&g_arm2); }
}

static void encoder_after_sample(uint8 sampleOk)
{
    if (sampleOk == 0u)
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
static void service_encoder(void) { encoder_after_sample(enc_sample()); }

/* Foreground encoder sampling, about once per JOG_ENC_SAMPLE_US.  Hardware read-out: never blocks, so a frame starts
 * whenever one is due and is picked up on a later pass.  Bit-banged read-out: the frame is read in a gap between
 * steps (jog_slot_free), or anyway after 5 sample periods. */
static void service_encoder_sampling(void)
{
#if (HW_SSI != 0u)
    {
        uint8 sampleOk;
        if (g_encFlight != 0u)
        {
            if (enc_flight_poll(&sampleOk) != 0u) { encoder_after_sample(sampleOk); }
        }
        else if (((g_arm1.hasEnc != 0u) || (g_arm2.hasEnc != 0u)) && (SU_ELAPSED(g_loopEnc) >= JOG_ENC_SAMPLE_CY))
        {
            g_loopEnc = SU_NOW();
            g_encTry = 0u;
            enc_flight_start(0u);
        }
    }
#else
    if (((g_arm1.hasEnc != 0u) || (g_arm2.hasEnc != 0u)) && (SU_ELAPSED(g_loopEnc) >= JOG_ENC_SAMPLE_CY))
    {
        if ((jog_slot_free(JOG_ENC_READ_US) != 0u) ||
            (SU_ELAPSED(g_loopEnc) >= (5u * JOG_ENC_SAMPLE_CY)))
        {
            g_loopEnc = SU_NOW();
            service_encoder();
        }
    }
#endif
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
    JLOG("\r\nSTAGE2 v10 latched stop framed control: both drivers disabled, reference unset; use home or explicit manual straight reference\r\n");
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
    deadman_arm();
}

static void app_poll(void)
{
    uint32 loopNow;
    FG_PASS_MARK();
    alive_kick();
    loopNow = wall_ms();
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
    service_encoder_sampling();

    stage_status_service();

#if (HW_TXDMA != 0u)
    if (txq_pending() != 0u) { txq_send_one(); }       /* the DMA needs no gap between steps */
#else
    if ((txq_pending() != 0u) && (jog_slot_free(JOG_TX_BYTE_US) != 0u))
    {
        txq_send_one();
    }
#endif
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
