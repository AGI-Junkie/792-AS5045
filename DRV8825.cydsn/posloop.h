/* posloop.h - closed-loop motor-shaft position control (build option POS_LOOP, on by default).
 * Header-only, like traj.h: main.c includes it once, time and positions come in as arguments, steps go out as an
 * array, so the offline fixture drives it with made-up times and encoder lags.
 *
 * The open-loop executor (traj_poll) turns each node into a fixed number of DDA steps at fixed due times; the encoders
 * only check.  Here a motion is a *reference* instead - a function of time - and the encoders close the loop:
 *
 *   reference   trajectory RUN: both axes read one plan clock; inside node k the reference is the straight line from
 *               node k-1 to node k (piecewise-linear interpolation in motor steps, arm2 already converted from the
 *               elbow angle q2-q1 by traj_joint_steps), and its slope is the velocity feedforward.
 *               PTP move: one straight line in step space, walked with a trapezoidal time law (accelerate, cruise,
 *               decelerate) on the longer axis; the other axis follows in proportion.
 *   feedback    every POS_CTRL_US, per axis:  e = ref - measured,  measured = cmd - lag,
 *               where cmd is the step counter (known exactly, now) and lag = cmd - encoder (from the last encoder
 *               sample, published by the foreground).  The sensor's own output delay is taken out of lag.
 *   control     v = v_ff + clamp(Kp * e), with the correction slew-limited (its own acceleration cap), the total
 *               clamped to the rate cap, and no reversal against a moving reference; a deadband with hysteresis
 *               (start beyond it, correct down to 0) keeps encoder noise from chattering the motor.
 *   pulses      per axis a phase accumulator (NCO): acc += v * dt; a full step's worth emits one STEP in v's
 *               direction.  Each axis has its own accumulator, so arm1 can speed up to catch up while arm2 keeps
 *               following its own reference - the DDA's fixed step ratio is gone, the shared clock is not.
 *   end         after the motion the reference stays at the final pose (hold); DONE once both errors stay in the
 *               deadband for POS_SETTLE_MS, a stop if that never happens within POS_HOLD_MAX_MS.
 *
 * The host (main.c) must provide, before this header is included (after traj.h):
 *   POS_CTRL_US, POS_KP_PER_S, POS_CORR_MAX, POS_CORR_ACCEL, POS_RATE_MAX, POS_DEADBAND_USTEP,
 *   POS_TRACK_MAX_USTEP, POS_SETTLE_MS, POS_HOLD_MAX_MS, POS_SENSOR_DELAY_US, and isqrt32()
 * Every gain and window is a first guess for the bench (docs/POSITION_LOOP.md), not a tuned value.
 */
#ifndef POSLOOP_H
#define POSLOOP_H

#if !defined(POS_CTRL_US) || !defined(POS_KP_PER_S) || !defined(POS_CORR_MAX) || !defined(POS_CORR_ACCEL) || \
    !defined(POS_RATE_MAX) || !defined(POS_DEADBAND_USTEP) || !defined(POS_TRACK_MAX_USTEP) || \
    !defined(POS_SETTLE_MS) || !defined(POS_HOLD_MAX_MS) || !defined(POS_SENSOR_DELAY_US)
#error "define the POS_* loop parameters before including posloop.h"
#endif

#define PL_CY_PER_S   (SU_CY_PER_US * 1000000u)
#define PL_CTRL_CY    (POS_CTRL_US * SU_CY_PER_US)
/* One control period's worth of correction-speed change, steps/s. */
#define PL_CORR_STEP  ((int32)((uint32)POS_CORR_ACCEL * POS_CTRL_US / 1000000u))

/* The trajectory reference product (segment steps) x (microseconds into the node) must stay in 32 bits; traj_push()
 * caps a node at TRAJ_RATE_MAX steps/s for at most TRAJ_NODE_MAX_MS. */
_Static_assert((uint64_t)TRAJ_RATE_MAX * TRAJ_NODE_MAX_MS / 1000u * TRAJ_NODE_MAX_MS * 1000u < 0x7FFFFFFFu,
               "posloop reference interpolation overflows int32");
_Static_assert((uint64_t)POS_RATE_MAX * PL_CTRL_CY < 0x7FFFFFFFu, "posloop accumulator overflows int32");
_Static_assert(PL_CORR_STEP > 0, "POS_CORR_ACCEL too small for POS_CTRL_US");

typedef struct {
    int32 from[2], to[2];   /* the current straight line, steps */
    int32 vff[2];           /* velocity feedforward, steps/s */
    int32 vcorr[2];         /* P correction after the slew limit, steps/s */
    int32 vcmd[2];          /* rate being pulsed, steps/s */
    int32 acc[2];           /* NCO phase, step * cycles/s units; one step = PL_CY_PER_S */
    int32 err[2];           /* last ref - measured, steps */
    int32 errPeak[2];       /* largest |err| this motion, steps (reported) */
    uint32 clock;           /* SysTick count at the last poll */
    uint32 ctrlCy, holdCy, calmCy;
    uint8 holding, ptp;
    uint8 fixing[2];        /* hysteresis: correcting since |err| left the deadband, until err reaches 0 */
    int32 rateMax[2];       /* per-axis cap on the pulsed rate, feedforward plus correction: the configured maximum */
    int32 lagBase[2];       /* lag (cmd - encoder) when tracking was last healthy: the steps already won back */
    /* PTP time law on the longer axis: distance (steps), top rate (steps/s), acceleration (steps/s^2), ramp and total
     * time (us); elapsed counts cycles in 64 bits because a slow long move outlasts the 89 s a uint32 holds at 48 MHz. */
    int32 dist; uint32 rate, accel, rampUs, totalUs;
    uint64_t elapsedCy, totalCy;
} pl_t;

static int32 pl_clamp(int32 v, int32 lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static void pl_hold(pl_t *pl)
{
    pl->holding = 1u;
    pl->from[0] = pl->to[0]; pl->from[1] = pl->to[1]; pl->vff[0] = pl->vff[1] = 0;
}

/* Aim at node t->index, starting from the previous node's pose (pl->to). */
static void pl_segment(const traj_t *t, pl_t *pl, const int32 origin[2])
{
    const traj_node_t *n = &t->nodes[t->index];
    uint8 i;
    for (i = 0u; i < 2u; i++) {
        pl->from[i] = pl->to[i];
        pl->to[i] = origin[i] - traj_joint_steps(i, n->q1, n->q2);
        pl->vff[i] = (pl->to[i] - pl->from[i]) * 1000 / (int32)n->ms;
    }
}

/* Keep each axis' cap within what the accumulator is sized for. */
static void pl_caps(pl_t *pl, const uint32 rateMax[2])
{
    pl->rateMax[0] = (int32)(rateMax[0] < (uint32)POS_RATE_MAX ? rateMax[0] : (uint32)POS_RATE_MAX);
    pl->rateMax[1] = (int32)(rateMax[1] < (uint32)POS_RATE_MAX ? rateMax[1] : (uint32)POS_RATE_MAX);
}

/* Trajectory RUN: start at node 1; `pos` is where the machine stands (node 0), `now` the SysTick count, rateMax the
 * per-axis cap (steps/s) the correction may not push past. */
static void pl_start(traj_t *t, pl_t *pl, const int32 origin[2], const int32 pos[2], const uint32 rateMax[2], uint32 now)
{
    memset(pl, 0, sizeof(*pl));
    pl_caps(pl, rateMax);
    pl->to[0] = pos[0]; pl->to[1] = pos[1]; pl->clock = now;
    t->state = TRAJ_RUNNING; t->index = 1u; t->elapsedCy = 0u; t->clock = now; t->lastStep = now; t->slipCy = 0u;
    pl_segment(t, pl, origin);
}

/* PTP move from `from` to `to` (steps), at most `rate` steps/s and `accel` steps/s^2 on the longer axis.
 * Trapezoid: T = D/R + R/A; when D < R^2/A there is no cruise and the top rate drops to sqrt(A*D) (the same formula
 * then gives the triangle's time). */
static void pl_ptp_start(pl_t *pl, const int32 from[2], const int32 to[2], uint32 rate, uint32 accel,
                         const uint32 rateMax[2], uint32 now)
{
    uint8 i;
    memset(pl, 0, sizeof(*pl));
    pl_caps(pl, rateMax);
    pl->ptp = 1u; pl->clock = now;
    for (i = 0u; i < 2u; i++) {
        pl->from[i] = from[i]; pl->to[i] = to[i];
        if (abs_i32(to[i] - from[i]) > pl->dist) { pl->dist = abs_i32(to[i] - from[i]); }
    }
    if (pl->dist == 0 || rate == 0u || accel == 0u) { pl_hold(pl); return; }
    if ((uint64_t)rate * rate > (uint64_t)accel * (uint32)pl->dist) { rate = isqrt32(accel * (uint32)pl->dist); }
    if (rate == 0u) { rate = 1u; }
    pl->rate = rate; pl->accel = accel;
    pl->rampUs = (uint32)((uint64_t)rate * 1000000u / accel);
    pl->totalUs = (uint32)((uint64_t)(uint32)pl->dist * 1000000u / rate) + pl->rampUs;
    pl->totalCy = (uint64_t)pl->totalUs * SU_CY_PER_US;
}

/* Where the PTP time law is at `us` into the move: s steps along the longer axis, v steps/s. */
static void pl_ptp_at(const pl_t *pl, uint32 us, int64_t *s, int32 *v)
{
    int64_t a = pl->accel, r = pl->rate, ta = pl->rampUs, t = us, left = (int64_t)pl->totalUs - us;
    if (t >= (int64_t)pl->totalUs) { *s = pl->dist; *v = 0; }
    else if (t < ta && t <= left) { *s = a * t * t / 2000000000000LL; *v = (int32)(a * t / 1000000); }
    else if (left > ta) { *s = r * r / (2 * a) + r * (t - ta) / 1000000; *v = (int32)r; }
    else { *s = pl->dist - a * left * left / 2000000000000LL; *v = (int32)(a * left / 1000000); }
    if (*s > pl->dist) { *s = pl->dist; }
}

/* One control update of axis i from its error e (steps).  The deadband has hysteresis: an error only starts a
 * correction once it leaves the band, and the correction then runs until the error is 0.  A plain band would stop
 * every correction at its edge, so a small steady lag (lost interrupt time, rounding) would end the move up to a
 * band's width short. */
static void pl_control(pl_t *pl, uint8 i, int32 e)
{
    int32 want, v;
    if (abs_i32(e) > POS_DEADBAND_USTEP) { pl->fixing[i] = 1u; }
    else if (e == 0) { pl->fixing[i] = 0u; }
    want = pl->fixing[i] ? pl_clamp(e * POS_KP_PER_S, POS_CORR_MAX) : 0;
    pl->vcorr[i] += pl_clamp(want - pl->vcorr[i], PL_CORR_STEP);
    v = pl_clamp(pl->vff[i] + pl->vcorr[i], pl->rateMax[i]);   /* the operator's per-axis maximum holds for corrections too */
    /* No backward step while the reference moves forward (and vice versa): wait for it instead of chattering the
     * direction pin on encoder noise or gearbox play. */
    if ((pl->vff[i] > 0 && v < 0) || (pl->vff[i] < 0 && v > 0)) { v = 0; }
    pl->vcmd[i] = v;
}

/* Advance the motion to `now` and pulse.  t is the trajectory (unused for a PTP move); cmd[i] is axis i's step counter,
 * lag[i] = cmd - encoder (steps, from the last sample).  Returns STEP (step[i] = 0 or the direction axis i pulses),
 * DONE (held in the deadband long enough), TRACKING (|ref - measured| beyond POS_TRACK_MAX_USTEP), UNSETTLED (hold
 * window expired), or IDLE. */
static traj_poll_t pl_poll(traj_t *t, pl_t *pl, const int32 origin[2], const int32 cmd[2], const int32 lag[2],
                           uint32 now, int8 step[2])
{
    uint32 dt = (pl->clock - now) & 0x00FFFFFFu;
    uint8 i, any = 0u;
    pl->clock = now;
    if (pl->holding) {
        pl->holdCy += dt;
    } else if (pl->ptp) {
        pl->elapsedCy += dt;
        if (pl->elapsedCy >= pl->totalCy) { pl_hold(pl); }
    } else {
        t->elapsedCy += dt; t->clock = now;
        while (t->elapsedCy >= (uint32)t->nodes[t->index].ms * 1000u * SU_CY_PER_US) {
            t->elapsedCy -= (uint32)t->nodes[t->index].ms * 1000u * SU_CY_PER_US;
            /* index == count, as the open-loop run ends; the hold reads no node */
            if (++t->index == t->count) { pl_hold(pl); t->elapsedCy = 0u; break; }
            pl_segment(t, pl, origin);
        }
    }
    pl->ctrlCy += dt;
    if (pl->ctrlCy >= PL_CTRL_CY) {
        uint8 calm = 1u;
        int64_t s = 0; int32 v = 0;
        uint32 us = 0u;
        pl->ctrlCy = (pl->ctrlCy >= 2u * PL_CTRL_CY) ? 0u : pl->ctrlCy - PL_CTRL_CY;  /* a late tick does not bunch up */
        if (!pl->holding) {
            if (pl->ptp) { pl_ptp_at(pl, (uint32)(pl->elapsedCy / SU_CY_PER_US), &s, &v); }
            else { us = t->elapsedCy / SU_CY_PER_US; }
        }
        for (i = 0u; i < 2u; i++) {
            int32 d = pl->to[i] - pl->from[i], ref, e;
            if (pl->holding) { ref = pl->to[i]; }
            else if (pl->ptp) { ref = pl->from[i] + (int32)(d * s / pl->dist); pl->vff[i] = (int32)((int64_t)d * v / pl->dist); }
            else { ref = pl->from[i] + d * (int32)us / ((int32)t->nodes[t->index].ms * 1000); }
            /* the encoder reports where the rotor was POS_SENSOR_DELAY_US ago: at speed v that overstates the lag */
            e = ref - cmd[i] + lag[i] - pl->vcmd[i] * (int32)POS_SENSOR_DELAY_US / 1000000;
            pl->err[i] = e;
            if (abs_i32(e) > pl->errPeak[i]) { pl->errPeak[i] = abs_i32(e); }
            if (abs_i32(e) > POS_TRACK_MAX_USTEP) { return TRAJ_POLL_TRACKING; }
            pl_control(pl, i, e);
            if (abs_i32(e) > POS_DEADBAND_USTEP || pl->fixing[i]) { calm = 0u; }   /* arrived = in the band, nothing still correcting */
            /* Recovered steps stay in cmd - encoder for good, so that alone says nothing; but lag that keeps growing while
             * tracking is unhealthy means steps are being lost faster than the loop wins them back (a stalled motor taking
             * the corrections too).  Healthy tracking moves the baseline, so every loss the loop made up is forgiven. */
            else { pl->lagBase[i] = lag[i]; }
            if (abs_i32(lag[i] - pl->lagBase[i]) > POS_TRACK_MAX_USTEP) { return TRAJ_POLL_FOLLOWING; }
        }
        if (pl->holding) {
            pl->calmCy = calm ? pl->calmCy + PL_CTRL_CY : 0u;
            if (pl->calmCy >= POS_SETTLE_MS * 1000u * SU_CY_PER_US) { return TRAJ_POLL_DONE; }
            if (pl->holdCy > POS_HOLD_MAX_MS * 1000u * SU_CY_PER_US) { return TRAJ_POLL_UNSETTLED; }
        }
    }
    for (i = 0u; i < 2u; i++) {
        int32 v = pl->vcmd[i], add;
        if ((v > 0 && pl->acc[i] < 0) || (v < 0 && pl->acc[i] > 0)) { pl->acc[i] = 0; }  /* fresh phase on reversal */
        /* A late interrupt keeps its time (up to two steps owed), so lateness does not quietly become lag the loop
         * must win back - with the rate at the operator's cap there is no headroom to win it back.  A longer stall owes
         * at most two steps, never a burst; v * dt is only formed where it cannot overflow. */
        add = (v == 0) ? 0 : (dt > 2u * PL_CY_PER_S / (uint32)abs_i32(v)) ? (v > 0 ? 2 : -2) * (int32)PL_CY_PER_S : v * (int32)dt;
        pl->acc[i] = pl_clamp(pl->acc[i] + add, 2 * (int32)PL_CY_PER_S);
        step[i] = 0;
        if (pl->acc[i] >= (int32)PL_CY_PER_S) { pl->acc[i] -= (int32)PL_CY_PER_S; step[i] = 1; any = 1u; }
        else if (pl->acc[i] <= -(int32)PL_CY_PER_S) { pl->acc[i] += (int32)PL_CY_PER_S; step[i] = -1; any = 1u; }
    }
    return any ? TRAJ_POLL_STEP : TRAJ_POLL_IDLE;
}

#endif /* POSLOOP_H */
