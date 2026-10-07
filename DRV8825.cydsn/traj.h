/* traj.h - the timed-trajectory executor's data and rules.  Header-only: main.c includes it once, so every function
 * is static and inlinable and the step path pays nothing for the split.
 *
 * What lives here: the numbered-node plan and its upload/validation rules, the two-axis DDA that turns one segment
 * into step pulses, and the due-time arithmetic.  What stays in main.c: the supervision (heartbeat, limits, encoder
 * following, final-position check), the pulses themselves, and the wire protocol.
 *
 * Time comes in as an argument (the SysTick count, 24 bits counting down), steps go out as an array: the executor
 * never reads a clock or touches a pin, so a test can drive it with made-up times.
 *
 * The host (main.c) must provide, before this header is included:
 *   TRAJ_CAPACITY, TRAJ_NODE_MIN_MS, TRAJ_NODE_MAX_MS, TRAJ_TOTAL_MAX_MS   (the host/firmware contract; the release
 *                                                                          gate reads them from main.c)
 *   TRAJ_STEP_MIN_US                                                       (never pulse faster than this, even when catching up)
 *   TRAJ_LATE_SOFT_US, TRAJ_SLIP_MAX_MS                                    (how late a step may be before the plan clock is held back,
 *                                                                          and how much total holding back a run may take)
 *   SU_CY_PER_US, abs_i32()
 * and, after it (a forward declaration here, the definition there):
 *   traj_joint_steps()                                                     (kinematics: joint angles -> steps)
 */
#ifndef TRAJ_H
#define TRAJ_H

#if !defined(TRAJ_CAPACITY) || !defined(TRAJ_NODE_MIN_MS) || !defined(TRAJ_NODE_MAX_MS) || !defined(TRAJ_TOTAL_MAX_MS) || \
    !defined(TRAJ_STEP_MIN_US) || !defined(TRAJ_LATE_SOFT_US) || !defined(TRAJ_SLIP_MAX_MS) || !defined(SU_CY_PER_US)
#error "define the TRAJ_* contract macros, TRAJ_STEP_MIN_US, TRAJ_LATE_SOFT_US, TRAJ_SLIP_MAX_MS and SU_CY_PER_US before including traj.h"
#endif

/* A node running this much longer than planned stops the run (the foreground has stalled). */
#define TRAJ_OVERRUN_US (20000u)

/* Steps of axis 0 (arm1) or 1 (arm2) that the pose (q1,q2) lies from the reference, in 0.01 degree inputs. */
static int32 traj_joint_steps(uint8 axis, int16 q1, int16 q2);

/* ---- two-axis DDA: one segment, both axes arriving together ------------------------------------------------- */

typedef struct {
    int32  target[2];
    uint32 distance[2], error[2], total, done;
    int8   sign[2];
} dda_t;

static void dda_start(dda_t *d, const int32 to[2], const int32 from[2])
{
    uint8 i;
    for (i = 0u; i < 2u; i++) {
        int32 delta = to[i] - from[i];
        d->target[i] = to[i];
        d->distance[i] = (uint32)abs_i32(delta); d->error[i] = 0u;
        d->sign[i] = (delta < 0) ? -1 : 1;
    }
    d->total = (d->distance[0] > d->distance[1]) ? d->distance[0] : d->distance[1];
    d->done = 0u;
}

/* The next step of the segment (call only while done < total): step[i] is 0, or the direction axis i pulses in.
 * Forced inline: the project builds at -Og, which would otherwise make this a call on every step. */
static inline __attribute__((always_inline)) void dda_step(dda_t *d, int8 step[2])
{
    uint8 i;
    for (i = 0u; i < 2u; i++) {
        d->error[i] += d->distance[i];
        step[i] = 0;
        if (d->error[i] >= d->total) { d->error[i] -= d->total; step[i] = d->sign[i]; }
    }
    d->done++;
}

/* When step k of a segment is due: floor(k * duration / total) cycles.  With q = duration / total and
 * r = duration % total that is k*q + k*r/total, which stays in 32 bits while total < 65536 (a node is at most 500 ms
 * at under 4000 steps/s), so the hot loop needs no 64-bit divide.  The last answer is kept because one pass of the main
 * loop asks for the same step from the trajectory service, the encoder slot check and the TX slot check. */
static uint32 traj_due_cy(uint32 k, uint32 duration, uint32 total)
{
    static uint32 memoK, memoDuration, memoTotal, memoDue; /* k >= 1, so the zero start never matches */
    if(k==memoK && duration==memoDuration && total==memoTotal) { return memoDue; }
    memoDue=(total<65536u) ? k*(duration/total)+k*(duration%total)/total : (uint32)((int64)k*duration/total);
    memoK=k; memoDuration=duration; memoTotal=total;
    return memoDue;
}

/* ---- the plan: a whole path preloaded, nothing moves until every numbered node is accepted ------------------ */

typedef enum { TRAJ_EMPTY = 0, TRAJ_UPLOADING = 1, TRAJ_READY = 2, TRAJ_RUNNING = 3, TRAJ_DONE = 4 } traj_state_t;
typedef struct { int16 q1, q2; uint16 ms; } traj_node_t;
typedef struct {
    uint8 state;   /* traj_state_t; DONE keeps the nodes, so a fresh RUN repeats the plan */
    uint16 count, expected, index;
    uint32 durationMs;
    uint32 elapsedCy, clock;  /* time spent in node `index`, and the SysTick count it was last advanced at */
    uint32 lastStep;          /* SysTick count of the last step pulse */
    uint32 lagMaxCy;          /* worst lateness of a step behind its due time; the owner clears it (ENABLE) */
    uint32 slipCy;            /* total plan-clock stretch of the current run (see TRAJ_LATE_SOFT_US); cleared by traj_start */
    int32 velocity[2];
    traj_node_t nodes[TRAJ_CAPACITY];
} traj_t;

/* Per-axis caps for one upload: steps/s and steps/s^2, already the smaller of the firmware cap and the configured one. */
typedef struct { uint32 rate[2], accel[2]; } traj_limits_t;
typedef enum { TRAJ_PUSH_OK = 0, TRAJ_PUSH_ORDER, TRAJ_PUSH_RATE } traj_push_t;

/* Forget the plan (the nodes stay in memory but are no longer a plan). */
static void traj_reset(traj_t *t)
{
    t->state = TRAJ_EMPTY; t->count = t->expected = t->index = 0u;
}

/* Node 0 is the start pose; `expected` nodes in all (the caller has range-checked it). */
static void traj_begin(traj_t *t, int16 q1, int16 q2, uint16 expected)
{
    t->nodes[0].q1 = q1; t->nodes[0].q2 = q2; t->nodes[0].ms = 0u;
    t->expected = expected; t->count = 1u; t->index = 0u; t->durationMs = 0u;
    t->velocity[0] = t->velocity[1] = 0;
    t->state = TRAJ_UPLOADING;
}

/* Append the next node.  seqOk is the caller's verdict on the numbering.  Refuses (changing nothing) on: wrong state,
 * a full plan, bad numbering, a node time outside [MIN, MAX], a plan over the total time (ORDER); or a node that is
 * faster than the rate cap, whose velocity change from the previous node exceeds the acceleration cap, or whose last
 * node does not come to rest within it (RATE). */
static traj_push_t traj_push(traj_t *t, const traj_limits_t *lim, uint8 seqOk, int16 q1, int16 q2, uint16 ms)
{
    const traj_node_t *last = &t->nodes[t->count ? t->count - 1u : 0u];
    int32 velocity[2]; uint16 previousMs; uint8 i;
    if (t->state != TRAJ_UPLOADING || t->count >= t->expected || !seqOk ||
        ms < TRAJ_NODE_MIN_MS || ms > TRAJ_NODE_MAX_MS || t->durationMs + ms > TRAJ_TOTAL_MAX_MS) { return TRAJ_PUSH_ORDER; }
    previousMs = last->ms ? last->ms : ms;
    for (i = 0u; i < 2u; i++) {
        int32 d = traj_joint_steps(i, q1, q2) - traj_joint_steps(i, last->q1, last->q2);
        velocity[i] = d * 1000 / (int32)ms;
        if ((uint32)abs_i32(d) * 1000u > lim->rate[i] * ms ||
            (uint32)abs_i32(velocity[i] - t->velocity[i]) * 2000u > lim->accel[i] * ((uint32)ms + previousMs) ||
            (t->count + 1u == t->expected && (uint32)abs_i32(velocity[i]) * 2000u > lim->accel[i] * ms)) { return TRAJ_PUSH_RATE; }
    }
    t->nodes[t->count].q1 = q1; t->nodes[t->count].q2 = q2;
    t->nodes[t->count++].ms = ms; t->durationMs += ms;
    t->velocity[0] = velocity[0]; t->velocity[1] = velocity[1];
    if (t->count == t->expected) { t->state = TRAJ_READY; }
    return TRAJ_PUSH_OK;
}

/* A complete plan, fresh or already run once, can be started. */
static uint8 traj_runnable(const traj_t *t)
{
    return (t->state == TRAJ_READY || t->state == TRAJ_DONE) && t->count == t->expected;
}

/* Aim `seg` at node `index`: both axes from `from` to the node's pose, with the reference at `origin`. */
static void traj_segment(const traj_t *t, dda_t *seg, const int32 origin[2], const int32 from[2])
{
    const traj_node_t *n = &t->nodes[t->index];
    int32 to[2];
    to[0] = origin[0] - traj_joint_steps(0u, n->q1, n->q2);
    to[1] = origin[1] - traj_joint_steps(1u, n->q1, n->q2);
    dda_start(seg, to, from);
}

/* Start at node 1 (node 0 is where the machine already stands, at `pos`); `now` is the SysTick count. */
static void traj_start(traj_t *t, dda_t *seg, const int32 origin[2], const int32 pos[2], uint32 now)
{
    t->state = TRAJ_RUNNING; t->index = 1u; t->elapsedCy = 0u; t->clock = now; t->lastStep = now; t->slipCy = 0u;
    traj_segment(t, seg, origin, pos);
}

typedef enum { TRAJ_POLL_IDLE = 0, TRAJ_POLL_STEP, TRAJ_POLL_DONE, TRAJ_POLL_OVERRUN, TRAJ_POLL_UNSUSTAINED } traj_poll_t;

/* Advance a running plan to `now`.  Call as often as the foreground allows; each call does at most one thing:
 *   STEP     one DDA event is due: step[i] says which axes pulse and which way.  The caller pulses them.
 *   DONE     the last node's time is up (the caller starts its settle check).
 *   OVERRUN  the current node has run TRAJ_OVERRUN_US past its time: the caller stops the machine.
 *   UNSUSTAINED  the run has been held back more than TRAJ_SLIP_MAX_MS in all: the caller stops the machine.
 *   IDLE     nothing is due (a node boundary may have been crossed).
 * A step more than TRAJ_LATE_SOFT_US behind its due time holds the plan clock back by the excess instead of letting the
 * lateness pile up to the overrun stop: the segment takes longer, its step count (the drawn shape) is unchanged.
 * A step is never issued sooner than TRAJ_STEP_MIN_US after the previous one, even when catching up.  The next
 * segment starts from where the last one aimed, which is where the pulses put the axes. */
static traj_poll_t traj_poll(traj_t *t, dda_t *seg, const int32 origin[2], uint32 now, int8 step[2])
{
    uint32 duration = (uint32)t->nodes[t->index].ms * 1000u * SU_CY_PER_US;
    t->elapsedCy += (t->clock - now) & 0x00FFFFFFu; t->clock = now;
    if (t->elapsedCy > duration + TRAJ_OVERRUN_US * SU_CY_PER_US) { return TRAJ_POLL_OVERRUN; }
    if (seg->done < seg->total) {
        uint32 due = traj_due_cy(seg->done + 1u, duration, seg->total);
        if (t->elapsedCy < due || ((t->lastStep - now) & 0x00FFFFFFu) < TRAJ_STEP_MIN_US * SU_CY_PER_US) { return TRAJ_POLL_IDLE; }
        {
            uint32 late = t->elapsedCy - due, soft = TRAJ_LATE_SOFT_US * SU_CY_PER_US;
            if (late > t->lagMaxCy) { t->lagMaxCy = late; }
            if (late > soft) {
                t->elapsedCy -= late - soft; t->slipCy += late - soft;
                if (t->slipCy > TRAJ_SLIP_MAX_MS * 1000u * SU_CY_PER_US) { return TRAJ_POLL_UNSUSTAINED; }
            }
        }
        t->lastStep = now;
        dda_step(seg, step);
        return TRAJ_POLL_STEP;
    }
    if (t->elapsedCy >= duration) {
        t->elapsedCy -= duration;
        if(++t->index==t->count) { return TRAJ_POLL_DONE; }
        traj_segment(t, seg, origin, seg->target);
    }
    return TRAJ_POLL_IDLE;
}

/* Cycles spent in the current node as of `now`. */
static uint32 traj_elapsed_cy(const traj_t *t, uint32 now)
{
    return t->elapsedCy + ((t->clock - now) & 0x00FFFFFFu);
}

/* For slot checks: how far into the node we are, and where in the node the next step (or the node's end) is due. */
static void traj_next_due(const traj_t *t, const dda_t *seg, uint32 now, uint32 *since, uint32 *due)
{
    uint32 duration = (uint32)t->nodes[t->index].ms * 1000u * SU_CY_PER_US;
    *since = traj_elapsed_cy(t, now);
    *due = (seg->done < seg->total) ? traj_due_cy(seg->done + 1u, duration, seg->total) : duration;
}

#endif /* TRAJ_H */
