/* traj.h - the timed-trajectory executor's data and rules.  Header-only: main.c includes it once, so every function
 * is static and inlinable and the step path pays nothing for the split.
 *
 * What lives here: the numbered-node plan and its upload/validation rules, the two-axis DDA that turns one segment
 * into step pulses, and the due-time arithmetic.  What stays in main.c: the supervision (heartbeat, limits, encoder
 * following, final-position check), the pulses themselves, and the wire protocol.
 *
 * The host (main.c) must provide, before this header is included:
 *   TRAJ_CAPACITY, TRAJ_NODE_MIN_MS, TRAJ_NODE_MAX_MS, TRAJ_TOTAL_MAX_MS   (the host/firmware contract; the release
 *                                                                          gate reads them from main.c)
 *   abs_i32()
 * and, after it (a forward declaration here, the definition there):
 *   traj_joint_steps()                                                     (kinematics: joint angles -> steps)
 */
#ifndef TRAJ_H
#define TRAJ_H

#if !defined(TRAJ_CAPACITY) || !defined(TRAJ_NODE_MIN_MS) || !defined(TRAJ_NODE_MAX_MS) || !defined(TRAJ_TOTAL_MAX_MS)
#error "define the TRAJ_* contract macros before including traj.h"
#endif

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
 * at 4000 steps/s), so the hot loop needs no 64-bit divide.  The last answer is kept because one pass of the main
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
    uint32 durationMs, elapsedCy, clock;
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

/* Start at node 1 (node 0 is where the machine already stands); `now` is the SysTick count at the start. */
static void traj_start(traj_t *t, uint32 now)
{
    t->state = TRAJ_RUNNING; t->index = 1u; t->elapsedCy = 0u; t->clock = now;
}

#endif /* TRAJ_H */
