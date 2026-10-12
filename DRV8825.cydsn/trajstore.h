/* trajstore.h -- trajectory archive in flash.
 *
 * The RAM plan g_traj (traj.h) is the only thing the executor reads.  This file keeps up to TS_SLOTS finished plans in a
 * reserved flash region so they survive power-off; loading one just refills g_traj node by node through the same
 * traj_begin()/traj_push() checks an upload goes through (main.c).
 *
 * Layout.  The region starts at TS_REGION_OFFSET (reserved for it by cm3gcc_trajstore.ld, nothing else may link there).
 * Each slot owns two blocks, A and B, of TS_BLOCK_ROWS flash rows.  A block is one header row followed by the nodes:
 * 6 bytes each, little endian, q1 (cdeg, int16) q2 (cdeg, int16) ms (uint16), TS_NODE_BYTES * count bytes.
 *
 * Header row (little endian; the rest of the row stays 0xFF):
 *    0 magic u32   4 version u16   6 contract u16   8 slot u16   10 count u16
 *   12 seq u32    16 durationMs u32   20 q1 s16   22 q2 s16   24 dataCrc u32   28 headerCrc u32 (bytes 0..27)
 * `contract` says which joint-angle convention the nodes use; raise TS_CONTRACT when that changes and every old slot
 * becomes invalid instead of being drawn wrong.  A block is valid only when the header and the data both check out.
 *
 * Description (optional).  The host may keep a short text with a plan (which figure and settings made it, so another computer
 * can show and restore it).  The device never reads it: it stores up to TS_META_MAX opaque bytes in row TS_META_ROW0, the first
 * row after the largest plan, and records them in the header row after the old fields:
 *   32 metaLen u16   34 0xFFFF   36 metaCrc u32 (of the text)   40 extCrc u32 (bytes 32..39)
 * Older firmware never looks past byte 31 and leaves 0xFF there, so a block without a description (or one written by older
 * firmware) reads as "no description", and a block with one is still a good plan to older firmware.  The text row is written
 * before the header, so the header row commits plan and description together.
 *
 * Saving never touches the newest valid block of the slot: the other block is invalidated, filled row by row (each row
 * read back), and only then given its header, with seq one higher.  Power lost at any point leaves the old block (or an
 * invalid new one), never a half plan.  Nothing here is called from an interrupt; the save runs one row per foreground
 * pass because a row write blocks for milliseconds (the measured worst case is reported, see ts_job_t.rowMaxCy).
 *
 * The hardware is behind one hook, ts_write_row(), defined by main.c; the simulation backs it with an array.
 */
#ifndef TRAJSTORE_H
#define TRAJSTORE_H

#define TS_SLOTS            (4u)
#define TS_ROW_BYTES        (256u)               /* one flash row (CYDEV_FLS_ROW_SIZE) */
#define TS_BLOCK_ROWS       (52u)                /* 13 KB: header row + up to 48 data rows + spare */
#define TS_BLOCK_BYTES      (TS_BLOCK_ROWS * TS_ROW_BYTES)
#define TS_REGION_OFFSET    (0x20000u)           /* second half of the 256 KB flash */
#define TS_REGION_BYTES     (TS_SLOTS * 2u * TS_BLOCK_BYTES)
#define TS_NODE_BYTES       (6u)
#define TS_DATA_ROWS_MAX    ((TRAJ_CAPACITY * TS_NODE_BYTES + TS_ROW_BYTES - 1u) / TS_ROW_BYTES)
#define TS_META_ROW0        (1u + TS_DATA_ROWS_MAX)   /* the description sits in the spare row after a full plan */
#define TS_META_MAX         (TS_ROW_BYTES)
#define TS_MAGIC            (0x534A5254u)        /* "TRJS" */
#define TS_VERSION          (1u)
#define TS_CONTRACT         (1u)
/* Proof that this image was linked with cm3gcc_trajstore.ld, which reserves the region.  With the stock script the symbol
 * is undefined (weak, so 0), TS_LINKED() is false and every archive operation is refused: the archive can then never be
 * written over program code, whatever the linker settings of the project happen to be. */
#ifndef TS_LINKED
extern const uint8 __trajstore_start[] __attribute__((weak));
#define TS_LINKED() ((uint32)__trajstore_start == (uint32)(CYDEV_FLASH_BASE + TS_REGION_OFFSET))
#endif
_Static_assert(TS_DATA_ROWS_MAX + 1u <= TS_BLOCK_ROWS, "a full plan must fit in one block");
_Static_assert(TS_META_ROW0 + 1u <= TS_BLOCK_ROWS, "the description row must fit in one block");
_Static_assert(TS_REGION_OFFSET + TS_REGION_BYTES <= 0x40000u, "the archive must fit in the 256 KB flash");

/* ---- little-endian helpers and CRC-32 (IEEE, reflected, the same as zlib/.NET callers compute) ----------------------- */

static void ts_put16(uint8 *d, uint16 v) { d[0] = (uint8)v; d[1] = (uint8)(v >> 8); }
static void ts_put32(uint8 *d, uint32 v) { ts_put16(d, (uint16)v); ts_put16(d + 2, (uint16)(v >> 16)); }
static uint16 ts_get16(const uint8 *d) { return (uint16)(d[0] | ((uint16)d[1] << 8)); }
static uint32 ts_get32(const uint8 *d) { return (uint32)ts_get16(d) | ((uint32)ts_get16(d + 2) << 16); }

/* Running CRC: start from 0, feed any number of pieces, the value after each call is the CRC of everything so far. */
static uint32 ts_crc32(uint32 crc, const uint8 *d, uint32 n)
{
    static const uint32 table[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu };
    crc = ~crc;
    while (n--) {
        crc ^= *d++;
        crc = (crc >> 4) ^ table[crc & 15u];
        crc = (crc >> 4) ^ table[crc & 15u];
    }
    return ~crc;
}

/* Byte k of the node stream of `nodes` (node k/6, then q1 lo/hi, q2 lo/hi, ms lo/hi). */
static uint8 ts_stream_byte(const traj_node_t *nodes, uint32 k)
{
    const traj_node_t *n = &nodes[k / TS_NODE_BYTES];
    uint16 v = (k % TS_NODE_BYTES) < 2u ? (uint16)n->q1 : (k % TS_NODE_BYTES) < 4u ? (uint16)n->q2 : n->ms;
    return (uint8)((k & 1u) ? (v >> 8) : v);
}

/* ---- reading ------------------------------------------------------------------------------------------------------ */

typedef struct {
    uint8  valid, which;          /* which = 0 (A) / 1 (B): the block these values came from */
    uint16 count;
    uint32 seq, durationMs, dataCrc;
    int16  q1, q2;                /* start pose, cdeg: node 0 */
    uint16 metaLen;               /* 0 = no (valid) description */
} ts_info_t;

static uint32 ts_block_offset(uint8 slot, uint8 which) { return TS_REGION_OFFSET + ((uint32)slot * 2u + which) * TS_BLOCK_BYTES; }

/* Full check of one block: header fields and CRC, count, and the CRC of the node bytes.  Fills *info when valid. */
static uint8 ts_check_block(const uint8 *blk, uint8 slot, ts_info_t *info)
{
    uint16 count = ts_get16(blk + 10);
    if (ts_get32(blk) != TS_MAGIC || ts_get16(blk + 4) != TS_VERSION || ts_get16(blk + 6) != TS_CONTRACT ||
        ts_get16(blk + 8) != slot || count < 2u || count > TRAJ_CAPACITY ||
        ts_crc32(0u, blk, 28u) != ts_get32(blk + 28) ||
        ts_crc32(0u, blk + TS_ROW_BYTES, (uint32)count * TS_NODE_BYTES) != ts_get32(blk + 24)) { return 0u; }
    info->valid = 1u; info->count = count; info->seq = ts_get32(blk + 12); info->durationMs = ts_get32(blk + 16);
    info->q1 = (int16)ts_get16(blk + 20); info->q2 = (int16)ts_get16(blk + 22); info->dataCrc = ts_get32(blk + 24);
    info->metaLen = 0u;
    {
        uint16 len = ts_get16(blk + 32);
        if (len >= 1u && len <= TS_META_MAX && ts_crc32(0u, blk + 32, 8u) == ts_get32(blk + 40) &&
            ts_crc32(0u, blk + TS_META_ROW0 * TS_ROW_BYTES, len) == ts_get32(blk + 36)) { info->metaLen = len; }
    }
    return 1u;
}

/* The newest valid block of a slot (seq compared modulo 2^32).  info->valid = 0 when neither block is good. */
static void ts_active(uint8 slot, ts_info_t *info)
{
    ts_info_t a, b;
    uint8 okA, okB;
    a.valid = b.valid = 0u;
    okA = ts_check_block(ts_flash(ts_block_offset(slot, 0u)), slot, &a);
    okB = ts_check_block(ts_flash(ts_block_offset(slot, 1u)), slot, &b);
    a.which = 0u; b.which = 1u;
    info->valid = 0u; info->which = 0u;
    if (okA && (!okB || (int32)(a.seq - b.seq) > 0)) { *info = a; }
    else if (okB) { *info = b; }
}

/* ---- writing: one row per ts_step() ------------------------------------------------------------------------------- */

typedef enum { TS_JOB_NONE = 0, TS_JOB_SAVE = 1, TS_JOB_ERASE = 2 } ts_kind_t;
typedef enum { TS_STEP_MORE = 0, TS_STEP_DONE = 1, TS_STEP_FAIL = 2 } ts_step_t;

typedef struct {
    uint8  busy, kind, slot, which;       /* which: the block being written (erase: the one being cleared) */
    uint8  phase;                         /* 0 clear the header, 1 data rows, 3 description row (if any), 2 commit the header */
    uint8  mismatch;                      /* the description was made for another plan: nothing was committed */
    uint16 row, rows;                     /* next data row (1-based) / how many there are */
    uint16 count, metaLen;
    uint32 seq, crc, durationMs, planCrc;
    const traj_node_t *nodes;             /* the RAM plan; the caller keeps it unchanged while busy */
    const uint8 *meta;                    /* the description (metaLen bytes, 0 = none); also kept unchanged while busy */
    uint32 rowMaxCy;                      /* slowest row write, SysTick cycles */
    uint32 rowsWritten;
    uint8  buf[TS_ROW_BYTES];
} ts_job_t;

static uint8 ts_write_row(uint32 offset, const uint8 *row);   /* main.c: program one row at offset and read it back; 1 = ok */

/* `meta` (metaLen > 0) is written with the plan, but only if planCrc is the CRC of the node stream actually written: a
 * description made for another plan is never committed. */
static void ts_job_save(ts_job_t *j, uint8 slot, const traj_node_t *nodes, uint16 count, uint32 durationMs,
                        const uint8 *meta, uint16 metaLen, uint32 planCrc)
{
    ts_info_t cur;
    ts_active(slot, &cur);
    memset(j, 0, sizeof(*j));
    j->busy = 1u; j->kind = TS_JOB_SAVE; j->slot = slot;
    j->which = cur.valid ? (uint8)(cur.which ^ 1u) : 0u;
    j->seq = cur.valid ? cur.seq + 1u : 1u;
    j->nodes = nodes; j->count = count; j->durationMs = durationMs;
    j->meta = meta; j->metaLen = meta ? metaLen : 0u; j->planCrc = planCrc;
    j->rows = (uint16)(((uint32)count * TS_NODE_BYTES + TS_ROW_BYTES - 1u) / TS_ROW_BYTES);
    j->row = 1u;
}

static void ts_job_erase(ts_job_t *j, uint8 slot)
{
    memset(j, 0, sizeof(*j));
    j->busy = 1u; j->kind = TS_JOB_ERASE; j->slot = slot; j->which = 0u;
}

static ts_step_t ts_step(ts_job_t *j)
{
    uint32 base = ts_block_offset(j->slot, j->which);
    uint32 start = SU_NOW();
    uint32 took;
    uint8 ok;
    if (j->kind == TS_JOB_ERASE || j->phase == 0u) {
        memset(j->buf, 0xFF, TS_ROW_BYTES);
        ok = ts_write_row(base, j->buf);
        j->rowsWritten++;
        if (ok && j->kind == TS_JOB_ERASE && j->which == 0u) { j->which = 1u; }
        else if (ok && j->kind == TS_JOB_ERASE) { j->busy = 0u; }
        else if (ok) { j->phase = 1u; }
    } else if (j->phase == 1u) {
        uint32 first = (uint32)(j->row - 1u) * TS_ROW_BYTES, total = (uint32)j->count * TS_NODE_BYTES, n, i;
        n = total - first < TS_ROW_BYTES ? total - first : TS_ROW_BYTES;
        for (i = 0u; i < n; i++) { j->buf[i] = ts_stream_byte(j->nodes, first + i); }
        for (; i < TS_ROW_BYTES; i++) { j->buf[i] = 0xFFu; }
        j->crc = ts_crc32(j->crc, j->buf, n);
        ok = ts_write_row(base + (uint32)j->row * TS_ROW_BYTES, j->buf);
        j->rowsWritten++;
        if (ok && ++j->row > j->rows) {
            if (j->metaLen && j->planCrc != j->crc) { ok = 0u; j->mismatch = 1u; }
            else { j->phase = j->metaLen ? 3u : 2u; }
        }
    } else if (j->phase == 3u) {
        memset(j->buf, 0xFF, TS_ROW_BYTES);
        memcpy(j->buf, j->meta, j->metaLen);
        ok = ts_write_row(base + TS_META_ROW0 * TS_ROW_BYTES, j->buf);
        j->rowsWritten++;
        if (ok) { j->phase = 2u; }
    } else {
        memset(j->buf, 0xFF, TS_ROW_BYTES);
        ts_put32(j->buf, TS_MAGIC); ts_put16(j->buf + 4, TS_VERSION); ts_put16(j->buf + 6, TS_CONTRACT);
        ts_put16(j->buf + 8, j->slot); ts_put16(j->buf + 10, j->count); ts_put32(j->buf + 12, j->seq);
        ts_put32(j->buf + 16, j->durationMs);
        ts_put16(j->buf + 20, (uint16)j->nodes[0].q1); ts_put16(j->buf + 22, (uint16)j->nodes[0].q2);
        ts_put32(j->buf + 24, j->crc);
        ts_put32(j->buf + 28, ts_crc32(0u, j->buf, 28u));
        if (j->metaLen) {
            ts_put16(j->buf + 32, j->metaLen);
            ts_put32(j->buf + 36, ts_crc32(0u, j->meta, j->metaLen));
            ts_put32(j->buf + 40, ts_crc32(0u, j->buf + 32, 8u));
        }
        ok = ts_write_row(base, j->buf);
        j->rowsWritten++;
        if (ok) { j->busy = 0u; }
    }
    took = SU_ELAPSED(start);
    if (took > j->rowMaxCy) { j->rowMaxCy = took; }
    if (!ok) { j->busy = 0u; return TS_STEP_FAIL; }
    return j->busy ? TS_STEP_MORE : TS_STEP_DONE;
}

#endif /* TRAJSTORE_H */
