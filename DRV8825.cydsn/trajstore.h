/* trajstore.h -- trajectory archive in flash.
 *
 * The RAM plan g_traj (traj.h) is the only thing the executor reads.  This file keeps up to TS_SLOTS finished plans in a
 * reserved flash region so they survive power-off; loading one just refills g_traj node by node through the same
 * traj_begin()/traj_push() checks an upload goes through (main.c).
 *
 * Layout.  The region starts at TS_REGION_OFFSET (reserved for it by cm3gcc_trajstore.ld, nothing else may link there) and is
 * two banks of TS_BANK_ROWS flash rows.  One bank holds the archive.  Every change (save, replace, erase) builds the whole next
 * archive in the other bank, the records that stay copied row by row, and commits it last; the old bank is not touched, so power
 * lost at any point leaves the old archive whole or the new one whole.  Each record takes only the rows it needs.
 *
 * Commit row, the first row of a bank (little endian; the rest of the row stays 0xFF):
 *    0 magic u32   4 version u16   6 contract u16   8 gen u32   12 records u16   14 usedRows u16
 *   16 bankCrc u32 (record rows 1..usedRows, whole rows)   20 headCrc u32 (bytes 0..19)
 * The records follow it back to back, each starting on a row boundary: a TS_REC_HEAD byte header, the description (metaLen
 * bytes), the nodes, then 0xFF up to the next row.
 *    0 slot u16   2 count u16   4 seq u32   8 durationMs u32   12 dataCrc u32 (node stream)   16 metaLen u16   18 0xFFFF
 *   20 headCrc u32 (bytes 0..19)
 * Nodes are 6 bytes each, little endian: q1 (cdeg, int16) q2 (cdeg, int16) ms (uint16).  The description (optional) is up to
 * TS_META_MAX opaque bytes the host keeps with the plan; the device never reads it.  A record's seq is the generation of the
 * bank it was first written in: a slot that gets another plan gets a new seq, and seq values never repeat.  `contract` says
 * which joint-angle convention the nodes use; raise TS_CONTRACT when that changes and the archive becomes empty instead of
 * drawn wrong.  The newer valid bank (gen compared modulo 2^32) is the archive; its records are listed in g_tsDir (RAM).
 *
 * Nothing here is called from an interrupt; a change runs one row per foreground pass because a row write blocks for
 * milliseconds (the measured worst case is reported, see ts_job_t.rowMaxCy).
 *
 * The hardware is behind one hook, ts_write_row(), defined by main.c; the simulation backs it with an array.
 */
#ifndef TRAJSTORE_H
#define TRAJSTORE_H

#define TS_SLOTS            (32u)
#define TS_ROW_BYTES        (256u)               /* one flash row (CYDEV_FLS_ROW_SIZE) */
#define TS_BANK_ROWS        (256u)               /* 64 KB: the commit row and TS_DATA_ROWS rows of records */
#define TS_DATA_ROWS        (TS_BANK_ROWS - 1u)
#define TS_BANK_BYTES       (TS_BANK_ROWS * TS_ROW_BYTES)
#define TS_REGION_OFFSET    (0x20000u)           /* second half of the 256 KB flash */
#define TS_REGION_BYTES     (2u * TS_BANK_BYTES)
#define TS_NODE_BYTES       (6u)
#define TS_REC_HEAD         (24u)
#define TS_META_MAX         (TS_ROW_BYTES)
#define TS_MAGIC            (0x534A5254u)        /* "TRJS" */
#define TS_VERSION          (2u)
#define TS_CONTRACT         (1u)
#define TS_REC_ROWS(count, metaLen) ((uint16)((TS_REC_HEAD + (uint32)(metaLen) + (uint32)(count) * TS_NODE_BYTES + TS_ROW_BYTES - 1u) / TS_ROW_BYTES))
/* Proof that this image was linked with cm3gcc_trajstore.ld, which reserves the region.  With the stock script the symbol
 * is undefined (weak, so 0), TS_LINKED() is false and every archive operation is refused: the archive can then never be
 * written over program code, whatever the linker settings of the project happen to be. */
#ifndef TS_LINKED
extern const uint8 __trajstore_start[] __attribute__((weak));
#define TS_LINKED() ((uint32)__trajstore_start == (uint32)(CYDEV_FLASH_BASE + TS_REGION_OFFSET))
#endif
_Static_assert(TS_REC_ROWS(TRAJ_CAPACITY, TS_META_MAX) <= TS_DATA_ROWS, "a full plan with the largest description must fit in a bank");
_Static_assert(TS_SLOTS <= 32u, "the inventory reports the slots as a 32-bit mask");
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

/* ---- the archive in RAM: which bank, and where each slot's record is ---------------------------------------------- */

typedef struct {
    uint8  valid;
    uint16 count, metaLen;        /* metaLen 0 = no description */
    uint16 row;                   /* the record's first row in the active bank */
    uint32 seq, durationMs, dataCrc;
    int16  q1, q2;                /* start pose, cdeg: node 0 */
} ts_info_t;

static ts_info_t g_tsDir[TS_SLOTS];
static struct { uint8 valid, bank; uint16 records, usedRows; uint32 gen; } g_tsBank;

static uint32 ts_bank_offset(uint8 bank) { return TS_REGION_OFFSET + (uint32)bank * TS_BANK_BYTES; }
static uint16 ts_rows(const ts_info_t *r) { return TS_REC_ROWS(r->count, r->metaLen); }
/* A valid slot's record: header, then the description, then the nodes. */
static const uint8 *ts_record(uint8 slot) { return ts_flash(ts_bank_offset(g_tsBank.bank) + (uint32)g_tsDir[slot].row * TS_ROW_BYTES); }
static const uint8 *ts_nodes(uint8 slot) { return ts_record(slot) + TS_REC_HEAD + g_tsDir[slot].metaLen; }
/* The flash may change after the boot check (a flipped bit): a plan is loaded or copied only while its nodes still match its CRC. */
static uint8 ts_nodes_ok(uint8 slot) { return (uint8)(ts_crc32(0u, ts_nodes(slot), (uint32)g_tsDir[slot].count * TS_NODE_BYTES) == g_tsDir[slot].dataCrc); }

/* Is `bank` a complete, committed archive?  `full` also checks the CRC of every used row (at boot; a bank just written was read
 * back row by row).  `fill` (only for a bank that already passed) loads g_tsDir from it. */
static uint8 ts_check_bank(uint8 bank, uint8 full, uint8 fill)
{
    const uint8 *b = ts_flash(ts_bank_offset(bank));
    uint16 records = ts_get16(b + 12), used = ts_get16(b + 14), row = 1u, i;
    uint32 seen = 0u;
    if (ts_get32(b) != TS_MAGIC || ts_get16(b + 4) != TS_VERSION || ts_get16(b + 6) != TS_CONTRACT || records > TS_SLOTS ||
        used > TS_DATA_ROWS || ts_crc32(0u, b, 20u) != ts_get32(b + 20) ||
        (full && ts_crc32(0u, b + TS_ROW_BYTES, (uint32)used * TS_ROW_BYTES) != ts_get32(b + 16))) { return 0u; }
    if (fill) { memset(g_tsDir, 0, sizeof(g_tsDir)); }
    for (i = 0u; i < records; i++) {
        const uint8 *h = b + (uint32)row * TS_ROW_BYTES;
        uint16 slot, count, metaLen;
        if (row > used || ts_crc32(0u, h, 20u) != ts_get32(h + 20)) { return 0u; }
        slot = ts_get16(h); count = ts_get16(h + 2); metaLen = ts_get16(h + 16);
        if (slot >= TS_SLOTS || (seen & (1ul << slot)) || count < 2u || count > TRAJ_CAPACITY || metaLen > TS_META_MAX ||
            (uint32)row + TS_REC_ROWS(count, metaLen) > (uint32)used + 1u) { return 0u; }
        seen |= 1ul << slot;
        if (fill) {
            ts_info_t *r = &g_tsDir[slot];
            const uint8 *n = h + TS_REC_HEAD + metaLen;
            r->valid = 1u; r->count = count; r->metaLen = metaLen; r->row = row;
            r->seq = ts_get32(h + 4); r->durationMs = ts_get32(h + 8); r->dataCrc = ts_get32(h + 12);
            r->q1 = (int16)ts_get16(n); r->q2 = (int16)ts_get16(n + 2);
        }
        row = (uint16)(row + TS_REC_ROWS(count, metaLen));
    }
    return (uint8)(row == used + 1u);
}

/* Make `bank` the archive (it is checked first, without the row CRC). */
static uint8 ts_use(uint8 bank)
{
    const uint8 *b = ts_flash(ts_bank_offset(bank));
    if (!ts_check_bank(bank, 0u, 1u)) { return 0u; }
    g_tsBank.valid = 1u; g_tsBank.bank = bank; g_tsBank.gen = ts_get32(b + 8);
    g_tsBank.records = ts_get16(b + 12); g_tsBank.usedRows = ts_get16(b + 14);
    return 1u;
}

/* At boot: the newer of the two fully checked banks, or an empty archive. */
static void ts_mount(void)
{
    uint8 okA, okB;
    memset(g_tsDir, 0, sizeof(g_tsDir)); memset(&g_tsBank, 0, sizeof(g_tsBank));
    if (!TS_LINKED()) { return; }
    okA = ts_check_bank(0u, 1u, 0u); okB = ts_check_bank(1u, 1u, 0u);
    if (okA && (!okB || (int32)(ts_get32(ts_flash(ts_bank_offset(0u)) + 8) - ts_get32(ts_flash(ts_bank_offset(1u)) + 8)) > 0)) { (void)ts_use(0u); }
    else if (okB) { (void)ts_use(1u); }
}

/* The archive's bank no longer matches its CRC (the flash changed after the boot check): use what a reboot would find, the
 * other bank if it is whole, else an empty archive. */
static void ts_drop_active(void)
{
    uint8 other = (uint8)(g_tsBank.bank ^ 1u);
    uint32 gen = g_tsBank.gen;
    memset(g_tsDir, 0, sizeof(g_tsDir)); memset(&g_tsBank, 0, sizeof(g_tsBank));
    if (ts_check_bank(other, 1u, 0u)) { (void)ts_use(other); }
    g_tsBank.gen = gen;                   /* the next bank still outranks the dropped one, so its new records get unused seq values */
}

/* Rows left for a record in `slot` (its present record, if any, would be replaced). */
static uint16 ts_free_rows(uint8 slot)
{
    uint16 used = 0u; uint8 i;
    for (i = 0u; i < TS_SLOTS; i++) { if (g_tsDir[i].valid && i != slot) { used = (uint16)(used + ts_rows(&g_tsDir[i])); } }
    return (uint16)(TS_DATA_ROWS - used);
}

/* ---- writing: one row per ts_step() ------------------------------------------------------------------------------- */

typedef enum { TS_JOB_NONE = 0, TS_JOB_SAVE = 1, TS_JOB_ERASE = 2 } ts_kind_t;
typedef enum { TS_STEP_MORE = 0, TS_STEP_DONE = 1, TS_STEP_FAIL = 2 } ts_step_t;

typedef struct {
    uint8  busy, kind, slot, bank;        /* bank: the one being built */
    uint8  phase;                         /* 4 check the records that stay, 0 clear its commit row, 1 copy them, 2 the new record, 3 commit */
    uint8  mismatch;                      /* the description was made for another plan: nothing is written */
    uint8  damaged;                       /* the bank to copy from no longer matches its CRC: nothing written, it was dropped */
    uint32 check;                         /* phase 4: CRC of the active bank's rows so far */
    uint8  src;                           /* phase 1: the slot being copied */
    uint16 part;                          /* row within the record being written */
    uint16 next;                          /* next record row of the bank */
    uint16 row, rows;                     /* rows written / to write in all */
    uint16 count, metaLen, records;
    uint32 gen, crc, durationMs, dataCrc;
    const traj_node_t *nodes;             /* the RAM plan; the caller keeps it unchanged while busy */
    const uint8 *meta;                    /* the description (metaLen bytes, 0 = none); also kept unchanged while busy */
    uint32 rowMaxCy;                      /* slowest row write, SysTick cycles */
    uint32 rowsWritten;
    uint8  head[TS_REC_HEAD];             /* the new record's header */
    uint8  buf[TS_ROW_BYTES];
} ts_job_t;

static uint8 ts_write_row(uint32 offset, const uint8 *row);   /* main.c: program one row at offset and read it back; 1 = ok */

/* The next archive is the present one without `slot`, plus (a save) the new record. */
static void ts_job_begin(ts_job_t *j, uint8 kind, uint8 slot)
{
    uint8 i;
    memset(j, 0, sizeof(*j));
    j->busy = 1u; j->kind = kind; j->slot = slot; j->next = 1u; j->phase = 4u;
    j->bank = g_tsBank.valid ? (uint8)(g_tsBank.bank ^ 1u) : 0u;
    j->gen = g_tsBank.gen + 1u;
    for (i = 0u; i < TS_SLOTS; i++) {
        if (g_tsDir[i].valid && i != slot) { j->records++; j->rows = (uint16)(j->rows + ts_rows(&g_tsDir[i])); }
    }
    j->rows += 2u;                        /* the commit row, cleared first and written last */
}

/* The caller checked the room (ts_free_rows).  `meta` (metaLen > 0) is written with the plan, but only if planCrc is the CRC of
 * the node stream: a description made for another plan is never committed. */
static void ts_job_save(ts_job_t *j, uint8 slot, const traj_node_t *nodes, uint16 count, uint32 durationMs,
                        const uint8 *meta, uint16 metaLen, uint32 planCrc)
{
    uint32 k, i, total = (uint32)count * TS_NODE_BYTES;
    ts_job_begin(j, TS_JOB_SAVE, slot);
    j->nodes = nodes; j->count = count; j->durationMs = durationMs;
    j->meta = meta; j->metaLen = meta ? metaLen : 0u;
    for (k = 0u; k < total; k += i) {
        for (i = 0u; i < TS_ROW_BYTES && k + i < total; i++) { j->buf[i] = ts_stream_byte(nodes, k + i); }
        j->dataCrc = ts_crc32(j->dataCrc, j->buf, i);
    }
    j->mismatch = (uint8)(j->metaLen && planCrc != j->dataCrc);
    memset(j->head, 0xFF, TS_REC_HEAD);
    ts_put16(j->head, slot); ts_put16(j->head + 2, count); ts_put32(j->head + 4, j->gen); ts_put32(j->head + 8, durationMs);
    ts_put32(j->head + 12, j->dataCrc); ts_put16(j->head + 16, j->metaLen); ts_put32(j->head + 20, ts_crc32(0u, j->head, 20u));
    j->records++; j->rows = (uint16)(j->rows + TS_REC_ROWS(count, j->metaLen));
}

/* 0: the slot is empty already and nothing is to be written. */
static uint8 ts_job_erase(ts_job_t *j, uint8 slot)
{
    if (!g_tsDir[slot].valid) { memset(j, 0, sizeof(*j)); j->kind = TS_JOB_ERASE; j->slot = slot; return 0u; }
    ts_job_begin(j, TS_JOB_ERASE, slot);
    return 1u;
}

static uint8 ts_new_byte(const ts_job_t *j, uint32 k)
{
    if (k < TS_REC_HEAD) { return j->head[k]; }
    k -= TS_REC_HEAD;
    return k < j->metaLen ? j->meta[k] : ts_stream_byte(j->nodes, k - j->metaLen);
}

static ts_step_t ts_step(ts_job_t *j)
{
    uint32 base = ts_bank_offset(j->bank), at = 0u, k, first = (uint32)j->part * TS_ROW_BYTES;
    uint32 size = TS_REC_HEAD + j->metaLen + (uint32)j->count * TS_NODE_BYTES;
    uint32 start = SU_NOW(), took;
    uint8 ok;
    if (j->mismatch) { j->busy = 0u; return TS_STEP_FAIL; }
    if (j->phase == 4u) {                 /* first, before any write: the bank copied from still matches its CRCs, 16 rows per pass */
        const uint8 *b = ts_flash(ts_bank_offset(g_tsBank.bank));
        if (g_tsBank.valid && j->part < g_tsBank.usedRows) {
            uint16 n = (uint16)(g_tsBank.usedRows - j->part < 16u ? g_tsBank.usedRows - j->part : 16u);
            j->check = ts_crc32(j->check, b + (uint32)(1u + j->part) * TS_ROW_BYTES, (uint32)n * TS_ROW_BYTES);
            j->part = (uint16)(j->part + n);
            return TS_STEP_MORE;
        }
        if (g_tsBank.valid && (j->check != ts_get32(b + 16) || ts_crc32(0u, b, 20u) != ts_get32(b + 20))) {
            j->damaged = 1u; j->busy = 0u;
            ts_drop_active();             /* never sealed into a new bank: what a reboot would find instead */
            return TS_STEP_FAIL;
        }
        j->phase = 0u; j->part = 0u;
    }
    while (j->phase == 1u && j->src < TS_SLOTS && (!g_tsDir[j->src].valid || j->src == j->slot)) { j->src++; }
    if (j->phase == 1u && j->src == TS_SLOTS) { j->phase = j->kind == TS_JOB_SAVE ? 2u : 3u; }
    memset(j->buf, 0xFF, TS_ROW_BYTES);
    if (j->phase == 1u) {
        memcpy(j->buf, ts_record(j->src) + first, TS_ROW_BYTES);
        at = j->next;
    } else if (j->phase == 2u) {
        for (k = 0u; k < TS_ROW_BYTES && first + k < size; k++) { j->buf[k] = ts_new_byte(j, first + k); }
        at = j->next;
    } else if (j->phase == 3u) {
        ts_put32(j->buf, TS_MAGIC); ts_put16(j->buf + 4, TS_VERSION); ts_put16(j->buf + 6, TS_CONTRACT); ts_put32(j->buf + 8, j->gen);
        ts_put16(j->buf + 12, j->records); ts_put16(j->buf + 14, (uint16)(j->next - 1u)); ts_put32(j->buf + 16, j->crc);
        ts_put32(j->buf + 20, ts_crc32(0u, j->buf, 20u));
    }
    ok = ts_write_row(base + at * TS_ROW_BYTES, j->buf);
    j->rowsWritten++;
    if (ok) {
        j->row++;
        if (j->phase == 0u) { j->phase = 1u; }
        else if (j->phase == 3u) {
            j->busy = 0u;
            ok = ts_use(j->bank);
            if (!ok) { ts_mount(); }      /* cannot happen with rows that read back right; the flash decides, as after a reboot */
        } else {
            j->crc = ts_crc32(j->crc, j->buf, TS_ROW_BYTES); j->next++; j->part++;
            if (j->phase == 1u && j->part == ts_rows(&g_tsDir[j->src])) { j->src++; j->part = 0u; }
            else if (j->phase == 2u && (uint32)j->part * TS_ROW_BYTES >= size) { j->phase = 3u; }
        }
    }
    took = SU_ELAPSED(start);
    if (took > j->rowMaxCy) { j->rowMaxCy = took; }
    if (!ok) { j->busy = 0u; return TS_STEP_FAIL; }
    return j->busy ? TS_STEP_MORE : TS_STEP_DONE;
}

#endif /* TRAJSTORE_H */
