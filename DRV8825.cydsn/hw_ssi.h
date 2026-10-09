/* hw_ssi.h - AS5045 serial read-out behind one small interface (docs/HARDWARE_OFFLOAD_PLAN.md, P2).
 *
 *   ssi_begin(clocks)   start a frame of 18 (board 1 only) or 38 (both boards) data clocks
 *   ssi_done()          the frame is complete and ssi_take() may be called
 *   ssi_take(&a, &b)    the two raw 18-bit words (b is 0 for an 18-clock frame)
 *   ssi_abort()         forget a frame in flight (leaves the interface idle)
 *   as5045_read_clocks  begin + wait + take for callers that need the answer now
 *
 * HW_SSI = 0: bit-banged on CLK/CSn/DO.  ssi_begin() runs the whole frame in the foreground (about 115 us for 18
 *             clocks, 230 us for 38, each clock with interrupts masked) and ssi_done() is always true.
 * HW_SSI = 1: the UDB SPI Master "Ssi" clocks the frame and drives CSn itself; ssi_begin() only queues the words and
 *             returns, the CPU is never held up and interrupts are never masked.
 *
 * Needs before inclusion: PIN_CSN/PIN_CLK/PIN_DO and SSI_HALF_US/SSI_SAMPLE_US (software), FRAME_MASK, delay_us(),
 * SU_NOW()/SU_ELAPSED()/SU_CY_PER_US, g_encIrqMaxCy, CyEnterCriticalSection()/CyExitCriticalSection(). */
#ifndef HW_SSI_H
#define HW_SSI_H

#define SSI_TIMEOUT_US              (1000u)  /* a frame is 160 us (hardware) / 230 us (software) at most */

#if (HW_SSI == 0u)

static uint32 g_ssiA, g_ssiB;

/* Daisy chain: 2 x 19 clocks (18 data + 1 dummy per device).  Board 1 (wired to the MCU) comes first, board 2 second.
 * With 18 clocks only board 1 is read and the second word is 0. */
static void ssi_begin(uint8 clocks)
{
    uint32 a = 0u, b = 0u;
    uint8 i, ints;
    uint32 started, cost;

    CyPins_ClearPin(PIN_CSN);
    delay_us(2u);
    for (i = 0u; i < clocks; i++)
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
    g_ssiA = a & FRAME_MASK;
    g_ssiB = b & FRAME_MASK;
}
static uint8 ssi_done(void) __attribute__((unused));
static uint8 ssi_done(void)                          { return 1u; }
static void ssi_take(uint32 *first, uint32 *second)  { *first = g_ssiA; *second = g_ssiB; }
static void ssi_abort(void) __attribute__((unused));
static void ssi_abort(void)                          { }

#else  /* HW_SSI */

/* SPI Master "Ssi": 10-bit words, mode 2 (CPOL = 1, CPHA = 0), 250 kHz, hardware SS on CSn, MISO on DO, MOSI unused.
 * The AS5045 changes DO on CLK rising edges and the master samples on the falling edge that follows, so stream bit 0
 * is DO right after CSn falls (a 1), bits 1..18 are board 1's 18 bits (D11 first), bit 19 is a dummy and bits 20..37
 * are board 2's: exactly the software read's bit i = j - 1.  Four words give 40 clocks (38 are needed) and fill the
 * 4-deep TX FIFO; two words (20 clocks) are enough for board 1 alone.  Drain received words as they arrive and
 * complete when all queued words are collected.  Mode and bit alignment are to be confirmed on the bench. */
#define SSI_WORD_BITS               (10u)
#define SSI_WORD_MASK               (0x03FFu)
#define SSI_WORDS_MAX               (4u)

static uint8  g_ssiWords, g_ssiRead;
static uint16 g_ssiRx[SSI_WORDS_MAX];
static uint32 g_ssiAt;                 /* SU_NOW() when the frame was queued */

static void ssi_begin(uint8 clocks)
{
    uint8 i;
    g_ssiWords = (clocks > 19u) ? 4u : 2u;
    g_ssiRead = 0u;
    g_ssiAt = SU_NOW();
    Ssi_ClearRxBuffer();
    for (i = 0u; i < g_ssiWords; i++) { Ssi_WriteTxData(SSI_WORD_MASK); }  /* MOSI is not connected */
}
static uint8 ssi_done(void)
{
    /* With the hardware FIFO, GetRxBufferSize reports only 0/1, not the number of words. */
    while (g_ssiRead < g_ssiWords && Ssi_GetRxBufferSize() != 0u)
    { g_ssiRx[g_ssiRead++] = (uint16)(Ssi_ReadRxData() & SSI_WORD_MASK); }
    return (g_ssiRead >= g_ssiWords) ? 1u : 0u;
}
static uint32 ssi_stream(uint8 from, uint8 count)    /* `count` stream bits starting at bit `from`, first bit is the MSB */
{
    uint32 value = 0u;
    uint8 j;
    for (j = from; j < (uint8)(from + count); j++)
    {
        uint8 word = (uint8)(j / SSI_WORD_BITS), shift = (uint8)(SSI_WORD_BITS - 1u - (j % SSI_WORD_BITS));
        value = (value << 1) | (uint32)(((word < g_ssiWords) ? (g_ssiRx[word] >> shift) : 0u) & 1u);
    }
    return value;
}
static void ssi_take(uint32 *first, uint32 *second)
{
    *first  = ssi_stream(1u, 18u) & FRAME_MASK;
    *second = (g_ssiWords > 2u) ? (ssi_stream(20u, 18u) & FRAME_MASK) : 0u;
    g_ssiWords = 0u;
}
static void ssi_abort(void)
{
    while (ssi_done() == 0u && SU_ELAPSED(g_ssiAt) < SSI_TIMEOUT_US * SU_CY_PER_US) { }  /* let a started frame finish */
    Ssi_ClearTxBuffer();
    Ssi_ClearRxBuffer();
    g_ssiWords = 0u;
}

#endif /* HW_SSI */

static void as5045_read_clocks(uint32 *first, uint32 *second, uint8 clocks)
{
    ssi_begin(clocks);
#if (HW_SSI != 0u)
    while (ssi_done() == 0u)
    {
        if (SU_ELAPSED(g_ssiAt) >= SSI_TIMEOUT_US * SU_CY_PER_US) { ssi_abort(); *first = 0u; *second = 0u; return; }
    }
#endif
    ssi_take(first, second);
}

#endif /* HW_SSI_H */
