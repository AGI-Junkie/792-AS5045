"""Actual firmware C with simulated GPIO/encoder/time; never touches serial or a board.
Run: python tests/test_stage1.py (requires installed Visual Studio C++ tools).  Options: --sim-mhz=N (bus clock of the simulation, default 48), --define=NAME=VALUE (e.g. HW_SSI=1u), --emit-c=DIR.
This verifies control flow, not real UART timing, motor torque or sensor wiring.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import sys

root = Path(__file__).resolve().parents[1]
host_plan = next((Path(a.split('=', 1)[1]).resolve() for a in sys.argv if a.startswith('--host-plan=')), None)
host_control = next((Path(a.split('=', 1)[1]).resolve() for a in sys.argv if a.startswith('--host-control=')), None)
emit_c = next((Path(a.split('=', 1)[1]).resolve() for a in sys.argv if a.startswith('--emit-c=')), None)
sim_mhz = int(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--sim-mhz=')), '48'))
extra_defs = ''.join('#define %s %s\n' % tuple(a[9:].split('=', 1)) for a in sys.argv if a.startswith('--define='))
sys.argv = [a for a in sys.argv if not a.startswith(('--host-plan=', '--host-control=', '--emit-c=', '--sim-mhz=', '--define='))]
source = (root / 'DRV8825.cydsn/main.c').read_text(encoding='utf-8')
# main.c includes traj.h; the simulation compiles one file, so put the header's text where the include is.
for header in ['traj.h', 'hw_ssi.h']:
    source = source.replace(f'#include "{header}"', (root / 'DRV8825.cydsn' / header).read_text(encoding='utf-8'), 1)
    assert f'#include "{header}"' not in source

def replace_once(text, old, new):
    count = text.count(old)
    assert count == 1, f'Simulation hook expected one match, got {count}: {old!r}'
    return text.replace(old, new, 1)

pins = sorted(set(re.findall(r'CYREG_PRT\d+_PC\d+', source)))
pin_defs = '\n'.join(f'#define {pin} {i}' for i, pin in enumerate(pins))
# Check node arrival at the actual transition, before the next node can pulse.
# A 1 ms fixture iteration may cross the boundary and already emit the next step.
source = replace_once(source, 'if(++t->index==t->count)',
                      'sim_check_traj_boundary(); if(++t->index==t->count)')
# Deliver the final event at ordinary-memory boundaries the register mock cannot intercept.
source = replace_once(source, '    if(g_traj.state==TRAJ_RUNNING) {\n#if (HW_EVT != 0u)',
                      '    if(simBeforeTrajCheck) { void (*hook)(void)=simBeforeTrajCheck; simBeforeTrajCheck=0; hook(); }\n'
                      '    if(g_traj.state==TRAJ_RUNNING) {\n#if (HW_EVT != 0u)')
source = replace_once(source, '    if (g_ptp.active==1u) {\n        uint32 period, since;',
                      '    if (g_ptp.active==1u) {\n'
                      '        if(simBeforeSlotCheck) { void (*hook)(void)=simBeforeSlotCheck; simBeforeSlotCheck=0; hook(); }\n'
                      '        uint32 period, since;')

mock = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int8_t int8; typedef int16_t int16; typedef int32_t int32; typedef int64_t int64;
#define __attribute__(x)
#ifndef SIM_MHZ
#define SIM_MHZ 24u
#endif
#define BCLK__BUS_CLK__HZ (SIM_MHZ*1000000u)
#define CYREG_NVIC_SYSTICK_CURRENT 100
#define CYREG_NVIC_SYSTICK_RELOAD 101
#define CYREG_NVIC_SYSTICK_CTL 102
#define CYREG_NVIC_INTR_CTRL_STATE 103
#define CY_INT_SYSTICK_IRQN 15
#define CY_ISR(name) void name(void)
#define CYWDT_128_TICKS 2
#define CYWDT_LPMODE_NOCHANGE 0
#define CY_PINS_DM_STRONG 0
#define CY_PINS_PC_DATAOUT 1
#define CY_PINS_DM_DIG_HIZ 0
#define CY_PINS_DM_RES_UP 0
#define CyGlobalIntEnable ((void)0)
static uint64_t simClock, simClockBase, simClockWraps;
static void (*simTickIsr)(void);
static void (*simBeforeTrajCheck)(void), (*simBeforeSlotCheck)(void);
static void (*simAfterCriticalExit)(void);
static int simInts=1, simTickPending;
static unsigned simWdtStarted, simWdtFeeds;
static int simIlo1K;
static void CyILO_Start1K(void){simIlo1K=1;}
static void CyWdtStart(uint8 ticks,uint8 mode){assert(simIlo1K && ticks==2 && mode==0);simWdtStarted++;}
static void CyWdtClear(void){assert(simWdtStarted);simWdtFeeds++;}
static void CyIntSetSysVector(uint8 n,void (*isr)(void)){assert(n==15);simTickIsr=isr;}
static void sim_evt_service(void);
static void sim_tick_sync(void) {
    uint64_t wraps=(simClock-simClockBase)>>24;
    while(simClockWraps<wraps){simClockWraps++;simTickPending=1;
        if(simInts && simTickIsr){simTickPending=0;simTickIsr();}}
    if(simInts && simTickPending && simTickIsr){simTickPending=0;simTickIsr();}
    sim_evt_service();
}
static uint8 mock_critical_enter(void){uint8 old=(uint8)simInts;sim_tick_sync();simInts=0;return old;}
static void mock_critical_exit(uint8 old){simInts=old;sim_tick_sync();
    if(simInts && simAfterCriticalExit){void (*hook)(void)=simAfterCriticalExit;simAfterCriticalExit=0;hook();}}
static int simPins[128], simStep1, simStep2, simBadEncoder, simBadReads;
static int simFault, simLimit, simHome1Mode, simHome2Mode, simStall, simPolarity=1, simStiction;
static int simHome1At=400, simHome2At=120;
static int simRateLimited; static uint32 simLastMotorPulse;
static int simRxActive; static uint64_t simRxBase, simRxOffset;
static uint8 simPtpWire[36], simPtpWireN;
static uint8 simPtpReceived[256];static unsigned simPtpReceivedN;
static int simAngleOverride=-1;
static int simEncoderBias, simEncoderBias2, simBadEncoder2, simPolarity2=1, simStall2;
static unsigned simBoundaryChecks;
static void sim_check_traj_boundary(void);
static unsigned simPulse1, simPulse2;
static uint32 simTrace; static uint64_t simTraceBase; /* FNV-1a over (clock since base, axis, direction) of every step pulse */
static unsigned simDriverSwitches;
static char simLog[1048576]; static unsigned simLogN;
/* SysTick read cost: the coarse 1 us keeps busy-wait loops fast, but inside an SSI frame (CSn low) the firmware
 * busy-waits on it for ~115 us, so a read costs 0.25 us there and the frame lasts as long as the real one. */
/* Inject target-library formatting cost; this is a scheduling stress model, not a measured printf profile. */
static unsigned simFormatCostUs;
static unsigned simFormatCalls;
static int simRequireFormatInts;
static int mock_snprintf(char *dst,size_t size,const char *fmt,...) {
    if(simRequireFormatInts)assert(simInts);simFormatCalls++;
    int n; va_list ap; va_start(ap,fmt);n=vsnprintf(dst,size,fmt,ap);va_end(ap);
    simClock+=(uint64_t)simFormatCostUs*SIM_MHZ;sim_tick_sync();return n;
}
#define snprintf mock_snprintf
static unsigned simRegCost=SIM_MHZ;
static uint32 mock_reg(uint32 reg) {
    simClock+=simRegCost;sim_tick_sync();
    if(reg==CYREG_NVIC_INTR_CTRL_STATE)return simTickPending?0x04000000u:0;
    return (uint32)(0xffffffu-(simClock-simClockBase))&0xffffffu;
}
static void mock_reg_set(uint32 reg,uint32 value){
    if(reg==CYREG_NVIC_SYSTICK_CURRENT){simClockBase=simClock;simClockWraps=0;simTickPending=0;}
    (void)value;
}
static void mock_log(uint8 c) { if(simLogN+1<sizeof(simLog)) {simLog[simLogN++]=(char)c;simLog[simLogN]=0;} }
/* Model the generated UART API boundary, not GPIO bit sampling. Bytes arrive
 * back-to-back at 115200, independently of foreground encoder/motion work. */
#define HostUart_TX_BUFFER_SIZE 256u
#define HostUart_RX_STS_OVERRUN 16u
#define HostUart_RX_STS_STOP_ERROR 8u
#define HostUart_RX_STS_PAR_ERROR 4u
#define HostUart_RX_STS_BREAK 2u
static uint8 HostUart_rxBufferOverflow, HostUart_errorStatus;
static uint8 simUartRx[256]; static unsigned simUartHead,simUartTail,simUartCount;
static unsigned simUartPacketIndex,simUartHeartbeat,simUartTxFull,simUartStarted;
static uint64_t simUartTxBusyUntil;
static unsigned simUartChunk;
static void sim_uart_heartbeat(void);
static void sim_uart_push(uint8 b) {
    if(simUartCount==256){HostUart_rxBufferOverflow=1;return;}
    simUartRx[simUartHead++%256]=b;simUartCount++;
}
static void sim_uart_fill(void) {
    uint32 us;
    if(!simRxActive)return;
    us=(simClock-simRxBase+simRxOffset)/SIM_MHZ;
    while(simUartPacketIndex<simPtpWireN && us>=5000u+(simUartChunk?
        (simUartPacketIndex/simUartChunk)*40000u+(simUartPacketIndex%simUartChunk)*87u:simUartPacketIndex*87u))
        sim_uart_push(simPtpWire[simUartPacketIndex++]);
    if((simUartPacketIndex==0 || simUartPacketIndex==simPtpWireN) && us>=500u+simUartHeartbeat*40000u) {
        sim_uart_heartbeat();simUartHeartbeat=us/40000u+1u;
    }
}
static void HostUart_Start(void){simUartStarted=1;}
static unsigned HostUart_GetRxBufferSize(void){sim_uart_fill();return simUartCount;}
static uint16 HostUart_GetByte(void){
    uint16 b=(uint16)HostUart_errorStatus<<8;HostUart_errorStatus=0;
    if(simUartCount){b|=simUartRx[simUartTail++%256];simUartCount--;
        if(simPtpWireN && simPtpReceivedN<256)simPtpReceived[simPtpReceivedN++]=(uint8)b;}
    return b;
}
static void HostUart_ClearRxBuffer(void){simUartHead=simUartTail=simUartCount=0;HostUart_rxBufferOverflow=0;}
static unsigned HostUart_GetTxBufferSize(void){return (simUartTxFull || simClock<simUartTxBusyUntil)?255:0;}
static void HostUart_PutChar(uint8 b){
    assert(!simUartTxFull);
    while(simClock<simUartTxBusyUntil){simClock+=SIM_MHZ;sim_tick_sync();}
    mock_log(b);
}
/* USB CDC "UsbCdc" (USBFS macro): configured + DTR = the port is open.  Packets of at most 64 bytes, like the endpoint. */
#define UsbCdc_5V_OPERATION 1u
#define UsbCdc_LINE_CONTROL_DTR 1u
static uint8 simUsbCfg, simUsbCfgChanged, simUsbDtr, simUsbStarted, simUsbInit, simUsbReady=1;
static uint8 simUsbRx[512]; static unsigned simUsbRxN, simUsbRxI;
static char simUsbSent[4096]; static unsigned simUsbSentN, simUsbPackets, simUsbMaxPacket;
static void UsbCdc_Start(uint8 device, uint8 mode){(void)device;(void)mode;simUsbStarted=1;}
static uint8 UsbCdc_GetConfiguration(void){return simUsbCfg;}
static uint8 UsbCdc_IsConfigurationChanged(void){uint8 c=simUsbCfgChanged;simUsbCfgChanged=0;return c;}
static uint8 UsbCdc_CDC_Init(void){simUsbInit++;return 0;}
static uint16 UsbCdc_GetLineControl(void){return simUsbDtr;}
static uint8 UsbCdc_DataIsReady(void){return simUsbRxI<simUsbRxN;}
static uint16 UsbCdc_GetAll(uint8 *p){unsigned n=simUsbRxN-simUsbRxI;if(n>64u)n=64u;memcpy(p,simUsbRx+simUsbRxI,n);simUsbRxI+=n;return (uint16)n;}
static uint8 UsbCdc_CDCIsReady(void){return simUsbReady;}
static void UsbCdc_PutData(const uint8 *p, uint16 n){
    assert(simUsbReady && n>0 && n<=64u && simUsbSentN+n<sizeof(simUsbSent));
    memcpy(simUsbSent+simUsbSentN,p,n);simUsbSentN+=n;simUsbPackets++;if(n>simUsbMaxPacket)simUsbMaxPacket=n;
}
static void sim_usb_reset(void){simUsbCfg=simUsbCfgChanged=simUsbDtr=0;simUsbReady=1;simUsbRxN=simUsbRxI=0;simUsbSentN=simUsbPackets=simUsbMaxPacket=0;simUsbInit=0;}
static void sim_usb_push(const void *p, unsigned n){memcpy(simUsbRx+simUsbRxN,p,n);simUsbRxN+=n;}
/* DMA channel "TxDma": text queue -> UART TX FIFO.  The channel moves one byte per simDmaByteCy cycles (0 = the UART
 * keeps up instantly, like the PutChar model above) and the bytes appear in the log as they are moved, so a stuck wire
 * (a huge simDmaByteCy) sends nothing and a disabled channel stops where it was. */
#define HI16(x) ((uint16)(((uint32)(x))>>16))
#define LO16(x) ((uint16)(x))
#define CYDEV_SRAM_BASE 0x20000000u
#define CYDEV_PERIPH_BASE 0x40000000u
#define CY_DMA_DISABLE_TD 0xFEu
#define CY_DMA_TD_INC_SRC_ADR 1u
#define CY_DMA_STATUS_CHAIN_ACTIVE 1u
#define CY_DMA_STATUS_TD_ACTIVE 2u
#define CY_DMA_CH_BASIC_CFG_EN 1u
static struct { volatile uint8 basic_cfg[4]; } simDmaChannels[2];
#define CY_DMA_CH_STRUCT_PTR simDmaChannels
#define simDmaEnabled simDmaChannels[1].basic_cfg[0]
typedef int cystatus;
static uint8 simUartTxFifoReg;
#define HostUart_TXDATA_PTR ((volatile uint8 *)&simUartTxFifoReg)
static const void *simDmaPtrs[8]; static unsigned simDmaPtrN;
static uint16 sim_dma_addr(const void *p){ if(simDmaPtrN>=8) simDmaPtrN=0; simDmaPtrs[simDmaPtrN]=p; return (uint16)simDmaPtrN++; }
#define TXDMA_ADDR(p) sim_dma_addr((const void *)(p))
static unsigned simDmaLen, simDmaLogged, simDmaTransfers, simDmaBytes, simDmaInit, simDmaByteCy; static const uint8 *simDmaSrc;
static uint64_t simDmaStart; static int simDmaActive, simDmaHoldDrq;
static void sim_dma_progress(void){
    uint64_t n;
    if(!simDmaEnabled) return;
    if(!simDmaActive) {
        if(simDmaHoldDrq) return;
        simDmaActive=1;simDmaStart=simClock;
    }
    n=simDmaByteCy?(simClock-simDmaStart)/simDmaByteCy:simDmaLen;
    if(n>simDmaLen) n=simDmaLen;
    while(simDmaLogged<n){ mock_log(simDmaSrc[simDmaLogged++]); simDmaBytes++; }
    if(simDmaLogged>=simDmaLen) {simDmaActive=0;simDmaEnabled=0;}
}
static uint8 TxDma_DmaInitialize(uint8 b,uint8 r,uint16 hs,uint16 hd){(void)b;(void)r;(void)hs;(void)hd;simDmaInit=1;return 1;}
static uint8 CyDmaTdAllocate(void){return 3;}
static cystatus CyDmaTdSetConfiguration(uint8 td,uint16 n,uint8 next,uint8 cfg){(void)td;assert(next==CY_DMA_DISABLE_TD && cfg==CY_DMA_TD_INC_SRC_ADR && n>=1u && n<=4095u);simDmaLen=n;return 0;}
static cystatus CyDmaTdSetAddress(uint8 td,uint16 src,uint16 dst){(void)td;(void)dst;simDmaSrc=(const uint8 *)simDmaPtrs[src];return 0;}
static cystatus CyDmaChSetInitialTd(uint8 ch,uint8 td){(void)ch;(void)td;return 0;}
static cystatus CyDmaChEnable(uint8 ch,uint8 keep){(void)ch;(void)keep;assert(simDmaInit && !simDmaEnabled);
    simDmaActive=0;simDmaEnabled=1;simDmaLogged=0;simDmaTransfers++;sim_dma_progress();return 0;}
static cystatus CyDmaChDisable(uint8 ch){(void)ch;simDmaActive=0;simDmaEnabled=0;return 0;}
static cystatus CyDmaChStatus(uint8 ch,uint8 *td,uint8 *state){(void)ch;*td=0;simClock+=simRegCost;
    sim_dma_progress();
    *state=simDmaActive?CY_DMA_STATUS_CHAIN_ACTIVE:0u;return 0;}
static uint32 mock_frame(void) {
    uint32 angle, frame, bits, parity=0;
    if(simBadEncoder) return 0;
    if(simBadReads>0){simBadReads--;return 0;}
    angle=(uint32)(1000+(int64)simStep1*simPolarity*256/100+simEncoderBias)&4095u;
    if(simAngleOverride>=0)angle=(uint32)simAngleOverride;
    frame=(angle<<6)|0x26u;
    for(bits=frame; bits; bits>>=1) parity^=bits&1;
    return frame|parity;
}
/* AS5045 at pin level: CSn falling latches a frame, every CLK rising edge while CSn is low presents the next bit, MSB first. */
static uint32 simSsiFrame, simSsiFrame2; static int simSsiBits;
static int simSsiSecondFrame=-1, simSsiDummy;
/* DO after n CLK rising edges (n = 0 right after CSn fell); the daisy chain is board 1, a dummy, board 2, a dummy. */
static int sim_ssi_do(int n) {
    if(n>=20 && n<=37) return ((((simSsiSecondFrame>=0)?(uint32)simSsiSecondFrame:simSsiFrame2)>>(37-n))&1u)?1:0;
    if(simSsiSecondFrame>=0 && (n==19 || n==38)) return simSsiDummy?1:0;
    return (n>=1 && n<=18 && ((simSsiFrame>>(18-n))&1u))?1:0;
}
static int mock_read(int pin) {
    if(pin==CYREG_PRT1_PC3) return sim_ssi_do(simSsiBits)?16:0;
    if(pin==CYREG_PRT12_PC5) return simFault?0:16;
    if(pin==CYREG_PRT0_PC4) return simLimit?16:0;
    if(pin==CYREG_PRT0_PC3) return (simHome1Mode<0?0:(simHome1Mode>0?1:simStep1>=simHome1At))?16:0;
    if(pin==CYREG_PRT1_PC0) return (simHome2Mode<0?0:(simHome2Mode>0?1:simStep2>=simHome2At))?0:16;
    return simPins[pin];
}
/* CSn fell: both boards latch their angle. */
static void sim_ssi_latch(void) {
    uint32 bits, parity=0;
    simSsiFrame=mock_frame();
    simSsiFrame2=(((uint32)(1000+(int64)simStep2*simPolarity2*256/100+simEncoderBias2)&4095u)<<6)|0x26u;
    for(bits=simSsiFrame2;bits;bits>>=1)parity^=bits&1;
    simSsiFrame2=simBadEncoder2?0:simSsiFrame2|parity;
}
/* UDB SPI Master "Ssi": 10-bit words, 250 kHz (4 us a bit, 40 us a word), hardware SS 2 us before the first clock.  The
 * first word written to an idle master drops SS (the boards latch); later words follow back to back.  Stream bit j
 * is DO at the j-th sampling edge: a 1 right after CSn falls, then the daisy chain.  Word k is in the receive FIFO
 * when it has been shifted completely. */
static uint64_t simSsiT0; static unsigned simSsiQueued, simSsiRead, simSsiStarted, simSsiFrames;
static unsigned simSsiDead; /* the master never finishes a frame */
static unsigned sim_ssi_complete(void) {
    uint64_t elapsed; unsigned n;
    if(simSsiDead) return simSsiRead; /* a stuck master finishes nothing more */
    if(!simSsiQueued || simClock<simSsiT0) return 0;
    elapsed=simClock-simSsiT0; n=(unsigned)(elapsed/((uint64_t)SIM_MHZ*40u));
    return n>simSsiQueued?simSsiQueued:n;
}
static void Ssi_Start(void){simSsiStarted=1;}
static void Ssi_WriteTxData(uint16 w){
    (void)w;
    if(!simSsiQueued || (sim_ssi_complete()==simSsiQueued && simSsiRead==simSsiQueued)) {
        sim_ssi_latch();simSsiQueued=0;simSsiRead=0;simSsiT0=simClock+(uint64_t)SIM_MHZ*2u;simSsiFrames++;
    }
    assert(simSsiQueued<4); /* the TX FIFO is four words deep */
    simSsiQueued++;
}
/* Creator's hardware FIFO API reports only empty/nonempty, never the word count. */
static unsigned Ssi_GetRxBufferSize(void){return sim_ssi_complete()>simSsiRead?1u:0u;}
static uint16 Ssi_ReadRxData(void){
    uint16 w=0; unsigned j;
    assert(simSsiRead<sim_ssi_complete());
    for(j=simSsiRead*10u;j<simSsiRead*10u+10u;j++) w=(uint16)((w<<1)|(j==0?1:sim_ssi_do((int)j)));
    simSsiRead++;return w;
}
static void Ssi_ClearRxBuffer(void){simSsiRead=sim_ssi_complete();}
static void Ssi_ClearTxBuffer(void){simSsiQueued=sim_ssi_complete();if(simSsiRead>simSsiQueued)simSsiRead=simSsiQueued;}
/* EvtTimer (fixed-function one-shot) + EvtRestart (PulseMode reset) + EvtIsr: WritePeriod loads the Period register;
 * a restart pulse reloads the counter from it, and the terminal count comes (Period + 1 + lag) us later.  The lag is
 * the restart latency the firmware compensates (EVT_RESTART_LAG_US).  The interrupt is taken at the first moment
 * simulated time moves after the terminal count, with interrupts enabled, and never inside itself. */
#define SIM_EVT_LAG_US 3u
static uint64_t simEvtDue, simEvtLateMaxCy; static int simEvtArmed, simEvtInIsr, simEvtIsrOn, simEvtPendingIrq;
static void (*simEvtHandler)(void);
static unsigned simEvtPeriod, simEvtStatusTc, simEvtStarted, simEvtRestarts, simEvtIsrCount;
static void EvtTimer_Start(void){simEvtStarted=1;}
static void EvtTimer_WritePeriod(uint16 p){ assert(p>=1u); simEvtPeriod=p; }
static uint8 EvtTimer_ReadStatusRegister(void){ uint8 s=simEvtStatusTc?1u:0u; simEvtStatusTc=0u; return s; }
static void EvtRestart_Write(uint8 v){
    if(v&1u){ simEvtArmed=1; simEvtDue=simClock+(uint64_t)(simEvtPeriod+1u+SIM_EVT_LAG_US)*SIM_MHZ; simEvtRestarts++; }
}
static void EvtIsr_StartEx(void (*h)(void)){ simEvtHandler=h; }
static void EvtIsr_SetPriority(uint8 prio){ (void)prio; }
static void EvtIsr_Enable(void){ simEvtIsrOn=1; }
static void EvtIsr_Disable(void){ simEvtIsrOn=0; }
static void EvtIsr_ClearPending(void){ simEvtPendingIrq=0; }
/* Pin interrupts: PinIsr fires on nFAULT falling (simFault 0 -> 1) and P0[4] rising (simLimit 0 -> 1); StepKill is the
 * Control_Reg wired into the PWMs' kill input (1 = no pulses leave the chip). */
static void (*simPinHandler)(void); static int simPinIrqPrio=-1, simPinPrevFault, simPinPrevLimit, simStepKill, simPinInIsr;
static unsigned simPinIrqCount;
static uint8 FAULT_ClearInterrupt(void){return 0;}
static uint8 LIMIT2_ClearInterrupt(void){return 0;}
static void StepKill_Write(uint8 v){simStepKill=v&1;}
static void FaultIsr_StartEx(void (*h)(void)){simPinHandler=h;}
static void LimitIsr_StartEx(void (*h)(void)){simPinHandler=h;}
static void FaultIsr_SetPriority(uint8 prio){simPinIrqPrio=prio;}
static void LimitIsr_SetPriority(uint8 prio){simPinIrqPrio=prio;}
static void sim_pin_service(void) {
    int edge=(simFault && !simPinPrevFault) || (simLimit && !simPinPrevLimit);
    if(simInts && !simPinInIsr) { simPinPrevFault=simFault; simPinPrevLimit=simLimit; }
    if(edge && simInts && !simPinInIsr && simPinHandler) { simPinInIsr=1; simPinIrqCount++; simPinHandler(); simPinInIsr=0; }
}
static void sim_evt_service(void) {
    sim_pin_service();
    if(simEvtArmed && simClock>=simEvtDue) {
        simEvtArmed=0; simEvtStatusTc=1u; simEvtPendingIrq=1;
        if(simClock-simEvtDue>simEvtLateMaxCy) simEvtLateMaxCy=simClock-simEvtDue;
    }
    while(simEvtPendingIrq && simEvtIsrOn && simInts && !simEvtInIsr && simEvtHandler) {
        simEvtPendingIrq=0; simEvtInIsr=1; simEvtIsrCount++; simEvtHandler(); simEvtInIsr=0;
    }
}
/* AliveReq (PulseMode) resets DeadTimer; 100 ms without a write is a terminal count, which latches a kill on both STEP
 * PWMs until they are restarted (StepArm*_Start). */
#define DeadTimer_STATUS_TC 1u
#define SIM_DEAD_MS 100u
static uint64_t simDeadAt; static int simDeadStarted, simDeadTripped, simDeadKilled; static unsigned simAliveKicks;
static void sim_dead_check(void){ if(simDeadStarted && !simDeadKilled && simClock>simDeadAt){ simDeadKilled=1; simDeadTripped=1; } }
static void AliveReq_Write(uint8 v){ if(v&1u){ sim_dead_check(); if(!simDeadKilled) simDeadAt=simClock+(uint64_t)SIM_DEAD_MS*SIM_MHZ*1000u; simAliveKicks++; } }
static void DeadTimer_Start(void){ simDeadStarted=1; simDeadAt=simClock+(uint64_t)SIM_DEAD_MS*SIM_MHZ*1000u; }
static uint8 DeadTimer_ReadStatusRegister(void){ uint8 s; sim_dead_check(); s=simDeadTripped?DeadTimer_STATUS_TC:0u; simDeadTripped=0; return s; }
/* One STEP pulse reaching a driver that is awake and enabled: the arm's motor moves one microstep along DIR.  The
 * bit-banged path calls these on the pin edge that ends the pulse, the PWM path when the pulse is requested. */
static void sim_pulse_arm1(void) {
    sim_dead_check(); if(simDeadKilled || simStepKill) return;
    if(!simPins[CYREG_PRT1_PC7] && simPins[CYREG_PRT12_PC2] && simPins[CYREG_PRT1_PC2]) {
        int tracks=!simRateLimited || simClock-simLastMotorPulse>=SIM_MHZ*1250u;
        simLastMotorPulse=simClock; /* fixture motor tracks at most the configured 800 ustep/s jog rate */
        simPulse1++; simTrace=(simTrace^(uint32)((simClock-simTraceBase)*24u/SIM_MHZ))*16777619u; simTrace=(simTrace^(simPins[CYREG_PRT12_PC4]?1u:2u))*16777619u;
        if(tracks && !simStall && !(simStiction && simPulse1<=24)) simStep1+=simPins[CYREG_PRT12_PC4]?1:-1;
    }
}
static void sim_pulse_arm2(void) {
    sim_dead_check(); if(simDeadKilled || simStepKill) return;
    if(!simPins[CYREG_PRT2_PC7] && simPins[CYREG_PRT2_PC2] && simPins[CYREG_PRT2_PC3]) {
        simPulse2++; simTrace=(simTrace^(uint32)((simClock-simTraceBase)*24u/SIM_MHZ))*16777619u; simTrace=(simTrace^(simPins[CYREG_PRT2_PC0]?3u:4u))*16777619u;
        if(!simStall2)simStep2+=simPins[CYREG_PRT2_PC0]?1:-1;
    }
}
/* Control_Regs "StepArm1Req" / "StepArm2Req" (PulseMode, one bit each) + one-shot PWMs "StepArm1" / "StepArm2": writing 1
 * requests one pulse.  A write of 0 requests nothing. */
static unsigned simStepStarted, simStepWrites1, simStepWrites2;
static void StepArm1_Start(void){simStepStarted|=1u; simDeadKilled=0;}
static void StepArm2_Start(void){simStepStarted|=2u; simDeadKilled=0;}
static void StepArm1_Stop(void){simStepStarted&=~1u;}
static void StepArm2_Stop(void){simStepStarted&=~2u;}
static void StepRearm_Write(uint8 v){ if(v&1u) simDeadKilled=0; }   /* reset input of both PWMs: clears a latched kill */
static void StepArm1Req_Write(uint8 v){ if(v&1u){simStepWrites1++; sim_pulse_arm1();} }
static void StepArm2Req_Write(uint8 v){ if(v&1u){simStepWrites2++; sim_pulse_arm2();} }
static void mock_set(int pin,int value) {
    if(pin==CYREG_PRT12_PC1 && value!=simPins[pin]) {
        if(!value){ sim_ssi_latch();simSsiBits=0;simRegCost=SIM_MHZ/4u; } else simRegCost=SIM_MHZ;
    }
    if(pin==CYREG_PRT12_PC0 && value && !simPins[pin] && !simPins[CYREG_PRT12_PC1])simSsiBits++;
    if(value!=simPins[pin] && (pin==CYREG_PRT1_PC7 || pin==CYREG_PRT12_PC2 || pin==CYREG_PRT1_PC2 ||
        pin==CYREG_PRT2_PC7 || pin==CYREG_PRT2_PC2 || pin==CYREG_PRT2_PC3))simDriverSwitches++;
    if(value && !simPins[pin]) {
        if(pin==CYREG_PRT12_PC3) sim_pulse_arm1();
        if(pin==CYREG_PRT2_PC1) sim_pulse_arm2();
    }
    simPins[pin]=value;
}
#define CY_GET_REG32(p) mock_reg(p)
#define CY_SET_REG32(p,v) mock_reg_set(p,v)
#define CY_SET_REG8(p,v) mock_set((p),((v)&1))
#define CyPins_ReadPin(p) mock_read(p)
#define CyPins_SetPin(p) mock_set((p),1)
#define CyPins_ClearPin(p) mock_set((p),0)
#define CyEnterCriticalSection() mock_critical_enter()
#define CyExitCriticalSection(x) mock_critical_exit(x)
static void CyDelay(uint32 ms) { simClock+=ms*(SIM_MHZ*1000u); }
'''

harness = r'''
#undef assert
#define assert(ok) do { if(!(ok)) { fprintf(stderr,"check: %s line %d; encLost=%u encGap=%lu\n%s\n", \
    #ok,__LINE__,g_enc.trackingLost,(unsigned long)g_encMaxGapMs,simLog+(simLogN>1600?simLogN-1600:0)); abort(); } } while(0)
/* A wire that takes nothing more: the UART's TX FIFO stays full / the DMA channel never finishes. */
#if HW_TXDMA
#define SIM_TX_STUCK(on) (simDmaByteCy=(on)?SIM_MHZ*1000000u:0u)
#else
#define SIM_TX_STUCK(on) (simUartTxFull=(on))
#endif
/* Clocks of the last AS5045 frame: counted CLK edges for the bit-banged read, 10 per word for the SPI Master. */
#if HW_SSI
#define SIM_SSI_LAST_CLOCKS() (simSsiQueued*10u)
#define SIM_SSI_CLK18 20u
#define SIM_SSI_CLK38 40u
#else
#define SIM_SSI_LAST_CLOCKS() ((unsigned)simSsiBits)
#define SIM_SSI_CLK18 18u
#define SIM_SSI_CLK38 38u
#endif
static void sim_keys(uint8 mask);
static uint32 simControlSequence;
static void sim_uart_heartbeat(void) {
    uint8 data[17]={6},i;uint16 crc;
    if(simControlSequence<g_controlSequence)simControlSequence=g_controlSequence;
    simControlSequence++;
    for(i=0;i<4;i++){data[1+i]=(uint8)(simControlSequence>>(8*i));data[5+i]=(uint8)(g_ptp.epoch>>(8*i));}
    crc=ptp_crc(data,15);data[15]=(uint8)crc;data[16]=(uint8)(crc>>8);
    sim_uart_push(PTP_FRAME_START);
    for(i=0;i<34;i++)sim_uart_push((uint8)(0x81u|((i&1u)<<5)|(((data[i/2]>>((i&1)*4))&15u)<<1)));
    sim_uart_push(PTP_FRAME_END);
}
static void setup(void) {
    hw_init();
    assert(enc_set_origin());
    jog_axis_init(&g_arm1,"arm1",PIN_STEP2,PIN_DIR2,1,1,ARM1_LEFT_DIR_LEVEL,800,200,20000,40000);
    jog_axis_init(&g_arm2,"arm2",PIN_STEP1,PIN_DIR1,0,0,ARM2_LEFT_DIR_LEVEL,600,200,12000,24000);
    sim_keys(0);
    stage_enable();
    assert(g_motionEnabled && !simPulse1 && !simPulse2 && !g_homed1 && !g_homed2);
}
static void status_drain(void) { /* Test callers that need a completed idle status block. */
    assert(!status_moving());
    while(g_statusNext){stage_status_service();txq_flush_blocking();}
    txq_flush_blocking();
}
static void sim_check_traj_boundary(void) {
    const traj_node_t *node=&g_traj.nodes[g_traj.index];
    assert(g_arm1.usteps==g_ptp.origin[0]-ptp_offset(0,node->q1));
    assert(g_arm2.usteps==g_ptp.origin[1]-ptp_offset(1,node->q2-node->q1));
    simBoundaryChecks++;
}
/* Let `us` microseconds pass in 20 us steps, each ending in a register read, so that an interrupt due in between is
 * taken about when it is due and not at the end of the whole span. */
static void sim_advance_us(unsigned us) {
    unsigned q;
    for(q=0;q<us/20u;q++){simClock+=(uint64_t)SIM_MHZ*20u;(void)SU_NOW();}
}
/* A traj run is advanced in 100 us slices (1 ms in all); with the event timer the slices are 20 us, because the
 * interrupt is taken at the first register read after its terminal count. */
#if HW_EVT
#define SIM_SUBS 50u
#define SIM_SUB_US 20u
#else
#define SIM_SUBS 10u
#define SIM_SUB_US 100u
#endif
static void cycle(int heartbeat) {
    if(g_traj.state==3u && g_ptp.active==1u) {
        /* The real foreground services pulses continuously, independently of
         * its 1 ms housekeeping tick. A single service per mock millisecond
         * falsely caps 1667 us pulse spacing at 500 Hz. Preserve elapsed time,
         * but service at 100 us resolution for the newly uncapped host plans. */
        unsigned sub;
        for(sub=0;sub<SIM_SUBS;sub++) {
            simClock+=SIM_MHZ*SIM_SUB_US;FG_PASS_MARK();alive_kick();jog_update_time();
            if(!jog_safety_blocked() && g_ptp.active)ptp_service();
        }
    } else simClock+=(SIM_MHZ*1000u);
    FG_PASS_MARK();alive_kick();
    jog_update_time();
    if(heartbeat && (g_jogMs-g_lastKeyMs>=40u)) sim_keys(0);
#if HW_SSI
    /* the hardware frame takes 160 us: one pass starts it, later passes (200 us apart) collect it, retries included */
    if(g_arm1.hasEnc){unsigned k;service_encoder_sampling();for(k=0;k<8 && g_encFlight;k++){sim_advance_us(200u);service_encoder_sampling();}}
#else
    if(g_arm1.hasEnc)service_encoder();
#endif
    jog_tick();
    service_motion();
    stage_status_service();
    txq_flush_blocking();
}
/* The real foreground loop for 2 s of simulated time. init=1 also runs the firmware's own axis set-up and banner;
 * init=0 keeps the axes and enable state that setup() prepared (used when a wire image is already loaded). */
static void sim_run(int init) {
    if(init)app_init(1);
    app_start();
    if(!init)simRxBase=simClock;
    while(simClock-simRxBase<(SIM_MHZ*1000000u)*2u)app_poll();
}
static void run_until(home_phase_t phase, int arm) {
    unsigned i;
    for(i=0;i<250000 && g_motionEnabled;i++) {
        if(g_home.phase==phase && g_home.arm==arm) return;
        cycle(1);
    }
    fprintf(stderr,"missing phase %d axis %d; logs:\n%s",phase,arm,simLog);
    assert(0);
}
static void assert_stopped(void) {
    unsigned p1=simPulse1,p2=simPulse2,i;
    assert(!g_motionEnabled && g_home.phase==HOME_IDLE && !g_homed1 && !g_homed2);
    for(i=0;i<400;i++) cycle(1);
    assert(simPulse1==p1 && simPulse2==p2);
}
static uint8 packetType=1u;
static void ptp_send(uint32 seq, uint32 epoch, int16 q1, int16 q2, uint16 rate, int mode) {
    uint8 data[17]={1}, i, copy, b; uint16 crc;
    data[0]=packetType;
    for(i=0;i<4;i++){data[1+i]=(uint8)(seq>>(8*i));data[5+i]=(uint8)(epoch>>(8*i));}
    data[9]=(uint8)q1;data[10]=(uint8)((uint16)q1>>8);
    data[11]=(uint8)q2;data[12]=(uint8)((uint16)q2>>8);
    data[13]=(uint8)rate;data[14]=(uint8)(rate>>8);
    crc=ptp_crc(data,15);data[15]=(uint8)crc;data[16]=(uint8)(crc>>8);
    if(mode==1)data[9]^=1; /* CRC corruption */
    if(mode==4) {simPtpWireN=0;simPtpWire[simPtpWireN++]=PTP_FRAME_START;}
    else for(copy=0;copy<3;copy++)jog_handle_byte(PTP_FRAME_START);
    for(i=0;i<34;i++) {
        if(mode==2&&i==10)continue; /* dropped entire nibble group */
        if(mode==3&&i==12)return; /* incomplete frame */
        b=(uint8)(0x81u|((i&1u)<<5)|(((data[i/2]>>((i&1)*4))&15u)<<1));
        if(mode==4){simPtpWire[simPtpWireN++]=b;continue;}
        for(copy=0;copy<3;copy++)jog_handle_byte(b);
        if(i%8==0 && mode!=5)jog_handle_byte(0x41); /* mode 5 intentionally has no heartbeat */
    }
    if(mode==4)simPtpWire[simPtpWireN++]=PTP_FRAME_END;
    else for(copy=0;copy<3;copy++)jog_handle_byte(PTP_FRAME_END);
}
static void sim_keys(uint8 mask) {
    uint8 saved=packetType;
    packetType=6;ptp_send(g_controlSequence+1u,g_ptp.epoch,0,0,mask,0);packetType=saved;
}
/* Compact heartbeat: D5 <lo/hi keys> <lo/hi crc8> D7, three wire copies each. */
static void short_send(uint8 keys, int corrupt) {
    uint8 data[2]={0}, i, b, copy;
    data[0]=keys & 15u;
    data[1]=ptp_crc8(data,1u);
    if(corrupt)data[1]^=1u;
    for(copy=0;copy<3;copy++)jog_handle_byte(PTP_SHORT_START);
    for(i=0;i<4;i++) {
        b=(uint8)(0x81u|((i&1u)<<5)|(((data[i/2]>>((i&1)*4))&15u)<<1));
        for(copy=0;copy<3;copy++)jog_handle_byte(b);
    }
    for(copy=0;copy<3;copy++)jog_handle_byte(PTP_SHORT_END);
}
static void limits_send(uint32 seq,uint32 epoch,uint16 axis,int16 min,int16 max) {
    packetType=9;ptp_send(seq,epoch,min,max,axis,5);packetType=1;txq_flush_blocking();
}
static void telem_test(const char *t) {
    uint32 seq,epoch,sent,skipped,dropped,started,elapsed; unsigned i,lines; const char *at;
    static const uint16 badHz[]={1,4,21,100,65535};
    /* The streamed TELEM line (control type 11) and its protections. setup() leaves the stage enabled. */
    seq=g_controlSequence+1u;epoch=g_ptp.epoch;
    if(!strcmp(t,"telem_rate_invalid")) {
        packetType=11;
        for(i=0;i<sizeof(badHz)/sizeof(badHz[0]);i++)ptp_send(seq,epoch,0,0,badHz[i],5);
        ptp_send(seq,epoch,1,0,10,5);ptp_send(seq,epoch,0,1,10,5);   /* the angle fields must be zero */
        ptp_send(0,epoch,0,0,10,5);ptp_send(seq,epoch-1u,0,0,10,5);  /* sequence 0, stale epoch */
        ptp_send(seq,epoch,0,0,10,1);                                 /* corrupted CRC */
        packetType=1;
        assert(g_telemHz==0 && g_controlSequence<seq && g_telemSent==0);
        return;
    }
    if(!strcmp(t,"telem_rate")) {
        packetType=11;ptp_send(seq,epoch,0,0,10,5);packetType=1;txq_flush_blocking();
        assert(g_telemHz==10 && g_controlSequence==seq && strstr(simLog,"TELEM_RATE epoch=") && strstr(simLog,"hz=10"));
        simLogN=0;simLog[0]=0;packetType=11;ptp_send(seq,epoch,0,0,10,5);packetType=1;txq_flush_blocking();
        assert(g_controlSequence==seq && strstr(simLog,"TELEM_RATE epoch=") && strstr(simLog,"hz=10")); /* an exact retry only repeats the receipt */
        packetType=11;ptp_send(seq,epoch,0,0,20,5);packetType=1;
        assert(g_telemHz==10);                                         /* the same sequence with another rate is stale */
        packetType=11;ptp_send(g_controlSequence+1u,epoch,0,0,0,5);packetType=1;
        assert(g_telemHz==0);                                          /* 0 switches the stream off */
        simLogN=0;simLog[0]=0;stage_status();status_drain();
        assert(strstr(simLog," telem_hz=0 proto=") && strstr(simLog," traj_repeat=1") && strstr(simLog," fault_snapshot=1") && strstr(simLog,"TELEM_DIAG hz=0 sent=0"));
        return;
    }
    if(!strcmp(t,"telem_stream")) {
        /* Off: only the 500 ms status block carries TELEM, as before. */
        simLogN=0;simLog[0]=0;
        for(i=0;i<1000;i++)cycle(1);
        assert(g_motionEnabled && g_telemSent==0);
        for(lines=0,at=simLog;(at=strstr(at,"TELEM t="))!=NULL;at++)lines++;
        assert(lines>=1 && lines<=3);
        /* 20 Hz while enabled: one standalone line per 50 ms of device time, on top of the status blocks, with nothing dropped.
         * (cycle() also pays for the heartbeat frames, so count against the device clock, not the loop count.) */
        packetType=11;ptp_send(g_controlSequence+1u,epoch,0,0,20,5);packetType=1;
        sent=g_telemSent;dropped=g_txqDropped;started=g_jogMs;simLogN=0;simLog[0]=0;
        for(i=0;i<1000;i++)cycle(1);
        elapsed=g_jogMs-started;
        assert(g_motionEnabled && elapsed>=1000u && elapsed<=1400u && g_txqDropped==dropped && g_telemSkipped==0);
        assert(g_telemSent-sent+1u>=elapsed/50u && g_telemSent-sent<=elapsed/50u+1u);
        for(lines=0,at=simLog;(at=strstr(at,"TELEM t="))!=NULL;at++)lines++;
        assert(lines>=g_telemSent-sent && lines<=g_telemSent-sent+3);
        /* The slowest setting, 5 Hz. */
        packetType=11;ptp_send(g_controlSequence+1u,epoch,0,0,5,5);packetType=1;
        sent=g_telemSent;started=g_jogMs;
        for(i=0;i<1000;i++)cycle(1);
        elapsed=g_jogMs-started;
        assert(g_telemSent-sent+1u>=elapsed/200u && g_telemSent-sent<=elapsed/200u+1u);
        /* Disabled: no stream. */
        jog_handle_byte(JOG_CMD_STOP);short_send(0,0);txq_flush_blocking();
        sent=g_telemSent;g_telemLastMs=g_jogMs-1000u;telem_service();
        assert(!g_motionEnabled && g_telemSent==sent);
        return;
    }
    if(!strcmp(t,"telem_skip")) {
        /* A nearly full transmit queue skips the stream line (counted) instead of dropping anything. */
        char filler[101];memset(filler,'x',100);filler[100]=0;
        g_telemHz=20;g_telemLastMs=g_jogMs-100u;
        while(txq_free()>=TELEM_MIN_FREE_BYTES)txq_puts(filler);
        sent=g_telemSent;skipped=g_telemSkipped;dropped=g_txqDropped;
        telem_service();
        assert(g_telemSkipped==skipped+1 && g_telemSent==sent && g_txqDropped==dropped);
        txq_flush_blocking();
        g_telemLastMs=g_jogMs-100u;telem_service();
        assert(g_telemSent==sent+1);
        return;
    }
    assert(!"unknown telem test");
}
static void fault_snapshot_test(void) {
    /* One structured line per stop, printed BEFORE the stop resets anything. */
    int32 beforeP1;
    simLogN=0;simLog[0]=0;
    g_ptp.origin[0]=100;g_ptp.origin[1]=200;g_arm1.usteps=40;g_arm2.usteps=50;
    fault_snapshot("fixture protection",0u);txq_flush_blocking();
    assert(strstr(simLog,"FAULT_SNAPSHOT t=") && strstr(simLog," phase=idle ") && strstr(simLog," p1=60 p2=150 ") &&
           strstr(simLog,"reason=\"fixture protection\"") && strstr(simLog," drv=ok ") && strstr(simLog," hb_age_ms=") && strstr(simLog," inputs=0 "));
    beforeP1=g_ptp.origin[0]-g_arm1.usteps;
    simLogN=0;simLog[0]=0;
    stage_stop("fixture protection",0u);txq_flush_blocking();
    assert(beforeP1==60 && strstr(simLog,"FAULT_SNAPSHOT") && strstr(simLog,"STOP: fixture protection"));
    assert(strstr(simLog,"FAULT_SNAPSHOT")<strstr(simLog,"STOP context")); /* before any reset */
    assert(strstr(simLog," phase=idle ")); /* the phase is the one the machine was in, not "disabled" after the stop */
    /* The phase names. */
    g_motionEnabled=1u;
    { static const struct { const char *name; } phases[]={{"home"},{"ptp"},{"traj_run"},{"traj_upload"},{"jog"},{"idle"},{"disabled"}}; unsigned k;
      for(k=0;k<sizeof(phases)/sizeof(phases[0]);k++) {
        g_home.phase=HOME_IDLE;g_ptp.active=0u;g_traj.state=0u;g_arm1.moving=0;g_motionEnabled=1u;
        if(k==0)g_home.phase=HOME_FAST;
        if(k==1)g_ptp.active=1u;
        if(k==2){g_ptp.active=1u;g_traj.state=3u;}
        if(k==3)g_traj.state=1u;
        if(k==4)g_arm1.moving=1;
        if(k==6)g_motionEnabled=0u;
        simLogN=0;simLog[0]=0;fault_snapshot("x",0u);txq_flush_blocking();
        { char want[40]; sprintf(want," phase=%s ",phases[k].name); assert(strstr(simLog,want)); }
      } }
    g_home.phase=HOME_IDLE;g_ptp.active=0u;g_traj.state=0u;g_arm1.moving=0;
    /* A driver fault is never explained: the pin cannot tell over-current from over-temperature. */
    simLogN=0;simLog[0]=0;fault_snapshot("DRV8825 nFAULT; drivers OFF",1u);txq_flush_blocking();
    assert(strstr(simLog," drv=nfault_undetermined "));
    assert(!strstr(simLog,"thermal") && !strstr(simLog,"over-temp") && !strstr(simLog,"overheat"));
    simLogN=0;simLog[0]=0;simFault=1;fault_snapshot("fixture",0u);txq_flush_blocking();simFault=0;
    assert(strstr(simLog," drv=nfault_undetermined ") && strstr(simLog," inputs=8 "));
    /* An over-long reason is cut off by the debug buffer, never overflowed. */
    simLogN=0;simLog[0]=0;
    fault_snapshot("HOME direction: no credible rotor motion; check VMOT/motor/encoder; UART receive error/overflow; buffered commands discarded",0u);txq_flush_blocking();
    assert(strstr(simLog,"FAULT_SNAPSHOT") && strstr(simLog,"reason=\"HOME direction"));
}
static void rates_live_test(void);
static void rates_test(const char *t) {
    uint32 seq,epoch; unsigned i; uint32 before;
    if(!strcmp(t,"rates_live")) { rates_live_test(); return; }
    /* A held key means the axis is about to move: its speed must not change under it (mode 5: no neutral byte that would release it). */
    g_arm1.want=1;packetType=10;ptp_send(g_controlSequence+1u,g_ptp.epoch,200,500,0,5);packetType=1;g_arm1.want=0;
    assert(g_motionRate[0]==800);
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);txq_flush_blocking();
    seq=g_controlSequence+1u;epoch=g_ptp.epoch;
    packetType=10;
    ptp_send(seq,epoch,19,500,0,0);ptp_send(seq,epoch,CFG_RATE_MAX_AXIS1+1,500,0,0);
    ptp_send(seq,epoch,200,99,0,0);ptp_send(seq,epoch,200,20001,0,0);
    ptp_send(seq,epoch-1u,200,500,0,0);
    assert(g_motionRate[0]==800 && g_controlSequence<seq);
    ptp_send(seq,epoch,200,500,0,0);
    assert(g_motionRate[0]==200 && g_motionAccel[0]==500 && g_arm1.accel==500 && g_arm1.rateStart==20);
    packetType=1;sim_keys(0);packetType=10;
    ptp_send(seq,epoch,200,500,0,0); /* exact retry after heartbeat */
    ptp_send(seq,epoch,300,500,0,0);assert(g_motionRate[0]==200);
    ptp_send(g_controlSequence+1u,epoch,150,300,1,0);packetType=1;
    status_drain();assert(strstr(simLog,"PTP_RATES epoch=") && strstr(simLog,"motion_rates=1"));
    stage_enable(); assert(g_motionEnabled && g_motionRate[0]==200 && g_motionRate[1]==150);
    if(!strcmp(t,"rates_jog")) {
        g_arm2.want=1; jog_axis_tick(&g_arm2);before=g_arm2.rate;
        for(i=0;i<100;i++)jog_axis_tick(&g_arm2);
        assert(g_arm2.rate==before+30 && g_arm2.rate<=150);
        for(i=0;i<1000;i++)jog_axis_tick(&g_arm2);assert(g_arm2.rate==150);
        g_arm2.want=0;for(i=0;i<1000;i++)jog_axis_tick(&g_arm2);assert(!g_arm2.moving);
    } else if(!strcmp(t,"rates_home")) {
        g_home.arm=2;home_enter(HOME_FAST);
        assert(g_home.ramp.cMin==step_period_us(150) && g_home.ramp.c0>=55000);
        home_enter(HOME_SLOW);assert(g_home.ramp.cMin==step_period_us(150));
        /* The fast touch and the move to straight follow the global speed and acceleration, above the old fixed 600/800 too.
         * Release, backoff and the slow touch keep their fixed rates (600, 600, 400) and the 4000 acceleration cap; the global
         * values only lower them. */
        g_motionRate[0]=3000;g_motionAccel[0]=15000;g_motionRate[1]=1000;g_motionAccel[1]=3000;
        g_home.arm=1;
        home_enter(HOME_FAST);assert(g_home.ramp.cMin==step_period_us(3000) && g_home.ramp.c0==motion_min(600000u,956000u/isqrt32(15000u)));
        home_enter(HOME_STRAIGHT);assert(g_home.ramp.cMin==step_period_us(3000) && g_home.ramp.c0==motion_min(600000u,956000u/isqrt32(15000u)));
        home_enter(HOME_RELEASE);assert(g_home.ramp.cMin==step_period_us(HOME_RELEASE_RATE_USTEP_S) && g_home.ramp.c0==motion_min(600000u,956000u/isqrt32(HOME_ACCEL_USTEP_S2)));
        home_enter(HOME_BACKOFF);assert(g_home.ramp.cMin==step_period_us(HOME_RELEASE_RATE_USTEP_S) && g_home.ramp.c0==motion_min(600000u,956000u/isqrt32(HOME_ACCEL_USTEP_S2)));
        home_enter(HOME_SLOW);assert(g_home.ramp.cMin==step_period_us(HOME_SLOW_RATE_USTEP_S) && g_home.ramp.c0==motion_min(600000u,956000u/isqrt32(HOME_ACCEL_USTEP_S2)));
        g_home.arm=2;
        home_enter(HOME_FAST);assert(g_home.ramp.cMin==step_period_us(1000));
        home_enter(HOME_STRAIGHT);assert(g_home.ramp.cMin==step_period_us(1000) && g_home.ramp.c0==motion_min(600000u,956000u/isqrt32(3000u)));
        home_enter(HOME_RELEASE);assert(g_home.ramp.cMin==step_period_us(HOME_RELEASE_RATE_USTEP_S));
        g_motionRate[1]=300;
        home_enter(HOME_FAST);assert(g_home.ramp.cMin==step_period_us(300));
        home_enter(HOME_STRAIGHT);assert(g_home.ramp.cMin==step_period_us(300));
        home_enter(HOME_RELEASE);assert(g_home.ramp.cMin==step_period_us(300)); /* a lower global speed lowers release, backoff and the slow touch too */
        home_enter(HOME_SLOW);assert(g_home.ramp.cMin==step_period_us(300));
    } else if(!strcmp(t,"rates_traj") && PTP_COMMISSIONED) {
        /* Simulated calibration: board2 polarity was previously learned. */ g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);
        packetType=2;ptp_send(2,g_ptp.epoch,0,0,4,0);
        assert(g_traj.count==1);
        /* 200 pulses/s passes legacy 600/800 caps but exceeds configured 150. */
        packetType=3;ptp_send(3,g_ptp.epoch,0,44,500,0);
        assert(g_traj.count==1);
        /* 80 pulses/s passes rate, but 320 acceleration exceeds configured 300. */
        ptp_send(3,g_ptp.epoch,0,9,250,0);assert(g_traj.count==1);
        ptp_send(3,g_ptp.epoch,0,4,250,0);assert(g_traj.count==2);
        packetType=1;
    } else if(!strcmp(t,"rates_ptp") && PTP_COMMISSIONED) {
        /* Simulated calibration: board2 polarity was previously learned. */ g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;
        ptp_send(2,g_ptp.epoch,100,200,600,0); /* absolute q2: elbow = 100 cd, so both motors move and the axis-2 rate applies */
        assert(g_ptp.active && g_ptp.ramp.cMin==step_period_us(150) && g_ptp.ramp.c0>=55000);
    }
    (void)t;
}
static void limits_test(const char *t) {
    uint32 seq,epoch;unsigned i;
    if(!strcmp(t,"limits_enabled")) {
        limits_send(g_controlSequence+1u,g_ptp.epoch,0,-13500,13500);
        assert(!ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)));
        return;
    }
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);status_drain();
    seq=g_controlSequence+1u;epoch=g_ptp.epoch;
    if(!strcmp(t,"limits_invalid")) {
        const int16 bounds[][2]={{-18001,13500},{-13500,18001},{100,200},{-200,-100},{1,1},{100,-100}};
        for(i=0;i<sizeof(bounds)/sizeof(bounds[0]);i++)limits_send(seq,epoch,0,bounds[i][0],bounds[i][1]);
        limits_send(seq,epoch,3,-13500,13500);limits_send(0,epoch,0,-13500,13500);
        { const int16 elbow[][2]={{-18001,12000},{-9000,18001},{100,12000},{-9000,-100},{1,1},{100,-100}}; /* axis 2 = elbow: same rules as the boxes */
          for(i=0;i<sizeof(elbow)/sizeof(elbow[0]);i++)limits_send(seq,epoch,2,elbow[i][0],elbow[i][1]); }
        limits_send(seq,epoch-1u,0,-13500,13500);
        packetType=9;ptp_send(seq,epoch,-13500,13500,0,1);packetType=1;
        assert(g_controlSequence<seq && g_ptp.epoch==epoch && !simPulse1 && !simPulse2);
        assert(!ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)));
        return;
    }
    if(!strcmp(t,"limits_busy")) {
        for(i=0;i<8;i++) {
            if(i==0)g_home.phase=HOME_FAST;if(i==1)g_ptp.reference=1;if(i==2)g_ptp.active=1;
            if(i==3)g_traj.state=1;if(i==4)g_arm1.want=1;if(i==5)g_arm2.moving=1;
            if(i==6)g_arm1.settleMs=1;if(i==7)g_arm2.holding=1;
            limits_send(seq,epoch,0,-13500,13500);
            assert(g_controlSequence<seq && !ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)));
            g_home.phase=HOME_IDLE;g_ptp.reference=g_ptp.active=g_traj.state=0;
            g_arm1.want=g_arm2.moving=0;g_arm1.settleMs=0;g_arm2.holding=0;
        }
        return;
    }
    limits_send(seq,epoch,0,-13500,13500);
    assert(g_controlSequence==seq && g_ptp.epoch==epoch);
    assert(ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)));
    assert(!ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13501)));
    assert(strstr(simLog,"axis=0 min=-13500 max=13500") && strstr(simLog,"joint_limits=1"));
    if(!strcmp(t,"limits_retry")) {
        sim_keys(0);assert(g_controlSequence>seq);simLogN=0;simLog[0]=0;
        limits_send(seq,epoch,0,-13500,13500);
        assert(strstr(simLog,"PTP_LIMITS ") && strstr(simLog,"axis=0 min=-13500 max=13500"));
        simLogN=0;simLog[0]=0;limits_send(seq,epoch,0,-10000,10000);
        assert(!strstr(simLog,"PTP_LIMITS epoch=") && ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)));
    }
    limits_send(g_controlSequence+1u,epoch,1,-15000,15000);
    assert(strstr(simLog,"min1=-13500 max1=13500 min2=-15000 max2=15000"));
    if(!strncmp(t,"limits_elbow",12)) {
        /* Boot default first: straight is inside, +/-155 deg is the provisional bound. */
        assert(g_jointMinCd[JOINT_ELBOW]==-15500 && g_jointMaxCd[JOINT_ELBOW]==15500 && strstr(simLog,"emin=-15500 emax=15500"));
        simLogN=0;simLog[0]=0;limits_send(g_controlSequence+1u,epoch,JOINT_ELBOW,-9000,12000);
        assert(g_jointMinCd[JOINT_ELBOW]==-9000 && g_jointMaxCd[JOINT_ELBOW]==12000);
        assert(strstr(simLog,"PTP_LIMITS epoch=") && strstr(simLog,"axis=2 min=-9000 max=12000") && strstr(simLog,"emin=-9000 emax=12000"));
        assert(g_jointMinCd[0]==-13500 && g_jointMaxCd[1]==15000); /* the boxes did not move */
    }
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);stage_enable();txq_flush_blocking();
    assert(g_motionEnabled && !simPulse1 && !simPulse2);
    assert(ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)) && (!strncmp(t,"limits_elbow",12) || ptp_counts_in_range(1u,g_ptp.origin[1]-ptp_offset(1u,15000)))); /* the elbow cases narrow the elbow below q2's +150 */
    if(!strcmp(t,"limits_configure") || !strcmp(t,"limits_retry"))return;
    /* Simulated calibration: board2 polarity was previously learned. */ g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;
    assert(g_ptp.reference && g_homed1 && g_homed2);
    if(!strcmp(t,"limits_ptp")) {
        ptp_send(2,g_ptp.epoch,13501,0,600,0);assert(!g_ptp.active);
        packetType=2;ptp_send(2,g_ptp.epoch,13501,0,2,0);packetType=1;assert(!g_traj.state);
        ptp_send(2,g_ptp.epoch,13500,0,600,0);assert(g_ptp.active && g_ptp.seg.target[0]==g_ptp.origin[0]-ptp_offset(0u,13500));
        return;
    }
    if(!strcmp(t,"limits_elbow")) {
        /* q1 and q2 are inside their boxes in every refusal below: only the elbow range (-90..+120 deg) can say no. */
        ptp_send(2,g_ptp.epoch,0,12001,600,0);assert(!g_ptp.active);
        ptp_send(2,g_ptp.epoch,-3000,9001,600,0);assert(!g_ptp.active);   /* elbow +120.01 */
        ptp_send(2,g_ptp.epoch,3000,-6001,600,0);assert(!g_ptp.active);   /* elbow -90.01 */
        packetType=2;ptp_send(2,g_ptp.epoch,0,12001,2,0);packetType=1;assert(!g_traj.state);
        ptp_send(2,g_ptp.epoch,0,12000,600,0);
        assert(g_ptp.active && g_ptp.seg.target[1]==g_ptp.origin[1]-ptp_offset(1u,12000)); /* exactly on the bound is allowed */
        return;
    }
    if(!strcmp(t,"limits_elbow_traj")) {
        packetType=2;ptp_send(2,g_ptp.epoch,0,0,3,0);assert(g_traj.count==1);
        simLogN=0;simLog[0]=0;
        packetType=3;ptp_send(3,g_ptp.epoch,0,12001,250,0);txq_flush_blocking();
        assert(g_traj.count==1 && strstr(simLog,"joint range/rate")); /* refused for the range, not for rate or order */
        simLogN=0;simLog[0]=0;
        ptp_send(3,g_ptp.epoch,0,-9001,250,0);txq_flush_blocking();
        assert(g_traj.count==1 && strstr(simLog,"joint range/rate"));
        packetType=1;
        return;
    }
    if(!strcmp(t,"limits_elbow_jog")) {
        jog_axis_t *a=&g_arm2;int32 count;
        g_ptp.reference=0;simHome1Mode=simHome2Mode=-1;g_arm1.usteps=g_ptp.origin[0];
        /* q2 would still allow +15000 here, so the elbow bound alone stops the outward step. */
        a->usteps=g_ptp.origin[1]-ptp_offset(1u,12000);count=a->usteps;
        jog_step(a,-1);assert(a->usteps==count);jog_step(a,1);assert(a->usteps==count+1);
        a->usteps=g_ptp.origin[1]-ptp_offset(1u,-9000);count=a->usteps;
        jog_step(a,1);assert(a->usteps==count);jog_step(a,-1);assert(a->usteps==count-1);
        /* Outside the bound the axis may only come back; the position check agrees. */
        a->usteps=g_ptp.origin[1]-ptp_offset(1u,12500);
        assert(!ptp_counts_in_range(1u,a->usteps) && !ptp_counts_in_range(0u,g_arm1.usteps)); /* the elbow is shared by both checks */
        return;
    }
    assert(!strcmp(t,"limits_jog"));
    simHome1Mode=simHome2Mode=-1;
    for(i=0;i<2;i++) {
        jog_axis_t *a=i?&g_arm2:&g_arm1;int16 bound=i?15000:13500;int32 count;
        g_ptp.reference=0; /* jogging invalidates PTP readiness, not the calibrated joint zero */
        a->usteps=g_ptp.origin[i]-ptp_offset((uint8)i,bound);count=a->usteps;
        if(i) g_arm1.usteps=g_ptp.origin[0];
        jog_step(a,-1);assert(a->usteps==count);jog_step(a,1);assert(a->usteps==count+1);
        a->usteps=g_ptp.origin[i]-ptp_offset((uint8)i,-bound);count=a->usteps;
        jog_step(a,1);assert(a->usteps==count);jog_step(a,-1);assert(a->usteps==count-1);
    }
    /* Encoder feedback independently blocks outward jog even when command counts are inside. */
    g_arm2.usteps=g_ptp.origin[1];
    g_arm1.usteps=g_ptp.origin[0];g_enc.pos=g_arm1SoftCenter-out_deg10_to_lsb(1350);
    jog_step(&g_arm1,-1);assert(g_arm1.usteps==g_ptp.origin[0]);
    jog_step(&g_arm1,1);assert(g_arm1.usteps==g_ptp.origin[0]+1);
    g_enc.pos=g_arm1SoftCenter;g_arm1.usteps=g_ptp.origin[0]-ptp_offset(0u,13500);
    g_home.phase=HOME_FAST;jog_step(&g_arm1,-1);
    assert(g_arm1.usteps==g_ptp.origin[0]-ptp_offset(0u,13500)-1);g_home.phase=HOME_IDLE;
    /* Beyond the old 90-degree arm1 envelope, configured inward/outward jog remains available. */
    g_arm1.usteps=g_ptp.origin[0]-ptp_offset(0u,10000);
    simStep1=g_arm1.usteps;g_enc.lastAngle=(uint16)((1000+(int64)simStep1*256/100)&4095u);
    g_enc.pos=g_arm1SoftCenter-out_deg10_to_lsb(1000);cmd_reset_to_measured();g_arm1.want=-1;
    jog_axis_tick(&g_arm1);assert(g_arm1.moving==-1 && !g_enc.trackingLost);
}
static void ptp_test(const char *t) {
    unsigned i,p1,p2; int32 start1,start2; uint32 epoch;
    int16 target1=125,target2=-250;
    if(!strcmp(t,"ptp_zero")||!strcmp(t,"ptp_axis2"))target1=0;
    if(!strcmp(t,"ptp_zero")||!strcmp(t,"ptp_axis1"))target2=0;
    if(!strcmp(t,"ptp_unhomed")) {
        ptp_send(1,g_ptp.epoch,125,-250,200,0);assert(!g_ptp.active&&!simPulse1&&!simPulse2);return;
    }
    jog_home_all();
    for(i=0;i<250000&&g_home.phase!=HOME_IDLE&&g_motionEnabled;i++)cycle(1);
    assert(g_homed1&&g_homed2&&g_ptp.reference);cycle(1);sim_keys(0);
    start1=g_arm1.usteps;start2=g_arm2.usteps;p1=simPulse1;p2=simPulse2;epoch=g_ptp.epoch;
    if(!strncmp(t,"ptp_final_",10)) {
        int error=67;
        if(!strcmp(t,"ptp_final_negative"))error=-53;
        if(!strcmp(t,"ptp_final_boundary"))error=228;
        if(!strcmp(t,"ptp_final_over_limit"))error=229;
        if(!strcmp(t,"ptp_final_following"))error=328;
        ptp_send(1,epoch,-1437,2904,600,0);assert(g_ptp.active==1);
        for(i=0;i<24000&&g_ptp.active==1&&g_motionEnabled;i++)cycle(1); /* elbow now travels q2-q1 = 43.41 deg, about 17.2 s at 600 usteps/s */
        assert(g_motionEnabled&&g_ptp.active==2&&g_arm1.usteps==g_ptp.seg.target[0]&&g_arm2.usteps==g_ptp.seg.target[1]);
        simAngleOverride=(g_enc.lastAngle+cmd_lsb()-g_enc.pos-error+4096)&4095;
        if(!strcmp(t,"ptp_final_counts"))g_arm2.usteps--;
        for(i=0;i<100&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
        if(error>228 || !strcmp(t,"ptp_final_counts")) {
            assert_stopped();assert(!g_ptp.reference&&!g_ptp.active);
            assert(strstr(simLog,error==328?"PTP encoder following error":"PTP final position mismatch"));
        } else {
            if(!g_motionEnabled)fprintf(stderr,"final error=%ld expected accepted error=%d\n",(long)(cmd_lsb()-g_enc.pos),error);
            assert(g_motionEnabled&&g_ptp.reference&&!g_ptp.active&&g_ptp.epoch==epoch);
            assert(cmd_lsb()-g_enc.pos==error&&strstr(simLog,"PTP done seq=1"));
        }
        return;
    }
    if(!strcmp(t,"ptp_power_stable") || !strcmp(t,"ptp_fast_stall")) {
        unsigned switches=simDriverSwitches;uint32 started=g_jogMs;
        if(!strcmp(t,"ptp_fast_stall"))simStall=1;
        ptp_send(1,epoch,-1437,2904,600,0);
        assert(g_ptp.active);
        for(i=0;i<24000&&g_ptp.active&&g_motionEnabled;i++) {
            cycle(1);
            assert(simDriverSwitches==switches); // Completion and protective stops retain driver power.
        }
        if(!strcmp(t,"ptp_fast_stall")) {
            assert_stopped();txq_flush_blocking();assert(strstr(simLog,"PTP encoder following error"));return;
        }
        assert(g_motionEnabled&&!g_ptp.active&&g_ptp.reference);
        assert(g_jogMs-started<30000 /* coarse 1 ms fixture plus dual SSI reads, not pulse scheduler throughput */ && g_arm1.usteps==start1-ptp_offset(0,-1437) && g_arm2.usteps==start2-ptp_offset(1,2904-(-1437)));
        jog_handle_byte(JOG_CMD_STOP);assert_stopped();
        assert(simDriverSwitches==switches+6u && simPins[PIN_EN1] && simPins[PIN_EN2]);
        puts("PTP 600: power stable through completion; operator STOP releases both drivers");return;
    }
    assert(ptp_crc((const uint8*)"123456789",9)==0x29b1u);
    if(!strcmp(t,"ptp_telemetry")) {
        char expected[170];
        assert(g_ptp.referenceSource==1u && g_ptp.encoderOrigin==g_enc.pos);
        ptp_send(1,epoch,-100,100,200,0);
        for(i=0;i<5000 && g_ptp.active;i++)cycle(1);   /* 212 steps at 200 steps/s: just over 1 s without the busy-waits */
        assert(!g_ptp.active);
        stage_status();for(i=0;i<100u && g_statusNext;i++)cycle(1);txq_flush_blocking();
        sprintf(expected,"p1=%ld p2=%ld e1=%ld err=%ld e2=%ld err2=%ld inputs=0 drops=0",
            (long)(g_ptp.origin[0]-g_arm1.usteps),(long)(g_ptp.origin[1]-g_arm2.usteps),
            (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),(long)(cmd_lsb()-g_enc.pos),
            (long)((g_enc2.pos-g_enc2.origin)*(g_enc2.incDir==ARM2_LEFT_DIR_LEVEL?1:-1)),(long)(g_enc2.cmdLsbX100/100-g_enc2.pos));
        if(!ARM2_ENCODER_FEEDBACK) sprintf(expected,"p1=%ld p2=%ld e1=%ld err=%ld inputs=0 drops=0",
            (long)(g_ptp.origin[0]-g_arm1.usteps),(long)(g_ptp.origin[1]-g_arm2.usteps),
            (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),(long)(cmd_lsb()-g_enc.pos));
        assert(strstr(simLog,expected));
        assert(g_ptp.origin[0]-g_arm1.usteps<0 && g_ptp.origin[1]-g_arm2.usteps>0);
        jog_handle_byte(JOG_CMD_STOP);status_drain();
        assert(g_ptp.referenceSource==0u && strstr(simLog,"ref=0 source=0"));return;
    }
    if(!strcmp(t,"ptp_logged_retry")) {
        /* Latest field target: lose a complete triplicated nibble, then retry unchanged. */
        ptp_send(2,epoch,-2195,4437,200,2);txq_flush_blocking();
        assert(!g_ptp.active && g_ptp.sequence==0 && p1==simPulse1 && p2==simPulse2);
        assert(strstr(simLog,"PTP RX dropped: incomplete"));
        ptp_send(2,epoch,-2195,4437,200,0);assert(g_ptp.active && g_ptp.sequence==2);
        for(i=0;i<100;i++)cycle(1);
        { uint32 done=g_ptp.seg.done; int32 a=g_arm1.usteps,b=g_arm2.usteps;
          ptp_send(2,epoch,-2195,4437,200,0);
          assert(g_ptp.seg.done==done && g_arm1.usteps==a && g_arm2.usteps==b); }
        jog_handle_byte(JOG_CMD_STOP);
        ptp_send(2,epoch,-2195,4437,200,0);assert_stopped();return;
    }
    if(!strncmp(t,"ptp_uart_",9)) {
        if(!strcmp(t,"ptp_uart_paced"))simUartChunk=8;
        int logged=!strncmp(t,"ptp_uart_logged_",16);
        ptp_send(1,epoch,logged?-2195:125,logged?4437:-250,200,4);
        simRxActive=1;simRxOffset=(uint32)atoi(t+(logged?16:9))*SIM_MHZ;
        simLogN=0;simLog[0]=0;
        sim_run(0);
        if(!g_ptp.active||g_ptp.sequence!=1){fprintf(stderr,"%s\nRX:",simLog);for(i=0;i<simPtpReceivedN;i++)fprintf(stderr,"%02X ",simPtpReceived[i]);}
        assert(g_motionEnabled&&g_ptp.active&&g_ptp.sequence==1&&g_ptp.seg.done>0);
        return;
    }
    if(!strcmp(t,"ptp_locked")) {
        assert(!PTP_COMMISSIONED);ptp_send(1,epoch,125,-250,200,0);
        assert(!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"ptp_crc")||!strcmp(t,"ptp_missing")||!strcmp(t,"ptp_partial")) {
        ptp_send(1,epoch,125,-250,200,!strcmp(t,"ptp_crc")?1:(!strcmp(t,"ptp_missing")?2:3));
        for(i=0;i<300;i++)cycle(1);
        assert(!g_ptpRx.active&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"ptp_quarantine")) {
        jog_handle_byte(PTP_FRAME_START);jog_handle_byte('q');jog_handle_byte('q');jog_handle_byte('q');
        assert(g_ptpRx.active&&g_home.phase==HOME_IDLE);jog_handle_byte(PTP_FRAME_END);assert(!g_ptpRx.active);return;
    }
    if(!strcmp(t,"ptp_partial_stop")) {
        ptp_send(1,epoch,125,-250,200,3);jog_handle_byte(JOG_CMD_STOP);jog_handle_byte(PTP_FRAME_END);
        assert(!g_ptp.active&&!g_ptp.reference&&!g_ptpRx.active);assert_stopped();return;
    }
    if(!strcmp(t,"ptp_stale_epoch")||!strcmp(t,"ptp_range")||!strcmp(t,"ptp_rate")) {
        ptp_send(1,!strcmp(t,"ptp_stale_epoch")?epoch-1:epoch,!strcmp(t,"ptp_range")?7000:125,-250,!strcmp(t,"ptp_rate")?PTP_RATE_MAX+1:200,0);
        assert(!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    ptp_send(1,epoch,target1,target2,200,0);assert(g_ptp.active&&g_ptp.sequence==1);
    if(!strcmp(t,"ptp_home_busy")){jog_home_all();assert(g_home.phase==HOME_IDLE&&g_ptp.active);}
    if(!strcmp(t,"ptp_duplicate")){ptp_send(1,epoch,125,-250,200,0);assert(g_ptp.active&&g_ptp.seg.done==0);}
    if(!strcmp(t,"ptp_busy")){ptp_send(2,epoch,250,250,200,0);assert(g_ptp.sequence==1&&g_ptp.seg.target[0]==start1-ptp_offset(0,125));}
    for(i=0;i<100;i++)cycle(1);
    if(!strcmp(t,"ptp_stop"))jog_handle_byte(JOG_CMD_STOP);
    if(!strcmp(t,"ptp_fault"))simFault=1;
    if(!strcmp(t,"ptp_limit"))simLimit=1;
    if(!strcmp(t,"ptp_home1"))simHome1Mode=1;
    if(!strcmp(t,"ptp_npn"))simHome2Mode=1;
    if(!strcmp(t,"ptp_encoder"))simBadEncoder=1;
    if(!strcmp(t,"ptp_stall"))simStall=1;
    if(!strcmp(t,"ptp_jog"))sim_keys(2);
    for(i=0;i<30000&&g_motionEnabled&&g_ptp.active;i++) {
        cycle(strcmp(t,"ptp_timeout")!=0);
        if(g_ptp.active&&g_motionEnabled&&g_ptp.seg.total) {
            assert(abs_i32(g_arm1.usteps-start1)==(int32)((int64)g_ptp.seg.distance[0]*g_ptp.seg.done/g_ptp.seg.total));
            assert(abs_i32(g_arm2.usteps-start2)==(int32)((int64)g_ptp.seg.distance[1]*g_ptp.seg.done/g_ptp.seg.total));
        }
    }
    assert(i<30000);
    if(!g_motionEnabled) {
        assert(!strcmp(t,"ptp_stop")||!strcmp(t,"ptp_fault")||!strcmp(t,"ptp_limit")||!strcmp(t,"ptp_home1")||
            !strcmp(t,"ptp_npn")||!strcmp(t,"ptp_encoder")||!strcmp(t,"ptp_stall")||!strcmp(t,"ptp_jog")||!strcmp(t,"ptp_timeout"));
        assert_stopped();assert(!g_ptp.reference&&!g_ptp.active);
        if(!strcmp(t,"ptp_fault"))assert(simPins[PIN_EN1] && simPins[PIN_EN2] &&
            !simPins[PIN_SLEEP1] && !simPins[PIN_SLEEP2] && !simPins[PIN_RST1] && !simPins[PIN_RST2]);
        return;
    }
    assert(!strcmp(t,"ptp_success")||!strcmp(t,"ptp_reverse")||!strcmp(t,"ptp_duplicate")||!strcmp(t,"ptp_busy")||!strcmp(t,"ptp_home_busy")||
        !strcmp(t,"ptp_zero")||!strcmp(t,"ptp_axis1")||!strcmp(t,"ptp_axis2"));
    assert(g_ptp.reference&&!g_ptp.active);
    assert(g_arm1.usteps==start1-ptp_offset(0,target1)&&g_arm2.usteps==start2-ptp_offset(1,target2-target1));
    p1=simPulse1;p2=simPulse2;
    ptp_send(1,epoch,125,-250,200,0);for(i=0;i<200;i++)cycle(1);
    assert(!g_ptp.active&&p1==simPulse1&&p2==simPulse2); /* completed duplicate cannot restart */
    ptp_send(2,epoch,0,0,200,0);assert(g_ptp.active);
    for(i=0;i<30000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
    assert(g_motionEnabled&&g_ptp.reference&&!g_ptp.active&&g_arm1.usteps==start1&&g_arm2.usteps==start2);
    ptp_send(1,epoch,125,-250,200,0);assert(!g_ptp.active); /* old seq rejected */
    sim_keys(2);assert(!g_ptp.reference); /* manual jog invalidates PTP reference */
}
static void traj_send(uint8 type, uint32 seq, int16 q1, int16 q2, uint16 value) {
    packetType=type; ptp_send(seq,g_ptp.epoch,q1,q2,value,0); packetType=1u; txq_flush_blocking();
}
/* Speeds and accelerations follow an enabled, referenced device while it is idle (rates_live=1); ranges still need a stop.
 * Mode 5 sends no neutral byte, so a held key or a running move is still there when the packet is judged. */
static void rates_send(uint32 seq,uint32 epoch,uint16 axis,int16 rate,int16 accel) {
    packetType=10;ptp_send(seq,epoch,rate,accel,axis,5);packetType=1;txq_flush_blocking();
}
static void rates_live_test(void) {
    uint32 epoch,seq,sequence; unsigned i; int32 u1,u2; int16 min1;
    short_send(0,0);/* Simulated calibration: board2 polarity was previously learned. */ g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;txq_flush_blocking();
    assert(g_motionEnabled && g_ptp.reference && g_homed1 && g_homed2);
    epoch=g_ptp.epoch;u1=g_arm1.usteps;u2=g_arm2.usteps;
    /* Idle and referenced: taken at once; the reference, the epoch and the position stay. */
    seq=g_controlSequence+1u;simLogN=0;simLog[0]=0;
    rates_send(seq,epoch,0,400,1500);
    assert(g_motionRate[0]==400 && g_motionAccel[0]==1500 && g_arm1.rateMax==400 && g_arm1.accel==1500 && g_controlSequence==seq);
    assert(g_motionEnabled && g_ptp.reference && g_ptp.epoch==epoch && !simPulse1 && !simPulse2 && g_arm1.usteps==u1 && g_arm2.usteps==u2);
    assert(strstr(simLog,"PTP_RATES epoch=") && strstr(simLog,"axis=0 min=400 max=1500") && strstr(simLog,"r1=400"));
    rates_send(seq,epoch,0,400,1500);assert(g_controlSequence==seq && g_motionRate[0]==400); /* exact retry only repeats the receipt */
    /* Anything moving, homing or uploading refuses, and nothing changes. */
    for(i=0;i<8;i++) {
        seq=g_controlSequence+1u;
        if(i==0)g_home.phase=HOME_FAST;if(i==1)g_ptp.active=1;if(i==2)g_traj.state=1;if(i==3)g_traj.state=3;
        if(i==4)g_arm1.want=1;if(i==5)g_arm2.moving=1;if(i==6)g_arm1.settleMs=1;if(i==7)g_uploadRxQuarantine=1;
        rates_send(seq,epoch,0,300,1000);
        assert(g_controlSequence<seq && g_motionRate[0]==400 && g_motionAccel[0]==1500 && g_arm1.rateMax==400);
        g_home.phase=HOME_IDLE;g_ptp.active=0;g_traj.state=0;g_arm1.want=0;g_arm2.moving=0;g_arm1.settleMs=0;g_uploadRxQuarantine=0;
    }
    /* The closed-loop hold is not motion. */
    g_arm1.holding=1;seq=g_controlSequence+1u;rates_send(seq,epoch,0,350,1200);
    assert(g_motionRate[0]==350 && g_motionAccel[0]==1200 && g_controlSequence==seq && g_arm1.holding);g_arm1.holding=0;
    /* A held plan was checked against the old speeds: the same values keep it, a change drops it. */
    sequence=g_ptp.sequence;
    traj_send(2,sequence+1u,0,0,2);traj_send(3,sequence+2u,10,0,250);
    assert(g_traj.state==2 && g_traj.count==2);
    seq=g_controlSequence+1u;rates_send(seq,epoch,0,350,1200);
    assert(g_controlSequence==seq && g_traj.state==2 && g_traj.count==2);
    simLogN=0;simLog[0]=0;seq=g_controlSequence+1u;rates_send(seq,epoch,0,360,1200);
    assert(g_motionRate[0]==360 && !g_traj.state && !g_traj.count && g_ptp.reference && g_ptp.epoch==epoch);
    assert(strstr(simLog,"TRAJ held plan dropped") && strstr(simLog,"TRAJ_STATE state=0"));
    g_traj.state=4u;g_traj.count=g_traj.expected=2u; /* a plan that already ran */
    seq=g_controlSequence+1u;rates_send(seq,epoch,0,370,1200);assert(g_motionRate[0]==370 && !g_traj.state);
    /* Ranges still need a stop and no reference. */
    min1=g_jointMinCd[0];seq=g_controlSequence+1u;limits_send(seq,epoch,0,-2000,2000);
    assert(g_controlSequence<seq && g_jointMinCd[0]==min1 && g_motionEnabled && g_ptp.reference);
    /* A real move: refused while it runs, taken once it has settled, reference intact. */
    ptp_send(g_ptp.sequence+1u,g_ptp.epoch,20,30,200,0);assert(g_ptp.active);
    seq=g_controlSequence+1u;rates_send(seq,epoch,1,300,900);assert(g_controlSequence<seq && g_motionRate[1]!=300);
    for(i=0;i<5000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
    assert(g_motionEnabled&&!g_ptp.active&&g_ptp.reference);
    for(i=0;i<100;i++)cycle(1);
    seq=g_controlSequence+1u;rates_send(seq,epoch,1,300,900);
    assert(g_motionRate[1]==300 && g_motionAccel[1]==900 && g_arm2.rateMax==300 && g_motionEnabled && g_ptp.reference && g_ptp.epoch==epoch);
}
static void traj_rx_fault(const char *t) {
    unsigned i,p1=simPulse1,p2=simPulse2; uint32 epoch=g_ptp.epoch,seq=g_ptp.sequence;
    uint16 count=g_traj.count; uint8 state=g_traj.state;
    /* A partial real RX frame, then hardware framing and queued legacy commands. */
    packetType=state==2?4:3;ptp_send(seq+1,epoch,state==2?0:10,state==2?0:-10,state==2?0:250,4);packetType=1;
    for(i=0;i<8;i++)sim_uart_push(simPtpWire[i]);jog_rx_service();assert(g_ptpRx.active);
    HostUart_errorStatus=HostUart_RX_STS_STOP_ERROR;
    if(!strcmp(t,"traj_rx_overrun"))HostUart_errorStatus=HostUart_RX_STS_OVERRUN;
    if(!strcmp(t,"traj_rx_break"))HostUart_errorStatus=HostUart_RX_STS_BREAK;
    if(!strcmp(t,"traj_rx_parity"))HostUart_errorStatus=HostUart_RX_STS_PAR_ERROR;
    if(!strcmp(t,"traj_rx_mixed"))HostUart_errorStatus|=HostUart_RX_STS_OVERRUN;
    if(!strcmp(t,"traj_rx_overflow"))HostUart_rxBufferOverflow=1;
    if(!strcmp(t,"traj_rx_heartbeat"))g_lastKeyMs=g_jogMs-JOG_DEADMAN_MS-1;
    if(!strcmp(t,"traj_rx_holding"))g_arm1.holding=1;
    if(!strcmp(t,"traj_rx_fault"))simFault=1;
    if(!strcmp(t,"traj_rx_encoder"))g_enc.badStreak=1;
    sim_uart_push(JOG_CMD_ENABLE);sim_uart_push(JOG_CMD_HOME);sim_uart_push(0x45);jog_rx_service();
    assert(!simUartCount);txq_flush_blocking();
    if(strcmp(t,"traj_rx_retry")&&strcmp(t,"traj_rx_ready_retry")&&strcmp(t,"traj_rx_repeated")&&
       strcmp(t,"traj_rx_stop")&&strcmp(t,"traj_rx_timeout")) {
        assert_stopped();assert(!g_ptp.reference&&!g_traj.state);return;
    }
    assert(g_motionEnabled&&g_ptp.reference&&g_ptp.epoch==epoch&&g_ptp.sequence==seq&&g_traj.count==count&&g_traj.state==state);
    assert(strstr(simLog,"TRAJ RX framing discarded; awaiting retry"));
    if(!strcmp(t,"traj_rx_repeated")){HostUart_errorStatus=HostUart_RX_STS_STOP_ERROR;jog_rx_service();assert(g_ptp.reference);}
    /* END and parser timeout must not expose corrupt tails as C, R or jog. */
    for(i=8;i<36;i++)sim_uart_push(simPtpWire[i]);while(simUartCount)jog_rx_service();
    jog_handle_byte(PTP_FRAME_START);for(i=0;i<300;i++)cycle(1);
    sim_uart_push(JOG_CMD_ENABLE);sim_uart_push(JOG_CMD_HOME);sim_uart_push(0x45);while(simUartCount)jog_rx_service();
    cycle(1);assert(g_motionEnabled&&g_ptp.reference&&g_ptp.epoch==epoch&&g_ptp.sequence==seq&&g_traj.count==count);
    assert(!g_ptp.active&&g_home.phase==HOME_IDLE&&p1==simPulse1&&p2==simPulse2);
    if(!strcmp(t,"traj_rx_stop")){
        sim_uart_push(JOG_CMD_STOP);sim_uart_push(JOG_CMD_ENABLE);sim_uart_push(JOG_CMD_HOME);
        jog_rx_service();assert(!simUartCount);assert_stopped();return;
    }
    if(!strcmp(t,"traj_rx_timeout")){
        sim_uart_push(0x41);sim_uart_push(JOG_CMD_ENABLE);sim_uart_push(JOG_CMD_HOME);
        for(i=0;i<260;i++)cycle(0);assert(!simUartCount);assert_stopped();return;
    }
    packetType=state==2?4:3;ptp_send(seq+1,epoch,state==2?0:10,state==2?0:-10,state==2?0:250,1);packetType=1;
    assert(g_ptp.sequence==seq&&g_traj.count==count&&!g_ptp.active); /* corrupt retry cannot commit or RUN */
    simPtpWireN=0; /* later valid packets exercise the original sequence/CRC path */
}
/* The real host disables, sets both axes' rates (type 10) and only then enables. Host plans are built for the
 * configured speeds, so a fixture that leaves the boot defaults (800/600) rejects them as "rate/velocity change". */
static void host_rates_config(void) {
    unsigned axis; uint32 epoch;
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);txq_flush_blocking();
    epoch=g_ptp.epoch;packetType=10;
    for(axis=0;axis<2;axis++)
        ptp_send(g_controlSequence+1u,epoch,axis?CFG_RATE_MAX_AXIS2:CFG_RATE_MAX_AXIS1,axis?CFG_ACCEL_MAX_AXIS2:CFG_ACCEL_MAX_AXIS1,axis,0);
    packetType=1;txq_flush_blocking();
    assert(g_motionRate[0]==CFG_RATE_MAX_AXIS1&&g_motionRate[1]==CFG_RATE_MAX_AXIS2);
    stage_enable();assert(g_motionEnabled);
}
/* Characterization of the trajectory executor: the exact pulse schedule of fixed plans.  The plans are out-and-back smoothsteps
 * built with integer arithmetic only, so they are identical on every compiler; the hashes were recorded from the firmware as of
 * the time-in traj_poll (the move into traj.h left them unchanged; making time an argument removed SysTick reads, which moved
 * every pulse by less than one foreground pass, with the same pulse counts and axis order).  The rate cap and the ride-the-limit
 * code changed nothing here (with the log text of the previous build the old hashes come back exactly); the new hashes only
 * reflect the longer FW_BUILD_ID and the traj_slip_us field in the status lines.  Any other change is a change of behaviour. */
#define SCHED_A1 7112u
#define SCHED_A2 6808u
#define SCHED_B1 3200u
#define SCHED_B2 3178u
#define SCHED_C1 2844u
#define SCHED_C2 0u
/* The trace is taken in 24 MHz-equivalent cycles, so a different bus clock reproduces the pulse counts and the axis order
 * exactly but lands some pulses one foreground pass later: each clock has its own recorded hashes (48 MHz: 10-08).  The
 * hardware offload moves every pulse again (no busy-waits, the SPI Master, the event timer): a build with every switch
 * on has its own set as well. Removing foreground slot-clock reads for ISR-owned plans changes the timestamp hash;
 * Idle status slicing also changes clock phases. Both changes were characterized against the prior commit:
 * pulse counts, node endpoints and axis/direction order match; hardware modeled worst lag stays 21 us.
 * Any other combination checks the pulse counts and node endpoints only (SCHED_HASHED 0). */
#define SCHED_ALL_HW (HW_SSI && HW_STEP && HW_EVT && HW_DEAD && HW_PINIRQ && HW_TXDMA)
#define SCHED_NO_HW  (!HW_SSI && !HW_STEP && !HW_EVT && !HW_DEAD && !HW_PINIRQ && !HW_TXDMA)
#if SIM_MHZ == 48 && SCHED_ALL_HW
#define SCHED_HASHED 1
#if ARM2_ENCODER_FEEDBACK
#define SCHED_AH 0xd98b4cddu
#define SCHED_BH 0x9fe67478u
#define SCHED_CH 0xba97d7fdu
#else
#define SCHED_AH 0x3f7d68bdu
#define SCHED_BH 0xe88b3c78u
#define SCHED_CH 0x33299ecdu
#endif
#elif SIM_MHZ == 48 && SCHED_NO_HW
#define SCHED_HASHED 1
#if ARM2_ENCODER_FEEDBACK
#define SCHED_AH 0xcdd517bfu
#define SCHED_BH 0x6385ba80u
#define SCHED_CH 0x4f57bf0bu
#else
#define SCHED_AH 0x435fb50du
#define SCHED_BH 0x2d4f0882u
#define SCHED_CH 0x15b5256fu
#endif
#elif SIM_MHZ == 24 && SCHED_NO_HW
#define SCHED_HASHED 1
#if ARM2_ENCODER_FEEDBACK /* the second encoder frame lengthens the foreground pass, which moves where each pulse lands */
#define SCHED_AH 0x95834fe7u
#define SCHED_BH 0x6385ba80u
#define SCHED_CH 0xfe1892c3u
#else
#define SCHED_AH 0xa177853du
#define SCHED_BH 0xe03428feu
#define SCHED_CH 0x3f5f10ffu
#endif
#else
#define SCHED_HASHED 0
#define SCHED_AH 0u
#define SCHED_BH 0u
#define SCHED_CH 0u
#endif
typedef struct { unsigned n; int16 q1[128], q2[128]; uint16 ms[128]; } sched_plan_t;
static void sched_build(sched_plan_t *p, const uint16 *ms, unsigned n, int a1, int ae, unsigned dwellMs) {
    unsigned k; int64 total=0, t=0;
    for(k=0;k<n;k++)total+=ms[k];
    p->n=n+1; p->q1[0]=p->q2[0]=0; p->ms[0]=0;
    for(k=0;k<n;k++) {
        int64 span=total-2*(int64)dwellMs, x, u, s;
        t+=ms[k]; x=t-(int64)dwellMs; if(x<0)x=0; if(x>span)x=span;
        x=(2*x<=span)?2*x:2*span-2*x;
        u=x*1000000/span; s=(3*u*u*1000000-2*u*u*u)/1000000000000LL;
        p->q1[k+1]=(int16)(a1*s/1000000); p->q2[k+1]=(int16)(p->q1[k+1]+ae*s/1000000); p->ms[k+1]=ms[k];
    }
}
static void sched_run(const char *name, const sched_plan_t *p, uint32 *seq, unsigned exp1, unsigned exp2, uint32 expTrace) {
    unsigned i, p1=simPulse1, p2=simPulse2;
    traj_send(2,(*seq)++,p->q1[0],p->q2[0],(uint16)p->n);
    for(i=1;i<p->n;i++){traj_send(3,(*seq)++,p->q1[i],p->q2[i],p->ms[i]);cycle(1);assert(g_traj.count==i+1);}
    assert(g_traj.state==2&&p1==simPulse1&&p2==simPulse2);
    simTrace=2166136261u;simTraceBase=simClock;g_traj.lagMaxCy=0u;p1=simPulse1;p2=simPulse2;
    traj_send(4,(*seq)++,0,0,0);assert(g_traj.state==3);
    for(i=0;i<650000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
    if(!g_motionEnabled)fprintf(stderr,"%s\n",simLog);
    assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==p->n);
    printf("TRAJ_SCHEDULE %s pulses1=%u pulses2=%u trace=0x%08lx lag_us=%lu\n",name,simPulse1-p1,simPulse2-p2,
           (unsigned long)simTrace,(unsigned long)(g_traj.lagMaxCy/SU_CY_PER_US));fflush(stdout);
#ifndef SCHED_RECORD /* compile with /DSCHED_RECORD to print new values instead of checking them */
    assert(simPulse1-p1==exp1&&simPulse2-p2==exp2&&(!SCHED_HASHED||simTrace==expTrace));
#else
    (void)exp1;(void)exp2;(void)expTrace;
#endif
}
static void traj_schedule_test(uint32 *seq) {
    static const uint16 regular[60]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
                                     100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
    static const uint16 uneven[24]={50,137,500,73,250,311,90,444,50,200,350,125,50,137,500,73,250,311,90,444,50,200,350,125};
    static uint16 dwelled[48]; /* 1 s still, 4 s of arm1-only motion, 1 s still */
    static sched_plan_t plan;
    unsigned k;
    for(k=0;k<48;k++)dwelled[k]=(k<4||k>=44)?250:100;
    sched_build(&plan,regular,60,2000,1500,0);sched_run("regular",&plan,seq,SCHED_A1,SCHED_A2,SCHED_AH);
    sched_build(&plan,uneven,24,900,700,0);sched_run("uneven",&plan,seq,SCHED_B1,SCHED_B2,SCHED_BH);
    sched_build(&plan,dwelled,48,800,0,1000);sched_run("arm1_dwell",&plan,seq,SCHED_C1,SCHED_C2,SCHED_CH);
}
static void traj_test(const char *t) {
    unsigned i,p1,p2; int32 origin1,origin2; uint32 seq=1;
    if(!strncmp(t,"traj_host_plan",14)||!strcmp(t,"traj_manual_plan")||!strcmp(t,"traj_schedule")) host_rates_config();
    if(!strcmp(t,"traj_manual_plan")||!strcmp(t,"traj_schedule")) {
        /* Simulated prior polarity calibration; manual reference itself must not pulse. */
        g_enc2.directionKnown=1u;g_enc2.incDir=1u;
        traj_send(5,seq++,0,0,0);assert(!simPulse1&&!simPulse2);
    }
    else jog_home_all();
    for(i=0;i<250000&&g_home.phase!=HOME_IDLE&&g_motionEnabled;i++)cycle(1);
    assert(g_homed1&&g_homed2&&g_ptp.reference);cycle(1);sim_keys(0);
    p1=simPulse1;p2=simPulse2;origin1=g_arm1.usteps;origin2=g_arm2.usteps;
    if(!strcmp(t,"traj_schedule")) {
        traj_schedule_test(&seq);
        assert(g_arm1.usteps==origin1&&g_arm2.usteps==origin2); /* every plan is out and back */
        return;
    }
    if(!strcmp(t,"traj_host_plan")||!strcmp(t,"traj_manual_plan")||!strcmp(t,"traj_host_plan_offset")||!strcmp(t,"traj_host_plan_rx_retry")) {
        FILE *file=fopen("host-plan.bin","rb"); uint16 count; static traj_node_t nodes[2048];
        assert(file && fread(&count,2,1,file)==1 && count>=2 && count<=2048);
        assert(fread(nodes,6,count,file)==count);fclose(file);
        if(!strcmp(t,"traj_host_plan_offset"))simEncoderBias=67; /* persistent field-like residual through approach, upload and drawing */
        ptp_send(seq,g_ptp.epoch,nodes[0].q1,nodes[0].q2,200,0);
        for(i=0;i<200000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
        assert(g_motionEnabled&&!g_ptp.active&&g_arm1.usteps==origin1-ptp_offset(0,nodes[0].q1));
        p1=simPulse1;p2=simPulse2;traj_send(2,++seq,nodes[0].q1,nodes[0].q2,count);
        for(i=1;i<count;i++) {
            if(!strcmp(t,"traj_host_plan_rx_retry")&&i==16)traj_rx_fault("traj_rx_retry");
            traj_send(3,++seq,nodes[i].q1,nodes[i].q2,nodes[i].ms);cycle(1);assert(g_traj.count==i+1);
            if(!strcmp(t,"traj_host_plan_rx_retry")&&i==16) {
                traj_send(3,seq,nodes[i].q1,nodes[i].q2,nodes[i].ms);assert(g_traj.count==i+1); /* ACK retry is idempotent */
            }
        }
        assert(p1==simPulse1&&p2==simPulse2&&g_traj.state==2);
        traj_send(4,++seq,0,0,0);assert(g_ptp.active);
        for(i=0;i<650000&&g_ptp.active&&g_motionEnabled;i++) {
            cycle(1); /* sim_check_traj_boundary checks both axes at each actual node transition. */
        }
        if(!g_motionEnabled)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==count);
        assert(simBoundaryChecks==count-1u);
        assert(g_arm1.usteps==origin1-ptp_offset(0,nodes[count-1].q1));
        assert(g_arm2.usteps==origin2-ptp_offset(1,nodes[count-1].q2-nodes[count-1].q1));
        if(!strcmp(t,"traj_host_plan_offset"))assert(abs_i32(cmd_lsb()-g_enc.pos)>=66&&abs_i32(cmd_lsb()-g_enc.pos)<=68&&g_ptp.reference);
        if(!strcmp(t,"traj_host_plan")) { /* repeat: the finished plan survives the move back to its start and runs again on a fresh sequence */
            uint32 lastRun=seq;
            ptp_send(++seq,g_ptp.epoch,nodes[0].q1,nodes[0].q2,200,0);
            for(i=0;i<200000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
            assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.count==count&&g_arm1.usteps==origin1-ptp_offset(0,nodes[0].q1));
            traj_send(4,lastRun,0,0,0);assert(!g_ptp.active&&g_traj.state==4); /* the old RUN sequence is stale, not a replay */
            traj_send(4,++seq,0,0,0);assert(g_ptp.active&&g_traj.state==3);
            for(i=0;i<650000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
            assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==count);
            assert(g_arm1.usteps==origin1-ptp_offset(0,nodes[count-1].q1));
            assert(g_arm2.usteps==origin2-ptp_offset(1,nodes[count-1].q2-nodes[count-1].q1));
        }
        return;
    }
    if(!strcmp(t,"traj_locked")) {
        traj_send(2,1,0,0,3);assert(!g_traj.state&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"traj_capacity")) {
        traj_send(2,1,0,0,2049);assert(!g_traj.state);
        traj_send(2,1,0,0,2048);assert(g_traj.state==1);
        for(i=2;i<=2048;i++)traj_send(3,i,0,0,250);
        assert(g_traj.state==2&&g_traj.count==2048);
        traj_send(3,2049,0,0,250);assert(g_traj.count==2048&&g_ptp.sequence==2048);
        assert(p1==simPulse1&&p2==simPulse2);return;
    }
    traj_send(2,seq,(!strcmp(t,"traj_wrong_start")?10:0),0,5);
    if(!strcmp(t,"traj_wrong_start")) {assert(!g_ptp.active&&!g_traj.state&&p1==simPulse1&&p2==simPulse2);return;}
    assert(g_traj.state==1&&g_traj.count==1&&!g_ptp.active);
    if(!strncmp(t,"traj_rx_",8)&&strcmp(t,"traj_rx_ready_retry")&&strcmp(t,"traj_rx_motion")) {
        traj_rx_fault(t);
        if(!g_motionEnabled)return;
    }
    if(!strcmp(t,"traj_uart_upload") || !strcmp(t,"traj_uart_upload_paced")) {
        if(!strcmp(t,"traj_uart_upload_paced"))simUartChunk=8;
        packetType=3;ptp_send(2,g_ptp.epoch,10,-10,250,4);packetType=1;
        simRxActive=1;simRxOffset=137u*SIM_MHZ;sim_run(0);
        assert(g_motionEnabled&&g_traj.count==2&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"traj_partial") || !strcmp(t,"traj_order") || !strcmp(t,"traj_range") ||
       !strcmp(t,"traj_rate") || !strcmp(t,"traj_time") || !strcmp(t,"traj_accel")) {
        if(!strcmp(t,"traj_partial"))traj_send(4,2,0,0,0);
        if(!strcmp(t,"traj_order"))traj_send(3,3,10,0,250);
        if(!strcmp(t,"traj_range"))traj_send(3,2,6001,0,250);
        if(!strcmp(t,"traj_rate"))traj_send(3,2,1000,0,250);
        if(!strcmp(t,"traj_time"))traj_send(3,2,10,0,501);
        if(!strcmp(t,"traj_accel"))traj_send(3,2,10,0,50);
        assert(g_traj.count==1&&g_ptp.sequence==1&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"traj_upload_stop")) {jog_handle_byte(JOG_CMD_STOP);assert(!g_traj.state);assert_stopped();return;}
    if(!strcmp(t,"traj_upload_jog")) {sim_keys(2);assert(!g_traj.state&&!g_ptp.reference);return;}
    if(!strcmp(t,"traj_upload_timeout")) {for(i=0;i<260;i++)cycle(0);assert(!g_traj.state);assert_stopped();return;}
    if(!strcmp(t,"traj_crc")) {
        packetType=3;ptp_send(2,g_ptp.epoch,10,10,250,1);packetType=1;
        assert(g_traj.count==1&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"traj_ptp_busy")) {ptp_send(2,g_ptp.epoch,10,10,200,0);assert(!g_ptp.active&&g_traj.count==1);return;}
    for(i=1;i<=4;i++) {
        int16 q=(i==1?10:(i==2?20:(i==3?10:0)));
        if(!strcmp(t,"traj_dwell"))q=0;
        traj_send(3,++seq,q,0,250); /* q2 is the absolute link angle (elbow = q2 - q1): (q,0) moves the elbow by -q, as (q,-q) did when q2 was elbow-relative */
        assert(g_traj.count==i+1);
    }
    assert(g_traj.state==2&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);
    if(!strcmp(t,"traj_rx_ready_retry"))traj_rx_fault(t);
    if(!strcmp(t,"traj_uart_run") || !strcmp(t,"traj_uart_run_paced")) {
        if(!strcmp(t,"traj_uart_run_paced"))simUartChunk=8;
        packetType=4;ptp_send(++seq,g_ptp.epoch,0,0,0,4);packetType=1;
        simRxActive=1;simRxOffset=499u*SIM_MHZ;sim_run(0);
        if(!g_motionEnabled||g_traj.state!=4)fprintf(stderr,"DBG en=%d st=%d act=%d idx=%u/%u stop=%s pulses=%u/%u wall=%lu pi=%u/%u cnt=%u off=%lu act=%d rxbase=%llu clk=%llu\n%s\n",g_motionEnabled,g_traj.state,g_ptp.active,(unsigned)g_traj.index,(unsigned)g_traj.count,g_lastStop,simPulse1-p1,simPulse2-p2,(unsigned long)wall_ms(),simUartPacketIndex,simPtpWireN,simUartCount,(unsigned long)simRxOffset,simRxActive,(unsigned long long)simRxBase,(unsigned long long)simClock,simLog);
        assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_arm1.usteps==origin1&&g_arm2.usteps==origin2);
        assert(simPulse1-p1==72&&simPulse2-p2==90);return;
    }
    traj_send(4,++seq,0,0,0);
    assert(g_traj.state==3&&g_ptp.active==1);
    if(!strncmp(t,"traj_evt_",9)) {
#if HW_EVT
        unsigned isr0=simEvtIsrCount; unsigned long pulses;
        assert(g_evtRunning && simEvtArmed && simEvtIsrOn);
        if(!strcmp(t,"traj_evt_runs")) {
            /* the timer, not the foreground, steps the plan: one interrupt per coordinated step (90 of them carry the 72 + 90 pulses), every one re-armed */
            for(i=0;i<2000&&g_motionEnabled&&g_ptp.active;i++)cycle(1);
            assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==g_traj.count);
            assert(simPulse1-p1==72&&simPulse2-p2==90);
            assert(simEvtIsrCount-isr0>=90u && simEvtRestarts>=simEvtIsrCount-isr0 && !g_evtRunning && !g_evtStop);
            assert(g_evtEvents>=90u && g_evtMaxCy>0u);
        } else if(!strcmp(t,"traj_evt_stall")) {
            /* a foreground that stops passing is noticed by the interrupt itself and ends the run; the foreground logs it */
            for(i=0;i<50;i++)cycle(1);
            assert(g_motionEnabled&&g_ptp.active==1u&&g_evtRunning);
            simClock+=(uint64_t)SIM_MHZ*60000u; (void)SU_NOW();
            assert(!g_evtRunning && g_evtStop && strstr(g_evtStop,"foreground stalled"));
            pulses=simPulse1+simPulse2; simClock+=(uint64_t)SIM_MHZ*5000u; (void)SU_NOW();
            assert(pulses==simPulse1+simPulse2);   /* and not one more pulse */
            cycle(1); assert(!g_motionEnabled && strstr(g_lastStop,"foreground stalled") && !g_evtStop);
        } else if(!strcmp(t,"traj_evt_stop")) {
            /* an operator stop takes the timer away before it touches the plan again */
            for(i=0;i<50;i++)cycle(1);
            assert(g_evtRunning); jog_handle_byte(JOG_CMD_STOP);
            assert(!g_motionEnabled && !g_evtRunning && !simEvtIsrOn);
            pulses=simPulse1+simPulse2; isr0=simEvtIsrCount;
            simClock+=(uint64_t)SIM_MHZ*20000u; (void)SU_NOW();
            assert(pulses==simPulse1+simPulse2 && isr0==simEvtIsrCount);
        }
#endif
        return;
    }
    if(!strcmp(t,"traj_rx_motion")){HostUart_errorStatus=HostUart_RX_STS_STOP_ERROR;jog_rx_service();assert_stopped();return;}
    if(!strcmp(t,"traj_duplicate")) {traj_send(4,seq,0,0,0);assert(g_traj.index==1);}
    if(!strcmp(t,"traj_overrun"))simClock+=SIM_MHZ*300000u;
    for(i=0;i<100;i++)cycle(1);
    if(!strcmp(t,"traj_stop"))jog_handle_byte(JOG_CMD_STOP);
    if(!strcmp(t,"traj_fault"))simFault=1;
    if(!strcmp(t,"traj_limit"))simLimit=1;
    if(!strcmp(t,"traj_npn"))simHome2Mode=1;
    if(!strcmp(t,"traj_home1"))simHome1Mode=1;
    if(!strcmp(t,"traj_encoder"))simBadEncoder=1;
    for(i=0;i<2000&&g_motionEnabled&&g_ptp.active;i++)cycle(strcmp(t,"traj_timeout")!=0);
    if(!strcmp(t,"traj_stop")||!strcmp(t,"traj_fault")||!strcmp(t,"traj_limit")||!strcmp(t,"traj_npn")||
       !strcmp(t,"traj_home1")||!strcmp(t,"traj_encoder")||!strcmp(t,"traj_timeout")||!strcmp(t,"traj_overrun")) {
        assert_stopped();assert(!g_traj.state&&!g_ptp.reference);return;
    }
    if(!g_motionEnabled)fprintf(stderr,"%s\n",simLog);
    assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==g_traj.count);
    assert(g_arm1.usteps==origin1&&g_arm2.usteps==origin2);
    if(!strcmp(t,"traj_dwell"))assert(p1==simPulse1&&p2==simPulse2);
    else assert(simPulse1-p1==72&&simPulse2-p2==90); /* out and back, both axes */
    p1=simPulse1;p2=simPulse2;traj_send(4,seq,0,0,0);for(i=0;i<100;i++)cycle(1);
    assert(!g_ptp.active&&p1==simPulse1&&p2==simPulse2); /* never replay completed start */
}
static void manual_test(const char *t) {
    /* This fixture represents a previously learned board2 polarity. */
    g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;
    uint32 epoch=g_ptp.epoch; unsigned i,p1=simPulse1,p2=simPulse2;
    int bad=strcmp(t,"manual_success")&&strcmp(t,"manual_duplicate")&&strcmp(t,"manual_stop")&&strcmp(t,"manual_old_epoch");
    /* Current counters can be outside the old reference envelope: no physical motion. */
    g_arm1.usteps=12345;g_arm2.usteps=-23456;
    if(!strcmp(t,"manual_disabled"))g_motionEnabled=0;
    if(!strcmp(t,"manual_moving"))g_arm1.moving=1;
    if(!strcmp(t,"manual_busy"))g_ptp.active=1;
    if(!strcmp(t,"manual_upload"))g_traj.state=1;
    if(!strcmp(t,"manual_sensor"))simHome2Mode=1;
    if(!strcmp(t,"manual_fault"))simFault=1;
    if(!strcmp(t,"manual_encoder"))simBadEncoder=1;
    if(!strcmp(t,"manual_timeout"))g_keysSeen=0;
    /* Simulated calibration: board2 polarity was previously learned. */ g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;packetType=5;
    ptp_send(1,!strcmp(t,"manual_stale")?epoch-1:epoch,!strcmp(t,"manual_payload")?1:0,0,0,!strcmp(t,"manual_crc")?1:(!strcmp(t,"manual_timeout")?5:0));
    packetType=1;
    assert(p1==simPulse1&&p2==simPulse2&&g_home.phase==HOME_IDLE);
    if(bad) {assert(!g_ptp.reference&&!g_homed1&&!g_homed2);return;}
    assert(g_ptp.reference&&g_homed1&&g_homed2&&g_directionKnown&&g_encIncDir==1&&g_ptp.epoch!=epoch);
    assert(g_ptp.origin[0]==12345&&g_ptp.origin[1]==-23456&&g_arm1SoftCenter==g_enc.pos);
    if(!strcmp(t,"manual_duplicate")) {
        uint32 newEpoch=g_ptp.epoch;/* Simulated calibration: board2 polarity was previously learned. */ g_enc2.directionKnown=1u;g_enc2.incDir=simPolarity2>0?1u:0u;packetType=5;ptp_send(1,epoch,0,0,0,0);packetType=1;
        assert(g_ptp.epoch==newEpoch&&g_ptp.sequence==1&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"manual_old_epoch")) {
        ptp_send(2,epoch,20,30,200,0);assert(!g_ptp.active&&g_ptp.sequence==1);return;
    }
    if(!strcmp(t,"manual_stop")) {
        jog_handle_byte(JOG_CMD_STOP);assert_stopped();assert(!g_ptp.reference);
        sim_keys(0);stage_enable();traj_send(5,2,0,0,0);
        /* A fresh enable invalidates polarity: the manual reference still works and takes the bench polarity for arm2. */
        assert(g_ptp.reference && g_enc2.directionKnown==(ARM2_ENCODER_FEEDBACK?1u:0u) && !g_ptp.active && p1==simPulse1 && p2==simPulse2);return;
    }
    ptp_send(2,g_ptp.epoch,20,30,200,0);assert(g_ptp.active);
    for(i=0;i<5000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
    assert(g_motionEnabled&&!g_ptp.active&&g_ptp.reference);
    assert(g_arm1.usteps==12345-ptp_offset(0,20)&&g_arm2.usteps==-23456-ptp_offset(1,10));
}
/* ---- fuzz: random events against the firmware's own loop pieces.  After every simulated millisecond the machine
 * must be coherent (one mode at a time, nothing runs while disabled, a reference implies homed axes, ...) and
 * no step pulse may be emitted while disabled or while nFAULT / the hard limit was already active. ---- */
static uint32 fuzzS; static unsigned fuzzOp, fuzzSeed;
static unsigned fuzzCov[6]; /* enable, stop, manual/home reference, ptp done, traj done, home done */
static uint8 fuzzKeys, fuzzHb=1;
static int16 fuzzQ1, fuzzQ2; static int fuzzClean;
static uint32 fuzz_rnd(uint32 n) { fuzzS=fuzzS*1664525u+1013904223u; return (fuzzS>>12)%n; }
static int16 fuzz_q(int lo, int hi) { return (int16)(lo+(int)fuzz_rnd((uint32)(hi-lo+1))); }
#define FZ(c) do { if(!(c)) { fprintf(stderr,"fuzz seed=%u op=%u: %s\n",fuzzSeed,fuzzOp,#c); assert(0); } } while(0)
static void fuzz_invariants(void) {
    FZ(g_motionEnabled || (g_home.phase==HOME_IDLE && !g_ptp.active && g_traj.state!=3u && !g_ptp.reference && !g_homed1 && !g_homed2));
    FZ(!(g_home.phase!=HOME_IDLE && g_ptp.active));
    FZ(!(g_ptp.active && (g_arm1.want || g_arm2.want)));
    FZ(!(g_home.phase!=HOME_IDLE && (g_arm1.want || g_arm2.want)));
    FZ(g_traj.state!=3u || g_ptp.active);
    FZ(!g_ptp.active || (g_ptp.reference && g_motionEnabled));
    FZ(!g_ptp.reference || (g_homed1 && g_homed2 && g_enc.directionKnown && (!ARM2_ENCODER_FEEDBACK || g_enc2.directionKnown)));
    FZ(!g_ptp.reference || g_ptp.active || (ptp_counts_in_range(0u,g_arm1.usteps) && ptp_counts_in_range(1u,g_arm2.usteps)));
}
static struct { uint8 en, ref, ptp, tr; home_phase_t ph; } fz;
static char fuzzReason[16][40]; static unsigned fuzzReasonN[16];
static void fuzz_note_stop(void) {
    const char *e=NULL, *nx; char r[40]; unsigned i,n;
    for(nx=strstr(simLog,"STOP: "); nx; nx=strstr(nx+1,"STOP: ")) e=nx;
    if(!e) return;
    e+=6; for(n=0; n<sizeof(r)-1u && e[n] && e[n]!=';' && e[n]!=13; n++) r[n]=e[n]; r[n]=0;
    for(i=0; i<16u && fuzzReason[i][0]; i++) if(!strcmp(fuzzReason[i],r)) { fuzzReasonN[i]++; return; }
    if(i<16u) { strcpy(fuzzReason[i],r); fuzzReasonN[i]=1u; }
}
static void fuzz_observe(void) {
    fuzzCov[0]+=(!fz.en && g_motionEnabled); fuzzCov[1]+=(fz.en && !g_motionEnabled);
    fuzzCov[2]+=(!fz.ref && g_ptp.reference);
    fuzzCov[3]+=(fz.ptp && !g_ptp.active && g_motionEnabled && fz.tr!=3u && g_traj.state!=3u && g_traj.state!=4u);
    fuzzCov[4]+=(fz.tr==3u && g_traj.state==4u);
    fuzzCov[5]+=(fz.ph!=HOME_IDLE && g_home.phase==HOME_IDLE && g_homed1 && g_homed2);
    if(fz.en && !g_motionEnabled) fuzz_note_stop();
    if(!fz.ref && g_ptp.reference) fuzzQ1=fuzzQ2=0;
    if(fz.tr==3u && g_traj.state==4u && g_traj.count) { fuzzQ1=g_traj.nodes[g_traj.count-1u].q1; fuzzQ2=g_traj.nodes[g_traj.count-1u].q2; }
    fz.en=g_motionEnabled; fz.ref=g_ptp.reference; fz.ptp=g_ptp.active; fz.tr=g_traj.state; fz.ph=g_home.phase;
}
static void fuzz_run(unsigned n) {
    while(n--) {
        unsigned p1=simPulse1, p2=simPulse2; uint8 en=g_motionEnabled;
        int hazard=simFault||simLimit; uint32 epoch=g_ptp.epoch, cseq=g_controlSequence;
        int32 u1=g_arm1.usteps, u2=g_arm2.usteps; int s1=simStep1, s2=simStep2, stall=simStall;
        home_phase_t ph0=g_home.phase; uint8 arm0=g_home.arm;
        if(simLogN>900000u) { simLogN=0; simLog[0]=0; }
        if(fuzzHb && g_jogMs-g_lastKeyMs>=40u) sim_keys(fuzzKeys);
        cycle(0);
        if(simPulse1!=p1 || simPulse2!=p2) FZ(en && !hazard);
        /* Position bookkeeping: what the firmware counts must be what the (unstalled) motors did; homing re-bases the counters. */
        if(g_home.phase==ph0 && g_home.arm==arm0) {
            FZ(g_arm2.usteps-u2==simStep2-s2);
            if(!stall && !simStall) FZ(g_arm1.usteps-u1==simStep1-s1);
        }
        /* Whatever the motor does (including a stall), the arm1 following error stays bounded while the firmware is still
         * enabled: either it is small, or the firmware has already stopped. */
        if(g_motionEnabled && g_directionKnown && g_arm1.hasEnc && !g_enc.badStreak)
            FZ(abs_i32(cmd_lsb()-g_enc.pos) < SLIP_FAIL_LSB+64);
        FZ(g_ptp.epoch>=epoch && g_controlSequence>=cseq);
        fuzz_invariants();
        fuzz_observe();
    }
}
static void fuzz_send(uint8 type, uint32 seq, int16 a, int16 b, uint16 v) {
    uint32 epoch=(fuzzClean||fuzz_rnd(12))?g_ptp.epoch:g_ptp.epoch-1u; uint32 mode=fuzzClean?99u:fuzz_rnd(24);
    if(!fuzzClean && !fuzz_rnd(10) && seq>2u) seq-=2u;
    packetType=type; ptp_send(seq,epoch,a,b,v,mode==0?1:(mode==1?2:(mode==2?3:0))); packetType=1u;
    txq_flush_blocking();
}
static void fuzz_goal(void) {
    /* Happy-path step for the current state, so the noise below has deep states to disturb. */
    fuzzClean=1;
    if(!g_motionEnabled) {
        if(fuzz_rnd(2)) { simFault=0; simLimit=0; simBadEncoder=0; simStall=0; }
        simHome1Mode=simHome2Mode=0; fuzzKeys=0; fuzzHb=1u; sim_keys(0);
        fuzz_run(fuzz_rnd(3)?1u:60u);
        fuzz_send(7,g_controlSequence+1u,0,0,0);
    } else if(g_home.phase!=HOME_IDLE || g_ptp.active) {
        fuzz_run(1u+fuzz_rnd(fuzz_rnd(3)?2000u:8000u));
    } else if(!g_ptp.reference) {
        fuzzKeys=0; fuzzHb=1u; sim_keys(0);
        if(fuzz_rnd(4)) fuzz_send(5,g_ptp.sequence+1u,0,0,0); else fuzz_send(8,g_controlSequence+1u,0,0,0);
    } else if(g_traj.state==1u) {
        fuzz_send(3,g_ptp.sequence+1u,(int16)(fuzzQ1+fuzz_q(-4,4)),(int16)(fuzzQ2+fuzz_q(-4,4)),(uint16)fuzz_q(100,500));
    } else if(g_traj.state==2u) {
        fuzz_send(4,g_ptp.sequence+1u,0,0,0);
    } else if(fuzz_rnd(3)==0) {
        fuzz_send(2,g_ptp.sequence+1u,fuzzQ1,fuzzQ2,(uint16)fuzz_q(3,6));
    } else {
        int16 q1=fuzz_q(g_jointMinCd[0]-20,g_jointMaxCd[0]+20), q2=fuzz_q(g_jointMinCd[1]-20,g_jointMaxCd[1]+20);
        uint32 before=g_ptp.sequence;
        fuzz_send(1,g_ptp.sequence+1u,q1,q2,(uint16)fuzz_q(20,600));
        if(g_ptp.sequence!=before) { fuzzQ1=q1; fuzzQ2=q2; }
    }
    fuzzClean=0;
}
static void fuzz_test(uint32 seed) {
    fuzzSeed=seed; fuzzS=seed*2654435761u+12345u;
    for(fuzzOp=0; fuzzOp<6000u; fuzzOp++) {
        uint32 pseq=g_ptp.sequence+1u, cseq=g_controlSequence+1u;
        if(fuzz_rnd(100) < (((fuzzOp/250u)&1u) ? 55u : 100u)) { fuzz_goal(); fuzz_invariants(); fuzz_observe(); continue; } /* quiet epochs let long moves finish */
        switch(fuzz_rnd(30)) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: fuzz_run(1u+fuzz_rnd(fuzz_rnd(10)==0?4000u:80u)); break;
        case 8: fuzzKeys=(uint8)fuzz_rnd(16); sim_keys(fuzzKeys); break;
        case 9: fuzzKeys=0; sim_keys(0); break;
        case 10: if(fuzz_rnd(3)==0) {
                jog_handle_byte(JOG_CMD_STOP);
                FZ(!g_motionEnabled && simPins[PIN_EN1] && simPins[PIN_EN2]);
            } break;
        case 11: fuzz_send(7,cseq,0,0,0); break;
        case 12: fuzz_send(8,cseq,0,0,0); break;
        case 13: fuzz_send(5,pseq,0,0,0); fuzzQ1=fuzzQ2=0; break;
        case 14: case 15: {
                int16 q1=fuzz_q(g_jointMinCd[0]-150,g_jointMaxCd[0]+150), q2=fuzz_q(g_jointMinCd[1]-150,g_jointMaxCd[1]+150);
                uint32 before=g_ptp.sequence;
                fuzz_send(1,pseq,q1,q2,(uint16)fuzz_q(15,610));
                if(g_ptp.sequence!=before) { fuzzQ1=q1; fuzzQ2=q2; }
            } break;
        case 16: fuzz_send(2,pseq,fuzzQ1,fuzzQ2,(uint16)fuzz_q(2,6)); break;
        case 17: case 18: fuzz_send(3,pseq,(int16)(fuzzQ1+fuzz_q(-4,4)),(int16)(fuzzQ2+fuzz_q(-4,4)),(uint16)fuzz_q(100,500)); break;
        case 19: fuzz_send(4,pseq,0,0,0); break;
        case 20: if(fuzz_rnd(3)==0) fuzzHb^=1u; else fuzzHb=1u; break;
        case 21: if(!fuzz_rnd(6)) simFault=!simFault; else if(fuzz_rnd(3)==0) simFault=0;
                 if(!fuzz_rnd(8)) simLimit=!simLimit; else if(fuzz_rnd(3)==0) simLimit=0; break;
        case 22: simHome1Mode=(int)fuzz_rnd(3)-1; simHome2Mode=(int)fuzz_rnd(3)-1; if(fuzz_rnd(2)) simHome1Mode=simHome2Mode=0; break;
        case 23: simBadEncoder=!fuzz_rnd(12) ? !simBadEncoder : (fuzz_rnd(3)==0 ? 0 : simBadEncoder);
                 simStall=!fuzz_rnd(12) ? !simStall : (fuzz_rnd(3)==0 ? 0 : simStall); break;
        case 24: sim_uart_push(0x41); HostUart_errorStatus=(uint8)(1u<<fuzz_rnd(5)); jog_rx_service(); break;
        case 25: {
                uint8 ax=(uint8)fuzz_rnd(2), rates=(uint8)fuzz_rnd(2);
                if(rates) fuzz_send(10,cseq,fuzz_q(19,ax?610:810),fuzz_q(90,ax?12100:20100),ax);
                else fuzz_send(9,cseq,fuzz_q(-6000,100),fuzz_q(-100,6000),ax);
            } break;
        default: fuzz_run(1u+fuzz_rnd(20u)); break;
        }
        fuzz_invariants();
        fuzz_observe();
    }
    printf("FUZZ enable=%u stop=%u ref=%u ptp=%u traj=%u home=%u\n",fuzzCov[0],fuzzCov[1],fuzzCov[2],fuzzCov[3],fuzzCov[4],fuzzCov[5]);
    { unsigned i; for(i=0; i<16u && fuzzReason[i][0]; i++) printf("FUZZSTOP %u %s\n",fuzzReasonN[i],fuzzReason[i]); }
}
static int32 lsb_to_out_deg100(int32 lsb) { return (int32)(((int64)lsb * 36000) / ((int64)ENCODER_CPR * GEAR_RATIO)); }
static void snapshot_interrupt(void) {
    assert(simInts); /* Inject an event immediately after the snapshot unlocks. */
    jog_step(&g_arm1,1);jog_step(&g_arm2,1);
    g_traj.lagMaxCy=301u*SU_CY_PER_US;g_traj.slipCy=402u*SU_CY_PER_US;
#if HW_EVT
    g_evtMaxCy=999u*SU_CY_PER_US;g_evtBusyCy=1888u*SU_CY_PER_US;g_evtEvents=777u;
#endif
}
static void status_upload_test(const char *t) {
    static const char *essential[]={"STATE fw=","CONTROL seq=","PTP_STATE ","TRAJ_STATE ","TELEM t="};
    static const char *diagnostic[]={"MOTION_RATES ","PTP_CHECK ","LINK ","PTP_RX ","TIMING ","ENC_DIAG ","TELEM_DIAG "};
    unsigned i,idleBytes,uploadBytes,queued,after,firstLine,crc,p1,p2;const char *at,*ack;
    setup();g_enc2.directionKnown=1u;g_enc2.incDir=1u;
    traj_send(5,1u,0,0,0);assert(g_ptp.reference && g_homed1 && g_homed2);status_drain();
    simLogN=0;simLog[0]=0;stage_status();status_drain();idleBytes=simLogN;
    for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(strstr(simLog,diagnostic[i]));
    p1=simPulse1;p2=simPulse2;
    traj_send(2,2u,0,0,!strcmp(t,"status_upload_stream")?176u:4u);assert(g_traj.state==TRAJ_UPLOADING && g_traj.count==1u);
    simLogN=0;simLog[0]=0;
    if(!strcmp(t,"status_upload_volume")) {
        stage_status();status_drain();uploadBytes=simLogN;
        printf("STATUS_UPLOAD idle_bytes=%u upload_bytes=%u wire_ms=%.2f/%.2f\n",idleBytes,uploadBytes,
               idleBytes*1000.0/11520.0,uploadBytes*1000.0/11520.0);fflush(stdout);
        assert(uploadBytes*2u<idleBytes);
        at=simLog;
        for(i=0;i<sizeof(essential)/sizeof(essential[0]);i++){at=strstr(at,essential[i]);assert(at);at++;}
        for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(!strstr(simLog,diagnostic[i]));
        assert(!strstr(simLog,"HWOFFLOAD ") && !g_txqDropped);
    } else if(!strcmp(t,"status_upload_backlog")) {
        /* An accepted node has queued its receipt; periodic work must not enlarge that backlog. */
        packetType=3u;ptp_send(3u,g_ptp.epoch,0,0,100u,0);packetType=1u;
        assert(g_traj.count==2u && g_ptp.sequence==3u && txq_pending());
        queued=TXQ_SIZE-1u-txq_free();stage_status();
        for(i=0;i<STATUS_LINES+2u;i++)stage_status_service();
        after=TXQ_SIZE-1u-txq_free();
        printf("STATUS_UPLOAD ack_queued_bytes=%u after_status_bytes=%u\n",queued,after);fflush(stdout);
        assert(after==queued && g_statusNext==1u);
        txq_flush_blocking();assert(strstr(simLog,"TRAJ_STATE state=1 count=2 total=4 index=0 seq=3 ")==simLog);
        status_drain();
        /* If a status line entered first, ACK latency is bounded by that one complete line. */
        simLogN=0;simLog[0]=0;stage_status();stage_status_service();
        firstLine=TXQ_SIZE-1u-txq_free();assert(firstLine && firstLine<200u);
        for(i=0;i<STATUS_LINES+2u;i++)stage_status_service();
        queued=TXQ_SIZE-1u-txq_free();
        packetType=3u;ptp_send(4u,g_ptp.epoch,0,0,100u,0);packetType=1u;
        assert(g_traj.count==3u && g_ptp.sequence==4u);
        after=TXQ_SIZE-1u-txq_free();
        for(i=0;i<STATUS_LINES+2u;i++)stage_status_service();
        assert(TXQ_SIZE-1u-txq_free()==after && queued==firstLine);
        txq_flush_blocking();ack=strstr(simLog,"TRAJ_STATE state=1 count=3 total=4 index=0 seq=4 ");
        assert(strstr(simLog,"STATE fw=")==simLog && ack && (unsigned)(ack-simLog)==firstLine);
        printf("STATUS_UPLOAD ahead_of_ack_bytes=%u wire_ms=%.2f\n",firstLine,firstLine*1000.0/11520.0);
        status_drain();assert(!g_txqDropped);
    } else if(!strcmp(t,"status_upload_partial")) {
        stage_status();stage_status_service();txq_flush_blocking();assert(g_statusNext==2u);
        stage_status();assert(g_statusNext==2u); /* Periodic requests cannot restart the unfinished block. */
        packetType=2u;ptp_send(2u,g_ptp.epoch,0,0,4u,0);packetType=1u;
        assert(g_traj.count==1u && g_ptp.sequence==2u && g_statusNext==2u);
        txq_flush_blocking();assert(strstr(simLog,"PTP duplicate seq=2; not restarted"));
        status_drain();
        /* The lost original ACK is recovered by periodic TRAJ_STATE without committing the retry twice. */
        at=simLog;
        for(i=0;i<sizeof(essential)/sizeof(essential[0]);i++){at=strstr(at,essential[i]);assert(at);at++;}
        assert(strstr(simLog,"TRAJ_STATE state=1 count=1 total=4 index=0 seq=2 ") && !g_txqDropped);
    } else if(!strcmp(t,"status_upload_stream")) {
        static const char *fresh[]={"STATE fw=","PTP_STATE ","TRAJ_STATE ","TELEM t="};
        unsigned last[4]={0},gap[4]={0},seen[4]={0},scan=0,peak=0,j,budget,elapsed;
        uint32 started;char *end,*line;
        jog_update_time();started=g_jogMs;g_telemHz=10u;g_telemLastMs=g_jogMs;
        /* Same parser/ACK and periodic services, with a conservative 11-byte/ms TX drain budget (115200 8N1). */
        for(i=0;i<8750u;i++) {
            simClock+=(uint64_t)SIM_MHZ*1000u;jog_update_time();elapsed=g_jogMs-started;
            if(i%40u==0u)sim_keys(0);
            if(i%50u==0u && i<8700u){packetType=3u;ptp_send(3u+i/50u,g_ptp.epoch,0,0,100u,0);packetType=1u;}
            if(i%500u==0u)stage_status();
            telem_service();stage_status_service();
            queued=TXQ_SIZE-1u-txq_free();if(queued>peak)peak=queued;
            for(budget=0;budget<11u && txq_pending();budget++)txq_send_one();
            while((end=strchr(simLog+scan,'\n'))!=NULL) {
                line=simLog+scan;
                for(j=0;j<4u;j++)if(!strncmp(line,fresh[j],strlen(fresh[j]))) {
                    if(elapsed-last[j]>gap[j])gap[j]=elapsed-last[j];last[j]=elapsed;seen[j]++;
                }
                scan=(unsigned)(end-simLog)+1u;
            }
        }
        assert(g_motionEnabled && g_traj.state==TRAJ_UPLOADING && g_traj.count==175u && g_ptp.sequence==176u);
        for(j=0;j<4u;j++){if(elapsed-last[j]>gap[j])gap[j]=elapsed-last[j];assert(seen[j]>=10u && gap[j]<750u);}
        assert(gap[3]<250u && g_telemSent>=80u && !g_txqDropped);
        printf("STATUS_UPLOAD stream_nodes=175 state_gap_ms=%u ptp_gap_ms=%u traj_gap_ms=%u telem_gap_ms=%u peak_queue_bytes=%u\n",
               gap[0],gap[1],gap[2],gap[3],peak);
        txq_flush_blocking();status_drain();
        for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(!strstr(simLog,diagnostic[i]));
        traj_send(3u,177u,0,0,100u);assert(g_traj.state==TRAJ_READY && g_traj.count==176u);
    } else if(!strcmp(t,"status_upload_restore")) {
        for(i=3u;i<=5u;i++)traj_send(3u,i,0,0,100u);
        assert(g_traj.state==TRAJ_READY && g_traj.count==4u);
        simLogN=0;simLog[0]=0;stage_status();status_drain();
        for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(strstr(simLog,diagnostic[i]));
        traj_send(4u,6u,0,0,0);assert(g_traj.state==TRAJ_RUNNING && g_ptp.active);
        simLogN=0;simLog[0]=0;stage_status();
        for(i=0;i<100u && g_statusNext;i++)cycle(1);
        assert(g_traj.state==TRAJ_RUNNING && !g_statusNext);
        for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(strstr(simLog,diagnostic[i]));
        for(i=0;i<5000u && g_ptp.active;i++)cycle(1);
        assert(g_traj.state==TRAJ_DONE && g_motionEnabled);
        simLogN=0;simLog[0]=0;stage_status();status_drain();
        for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(strstr(simLog,diagnostic[i]));
    } else {
        /* CRC and ordering rejection remain on the actual parser/accept path during reduced status. */
        crc=g_ptpRxCrc;
        packetType=3u;ptp_send(3u,g_ptp.epoch,0,0,100u,1);packetType=1u;
        assert(g_ptpRxCrc==crc+1u && g_traj.count==1u && g_ptp.sequence==2u);
        txq_flush_blocking();assert(strstr(simLog,"PTP RX dropped: CRC"));
        packetType=3u;ptp_send(4u,g_ptp.epoch,0,0,100u,0);packetType=1u;
        assert(g_traj.count==1u && g_ptp.sequence==2u);
        txq_flush_blocking();assert(strstr(simLog,"PTP refused: trajectory order/time"));
        simLogN=0;simLog[0]=0;
        packetType=3u;ptp_send(3u,g_ptp.epoch,0,0,100u,0);packetType=1u;
        assert(g_traj.count==2u && txq_pending());stage_status();
        if(!strcmp(t,"status_upload_fault")){simFault=1;assert(jog_safety_blocked());assert(g_jogFaulted);}
        else {assert(!strcmp(t,"status_upload_stop"));jog_handle_byte(JOG_CMD_STOP);}
        assert(!g_motionEnabled && !g_traj.state && !g_ptp.reference);
        txq_flush_blocking(); /* No periodic status service is needed to publish the fault/STOP. */
        ack=strstr(simLog,"TRAJ_STATE state=1 count=2 ");at=strstr(simLog,"FAULT_SNAPSHOT ");
        assert(ack && at && ack<at && strstr(simLog,"STOP: ") && strstr(at,"phase=traj_upload"));
        assert(!strstr(simLog,"LINK "));
        status_drain();
        assert(strstr(simLog,"LAST_STOP ") && strstr(simLog,"STATE fw=stage2-v11 enabled=0"));
        for(i=0;i<sizeof(diagnostic)/sizeof(diagnostic[0]);i++)assert(strstr(simLog,diagnostic[i]));
        assert(!g_txqDropped);
    }
    assert(simPulse1==p1 && simPulse2==p2);
}
int main(int argc,char **argv) {
    const char *t=argv[1]; unsigned i, p1, p2;
    assert(argc==2);
    if(!strncmp(t,"status_upload_",14)){status_upload_test(t);puts(t);return 0;}
    if(!strncmp(t,"status_idle_",12)) {
        static const char *prefix[]={"STATE fw=","CONTROL seq=","PTP_STATE ","MOTION_RATES ","PTP_CHECK ",
            "TRAJ_STATE ","LINK ","PTP_RX ","TIMING ","ENC_DIAG ","TELEM_DIAG ","TELEM t="};
        unsigned before,feeds;const char *at,*state;uint32 epoch;
        setup();status_drain();app_start();g_telemHz=0u;
        simLogN=0;simLog[0]=0;simFormatCalls=0u;simRequireFormatInts=1;
        before=simFormatCalls;epoch=g_ptp.epoch;
        if(!strcmp(t,"status_idle_stop")) {
            g_statusNext=8u;g_statusEpoch=epoch;stage_stop("status slice regression",0u);
            txq_flush_blocking();assert(strstr(simLog,"LAST_STOP ") && !strstr(simLog,"STATE fw="));
        } else {
            if(!strcmp(t,"status_idle_epoch")) {g_statusNext=8u;g_statusEpoch=epoch;ptp_invalidate();}
            if(!strcmp(t,"status_idle_repeat")) {g_statusNext=8u;g_statusEpoch=epoch;}
            stage_status();assert(simFormatCalls==before);
        }
        assert(g_statusNext==1u && g_statusEpoch==g_ptp.epoch);
        simFormatCostUs=2500u;feeds=simWdtFeeds;
        for(i=0;i<STATUS_LINES;i++) {
            before=simFormatCalls;app_poll();
            assert(simFormatCalls==before+1u);
            assert(g_statusNext==(i+1u<STATUS_LINES ? i+2u : 0u));
            txq_flush_blocking();
        }
        assert(simWdtFeeds==feeds+STATUS_LINES && wall_ms()<JOG_STATUS_MS);
        at=simLog;
        for(i=0;i<sizeof(prefix)/sizeof(prefix[0]);i++){at=strstr(at,prefix[i]);assert(at);at++;}
        if(STATUS_LINES==13u)assert(strstr(at,"HWOFFLOAD "));
#if !HW_EVT
        if(STATUS_LINES==13u)assert(strstr(simLog,"evt_isr_max_us=0 evt_isr_busy_permille=0 evt_events=0"));
#endif
        state=strstr(simLog,"STATE fw=");
        if(!strcmp(t,"status_idle_stop"))assert(strstr(simLog,"LAST_STOP ")<state);
        if(!strcmp(t,"status_idle_epoch"))assert(g_ptp.epoch!=epoch);
        at=strstr(simLog,"PTP_STATE ");at=strstr(at,"epoch=");
        assert(at && strtoul(at+6,0,10)==g_ptp.epoch);
        puts(t);return 0;
    }
    if(!strcmp(t,"snapshot_telem") || !strcmp(t,"snapshot_stats")) {
        char expected[160];int32 u1,u2;
        setup();txq_flush_blocking();simLogN=0;simLog[0]=0;simRequireFormatInts=1;
        u1=g_arm1.usteps;u2=g_arm2.usteps;
        if(!strcmp(t,"snapshot_telem")) {
            snprintf(expected,sizeof(expected),"p1=%ld p2=%ld e1=%ld err=%ld ",
                (long)(g_ptp.origin[0]-u1),(long)(g_ptp.origin[1]-u2),
                (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),(long)(cmd_lsb()-g_enc.pos));
            simAfterCriticalExit=snapshot_interrupt;telem_line();
            assert(!simAfterCriticalExit && g_arm1.usteps==u1+1 && g_arm2.usteps==u2+1);
            txq_flush_blocking();assert(strstr(simLog,expected));
#if ARM2_ENCODER_FEEDBACK
            assert(strstr(simLog,"e2=0 err2=0 ") && g_enc2.cmdLsbX100!=0);
#endif
        } else {
            g_traj.lagMaxCy=100u*SU_CY_PER_US;g_traj.slipCy=200u*SU_CY_PER_US;
            g_statusNext=9u;simAfterCriticalExit=snapshot_interrupt;stage_status_line();
            assert(!simAfterCriticalExit);txq_flush_blocking();
            assert(strstr(simLog,"traj_lag_max_us=100 traj_slip_us=200"));
#if HW_EVT
            g_jogMs=100u;g_evtMaxCy=10u*SU_CY_PER_US;g_evtBusyCy=100u*SU_CY_PER_US;g_evtEvents=20u;
            g_evtRunMs=g_jogMs-10u;g_statusNext=13u;
            simAfterCriticalExit=snapshot_interrupt;stage_status_line();
            assert(!simAfterCriticalExit);txq_flush_blocking();
            assert(strstr(simLog,"evt_isr_max_us=10 evt_isr_busy_permille=10 evt_events=20"));
#endif
        }
        puts(t);return 0;
    }
    if(!strcmp(t,"traj_isr_done") || !strcmp(t,"traj_isr_slot")) {
        setup();g_ptp.reference=1u;g_ptp.active=1u;g_ptp.sinceMs=g_jogMs;
        g_traj.state=TRAJ_RUNNING;g_traj.count=1u;g_traj.index=0u;
        g_traj.nodes[0].q1=g_traj.nodes[0].q2=0;g_traj.nodes[0].ms=20u;
        g_ptp.origin[0]=g_arm1.usteps;g_ptp.origin[1]=g_arm2.usteps;
        g_ptp.seg.target[0]=g_arm1.usteps;g_ptp.seg.target[1]=g_arm2.usteps;
        g_ptp.seg.done=g_ptp.seg.total=0u;
        g_traj.elapsedCy=20u*1000u*SU_CY_PER_US;g_traj.clock=SU_NOW();
        p1=simPulse1;p2=simPulse2;
#if HW_EVT
        g_evtRunning=1u;g_fgPassAt=SU_NOW();
        if(!strcmp(t,"traj_isr_slot")) {
            simBeforeSlotCheck=evt_isr;
            assert(jog_slot_free(50u));assert(!simBeforeSlotCheck);
        } else {
            simBeforeTrajCheck=evt_isr;ptp_service();assert(!simBeforeTrajCheck);
        }
        assert(!g_evtRunning && !g_evtStop);
#else
        assert(!jog_slot_free(50u)); /* software path still checks the due node */
        ptp_service();             /* software path still executes it */
#endif
        assert(g_traj.index==g_traj.count && g_ptp.active==2u && simBoundaryChecks==1u);
        assert(simPulse1==p1 && simPulse2==p2);
        simClock+=(uint64_t)SIM_MHZ*(HOME_SETTLE_MS+1u)*1000u;jog_update_time();
        ptp_service();assert(!g_ptp.active && g_traj.state==TRAJ_DONE && g_motionEnabled);
        puts(t);return 0;
    }
    if(!strcmp(t,"traj_status_cost")) {
        uint64_t start; unsigned requested=0;
        setup();g_enc2.directionKnown=1u;g_enc2.incDir=1u;
        packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;txq_flush_blocking();
        g_traj.state=3u;g_traj.count=2u;g_traj.index=1u;
        g_traj.nodes[1].q1=-139;g_traj.nodes[1].q2=207;g_traj.nodes[1].ms=397; /* 785 steps = 1977/s, 79% of TRAJ_RATE_MAX (was 248 ms = 3165/s = 79% of the old 4000 cap) */
        g_ptp.active=1u;{int32 pos[2];pos[0]=g_arm1.usteps;pos[1]=g_arm2.usteps;g_traj.elapsedCy=0;g_traj.clock=g_traj.lastStep=SU_NOW();traj_segment(&g_traj,&g_ptp.seg,g_ptp.origin,pos);}
#if HW_EVT
        evt_start(); /* This fixture bypasses RUN; start the hardware owner explicitly. */
#endif
        assert(g_ptp.seg.total==785u); CyILO_Start1K();CyWdtStart(CYWDT_128_TICKS,CYWDT_LPMODE_NOCHANGE);start=simClock;
        simFormatCostUs=2500u; /* 12 format calls in one burst cost about the observed 30 ms loop gap. */
        while(g_motionEnabled && g_traj.index<g_traj.count && simClock-start<600000u*SIM_MHZ) {
            if(!requested && simClock-start>=389000u*SIM_MHZ){requested=1;stage_status();} /* 8 ms before the planned end, as before */
            if(g_jogMs-g_lastKeyMs>=35u)short_send(0,0);
            app_poll();simClock+=SIM_MHZ;
        }
        if(!g_motionEnabled)fprintf(stderr,"status stress stop: %s\n",g_lastStop);
        assert(requested && g_motionEnabled && g_traj.index==g_traj.count);
        assert(simBoundaryChecks==1u && g_ptp.seg.done==785u);
        { unsigned long lag; const char *p; simLogN=0;simLog[0]=0;g_statusNext=9u;stage_status_line();txq_flush_blocking();
          p=strstr(simLog,"traj_lag_max_us="); assert(p && sscanf(p,"traj_lag_max_us=%lu",&lag)==1 && lag==g_traj.lagMaxCy/SU_CY_PER_US); }
        printf("%s: 397 ms / 785 steps completed with injected formatting cost\n",t);return 0;
    }
    if(!strcmp(t,"jog_arm2_limit")) {
        /* jog_axis_tick() must refuse an outward arm2 jog at the elbow bound and say so, with or without arm2 feedback. */
        jog_axis_t *a=&g_arm2;
        if(!ARM2_ENCODER_FEEDBACK) simBadEncoder2=1; /* the delivery profile has no arm2 encoder to learn a direction from */
        setup();
        if(ARM2_ENCODER_FEEDBACK) { g_enc2.directionKnown=1u; g_enc2.incDir=simPolarity2>0?1u:0u; }
        packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;
        assert(g_ptp.reference && g_homed1 && g_homed2 && g_directionKnown);
        assert(ARM2_ENCODER_FEEDBACK || !g_enc2.directionKnown);
        g_ptp.reference=0; simHome1Mode=simHome2Mode=-1;
        g_arm1.usteps=g_ptp.origin[0];
        a->usteps=g_ptp.origin[1]-ptp_offset(1u,g_jointMaxCd[JOINT_ELBOW]);
        assert(!joint_step_allowed(a,-1) && joint_step_allowed(a,1));
        txq_flush_blocking();simLogN=0;simLog[0]=0;
        a->want=-1; jog_axis_tick(a); txq_flush_blocking();
        assert(a->limitHit && a->moving==0 && strstr(simLog,"arm2: configured joint limit"));
        puts(t);return 0;
    }
    if(!strcmp(t,"origin_retry")) {
        /* enc_set_origin() averages 12 frames at ENABLE: a single bad frame is retried, a dead encoder still refuses. */
        setup();
        simBadReads=1; assert(enc_set_origin() && g_encRetries>0 && simBadReads==0);
        simBadReads=ENCODER_READ_RETRIES; assert(!enc_set_origin());
        simBadReads=0; simBadEncoder=1; assert(!enc_set_origin());
        puts(t);return 0;
    }
    if(!strcmp(t,"hot_arith")) {
        /* The 32-bit fast paths must equal the original 64-bit arithmetic, inside and outside their bounds. */
        const uint32 totals[]={1u,2u,3u,785u,2000u,65535u,65536u,70000u};
        const uint32 durations[]={(SIM_MHZ*1000u)*50u,(SIM_MHZ*1000u)*248u,(SIM_MHZ*1000u)*249u+7u,(SIM_MHZ*1000u)*500u};
        unsigned a, ti, di; int32 v;
        setup();
        for(a=0;a<2;a++) {
            int32 rev=a==0 ? MOTOR_FULLSTEPS_REV*USTEP_DIV*GEAR_RATIO : ARM2_MOTOR_FULLSTEPS_REV*USTEP_DIV*ARM2_GEAR_RATIO_X100/100;
            g_ptp.origin[a]=0;
            for(v=-150000;v<=150000;v+=(v>-70000 && v<70000)?1:97) {
                int64 scaled=(int64)v*36000; int32 want=(int32)(scaled>=0 ? (scaled+rev/2)/rev : -((-scaled+rev/2)/rev));
                assert(ptp_angle_from_counts((uint8)a,-v)==want);
            }
            for(v=-40000;v<=40000;v++) {
                int64 n=(int64)v*rev; int32 want=(int32)(n>=0 ? (n+18000)/36000 : -((-n+18000)/36000));
                assert(ptp_offset((uint8)a,v)==want);
            }
        }
        for(ti=0;ti<sizeof(totals)/sizeof(totals[0]);ti++) for(di=0;di<sizeof(durations)/sizeof(durations[0]);di++) {
            uint32 k;
            for(k=1;k<=totals[ti];k+=(k<3000u || totals[ti]-k<3000u)?1u:61u)
                assert(traj_due_cy(k,durations[di],totals[ti])==(uint32)((int64)k*durations[di]/totals[ti]));
            assert(traj_due_cy(totals[ti],durations[di],totals[ti])==durations[di]); /* the last step is due exactly at the end */
        }
        puts(t);return 0;
    }
    if(!strcmp(t,"traj_slip")) {
        /* Ride the limit: a step far behind its due time stretches the plan clock instead of stopping; only an
         * unsustainable total stretch (TRAJ_SLIP_MAX_MS) stops, with its own reason. */
        setup(); txq_flush_blocking(); simLogN=0;simLog[0]=0;
        g_traj.state=3u;g_traj.count=2u;g_traj.index=1u;g_traj.nodes[1].ms=248u;
        g_ptp.active=1u;g_ptp.seg.done=0u;g_ptp.seg.total=100u;g_traj.slipCy=0u;g_traj.lagMaxCy=0u;
        g_traj.lastStep=SU_NOW(); simClock+=1000u*SU_CY_PER_US;
        g_traj.elapsedCy=10000u*SU_CY_PER_US;g_traj.clock=SU_NOW(); /* step 1 is due at 2.48 ms: 7.52 ms late */
        traj_service();
        assert(g_motionEnabled && g_ptp.seg.done==1u);
        /* register reads cost a few cycles in the fixture, hence the +-5 us windows */
        assert(g_traj.lagMaxCy/SU_CY_PER_US>=7515u && g_traj.lagMaxCy/SU_CY_PER_US<=7525u);
        assert(g_traj.slipCy/SU_CY_PER_US>=6515u && g_traj.slipCy/SU_CY_PER_US<=6525u);   /* everything beyond the 1 ms soft margin */
        assert(g_traj.elapsedCy/SU_CY_PER_US>=3475u && g_traj.elapsedCy/SU_CY_PER_US<=3485u); /* 10 ms - 6.52 ms: now 1 ms behind step 1's due time */
        g_traj.lastStep=SU_NOW(); simClock+=1000u*SU_CY_PER_US;
        g_traj.elapsedCy=7960u*SU_CY_PER_US;g_traj.clock=SU_NOW();   /* step 2 due at 4.96 ms: 3 ms late */
        g_traj.slipCy=(TRAJ_SLIP_MAX_MS*1000u-1000u)*SU_CY_PER_US;    /* 1 ms of stretch left */
        traj_service();
        assert(!g_motionEnabled && strstr(g_lastStop,"reason=\"trajectory rate not sustained\"") && g_ptp.seg.done==1u);
        puts(t);return 0;
    }
    if(!strcmp(t,"pulse_blocked")) {
        /* The one place a trajectory/PTP step meets the pins: a step the axis refuses must stop the machine, with the reason. */
        int8 both[2]={1,1}, second[2]={0,1}; int32 u1,u2;
        setup(); txq_flush_blocking(); simLogN=0;simLog[0]=0;
        u1=g_arm1.usteps;u2=g_arm2.usteps;
        assert(pulse_axes(both,"trajectory pulse blocked") && g_arm1.usteps==u1+1 && g_arm2.usteps==u2+1 && g_motionEnabled);
        assert(pulse_axes((int8[2]){0,0},"trajectory pulse blocked") && g_arm1.usteps==u1+1 && g_arm2.usteps==u2+1); /* no step, no pulse */
        g_motionEnabled=0u; /* jog_step() refuses every step while motion is disabled */
        assert(!pulse_axes(second,"trajectory pulse blocked") && g_arm1.usteps==u1+1 && g_arm2.usteps==u2+1);
        assert(strstr(g_lastStop,"reason=\"trajectory pulse blocked\""));
        puts(t);return 0;
    }
    if(!strcmp(t,"stop_latch")) {
        const char *fill="xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\r\n";
        char saved[800];
        setup(); txq_flush_blocking(); simLogN=0;simLog[0]=0;
        g_traj.state=3u;g_traj.count=2u;g_traj.index=1u;g_traj.nodes[1].ms=248u;
        g_ptp.active=1u;g_ptp.seg.done=90u;g_ptp.seg.total=100u;
        g_traj.elapsedCy=268001u*SU_CY_PER_US;g_traj.clock=SU_NOW();
        while(txq_free()>=strlen(fill)) JLOG(fill); /* first stop messages cannot fit */
        traj_service(); assert(!g_motionEnabled && g_txqDropped);
        assert(strstr(g_lastStop,"reason=\"trajectory timing overrun\"") && strstr(g_lastStop,"idx=1/2") &&
               strstr(g_lastStop,"done=90 total=100"));
        { unsigned long elapsed,planned,late;
          assert(sscanf(strstr(g_lastStop,"elapsed_us="),"elapsed_us=%lu plan_us=%lu late_us=%lu",&elapsed,&planned,&late)==3);
          assert(elapsed>268000u && planned==248000u && late==elapsed-planned && late>20000u); }
        strcpy(saved,g_lastStop);txq_flush_blocking();simLogN=0;simLog[0]=0;
        stage_status();txq_flush_blocking();assert(strstr(simLog,saved)==simLog);
        jog_handle_byte(JOG_CMD_STOP);txq_flush_blocking();assert(!strcmp(saved,g_lastStop));
        simLogN=0;simLog[0]=0;stage_status();txq_flush_blocking();assert(strstr(simLog,saved));
        sim_keys(0);stage_enable();txq_flush_blocking();simLogN=0;simLog[0]=0;
        stage_status();txq_flush_blocking();assert(!strstr(simLog,"LAST_STOP")); /* no extra motion-time output */
        stage_stop("new test stop",0u);assert(g_lastStopId==2u && strstr(g_lastStop,"new test stop"));
        puts(t);return 0;
    }
    if(!strcmp(t,"optional_arm2")) {
        assert(!ARM2_ENCODER_FEEDBACK); simBadEncoder2=1;
        assert(app_boot()); setup(); assert(g_arm1.hasEnc && !g_arm2.hasEnc);
        assert(!g_enc2.directionKnown); packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;
        assert(g_ptp.reference && !simPulse1 && !simPulse2);
        stage_status();status_drain();assert(strstr(simLog,"arm2fb=0") && !strstr(simLog," e2="));
        { uint32 epoch=g_ptp.epoch; ptp_send(2,epoch,0,100,200,0);
          for(i=0;i<20000 && g_ptp.active;i++)cycle(1);
          assert(g_motionEnabled && !g_ptp.active && simPulse2); }
        jog_home_all(); run_until(HOME_STRAIGHT_SETTLE,2);
        for(i=0;i<2000 && g_home.phase!=HOME_IDLE;i++)cycle(1);
        assert(g_motionEnabled && g_home.phase==HOME_IDLE && g_ptp.reference && !g_arm2.hasEnc && !g_enc2.directionKnown);
        { /* arm2 feedback off: the closed loop reads board 1 only (18 clocks) and never touches the arm2 encoder state */
          int32 pos2=g_enc2.pos; uint8 bad2=g_enc2.badStreak, primed2=g_enc2.primed;
          simSsiBits=0; simClock+=(SIM_MHZ*1000u); assert(enc_sample() && SIM_SSI_LAST_CLOCKS()==SIM_SSI_CLK18);
          assert(g_enc2.pos==pos2 && g_enc2.badStreak==bad2 && g_enc2.primed==primed2); }
        g_enc.directionKnown=1u; simBadEncoder=1;
        for(i=0;i<ENC_BAD_LIMIT;i++)service_encoder(); assert(!g_motionEnabled);
        puts(t);return 0;
    }
    if(!strncmp(t,"txdma_",6)) {
#if HW_TXDMA
        char big[300]; unsigned n, sent=0u, k; uint16 tail0;
        hw_init(); assert(simDmaInit);
        simLogN=0; simLog[0]=0;
        if(!strcmp(t,"txdma_wait_drq")) {
            uint16 free0;
            simDmaHoldDrq=1;simDmaByteCy=SIM_MHZ*87u;
            JLOG("0123456789");tail0=g_txqTail;free0=txq_free();txq_send_one();
            assert(simDmaEnabled && !simDmaActive && g_txDmaLen==10u);
            for(k=0;k<3;k++) {
                simClock+=SIM_MHZ*100u;txq_send_one();
                assert(g_txqTail==tail0 && g_txDmaLen==10u && txq_free()==free0);
                assert(simDmaTransfers==1u && !simLogN);
            }
            simDmaHoldDrq=0;txq_send_one();
            assert(simDmaActive && simDmaEnabled && g_txqTail==tail0);
            simClock+=SIM_MHZ*1000u;txq_send_one();
            assert(!simDmaEnabled && !g_txDmaLen && !txq_pending());
            assert(g_txqTail==(uint16)((tail0+10u)%TXQ_SIZE) && !strcmp(simLog,"0123456789"));
            txq_send_one();assert(g_txqTail==(uint16)((tail0+10u)%TXQ_SIZE)); /* retire once */
        } else if(!strcmp(t,"txdma_order")) {
            /* more text than the ring holds, in pieces, across the wrap: it arrives complete and in order */
            for(n=0;n<sizeof(big)-1;n++) big[n]=(char)('a'+(n%26)); big[sizeof(big)-1]=0;
            for(k=0;k<20;k++) {
                JLOG(big); sent+=(unsigned)strlen(big);
                if(k%3==2) txq_flush_blocking();
            }
            txq_flush_blocking();
            assert(!txq_pending() && !g_txqDropped && simLogN==sent && simDmaTransfers>=2u);
            for(n=0;n<sent;n++) assert(simLog[n]==(char)('a'+((n%(sizeof(big)-1))%26)));
        } else {
            /* while a transfer is running the queue does not give its space back, and a finished one is retired */
            simDmaByteCy=SIM_MHZ*87u;                      /* 115200 baud: 87 us a byte */
            JLOG("0123456789"); tail0=g_txqTail; txq_send_one();
            assert(g_txDmaLen==10u && txq_pending());
            JLOG("abcdefghij");                             /* queued behind it */
            txq_send_one(); assert(g_txDmaLen==10u && g_txqTail==tail0);
            simClock+=(uint64_t)SIM_MHZ*300u; txq_send_one();   /* 300 us of the 870 us: still on the wire */
            assert(g_txDmaLen==10u && g_txqTail==tail0);
            simClock+=(uint64_t)SIM_MHZ*700u; txq_send_one();   /* 1000 us: retired, the next run starts */
            assert(g_txDmaLen==10u && g_txqTail==(uint16)((tail0+10u)%TXQ_SIZE));
            simClock+=(uint64_t)SIM_MHZ*2000u; txq_send_one();
            assert(g_txDmaLen==0u && !txq_pending() && !strcmp(simLog,"0123456789abcdefghij"));
            /* a stuck wire is given up after 100 ms like the interrupt-driven queue */
            simDmaByteCy=SIM_MHZ*1000000u; JLOG("xyz"); txq_flush_blocking();
            assert(g_txqDropped==1u && !txq_pending() && !simDmaEnabled);
        }
#endif
        puts(t);return 0;
    }
    if(!strcmp(t,"pin_irq_kill")) {
#if HW_PINIRQ
        int32 u1;
        setup(); assert(simPinIrqPrio==0 && !simStepKill);
        jog_step(&g_arm1,1); assert(simPulse1==1u);
        /* nFAULT drops for a moment: the interrupt has killed the PWMs before anyone looked at the pin, so a step that
         * the pin check no longer refuses still does not leave the chip */
        simFault=1; (void)SU_NOW(); assert(simPinIrqCount==1u && simStepKill);
        simFault=0; u1=g_arm1.usteps;
        jog_step(&g_arm1,1); assert(g_arm1.usteps==u1+1 && simPulse1==1u);
        service_motion(); assert(!g_motionEnabled && strstr(g_lastStop,"pin interrupt"));
        /* only an operator enable clears it */
        g_homeNeedKeyRelease=0u; sim_keys(0); stage_enable(); assert(g_motionEnabled && !simStepKill);
        jog_step(&g_arm1,1); assert(simPulse1==2u);
        /* the global limit does the same */
        simLimit=1; (void)SU_NOW(); assert(simPinIrqCount==2u && simStepKill);
#endif
        puts(t);return 0;
    }
    if(!strcmp(t,"dead_kill")) {
#if HW_DEAD
        int32 u1;
        setup(); assert(simDeadStarted && !simDeadKilled);
        /* fed by the foreground, the deadman never trips */
        for(i=0;i<500;i++){ simClock+=SIM_MHZ*1000u; alive_kick(); assert(!deadman_tripped()); }
        jog_step(&g_arm1,1); assert(simPulse1==1u);
        /* 150 ms without a pass: the PWMs are killed in hardware, a requested pulse never leaves the chip, and the next
         * pass notices and stops the stage */
        simClock+=(uint64_t)SIM_MHZ*150000u; u1=g_arm1.usteps;
        jog_step(&g_arm1,1); assert(g_arm1.usteps==u1+1 && simPulse1==1u);
        service_motion(); assert(!g_motionEnabled && strstr(g_lastStop,"hardware deadman"));
        /* only an operator enable clears the latch */
        g_homeNeedKeyRelease=0u; sim_keys(0); stage_enable(); assert(g_motionEnabled && !simDeadKilled);
        jog_step(&g_arm1,1); assert(simPulse1==2u);
#endif
        puts(t);return 0;
    }
    if(!strncmp(t,"step_hw_",8)) {
#if HW_STEP
        uint64_t t0; int32 u1, u2; int8 both[2];
        setup(); assert(simStepStarted==3u);
        if(!strcmp(t,"step_hw_request")) {
            /* one step = one register write and no busy-wait: only the DIR set-up (when the direction changes) takes time */
            u1=g_arm1.usteps; g_arm1.lastDirLevel=0xFFu; t0=simClock;
            jog_step(&g_arm1,1); assert(g_arm1.usteps==u1+1 && simStepWrites1==1u && simStepWrites2==0u && simPulse1==1u);
            assert(simClock-t0<(uint64_t)SIM_MHZ*(DIR_SETUP_US+8u));
            t0=simClock; jog_step(&g_arm1,1);
            assert(g_arm1.usteps==u1+2 && simStepWrites1==2u && simClock-t0<(uint64_t)SIM_MHZ*8u);   /* same direction: no wait at all */
            u2=g_arm2.usteps; jog_step(&g_arm2,-1);
            assert(g_arm2.usteps==u2-1 && simStepWrites2==1u && simPulse2==1u);
            /* both axes of one coordinated step are separate requests: neither can cancel the other */
            both[0]=1; both[1]=-1; assert(pulse_axes(both,"test"));
            assert(simStepWrites1==3u && simStepWrites2==2u && simPulse1==3u && simPulse2==2u);
        } else {
            /* the STEP pins (STEP1 is the blue LED too) are the PWMs', never driven from software */
            assert(!simPins[PIN_STEP1] && !simPins[PIN_STEP2]);
            jog_step(&g_arm1,1); jog_step(&g_arm2,1); stage_stop("test",0u);
            assert(!simPins[PIN_STEP1] && !simPins[PIN_STEP2]);
        }
#endif
        puts(t);return 0;
    }
    if(!strncmp(t,"usb_",4)) {
        char text[200]; unsigned k, n;
        setup(); txq_flush_blocking();
        for(k=0;k<199u;k++) text[k]=(char)('a'+k%26);
        text[199]=0;
        simLogN=0; simLog[0]=0;
#if HW_USB
        assert(simUsbStarted);
        sim_usb_reset(); simLogN=0; simLog[0]=0;
        if(!strcmp(t,"usb_tx")) {
            /* port open: the text queue leaves in packets of at most 64 bytes, in order, and the UART stays silent */
            simUsbCfg=1; simUsbCfgChanged=1; simUsbDtr=1;
            JLOG(text); txq_send_one();
            assert(g_usbUp && simUsbInit==1 && simUsbPackets==1 && simUsbSentN==64u);
            while(txq_pending()) txq_send_one();
            assert(simUsbSentN==199u && !memcmp(simUsbSent,text,199u) && simUsbPackets==4u && simUsbMaxPacket==64u);
            assert(simLogN==0 && g_txqDropped==0);
        } else if(!strcmp(t,"usb_fallback")) {
            /* port closed: UART.  Port opens mid-queue: the rest goes out by USB, nothing lost or repeated, then back to UART. */
            JLOG(text);
            for(k=0;k<10;k++) txq_send_one();
            n=simLogN; assert(n>0 && n<=10u && simUsbPackets==0);
            simUsbCfg=1; simUsbCfgChanged=1; simUsbDtr=1;
            while(txq_pending()) txq_send_one();
            assert(g_usbUp && n+simUsbSentN==199u && !memcmp(simLog,text,n) && !memcmp(simUsbSent,text+n,simUsbSentN));
            simUsbDtr=0; simLogN=0; simLog[0]=0; JLOG("x\r\n"); txq_flush_blocking();
            assert(!g_usbUp && simLogN==3u && !strcmp(simLog,"x\r\n"));
            /* configured but DTR low (nobody opened the port) is not "up" either */
            simUsbDtr=0; usb_poll(); assert(!g_usbUp);
        } else if(!strcmp(t,"usb_stall")) {
            /* the host stopped reading (CDCIsReady stays low): nothing is sent or lost silently, the flush gives up like a stuck UART */
            uint32 before;
            simUsbCfg=1; simUsbCfgChanged=1; simUsbDtr=1; simUsbReady=0;
            JLOG(text); txq_send_one(); assert(simUsbPackets==0 && txq_pending());
            before=wall_ms(); txq_flush_blocking();
            assert(!txq_pending() && wall_ms()-before>=100u && wall_ms()-before<110u && g_txqDropped==1u && simUsbPackets==0 && simLogN==0);
            simUsbReady=1; JLOG("ok\r\n"); txq_flush_blocking();
            assert(simUsbSentN==4u && !memcmp(simUsbSent,"ok\r\n",4));
        } else if(!strcmp(t,"usb_rx")) {
            /* USB bytes come out in order across 64-byte packets, then the UART's; a discard drops what is pending on both */
            uint8 data[150]; int16 b;
            for(k=0;k<150u;k++) data[k]=(uint8)(0x40u+k%32u);
            simUsbCfg=1; simUsbCfgChanged=1; simUsbDtr=1; sim_usb_push(data,150u);
            for(k=0;k<150u;k++) { b=uart_rx_service(); assert(b==(int16)data[k]); }
            assert(uart_rx_service()==-1 && simUsbInit==1);
            sim_uart_push(0x5A); assert(uart_rx_service()==0x5A && uart_rx_service()==-1);
            sim_usb_push(data,100u); sim_uart_push(0x5B); assert(uart_rx_service()==(int16)data[0]);
            uart_discard_rx(); assert(uart_rx_service()==-1 && simUsbRxI==simUsbRxN);
        } else if(!strcmp(t,"usb_link")) {
            stage_status(); status_drain(); assert(strstr(simLog," link=uart ") && !strstr(simLog," link=usb "));
            simUsbCfg=1; simUsbCfgChanged=1; simUsbDtr=1; simLogN=0; simLog[0]=0; simUsbSentN=0;
            stage_status(); status_drain(); usb_poll();
            simUsbSent[simUsbSentN]=0; assert(strstr(simUsbSent," link=usb ") && !strstr(simUsbSent," link=uart "));
        } else assert(0);
#else
        /* no USB build: the link is the UART and says so */
        assert(g_usbUp==0u);
        stage_status(); status_drain(); assert(strstr(simLog," link=uart ") && !strstr(simLog," link=usb "));
#endif
        puts(t);return 0;
    }
    if(!strncmp(t,"ssi_",4)) {
#if HW_SSI
        uint8 ok=0u; uint64_t t0; unsigned k; unsigned retries;
        setup(); simClock+=SIM_MHZ*2000u; g_enc.primed=1u;
        if(!strcmp(t,"ssi_fifo")) {
            uint32 first, second, expected;
            /* Accumulate each word while in flight; repeated polls must not consume it twice. */
            expected=mock_frame(); ssi_begin(38u);
            for(k=1u;k<=4u;k++) {
                simClock+=SIM_MHZ*44u;
                assert(ssi_done()==(k==4u) && simSsiRead==k);
                assert(ssi_done()==(k==4u) && simSsiRead==k);
            }
            ssi_take(&first,&second); assert(first==expected && second==simSsiFrame2 && ssi_done());
            /* The one-encoder frame has two words, with no stale words from the chain. */
            ssi_begin(18u); simClock+=SIM_MHZ*90u; assert(ssi_done() && simSsiRead==2u);
            ssi_take(&first,&second); assert(first==expected && second==0u);
        } else if(!strcmp(t,"ssi_nonblocking")) {
            /* queueing a frame is a few register writes and masks nothing; the answer is there once 160 us have passed */
            t0=simClock; g_encTry=0u; enc_flight_start(0u);
            assert(simClock-t0<(uint64_t)SIM_MHZ*20u && simInts && g_encFlight);
            simClock+=SIM_MHZ*20u; assert(!enc_flight_poll(&ok) && g_encFlight && simInts);
            simClock+=SIM_MHZ*200u; assert(enc_flight_poll(&ok) && ok && !g_encFlight);
            assert(!g_enc.badStreak && g_enc.pos==0);
        } else if(!strcmp(t,"ssi_snapshot")) {
            /* the command counted against the new angle is the one at the start of the frame, not at its end */
            int32 at; g_encTry=0u; enc_flight_start(0u); at=g_enc.cmdUsteps;
            g_enc.cmdUsteps+=7; g_enc.cmdLsbX100+=7*LSB_PER_USTEP_X100;
            simClock+=SIM_MHZ*300u; assert(enc_flight_poll(&ok) && ok);
            assert(g_enc.lastCmd==at && g_enc.cmdUsteps==at+7);
        } else if(!strcmp(t,"ssi_retry")) {
            /* two unusable frames, then a good one: same retries as the blocking read, no sample lost */
            retries=g_encRetries; simBadReads=2u; g_encTry=0u; enc_flight_start(0u);
            for(k=0;k<20 && !enc_flight_poll(&ok);k++) simClock+=SIM_MHZ*200u;
            assert(k<20 && ok && g_encRetries==retries+2u && !g_enc.badStreak);
        } else if(!strcmp(t,"ssi_timeout")) {
            /* a master that never finishes costs one timeout per attempt, then reads as an unusable frame */
            simSsiDead=1u; g_encTry=0u; enc_flight_start(0u);
            simClock+=SIM_MHZ*500u; assert(!enc_flight_poll(&ok) && g_encFlight);
            for(k=0;k<40 && !enc_flight_poll(&ok);k++) simClock+=SIM_MHZ*200u;
            assert(k<40 && !ok && !g_encFlight && g_enc.badStreak==1u);
            simSsiDead=0u; assert(enc_sample() && !g_enc.badStreak);   /* and the interface works again afterwards */
        } else if(!strcmp(t,"ssi_takeover")) {
            /* a synchronous read, or a re-anchored command, drops a frame that is still on the wire */
            g_encTry=0u; enc_flight_start(0u); assert(g_encFlight);
            assert(enc_sample() && !g_encFlight);
            g_encTry=0u; enc_flight_start(0u); assert(g_encFlight);
            cmd_reset_axis(&g_enc); assert(!g_encFlight);
            assert(enc_sample());
        }
#endif
        puts(t);return 0;
    }
    if(!strcmp(t,"chain_frames")) {
        const int angles[]={0,1234,2048,4095};
        const uint32 frames[]={0,1,0x20000u,0x3ffffu,0x1f8a6u};
        unsigned j;uint32 first,second,expected;
        hw_init();
        for(i=0;i<4;i++)for(j=0;j<5;j++)for(simSsiDummy=0;simSsiDummy<2;simSsiDummy++) {
            simAngleOverride=angles[i];simSsiSecondFrame=(int)frames[j];expected=mock_frame();
            as5045_read_chain(&first,&second);
            assert(first==expected && second==frames[j]);
            assert(SIM_SSI_LAST_CLOCKS()==SIM_SSI_CLK38 && (HW_SSI || simPins[PIN_CSN]) && simInts);
            assert(as5045_read_raw()==expected && SIM_SSI_LAST_CLOCKS()==SIM_SSI_CLK18);
        }
        puts(t);return 0;
    }

    if(!strncmp(t,"dual_",5) && strcmp(t,"dual_snapshot_length")) {
        if(!strcmp(t,"dual_home_reverse"))simPolarity2=-1;
        setup();
        if(!strcmp(t,"dual_home_reverse") || !strcmp(t,"dual_home_stall")) {
            if(!strcmp(t,"dual_home_stall"))simStall2=1;
            jog_home_all();for(i=0;i<250000 && g_motionEnabled && g_home.phase!=HOME_IDLE;i++)cycle(1);
            if(simStall2) { assert(!g_motionEnabled && !g_ptp.reference && strstr(simLog,"no credible rotor motion")); }
            else { assert(dual_ready() && g_homed2 && g_enc2.incDir==0 && g_ptp.reference && g_enc2.origin==g_enc2.pos); }
        } else if(!strcmp(t,"dual_wrap")) {
            int32 first=g_enc.pos;
            for(i=0;i<14;i++) {
                simEncoderBias2+=250;g_enc2.cmdUsteps+=98;g_enc2.cmdLsbX100+=25000;
                simClock+=(SIM_MHZ*1000u);assert(enc_sample());
            }
            assert(g_enc2.pos==3500 && g_enc.pos==first && g_enc2.lastAngle==404);
            for(i=0;i<20;i++) {simEncoderBias2-=250;g_enc2.cmdUsteps-=98;g_enc2.cmdLsbX100-=25000;simClock+=(SIM_MHZ*1000u);assert(enc_sample());}
            assert(g_enc2.pos==-1500 && g_enc.pos==first && g_enc2.lastAngle==3596);
        } else if(!strcmp(t,"dual_loss")) {
            simBadEncoder2=1;
            for(i=0;i<ENC_BAD_LIMIT;i++){simClock+=(SIM_MHZ*1000u);service_encoder();}
            assert(!g_motionEnabled && !g_arm2.hasEnc && g_arm1.hasEnc && !g_enc.badStreak && !g_ptp.reference);
            {
                uint32 epoch=g_ptp.epoch, faults=g_jogFaultCount;
                for(i=0;i<20;i++){simClock+=(SIM_MHZ*1000u);service_encoder();}
                assert(g_ptp.epoch==epoch && g_jogFaultCount==faults && !g_enc.badStreak);
            }
        } else if(!strcmp(t,"dual_gap")) {
            g_enc2.lastMs=wall_ms()-101u;
            service_encoder();assert(g_enc2.trackingLost && !g_enc.trackingLost && !g_motionEnabled);
        } else if(!strcmp(t,"dual_parity")) {
            simSsiSecondFrame=(int)(simSsiFrame2^1u);
            for(i=0;i<ENC_BAD_LIMIT;i++){simClock+=(SIM_MHZ*1000u);service_encoder();}
            assert(!g_motionEnabled && g_enc2.badStreak>=ENC_BAD_LIMIT && !g_enc.badStreak);
        } else if(!strcmp(t,"dual_hold")) {
            g_enc2.incDir=1u;g_enc2.directionKnown=1u;
            jog_axis_settled(&g_arm2);assert(g_arm2.holding && g_arm2.holdTarget==0);
            simEncoderBias2=20;simClock+=(SIM_MHZ*1000u);assert(enc_sample());
            p1=simPulse1;p2=simPulse2;
            for(i=0;i<150;i++)cycle(1);
            assert(g_motionEnabled && abs_i32(g_enc2.pos)<=CL_DEADBAND_LSB && simPulse2>p2 && simPulse1==p1);
        } else if(!strcmp(t,"dual_derate") || !strcmp(t,"dual_follow_stop")) {
            g_enc2.incDir=1u;g_enc2.directionKnown=1u;g_arm2.moving=1;
            g_arm2.rate=600;g_arm2.rateCap=600;g_arm2.periodCy=(SIM_MHZ*1000000u)/600;
            simStall2=1;
            for(i=0;i<(!strcmp(t,"dual_derate")?20u:150u);i++)jog_step(&g_arm2,1);
            simClock+=(SIM_MHZ*1000u);service_encoder();
            if(!strcmp(t,"dual_derate")) {assert(g_motionEnabled && g_arm2.derates==1 && g_arm2.rateCap==300 && !g_arm1.derates);}
            else {assert(!g_motionEnabled && !g_ptp.reference);}
        } else if(!strcmp(t,"dual_peer_bad") || !strcmp(t,"dual_peer_bad_reverse")) {
            jog_axis_t *healthy=!strcmp(t,"dual_peer_bad") ? &g_arm1 : &g_arm2;
            enc_t *e=axis_encoder(healthy);
            e->incDir=e->directionKnown=1u;e->cmdLsbX100=SLIP_FAIL_LSB*100;
            healthy->moving=1;healthy->rate=600;healthy->periodCy=(SIM_MHZ*1000000u)/600;
            if(healthy==&g_arm1)simBadEncoder2=1;else simBadEncoder=1;
            service_encoder();txq_flush_blocking();
            assert(!g_motionEnabled && !e->badStreak && strstr(simLog,"jog encoder following error"));
            assert((healthy==&g_arm1 ? g_enc2.badStreak : g_enc.badStreak)==1u);
        } else if(!strcmp(t,"dual_final")) {
            g_enc.directionKnown=g_enc2.directionKnown=1u;g_enc.incDir=g_enc2.incDir=1u;
            g_homed1=g_homed2=g_ptp.reference=1u;g_ptp.active=2u;
            g_ptp.settleMs=g_jogMs-HOME_SETTLE_MS;
            simEncoderBias2=240;assert(enc_sample());ptp_service();txq_flush_blocking();
            assert(!g_motionEnabled && strstr(simLog,"PTP final position mismatch"));
        } else if(!strcmp(t,"dual_limit")) {
            g_enc.directionKnown=g_enc2.directionKnown=1u;g_enc.incDir=g_enc2.incDir=1u;
            g_homed1=g_homed2=1u;
            g_jointMinCd[0]=-3000;g_jointMaxCd[0]=3000;
            g_jointMinCd[1]=-6000;g_jointMaxCd[1]=6000;
            g_jointMinCd[2]=-15500;g_jointMaxCd[2]=15500;
            g_enc2.pos=-(int32)((int64)g_jointMaxCd[1]*ENCODER_CPR*ARM2_GEAR_RATIO_X100/3600000)-10;
            assert(!joint_step_allowed(&g_arm2,-1) && joint_step_allowed(&g_arm2,1));
        } else if(!strcmp(t,"dual_manual_unknown")) {
            packetType = 5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;txq_flush_blocking();
            assert(!simPulse1 && !simPulse2);
            /* power-up, arm2 polarity not learned: the manual reference is accepted with the bench polarity (no pulses) */
            assert(g_ptp.reference && strstr(simLog,"MANUAL REF OK"));
            assert(g_enc2.directionKnown==(ARM2_ENCODER_FEEDBACK?1u:0u) && (!ARM2_ENCODER_FEEDBACK || (g_enc2.incDir==MANUAL_ENC2_INC_DIR && strstr(simLog,"bench calibration"))));
        } else if(!strcmp(t,"dual_ptp_follow")) {
            g_enc.directionKnown=g_enc2.directionKnown=1u;g_enc.incDir=g_enc2.incDir=1u;
            g_homed1=g_homed2=g_ptp.reference=1u;g_ptp.active=1u;
            g_enc2.cmdLsbX100=SLIP_FAIL_LSB*100;ptp_service();txq_flush_blocking();
            if(ARM2_ENCODER_FEEDBACK) assert(!g_motionEnabled && strstr(simLog,"PTP encoder following error") && !simPulse1 && !simPulse2);
            else assert(g_motionEnabled && !strstr(simLog,"PTP encoder following error"));
        } else if(!strcmp(t,"dual_enable_bad")) {
            stage_stop("test",0u);simBadEncoder2=1;sim_keys(0);stage_enable();
            assert(!g_motionEnabled && !g_arm1.hasEnc && !g_arm2.hasEnc);
        } else {assert(0);}
        puts(t);return 0;
    }
    if(!strcmp(t,"boot_tx_backlog")) {
        hw_init();assert(enc_set_origin());
        simRxActive=1;simRxBase=simClock;simUartTxBusyUntil=simClock+120u*(SIM_MHZ*1000u);
        sim_run(1);
        assert(!g_motionEnabled && !simPulse1 && !simPulse2);
        assert(g_arm1.hasEnc && !g_enc.trackingLost && g_encMaxGapMs<10u);
        assert(simWdtFeeds>1000u && strstr(simLog,"STAGE2 v10 latched stop framed control"));
        puts(t);return 0;
    }
    if(!strcmp(t,"dual_snapshot_length")) {
        const char *reason="HOME direction: no credible rotor motion; check VMOT/motor/encoder";
        setup();simLogN=0;simLog[0]=0;
        g_jogMs=g_ptp.epoch=g_ptp.sequence=UINT32_MAX;g_lastKeyMs=0;g_keyMaxGapMs=UINT32_MAX;
        g_traj.state=1u;g_traj.index=g_traj.count=2048u;
        g_ptp.origin[0]=g_ptp.origin[1]=1073741823;g_arm1.usteps=g_arm2.usteps=-1073741823;
        g_enc.pos=g_enc2.pos=-21474836;g_ptp.encoderOrigin=g_enc2.origin=21474836;
        fault_snapshot(reason,1u);txq_flush_blocking();
        assert(strlen(simLog)>320 && strstr(simLog,reason) && strstr(simLog,"encoder\"\r\n"));
        assert(strstr(simLog," e2=") && strstr(simLog," err2="));
        puts(t);return 0;
    }
    if(!strcmp(t,"boot") || !strcmp(t,"boot_bad") || !strcmp(t,"boot_limit")) {
        simBadEncoder=!strcmp(t,"boot_bad"); simLimit=!strcmp(t,"boot_limit");
        app_boot();
        assert(!simPulse1 && !simPulse2 && !g_motionEnabled);
        assert(simPins[PIN_EN1] && simPins[PIN_EN2]);
        assert(strstr(simLog,(simBadEncoder||simLimit)?"SELFTEST BLOCKED":"SELFTEST PASS"));
        puts(t);return 0;
    }
    if(!strcmp(t,"reverse_polarity")||!strcmp(t,"ptp_reverse")) simPolarity=-1;
    /* Operator aligned the links before power/enable; sensors remain at their real offsets. */
    if(!strcmp(t,"traj_manual_plan") || !strncmp(t,"fuzz_",5)) {simStep1=400-22400;simStep2=120-36763;}
    /* Logged circle endpoint: q1=-14.37 deg, q2=+29.04 deg from straight. */
    if(!strcmp(t,"home_from_drawing")) {simStep1=400-22400+2555;simStep2=120-36763-6590;}
    if(!strcmp(t,"home_active_at_start")) simStep1=450;
    if(!strcmp(t,"home_from_arm2_range") || !strcmp(t,"home_long_stop"))
        simStep2=120-36763-13616; /* +60 deg: farthest commissioned arm2 pose */
    setup();
    if(!strcmp(t,"traj_manual_plan") || !strncmp(t,"fuzz_",5)) {g_enc2.directionKnown=1u;g_enc2.incDir=1u;}
    if(!strncmp(t,"fuzz_",5)){fuzz_test((uint32)atoi(t+5));puts(t);return 0;}
    if(!strcmp(t,"link_capabilities")) {
        /* The LINK line is the only carrier of the host's compatibility contract (docs/PROTOCOL.md section 6). */
        char proto[24]; simLogN=0; simLog[0]=0;
        stage_status(); status_drain();
        sprintf(proto," proto=%u\r\n",(unsigned)PROTO_VERSION); /* with its end of line: the debug buffer must not cut the LINK line short */
        assert(strstr(simLog,"STATE fw=stage2-v11 ") && strstr(simLog,"LINK uart=hw") && strstr(simLog,"fw=" FW_BUILD_ID) && strstr(simLog,proto));
        assert(strstr(simLog," short_hb=1") && strstr(simLog," joint_limits=1") && strstr(simLog," motion_rates=1") && strstr(simLog," rates_live=1") && strstr(simLog," q2_absolute=1"));
        assert(strstr(simLog," traj_repeat=1") && strstr(simLog," fault_snapshot=1") && strstr(simLog," telem_hz=0 proto="));
        puts(t);return 0;
    }
    if(!strncmp(t,"limits_",7)){limits_test(t);puts(t);return 0;}
    if(!strncmp(t,"telem_",6)){telem_test(t);puts(t);return 0;}
    if(!strcmp(t,"fault_snapshot")){fault_snapshot_test();puts(t);return 0;}
    if(!strncmp(t,"rates_",6)){rates_test(t);puts(t);return 0;}
    if(!strncmp(t,"operator_stop_",14)) {
        uint32 epoch=g_ptp.epoch;
        if(!strcmp(t,"operator_stop_after_protection")) {
            stage_stop("fixture protection",0u);
            assert(!g_motionEnabled && !simPins[PIN_EN1] && !simPins[PIN_EN2] && !g_jogFaulted);
        } else if(!strcmp(t,"operator_stop_after_fault")) {
            simFault=1;jog_safety_blocked();assert(g_jogFaulted);
        } else {
            assert(!strcmp(t,"operator_stop_reenable"));
            sim_keys(2);jog_tick();jog_axis_service(&g_arm1);assert(simPulse1>0);
        }
        p1=simPulse1;p2=simPulse2;
        jog_handle_byte(JOG_CMD_STOP);
        assert(simPins[PIN_EN1] && simPins[PIN_EN2] && !simPins[PIN_SLEEP1] &&
            !simPins[PIN_SLEEP2] && !simPins[PIN_RST1] && !simPins[PIN_RST2]);
        assert(!g_motionEnabled && !g_ptp.reference && g_ptp.epoch>epoch);
        assert(g_jogFaulted==(uint8)!strcmp(t,"operator_stop_after_fault"));
        jog_handle_byte(JOG_CMD_STOP);assert_stopped();
        if(simFault){stage_enable();assert(!g_motionEnabled);simFault=0;}
        short_send(0,0);packetType=7;
        ptp_send(g_controlSequence+1u,g_ptp.epoch,0,0,0,0);packetType=1;
        assert(g_motionEnabled && !g_jogFaulted && !simPins[PIN_EN1] && !simPins[PIN_EN2]);
        assert(simPulse1==p1 && simPulse2==p2 && !g_ptp.reference);
        puts(t);return 0;
    }
    if(!strcmp(t,"rejected_byte_status")) {
        uint32 rejected=g_rxRejected;
        jog_handle_byte(0x02);assert(g_rxRejected==++rejected);
        stage_status();status_drain();assert(strstr(simLog,"last_rejected_byte=0x02"));
        g_uploadRxQuarantine=1u;jog_handle_byte(0x04);assert(g_rxRejected==++rejected);
        stage_status();status_drain();assert(strstr(simLog,"last_rejected_byte=0x04"));
        g_uploadRxQuarantine=0u;jog_handle_byte(JOG_CMD_ENABLE);assert(g_rxRejected==++rejected);
        stage_status();status_drain();assert(strstr(simLog,"last_rejected_byte=0x63"));
        packetType=6;ptp_send(g_controlSequence+1u,g_ptp.epoch-1u,0,0,0,0);packetType=1;
        assert(g_rxRejected==++rejected);
        simLogN=0;simLog[0]=0;stage_status();status_drain();
        assert(strstr(simLog,"last_rejected_byte=0x63")); /* epoch rejection is not an unknown wire byte */
        puts(t);return 0;
    }
    if(!strncmp(t,"control_",8)) {
        uint32 seq,epoch;
        stage_stop("fixture",0);sim_keys(0);seq=g_controlSequence+1;epoch=g_ptp.epoch;
        if(!strncmp(t,"control_host",12)) {
            FILE *f=fopen("host-control.bin","rb");int b;unsigned count=0;
            assert(f);g_ptp.epoch=1;g_controlSequence=0;g_keysSeen=0;
            while((b=fgetc(f))!=EOF){
                /* Lose alternating first/second byte copies, including delimiters. */
                if(!strcmp(t,"control_host") || count%2==(count/2)%2)jog_handle_byte((uint8)b);
                count++;
            }
            rewind(f);
            assert(count==288 && g_motionEnabled && g_home.phase!=HOME_IDLE && g_ptp.epoch==3 && g_controlSequence==4);
            epoch=g_ptp.epoch;
            while((b=fgetc(f))!=EOF)jog_handle_byte((uint8)b);
            fclose(f);assert(g_ptp.epoch==epoch && !simPulse1 && !simPulse2);
            jog_handle_byte(JOG_CMD_STOP);assert_stopped();
        } else if(!strcmp(t,"control_stop_before_enable")) {
            jog_handle_byte(JOG_CMD_STOP);
            packetType=7;ptp_send(seq,epoch,0,0,0,0);
            assert(!g_motionEnabled && g_ptp.epoch!=epoch && !simPulse1 && !simPulse2);
        } else if(!strcmp(t,"control_raw")) {
            for(i=0;i<256;i++) { if(i==PTP_FRAME_START)continue; jog_handle_byte((uint8)i); }
            assert(!g_motionEnabled && !simPulse1 && !simPulse2);
        } else if(!strcmp(t,"control_crc")) {
            packetType=7;ptp_send(seq,epoch,0,0,0,1);
            assert(!g_motionEnabled && g_controlSequence<seq);
        } else if(!strcmp(t,"control_stale")) {
            packetType=7;ptp_send(seq,epoch-1,0,0,0,0);assert(!g_motionEnabled);
        } else if(!strcmp(t,"control_payload")) {
            packetType=7;ptp_send(seq,epoch,1,0,0,0);assert(!g_motionEnabled);
        } else {
            packetType=7;ptp_send(seq,epoch,0,0,0,0);assert(g_motionEnabled && !simPulse1 && !simPulse2);
            if(!strcmp(t,"control_duplicate")) {
                uint32 current=g_ptp.epoch;
                ptp_send(seq,epoch,0,0,0,0);assert(g_ptp.epoch==current && !simPulse1);
            } else if(!strcmp(t,"control_home")) {
                sim_keys(0);packetType=8;
                seq=g_controlSequence+1;epoch=g_ptp.epoch;ptp_send(seq,epoch,0,0,0,0);
                assert(g_home.phase!=HOME_IDLE);
                for(i=0;i<20;i++)cycle(1);
                epoch=g_ptp.epoch;ptp_send(seq,epoch,0,0,0,0);
                assert(g_ptp.epoch==epoch);jog_handle_byte(JOG_CMD_STOP);assert_stopped();
            } else if(!strcmp(t,"control_home_epoch_ack")) {
                char expected[96];uint32 requested;
                /* Issue HOME just after periodic status: the next is 500 ms away. */
                while(g_jogMs<JOG_STATUS_MS)cycle(1);
                txq_flush_blocking();simLogN=0;simLog[0]=0;
                requested=wall_ms();sim_keys(0);packetType=8;
                g_statusNext=8u;g_statusEpoch=g_ptp.epoch; /* previous running block has already published its old epoch */
                seq=g_controlSequence+1;epoch=g_ptp.epoch;ptp_send(seq,epoch,0,0,0,0);
                /* Drain the requested sliced block without waiting for the next periodic status. */
                for(i=0;i<80u && g_statusNext;i++){simClock+=(SIM_MHZ*1000u);jog_update_time();stage_status_service();txq_flush_blocking();}
                txq_flush_blocking();
                assert(g_motionEnabled && g_home.phase!=HOME_IDLE && g_ptp.epoch==epoch+1);
                assert(JOG_STATUS_MS>JOG_DEADMAN_MS && wall_ms()-requested<JOG_DEADMAN_MS);
                snprintf(expected,sizeof(expected),"PTP_STATE lock=%u ref=0 active=0 epoch=%lu ",
                         (unsigned)!PTP_COMMISSIONED,(unsigned long)(epoch+1));
                assert(strstr(simLog,expected)); /* no jog_tick() or periodic status after HOME */
            } else if(!strcmp(t,"control_replay_deadman")) {
                packetType=6;seq=g_controlSequence+1;ptp_send(seq,g_ptp.epoch,0,0,0,0);
                for(i=0;i<260;i++) {
                    simClock+=(SIM_MHZ*1000u);jog_update_time();
                    ptp_send(seq,g_ptp.epoch,0,0,0,0);
                }
                assert_stopped();
            } else if(!strcmp(t,"control_raw_deadman")) {
                for(i=0;i<260;i++){simClock+=(SIM_MHZ*1000u);jog_update_time();jog_handle_byte(0x41);}
                assert_stopped();
            } else if(!strcmp(t,"control_stale_epoch_lease")) {
                /* Field defect: homing bumps the epoch, the host keeps sending the
                 * previous one, and every such heartbeat used to be discarded whole.
                 * The lease must stay alive; keys and sequence must not be taken. */
                uint32 staleEpoch=epoch-1;
                /* Drain the transmit ring every iteration: the real main loop does
                 * this byte by byte, and skipping it would fill the 2048-byte ring
                 * and silently drop the announcement this test is looking for. */
                txq_flush_blocking();
                for(i=0;i<300;i++) {
                    simClock+=(SIM_MHZ*1000u);jog_update_time();
                    packetType=6;ptp_send(g_controlSequence+1u,staleEpoch,0,0,0,0);packetType=1;
                    txq_flush_blocking();
                }
                assert(g_motionEnabled && !simPulse1 && !simPulse2);
                assert(!strstr(simLog,"heartbeat timeout"));
                assert(g_controlSequence==seq && g_ptpEpochRejected>0);
                {
                    char announced[32];
                    snprintf(announced,sizeof(announced),"PTP_EPOCH %lu\r\n",(unsigned long)g_ptp.epoch);
                    assert(strstr(simLog,announced));
                }
                jog_handle_byte(JOG_CMD_STOP);assert_stopped();
            } else if(!strcmp(t,"control_stale_epoch_no_keys")) {
                /* A stale epoch must never carry jog keys to the axes. */
                packetType=6;ptp_send(g_controlSequence+1u,epoch,0,0,0,0);packetType=1;
                assert(g_arm1.want==0 && g_arm2.want==0);
                packetType=6;ptp_send(g_controlSequence+1u,epoch-1,0,0,1,0);packetType=1;
                assert(g_motionEnabled && g_arm1.want==0 && g_arm2.want==0);
                jog_handle_byte(JOG_CMD_STOP);assert_stopped();
            } else if(!strcmp(t,"control_stale_epoch_home")) {
                /* The exact reported sequence: HOME bumps the epoch while the host is
                 * still on the old one.  Nothing may stop the machine. */
                sim_keys(0);packetType=8;
                seq=g_controlSequence+1;epoch=g_ptp.epoch;ptp_send(seq,epoch,0,0,0,0);packetType=1;
                assert(g_motionEnabled && g_home.phase!=HOME_IDLE && g_ptp.epoch==epoch+1);
                for(i=0;i<300;i++) {
                    simClock+=(SIM_MHZ*1000u);jog_update_time();
                    packetType=6;ptp_send(g_controlSequence+1u,epoch,0,0,0,0);packetType=1;
                    txq_flush_blocking();
                }
                assert(g_motionEnabled && g_home.phase!=HOME_IDLE);
                assert(!strstr(simLog,"heartbeat timeout"));
                jog_handle_byte(JOG_CMD_STOP);assert_stopped();
            } else if(!strcmp(t,"control_short_lease")) {
                /* A compact heartbeat is the shipping path: 6 wire bytes a beat.  It
                 * must hold the lease through a long stretch of clock advance. */
                uint32 before=g_shortRxOk;
                for(i=0;i<300;i++) {
                    simClock+=(SIM_MHZ*1000u);jog_update_time();
                    short_send(0,0);
                }
                txq_flush_blocking();
                assert(g_motionEnabled && !simPulse1 && !simPulse2);
                assert(g_shortRxOk>=before+250u && g_ptpEpochRejected==0u);
                jog_handle_byte(JOG_CMD_STOP);assert_stopped();
            } else if(!strcmp(t,"control_short_crc")) {
                /* A corrupt compact heartbeat must not renew the lease. */
                assert(g_shortRxOk==0u);
                for(i=0;i<300;i++) {
                    simClock+=(SIM_MHZ*1000u);   /* 1 ms per iteration: 300 ms total */
                    jog_update_time();
                    short_send(0,1);
                }
                assert(!g_motionEnabled && g_shortRxOk==0u);
            } else if(!strcmp(t,"control_short_keys")) {
                /* The compact frame carries the jog mask, and only that.  A neutral
                 * compact heartbeat first, exactly as the host sends before jogging. */
                short_send(0,0);
                assert(g_motionEnabled && g_arm1.want==0 && g_arm2.want==0);
                short_send(2,0);
                assert(g_motionEnabled && g_arm1.want==1 && g_arm2.want==0);
                short_send(0,0);
                assert(g_arm1.want==0);
                jog_handle_byte(JOG_CMD_STOP);assert_stopped();
            } else if(!strcmp(t,"control_jog")) {
                sim_keys(2);jog_tick();jog_axis_service(&g_arm1);assert(g_arm1.want==1);
                jog_handle_byte(0x41);assert(g_arm1.want==0);
            } else assert(0);
        }
        puts(t);return 0;
    }
    if(!strncmp(t,"encoder_guard_",14)) {
        int32 position=g_enc.pos;uint16 angle=g_enc.lastAngle;
        if(!strcmp(t,"encoder_guard_gap")) simClock+=101u*(SIM_MHZ*1000u);
        else if(!strcmp(t,"encoder_guard_jump"))simAngleOverride=(angle+1000u)&4095u;
        else if(!strcmp(t,"encoder_guard_overflow")) {
            g_enc.pos=INT32_MAX;position=g_enc.pos;simAngleOverride=(angle+1u)&4095u;
        } else assert(0);
        assert(!enc_sample() && g_enc.pos==position && g_enc.lastAngle==angle && g_enc.trackingLost);
        simAngleOverride=angle;assert(!enc_sample()); /* cannot silently re-prime */
        assert(enc_set_origin() && enc_sample() && !g_enc.trackingLost);
        puts(t);return 0;
    }
    if(!strncmp(t,"clock_",6)) {
        uint32 before;
        /* Register reads advance the mock clock: start exact-ms delta checks away from a floor boundary. */
        if(!strcmp(t,"clock_wraps") || !strcmp(t,"clock_ms_wrap"))
            simClock+=(SIM_MHZ*1000u)-(simClock-simClockBase)%(SIM_MHZ*1000u);
        before=wall_ms();
        if(!strcmp(t,"clock_wraps")) {
            simClock+=3u*(SIM_MHZ*1000000u);assert(wall_ms()-before==3000u);
            before=wall_ms();simInts=0;
            simClock+=0x1000000u; /* pending wrap must not be lost or counted twice */
            assert(wall_ms()-before>=SU_WRAP_MS);simInts=1;sim_tick_sync();
            assert(wall_ms()-before>=SU_WRAP_MS && wall_ms()-before<=SU_WRAP_MS+1u);
        } else if(!strcmp(t,"clock_ms_wrap")) {
            simClock+=(uint64_t)0xfffffff0u*(SIM_MHZ*1000u);before=wall_ms();
            simClock+=40u*(SIM_MHZ*1000u);assert(wall_ms()-before==40u);
        } else if(!strcmp(t,"clock_deadman_stall")) {
            jog_home_all();simClock+=251u*(SIM_MHZ*1000u);
            jog_update_time();assert(!g_motionEnabled && g_jogMs-g_lastKeyMs>=251u);
            sim_keys(0);assert_stopped();
        } else if(!strcmp(t,"clock_home_timeout")) {
            jog_home_all();home_enter(HOME_RELEASE);g_home.sinceMs=wall_ms();
            simClock+=(uint64_t)HOME_PHASE_TIMEOUT_MS*(SIM_MHZ*1000u);
            g_jogMs=wall_ms();g_lastKeyMs=g_jogMs;
            home_service();assert_stopped();
        } else if(!strcmp(t,"clock_tx_timeout")) {
            g_txqHead=g_txqTail=0;JLOG("stuck TX");SIM_TX_STUCK(1);
            before=wall_ms();txq_flush_blocking();
            assert(!txq_pending() && wall_ms()-before>=100u && wall_ms()-before<110u);
            SIM_TX_STUCK(0);
        } else assert(0);
        puts(t);return 0;
    }
    if(!strncmp(t,"manual_",7)){manual_test(t);puts(t);return 0;}
    if(!strncmp(t,"traj_",5)){traj_test(t);puts(t);return 0;}
    if(!strncmp(t,"ptp_",4)){ptp_test(t);puts(t);return 0;}
    if(!strcmp(t,"startup_stiction"))simStiction=1;
    if(!strcmp(t,"missing_sensor")) simHome1Mode=-1;
    if(!strcmp(t,"stuck_sensor")) simHome1Mode=1;
    if(!strcmp(t,"missing_npn")) simHome2Mode=-1;
    if(!strcmp(t,"stalled_motor")) simStall=1;
    if(!strcmp(t,"home_low_speed_tracking")) simRateLimited=1;
    if(!strcmp(t,"reject_moving")) {
        sim_keys(2); jog_tick(); jog_home_all(); assert(g_home.phase==HOME_IDLE);
        puts(t);return 0;
    }
    if(!strncmp(t,"hw_uart_",8)) {
        if(!strcmp(t,"hw_uart_bytes")) {
            for(i=0;i<256;i++)sim_uart_push((uint8)i);
            for(i=0;i<256;i++)assert(uart_rx_service()==(int16)i);
            assert(uart_rx_service()==-1 && !g_rxLate && !g_rxFraming);
        } else if(!strcmp(t,"hw_uart_boot_discard")) {
            sim_uart_push(0x41);sim_uart_push(JOG_CMD_ENABLE);sim_uart_push(JOG_CMD_HOME);
            HostUart_errorStatus=HostUart_RX_STS_OVERRUN;
            uart_discard_rx();assert(!simUartCount && !HostUart_errorStatus);
        } else if(!strcmp(t,"hw_uart_message_full")) {
            uint16 before; g_txqHead=TXQ_SIZE-3;g_txqTail=0;before=g_txqHead;
            JLOG("ACK\r\n");assert(g_txqHead==before && g_txqDropped==1);
            g_txqTail=g_txqHead;
        } else if(!strcmp(t,"hw_uart_tx_full")) {
            simLogN=0;simLog[0]=0;g_txqHead=g_txqTail=0;
            JLOG("ACK\r\n");SIM_TX_STUCK(1);txq_send_one();
            assert(txq_pending() && simLogN==0);
            SIM_TX_STUCK(0);txq_flush_blocking();assert(!strcmp(simLog,"ACK\r\n"));
        } else if(!strcmp(t,"hw_uart_stop")) {
            g_ptpRx.active=1;sim_uart_push(JOG_CMD_STOP);jog_rx_service();assert_stopped();
        } else {
            if(!strcmp(t,"hw_uart_overflow"))for(i=0;i<257;i++)sim_uart_push(0x41);
            else HostUart_errorStatus=!strcmp(t,"hw_uart_overrun")?HostUart_RX_STS_OVERRUN:
                !strcmp(t,"hw_uart_break")?HostUart_RX_STS_BREAK:
                !strcmp(t,"hw_uart_parity")?HostUart_RX_STS_PAR_ERROR:HostUart_RX_STS_STOP_ERROR;
            sim_uart_push(JOG_CMD_ENABLE);sim_uart_push(JOG_CMD_HOME);
            jog_rx_service();assert(!simUartCount && !g_keysSeen && g_ptpRxDropped==1);
            assert(g_rxLate+g_rxFraming==1);assert_stopped();
            txq_flush_blocking();assert(strstr(simLog,"UART receive error/overflow"));
            assert(strstr(simLog,"UART RX fault: flags="));
            if(!strcmp(t,"hw_uart_overflow"))assert(strstr(simLog,"flags=0x00 sw_overflow=1"));
            if(!strcmp(t,"hw_uart_overrun"))assert(strstr(simLog,"flags=0x10 sw_overflow=0")&&strstr(simLog,"overrun=1"));
            if(!strcmp(t,"hw_uart_break"))assert(strstr(simLog,"flags=0x02 sw_overflow=0")&&strstr(simLog,"break=1"));
            if(!strcmp(t,"hw_uart_parity"))assert(strstr(simLog,"flags=0x04 sw_overflow=0")&&strstr(simLog,"parity=1"));
            if(!strcmp(t,"hw_uart_framing"))assert(strstr(simLog,"flags=0x08 sw_overflow=0")&&strstr(simLog,"framing=1"));
        }
        puts(t);return 0;
    }
    jog_home_all(); assert(g_home.phase!=HOME_IDLE);
    if(!strcmp(t,"missing_sensor") || !strcmp(t,"missing_npn") ||
       !strncmp(t,"home_search_",12) || !strncmp(t,"home_slow_",10)) {
        int slow=!strncmp(t,"home_slow_",10);
        int arm=(!strcmp(t,"missing_sensor") || !strcmp(t,"home_slow_sensor"))?1:2;
        home_phase_t phase=slow?HOME_SLOW:HOME_FAST;
        run_until(phase,arm);
        if(arm==1)simHome1Mode=-1;else simHome2Mode=-1;
        for(i=0;i<130000 && g_motionEnabled;i++)cycle(1);
        assert(g_motionEnabled && g_home.phase==phase && g_home.arm==arm);
        assert(g_home.steps>(slow?512u:52000u) && g_jogMs-g_home.sinceMs>120000u);
        assert(g_home.ramp.cruising && g_home.periodUs==step_period_us(slow?HOME_SLOW_RATE_USTEP_S:g_motionRate[arm-1])); /* the fast touch runs at the global speed */
        if(!strcmp(t,"home_search_stop"))jog_handle_byte(JOG_CMD_STOP);
        else if(!strcmp(t,"home_search_heartbeat"))for(i=0;i<260;i++)cycle(0);
        else if(!strcmp(t,"home_search_limit")){simLimit=1;cycle(1);}
        else if(!strcmp(t,"home_search_fault")){simFault=1;cycle(1);}
        else {
            if(arm==1){simHome1At=simStep1+10;simHome1Mode=0;}
            else {simHome2At=simStep2+10;simHome2Mode=0;}
            run_until(slow?HOME_SETTLE:HOME_RELEASE,arm);
            assert(g_home.steps==0u); /* contact stopped search and reset phase accounting */
            for(i=0;i<250000 && g_home.phase!=HOME_IDLE && g_motionEnabled;i++)cycle(1);
            assert(g_motionEnabled && g_homed1 && g_homed2 && strstr(simLog,"HOME OK:"));
            assert(simStep1==simHome1At-22400 && simStep2==simHome2At-36763);
            puts(t);return 0;
        }
        assert_stopped();puts(t);return 0;
    }
    if(!strcmp(t,"home_release_time_limit")) {
        run_until(HOME_RELEASE,1);
        g_home.sinceMs=g_jogMs-HOME_PHASE_TIMEOUT_MS;
        cycle(1);assert_stopped();assert(strstr(simLog,"HOME phase time limit"));
        puts(t);return 0;
    }
    if(!strcmp(t,"home_long_stop")) {
        run_until(HOME_FAST,2);
        while(g_motionEnabled && g_home.phase==HOME_FAST && g_home.steps<=40000u)cycle(1);
        assert(g_motionEnabled && g_home.phase==HOME_FAST && g_home.steps>40000u);
        jog_handle_byte(JOG_CMD_STOP);assert_stopped();puts(t);return 0;
    }
    if(!strcmp(t,"jog_slip_follow") || !strcmp(t,"jog_slip_stall")) {
        /* Jog arm1 right after homing.  Through cycle() the encoder result reaches the firmware's own
         * slip check: a motor that follows stays quiet, a stalled one first derates, then stops. */
        for(i=0;i<250000 && g_home.phase!=HOME_IDLE && g_motionEnabled;i++)cycle(1);
        assert(g_motionEnabled && g_homed1 && g_homed2 && g_directionKnown);
        cycle(1);sim_keys(0);
        if(!strcmp(t,"jog_slip_stall"))simStall=1;
        p1=simPulse1;
        for(i=0;i<3000 && g_motionEnabled;i++) {if(g_jogMs-g_lastKeyMs>=40u)sim_keys(2);cycle(0);}
        if(!strcmp(t,"jog_slip_follow")) {
            assert(g_motionEnabled && simPulse1-p1>200 && !g_arm1.derates && g_arm1.peakSlip<SLIP_WARN_LSB);
        } else {
            assert_stopped();
            assert(strstr(simLog,"SLIP WARN") && strstr(simLog,"jog encoder following error"));
        }
        puts(t);return 0;
    }
    if(!strcmp(t,"home_following_stall")) {
        for(i=0;i<5000 && g_motionEnabled && !g_directionKnown;i++)cycle(1);
        assert(g_motionEnabled && g_directionKnown && g_home.phase==HOME_FAST);
        simStall=1;
        for(i=0;i<1000 && g_motionEnabled;i++)cycle(1);
        assert_stopped();assert(strstr(simLog,"HOME encoder following error"));
        puts(t);return 0;
    }
    if(!strncmp(t,"average_",8)) {
        int negative=!strcmp(t,"average_reverse_wrap");
        run_until(HOME_SETTLE,1);
        while(g_jogMs-g_home.sinceMs<HOME_SETTLE_MS-2u)cycle(1);
        assert(g_home.zeroSamples==0 && !g_homed1);
        simClock+=(HOME_SETTLE_MS-(g_jogMs-g_home.sinceMs))*(SIM_MHZ*1000u);
        simHome1Mode=1;
        g_enc.lastAngle=negative?1:4094;g_enc.pos=0;cmd_reset_to_measured();
        p1=simPulse1;p2=simPulse2;
        for(i=0;i<HOME_ZERO_SAMPLES;i++) {
            simAngleOverride=negative?((1-(int)(i%4)+4096)%4096):((4094+i%4)%4096);
            cycle(1);
            assert(g_home.zeroSamples==i+1 || g_home.phase==HOME_STRAIGHT);
            assert(simPulse1==p1 && simPulse2==p2);
            if(i==0) {
                home_service();assert(g_home.zeroSamples==1); /* no duplicate within same tick */
                if(!strcmp(t,"average_stop")){jog_handle_byte(JOG_CMD_STOP);assert_stopped();puts(t);return 0;}
                if(!strcmp(t,"average_timeout")){g_lastKeyMs=g_jogMs-JOG_DEADMAN_MS;cycle(0);assert_stopped();puts(t);return 0;}
                if(!strcmp(t,"average_bad_encoder")){simBadEncoder=1;cycle(1);assert_stopped();puts(t);return 0;}
                if(!strcmp(t,"average_contact_lost")){simHome1Mode=-1;cycle(1);assert_stopped();puts(t);return 0;}
            }
            if(i+1<HOME_ZERO_SAMPLES)assert(g_home.phase==HOME_SETTLE && !g_homed1);
        }
        assert(g_home.phase==HOME_STRAIGHT && g_homed1);
        assert(g_enc.pos==(negative?-2:2)); /* mean across wrap, not last sample */
        assert(strstr(simLog,negative?"samples=12 raw=0 lsb":"samples=12 raw=4095 lsb"));
        assert(enc_sample() && g_enc.pos==(negative?-2:2));
        puts(t);return 0;
    }
    if(!strncmp(t,"uart_load_",10)) {
        simRxActive=1;simRxBase=simClock;simRxOffset=(uint32)atoi(t+10)*SIM_MHZ;
        sim_run(1);
        if(!g_motionEnabled)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled && g_rxGood>15 && !strstr(simLog,"heartbeat timeout"));
        puts(t);return 0;
    }
    if(!strncmp(t,"stop_",5) || !strncmp(t,"timeout_",8) || !strcmp(t,"sensor_bounce")) {
        int timeout=!strncmp(t,"timeout_",8);
        const char *part=t+(timeout?8:5);
        int axis=1;home_phase_t phase=HOME_FAST;
        if(!strcmp(part,"release"))phase=HOME_RELEASE;
        if(!strcmp(part,"backoff"))phase=HOME_BACKOFF;
        if(!strcmp(part,"slow"))phase=HOME_SLOW;
        if(!strcmp(part,"settle"))phase=HOME_SETTLE;
        if(!strcmp(part,"straight"))phase=HOME_STRAIGHT;
        if(!strcmp(part,"straight_settle"))phase=HOME_STRAIGHT_SETTLE;
        if(!strcmp(part,"arm2")){axis=2;phase=HOME_FAST;}
        if(!strcmp(part,"arm2_straight")){axis=2;phase=HOME_STRAIGHT;}
        if(!strcmp(part,"arm2_straight_settle")){axis=2;phase=HOME_STRAIGHT_SETTLE;}
        run_until(phase,axis);
        if(!strcmp(t,"sensor_bounce")) {
            simHome1Mode=1;cycle(1);p1=simPulse1;home_service(); /* same timestamp: still debouncing */
            assert(g_home.phase==HOME_FAST && simPulse1==p1);
            simHome1Mode=-1;cycle(1);assert(g_home.phase==HOME_FAST);puts(t);return 0;
        }
        if(timeout){for(i=0;i<260;i++)cycle(0);}
        else jog_handle_byte(JOG_CMD_STOP);
        assert_stopped();puts(t);return 0;
    }
    if(!strncmp(t,"arm2_",5)) {
        run_until(HOME_STRAIGHT,2);
        assert(!g_homed2 && !strstr(simLog,"HOME OK:"));
        if(!strcmp(t,"arm2_fault"))simFault=1;
        if(!strcmp(t,"arm2_limit"))simLimit=1;
        if(!strcmp(t,"arm2_encoder_loss"))simBadEncoder=1;
        if(!strcmp(t,"arm2_jog_cancels"))sim_keys(2);
        if(!strcmp(t,"arm2_stuck_npn"))simHome2Mode=1;
        for(i=0;i<2000 && g_motionEnabled;i++)cycle(1);
        assert_stopped();assert(!strstr(simLog,"HOME OK:"));
        if(simFault)assert(simPins[PIN_EN1]&&simPins[PIN_EN2]);
        puts(t);return 0;
    }
    if(!strcmp(t,"fault")||!strcmp(t,"limit")||!strcmp(t,"encoder_loss")||!strcmp(t,"jog_cancels")) {
        run_until(HOME_STRAIGHT,1);
        if(!strcmp(t,"fault"))simFault=1;
        if(!strcmp(t,"limit"))simLimit=1;
        if(!strcmp(t,"encoder_loss"))simBadEncoder=1;
        if(!strcmp(t,"jog_cancels"))sim_keys(2);
        for(i=0;i<8;i++)cycle(1);
        assert_stopped();
        if(simFault)assert(simPins[PIN_EN1]&&simPins[PIN_EN2]);
        if(simFault||simLimit||simBadEncoder){stage_enable();assert(!g_motionEnabled);}
        puts(t);return 0;
    }
    for(i=0;i<250000 && g_home.phase!=HOME_IDLE && g_motionEnabled;i++) cycle(1);
    assert(i<250000);
    if(!strcmp(t,"stuck_sensor")||!strcmp(t,"stalled_motor")) {
        assert_stopped(); assert(!strstr(simLog,"HOME OK:"));
    } else {
        if(!g_homed2)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled && g_homed1 && g_homed2 && g_directionKnown);
        assert(home_straight_steps(1)==22400u);
        assert(home_straight_steps(2)==36763u);
        assert(g_arm2.usteps==-36763 && simStep2==120-36763 && g_arm1.usteps==-22400);
        assert(!ARM2_NPN_ACTIVE());
        assert(abs_i32(abs_i32(lsb_to_out_deg100(g_enc.pos))-12600)<10);
        assert(strstr(simLog,"HOME OK:"));
        p1=simPulse1;p2=simPulse2;for(i=0;i<200;i++)cycle(1);
        assert(simPulse1==p1 && simPulse2==p2);
        jog_home_all();for(i=0;i<250000&&g_home.phase!=HOME_IDLE&&g_motionEnabled;i++)cycle(1);
        assert(g_homed1 && g_homed2);
        assert(g_arm2.usteps==-36763 && simStep2==120-36763 && g_arm1.usteps==-22400);
    }
    puts(t);return 0;
}
'''

cases = ['dual_'+n for n in ['home_reverse','home_stall','wrap','loss','gap','parity','hold','derate','follow_stop','peer_bad','peer_bad_reverse','final','limit','enable_bad']]
cases += ['ssi_fifo','ssi_nonblocking','ssi_snapshot','ssi_retry','ssi_timeout','ssi_takeover']
cases += ['usb_tx','usb_fallback','usb_stall','usb_rx','usb_link']
cases += ['step_hw_request','step_hw_pins','dead_kill','txdma_order','txdma_busy','txdma_wait_drq','pin_irq_kill']
cases += ['dual_snapshot_length','chain_frames','boot','boot_bad','boot_limit','success','reverse_polarity','home_active_at_start',
         'missing_sensor','stuck_sensor','missing_npn','stalled_motor','reject_moving',
         'fault','limit','encoder_loss','jog_cancels','sensor_bounce']
cases += ['startup_stiction','home_low_speed_tracking','home_following_stall','jog_slip_follow','jog_slip_stall','link_capabilities']
cases += ['boot_tx_backlog','rejected_byte_status','operator_stop_reenable','operator_stop_after_protection','operator_stop_after_fault']
cases += ['limits_'+n for n in ['configure','retry','invalid','busy','enabled']]
cases += ['telem_rate','telem_rate_invalid','telem_stream','telem_skip','fault_snapshot']
cases += ['snapshot_telem','snapshot_stats']
cases += ['status_idle_'+n for n in ['slice','stop','epoch','repeat']]
cases += ['rates_config','rates_jog','rates_home']
cases += ['home_from_drawing','home_from_arm2_range','home_long_stop']
cases += ['home_search_stop','home_search_heartbeat','home_search_limit','home_search_fault','home_release_time_limit']
cases += ['clock_wraps','clock_ms_wrap','clock_deadman_stall','clock_home_timeout','clock_tx_timeout']
cases += ['control_'+n for n in ['raw','crc','stale','payload','duplicate','home','home_epoch_ack','replay_deadman','raw_deadman','jog','stop_before_enable','stale_epoch_lease','stale_epoch_no_keys','stale_epoch_home','short_lease','short_crc','short_keys']]
cases += ['encoder_guard_'+n for n in ['gap','jump','overflow']]
if host_control is not None: cases += ['control_host','control_host_single_copy']
cases += ['home_slow_sensor','home_slow_npn']
cases += ['hw_uart_'+n for n in ['bytes','boot_discard','message_full','tx_full','stop','overflow','overrun','break','parity','framing']]
cases += ['arm2_fault','arm2_limit','arm2_encoder_loss','arm2_jog_cancels','arm2_stuck_npn']
cases += ['average_wrap','average_reverse_wrap','average_stop','average_timeout',
          'average_bad_encoder','average_contact_lost']
cases += [f'uart_load_{offset}' for offset in [0, 137, 499, 950, 1750, 3100]]
cases += [f'{action}_{phase}' for action in ['stop','timeout'] for phase in
          ['fast','release','backoff','slow','settle','straight','straight_settle','arm2',
           'arm2_straight','arm2_straight_settle']]
cases += ['ptp_locked','ptp_unhomed']
cases += ['traj_locked']
cases += ['manual_locked']
ptp_cases = ['ptp_'+name for name in ['success','reverse','unhomed','crc','missing','partial','quarantine','partial_stop',
    'stale_epoch','range','rate','duplicate','busy','home_busy','stop','fault','limit','home1','npn','encoder','stall','jog','timeout']]
ptp_cases += [f'ptp_uart_{offset}' for offset in [0,137,499,950,1750,3100]]
ptp_cases += [f'ptp_uart_logged_{offset}' for offset in [0,499,1750]]
ptp_cases += ['ptp_uart_paced','traj_uart_upload_paced','traj_uart_run_paced']
ptp_cases += ['ptp_zero','ptp_axis1','ptp_axis2']
ptp_cases += ['ptp_power_stable','ptp_fast_stall']
ptp_cases += ['limits_ptp','limits_jog','limits_elbow','limits_elbow_traj','limits_elbow_jog']
ptp_cases += ['rates_ptp','rates_traj','rates_live']
ptp_cases += ['ptp_final_'+name for name in ['positive','negative','boundary','over_limit','following','counts']]
ptp_cases += ['ptp_logged_retry']
ptp_cases += ['traj_rx_'+name for name in ['retry','ready_retry','repeated','stop','timeout','overrun','overflow','break','parity','mixed','heartbeat','holding','fault','encoder','motion']]
ptp_cases += ['ptp_telemetry']
ptp_cases += ['traj_'+name for name in ['success','dwell','duplicate','capacity','wrong_start','partial','order','range','rate','time','accel',
    'upload_stop','upload_jog','upload_timeout','crc','ptp_busy','stop','fault','limit','home1','npn','encoder','timeout','overrun','uart_upload','uart_run']]
ptp_cases += ['manual_'+name for name in ['success','duplicate','stop','old_epoch','disabled','moving','busy','upload','sensor','fault','encoder','timeout','stale','payload','crc']]
ptp_cases += ['fuzz_%d' % n for n in range(1, 7)]   # the gate's share; long campaigns: --emit-c=DIR and run fuzz_<seed> in parallel
ptp_cases += ['traj_evt_runs','traj_evt_stall','traj_evt_stop']
ptp_cases += ['traj_isr_done','traj_isr_slot']
ptp_cases += ['status_upload_'+n for n in ['volume','backlog','partial','stream','restore','stop','fault']]
ptp_cases += ['dual_manual_unknown','dual_ptp_follow','stop_latch','traj_slip','traj_status_cost','hot_arith','jog_arm2_limit','origin_retry','traj_schedule','pulse_blocked']
if host_plan is not None: ptp_cases += ['traj_host_plan','traj_manual_plan','traj_host_plan_offset','traj_host_plan_rx_retry']
if len(sys.argv)>1:
    cases=[c for c in cases if c in sys.argv[1:]]
    ptp_cases=[c for c in ptp_cases if c in sys.argv[1:]]
    assert cases or ptp_cases, 'No matching test cases'
fuzz_sums = {}
fuzz_stops = {}
def fuzz_total(profile, stdout):
    for line in stdout.splitlines():
        if line.startswith('FUZZ '):
            sums = fuzz_sums.setdefault(profile, {})
            for item in line.split()[1:]:
                key, value = item.split('=')
                sums[key] = sums.get(key, 0) + int(value)
        elif line.startswith('FUZZSTOP '):
            _, count, reason = line.split(' ', 2)
            stops = fuzz_stops.setdefault(profile, {})
            stops[reason] = stops.get(reason, 0) + int(count)
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat')
assert vcvars.exists(), f'Install Visual Studio C++ tools: {vcvars}'
with tempfile.TemporaryDirectory(prefix='arm-stage1-') as temp:
    tmp = Path(temp)
    if host_plan is not None: (tmp/'host-plan.bin').write_bytes(host_plan.read_bytes())
    if host_control is not None: (tmp/'host-control.bin').write_bytes(host_control.read_bytes())
    for name in ['cytypes.h','cyfitter.h','cydevice_trm.h','cypins.h','CyLib.h','HostUart.h','Ssi.h','StepArm1Req.h','StepArm2Req.h','StepArm1.h','StepArm2.h','EvtTimer.h','EvtRestart.h','EvtIsr.h','AliveReq.h','DeadTimer.h','TxDma_dma.h','FAULT.h','LIMIT2.h','StepKill.h','FaultIsr.h','LimitIsr.h','StepRearm.h','CyDmac.h','UsbCdc.h','UsbCdc_cdc.h']:
        (tmp/name).write_text('',encoding='ascii')
    if emit_c is not None:
        emit_c.mkdir(parents=True, exist_ok=True)
        for name in ['cytypes.h','cyfitter.h','cydevice_trm.h','cypins.h','CyLib.h','HostUart.h','Ssi.h','StepArm1Req.h','StepArm2Req.h','StepArm1.h','StepArm2.h','EvtTimer.h','EvtRestart.h','EvtIsr.h','AliveReq.h','DeadTimer.h','TxDma_dma.h','FAULT.h','LIMIT2.h','StepKill.h','FaultIsr.h','LimitIsr.h','StepRearm.h','CyDmac.h','UsbCdc.h','UsbCdc_cdc.h']: (emit_c/name).write_text('',encoding='ascii')
        (emit_c/'check.c').write_text(pin_defs+f'\n#define SIM_MHZ {sim_mhz}u\n'+extra_defs+mock+'\n#define main firmware_main\n'+source+'\n#undef main\n'+harness,encoding='utf-8')
        print('wrote', emit_c/'check.c'); sys.exit(0)
    (tmp/'check.c').write_text(pin_defs+f'\n#define SIM_MHZ {sim_mhz}u\n'+extra_defs+mock+'\n#define main firmware_main\n'+source+'\n#undef main\n'+harness,encoding='utf-8')
    (tmp/'build.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /TC /std:c11 /W3 /DPTP_COMMISSIONED=0 /DARM2_ENCODER_FEEDBACK=1 /I"{tmp}" "{tmp / "check.c"}" /Fe:"{tmp / "check.exe"}" /Fo:"{tmp / "check.obj"}"\n',encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'{case}: {result.stdout}\n{result.stderr}'
        print('PASS',case)
    # Protocol regression profile uses artificial ranges only in the mock executable.
    defines='/DARM2_ENCODER_FEEDBACK=1 /DPTP_COMMISSIONED=1 /DPTP_MIN1_CD=-6000 /DPTP_MAX1_CD=6000 /DPTP_MIN2_CD=-9000 /DPTP_MAX2_CD=9000'
    (tmp/'build.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /TC /std:c11 /W3 {defines} /I"{tmp}" "{tmp / "check.c"}" /Fe:"{tmp / "check.exe"}" /Fo:"{tmp / "check.obj"}"\n',encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in ptp_cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'{case}: {result.stdout}\n{result.stderr}'
        fuzz_total('simulated-unlock',result.stdout)
        if case.startswith('status_upload_'): print(result.stdout, end='')
        print('PASS simulated-unlock',case)
    # Exercise the deliberately optional, absent arm2 encoder in the delivery profile.
    # Third profile: arm2 feedback off (the macro default is now 1), no other overrides.
    build = replace_once((tmp/'build.cmd').read_text(encoding='utf-8'), defines, '/DARM2_ENCODER_FEEDBACK=0')
    (tmp/'build.cmd').write_text(build, encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in ptp_cases + (["optional_arm2"] if len(sys.argv)==1 or host_plan is not None else []):
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'delivery {case}: {result.stdout}\n{result.stderr}'
        fuzz_total('delivery-default',result.stdout)
        if case.startswith('status_upload_'): print(result.stdout, end='')
        print('PASS delivery-default',case)
    # The base cases again with arm2 feedback off, as delivered.  dual_* describe the two-encoder behaviour (they cannot
    # pass without it); everything else, including the arm2_* and control_* cases, must hold in both configurations.
    default_cases = [c for c in cases if not c.startswith('dual_')]
    build = (tmp/'build.cmd').read_text(encoding='utf-8').replace('/W3 ', '/W3 /DPTP_COMMISSIONED=0 ', 1)
    assert '/DPTP_COMMISSIONED=0' in build and '/DARM2_ENCODER_FEEDBACK=0' in build
    (tmp/'build.cmd').write_text(build, encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in default_cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'delivery base {case}: {result.stdout}\n{result.stderr}'
        print('PASS delivery-base',case)
for profile, sums in fuzz_sums.items():
    print('fuzz coverage', profile, ' '.join(f'{k}={v}' for k, v in sums.items()))
    for reason, count in sorted(fuzz_stops.get(profile, {}).items(), key=lambda kv: -kv[1]): print(f'  stop x{count}: {reason}')
    # a fuzz run that never reached a mode proves nothing about it
    if len(sys.argv)==1: assert all(v > 0 for v in sums.values()) and len(sums)==6, f'fuzz did not reach every mode: {sums}'
print(f'{len(cases)+2*len(ptp_cases)+len(default_cases)+(1 if len(sys.argv)==1 or host_plan is not None else 0)} actual-C simulation cases passed; hardware validation still required.')
