"""Actual firmware C with simulated GPIO/encoder/time; never touches serial or a board.
Run: python tests/test_stage1.py (requires installed Visual Studio C++ tools).
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
sys.argv = [a for a in sys.argv if not a.startswith(('--host-plan=', '--host-control=', '--emit-c='))]
source = (root / 'DRV8825.cydsn/main.c').read_text(encoding='utf-8')

def replace_once(text, old, new):
    count = text.count(old)
    assert count == 1, f'Simulation hook expected one match, got {count}: {old!r}'
    return text.replace(old, new, 1)

pins = sorted(set(re.findall(r'CYREG_PRT\d+_PC\d+', source)))
pin_defs = '\n'.join(f'#define {pin} {i}' for i, pin in enumerate(pins))

mock = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int8_t int8; typedef int16_t int16; typedef int32_t int32; typedef int64_t int64;
#define __attribute__(x)
#define BCLK__BUS_CLK__HZ 24000000u
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
static int simInts=1, simTickPending;
static unsigned simWdtStarted, simWdtFeeds;
static int simIlo1K;
static void CyILO_Start1K(void){simIlo1K=1;}
static void CyWdtStart(uint8 ticks,uint8 mode){assert(simIlo1K && ticks==2 && mode==0);simWdtStarted++;}
static void CyWdtClear(void){assert(simWdtStarted);simWdtFeeds++;}
static void CyIntSetSysVector(uint8 n,void (*isr)(void)){assert(n==15);simTickIsr=isr;}
static void sim_tick_sync(void) {
    uint64_t wraps=(simClock-simClockBase)>>24;
    while(simClockWraps<wraps){simClockWraps++;simTickPending=1;
        if(simInts && simTickIsr){simTickPending=0;simTickIsr();}}
    if(simInts && simTickPending && simTickIsr){simTickPending=0;simTickIsr();}
}
static uint8 mock_critical_enter(void){uint8 old=(uint8)simInts;sim_tick_sync();simInts=0;return old;}
static void mock_critical_exit(uint8 old){simInts=old;sim_tick_sync();}
static int simPins[128], simStep1, simStep2, simBadEncoder;
static int simFault, simLimit, simHome1Mode, simHome2Mode, simStall, simPolarity=1, simStiction;
static int simHome1At=400, simHome2At=120;
static int simRateLimited; static uint32 simLastMotorPulse;
static int simRxActive; static uint32 simRxBase, simRxOffset;
static uint8 simPtpWire[36], simPtpWireN;
static uint8 simPtpReceived[256];static unsigned simPtpReceivedN;
static int simAngleOverride=-1;
static int simEncoderBias;
static unsigned simPulse1, simPulse2;
static unsigned simDriverSwitches;
static char simLog[1048576]; static unsigned simLogN;
/* SysTick read cost: the coarse 1 us keeps busy-wait loops fast, but inside an SSI frame (CSn low) the firmware
 * busy-waits on it for ~115 us, so a read costs 0.25 us there and the frame lasts as long as the real one. */
static unsigned simRegCost=24;
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
    us=(simClock-simRxBase+simRxOffset)/24u;
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
    while(simClock<simUartTxBusyUntil){simClock+=24u;sim_tick_sync();}
    mock_log(b);
}
static uint32 mock_frame(void) {
    uint32 angle, frame, bits, parity=0;
    if(simBadEncoder) return 0;
    angle=(uint32)(1000+(int64)simStep1*simPolarity*256/100+simEncoderBias)&4095u;
    if(simAngleOverride>=0)angle=(uint32)simAngleOverride;
    frame=(angle<<6)|0x26u;
    for(bits=frame; bits; bits>>=1) parity^=bits&1;
    return frame|parity;
}
/* AS5045 at pin level: CSn falling latches a frame, every CLK rising edge while CSn is low presents the next bit, MSB first. */
static uint32 simSsiFrame; static int simSsiBits;
static int mock_read(int pin) {
    if(pin==CYREG_PRT1_PC3) return (simSsiBits>=1 && simSsiBits<=18 && ((simSsiFrame>>(18-simSsiBits))&1u))?16:0;
    if(pin==CYREG_PRT12_PC5) return simFault?0:16;
    if(pin==CYREG_PRT0_PC4) return simLimit?16:0;
    if(pin==CYREG_PRT0_PC3) return (simHome1Mode<0?0:(simHome1Mode>0?1:simStep1>=simHome1At))?16:0;
    if(pin==CYREG_PRT1_PC0) return (simHome2Mode<0?0:(simHome2Mode>0?1:simStep2>=simHome2At))?0:16;
    return simPins[pin];
}
static void mock_set(int pin,int value) {
    if(pin==CYREG_PRT12_PC1 && value!=simPins[pin]) {
        if(!value){simSsiFrame=mock_frame();simSsiBits=0;simRegCost=6;} else simRegCost=24;
    }
    if(pin==CYREG_PRT12_PC0 && value && !simPins[pin] && !simPins[CYREG_PRT12_PC1])simSsiBits++;
    if(value!=simPins[pin] && (pin==CYREG_PRT1_PC7 || pin==CYREG_PRT12_PC2 || pin==CYREG_PRT1_PC2 ||
        pin==CYREG_PRT2_PC7 || pin==CYREG_PRT2_PC2 || pin==CYREG_PRT2_PC3))simDriverSwitches++;
    if(value && !simPins[pin]) {
        if(pin==CYREG_PRT12_PC3 && !simPins[CYREG_PRT1_PC7] && simPins[CYREG_PRT12_PC2] && simPins[CYREG_PRT1_PC2]) {
            int tracks=!simRateLimited || simClock-simLastMotorPulse>=24u*1250u;
            simLastMotorPulse=simClock; /* fixture motor tracks at most the configured 800 ustep/s jog rate */
            simPulse1++; if(tracks && !simStall && !(simStiction && simPulse1<=24)) simStep1+=simPins[CYREG_PRT12_PC4]?1:-1;
        }
        if(pin==CYREG_PRT2_PC1 && !simPins[CYREG_PRT2_PC7] && simPins[CYREG_PRT2_PC2] && simPins[CYREG_PRT2_PC3]) {
            simPulse2++; simStep2+=simPins[CYREG_PRT2_PC0]?1:-1;
        }
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
static void CyDelay(uint32 ms) { simClock+=ms*24000u; }
'''

harness = r'''
#undef assert
#define assert(ok) do { if(!(ok)) { fprintf(stderr,"check: %s line %d; encLost=%u encGap=%lu\n%s\n", \
    #ok,__LINE__,g_enc.trackingLost,(unsigned long)g_encMaxGapMs,simLog+(simLogN>1600?simLogN-1600:0)); abort(); } } while(0)
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
static void cycle(int heartbeat) {
    if(g_traj.state==3u && g_ptp.active==1u) {
        /* The real foreground services pulses continuously, independently of
         * its 1 ms housekeeping tick. A single service per mock millisecond
         * falsely caps 1667 us pulse spacing at 500 Hz. Preserve elapsed time,
         * but service at 100 us resolution for the newly uncapped host plans. */
        unsigned sub;
        for(sub=0;sub<10;sub++) {
            simClock+=2400u;jog_update_time();
            if(!jog_safety_blocked() && g_ptp.active)ptp_service();
        }
    } else simClock+=24000u;
    jog_update_time();
    if(heartbeat && (g_jogMs-g_lastKeyMs>=40u)) sim_keys(0);
    if(g_arm1.hasEnc)service_encoder();
    jog_tick();
    service_motion();
    txq_flush_blocking();
}
/* The real foreground loop for 2 s of simulated time. init=1 also runs the firmware's own axis set-up and banner;
 * init=0 keeps the axes and enable state that setup() prepared (used when a wire image is already loaded). */
static void sim_run(int init) {
    if(init)app_init(1);
    app_start();
    if(!init)simRxBase=simClock;
    while(simClock-simRxBase<24000000u*2u)app_poll();
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
static void rates_test(const char *t) {
    uint32 seq,epoch; unsigned i; uint32 before;
    packetType=10;ptp_send(g_controlSequence+1u,g_ptp.epoch,200,500,0,0);packetType=1;
    assert(g_motionRate[0]==800); /* enabled configuration is refused */
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);txq_flush_blocking();
    seq=g_controlSequence+1u;epoch=g_ptp.epoch;
    packetType=10;
    ptp_send(seq,epoch,19,500,0,0);ptp_send(seq,epoch,801,500,0,0);
    ptp_send(seq,epoch,200,99,0,0);ptp_send(seq,epoch,200,20001,0,0);
    ptp_send(seq,epoch-1u,200,500,0,0);
    assert(g_motionRate[0]==800 && g_controlSequence<seq);
    ptp_send(seq,epoch,200,500,0,0);
    assert(g_motionRate[0]==200 && g_motionAccel[0]==500 && g_arm1.accel==500 && g_arm1.rateStart==20);
    packetType=1;sim_keys(0);packetType=10;
    ptp_send(seq,epoch,200,500,0,0); /* exact retry after heartbeat */
    ptp_send(seq,epoch,300,500,0,0);assert(g_motionRate[0]==200);
    ptp_send(g_controlSequence+1u,epoch,150,300,1,0);packetType=1;
    txq_flush_blocking();assert(strstr(simLog,"PTP_RATES epoch=") && strstr(simLog,"motion_rates=1"));
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
    } else if(!strcmp(t,"rates_traj") && PTP_COMMISSIONED) {
        packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);
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
        packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;
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
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);txq_flush_blocking();
    seq=g_controlSequence+1u;epoch=g_ptp.epoch;
    if(!strcmp(t,"limits_invalid")) {
        const int16 bounds[][2]={{-18001,13500},{-13500,18001},{100,200},{-200,-100},{1,1},{100,-100}};
        for(i=0;i<sizeof(bounds)/sizeof(bounds[0]);i++)limits_send(seq,epoch,0,bounds[i][0],bounds[i][1]);
        limits_send(seq,epoch,2,-13500,13500);limits_send(0,epoch,0,-13500,13500);
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
    jog_handle_byte(JOG_CMD_STOP);short_send(0,0);stage_enable();txq_flush_blocking();
    assert(g_motionEnabled && !simPulse1 && !simPulse2);
    assert(ptp_counts_in_range(0u,g_ptp.origin[0]-ptp_offset(0u,13500)) && ptp_counts_in_range(1u,g_ptp.origin[1]-ptp_offset(1u,15000)));
    if(!strcmp(t,"limits_configure") || !strcmp(t,"limits_retry"))return;
    packetType=5;ptp_send(1,g_ptp.epoch,0,0,0,0);packetType=1;
    assert(g_ptp.reference && g_homed1 && g_homed2);
    if(!strcmp(t,"limits_ptp")) {
        ptp_send(2,g_ptp.epoch,13501,0,600,0);assert(!g_ptp.active);
        packetType=2;ptp_send(2,g_ptp.epoch,13501,0,2,0);packetType=1;assert(!g_traj.state);
        ptp_send(2,g_ptp.epoch,13500,0,600,0);assert(g_ptp.active && g_ptp.target[0]==g_ptp.origin[0]-ptp_offset(0u,13500));
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
        assert(g_motionEnabled&&g_ptp.active==2&&g_arm1.usteps==g_ptp.target[0]&&g_arm2.usteps==g_ptp.target[1]);
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
        assert(g_jogMs-started<24000 && g_arm1.usteps==start1-ptp_offset(0,-1437) && g_arm2.usteps==start2-ptp_offset(1,2904-(-1437)));
        jog_handle_byte(JOG_CMD_STOP);assert_stopped();
        assert(simDriverSwitches==switches+6u && simPins[PIN_EN1] && simPins[PIN_EN2]);
        puts("PTP 600: power stable through completion; operator STOP releases both drivers");return;
    }
    assert(ptp_crc((const uint8*)"123456789",9)==0x29b1u);
    if(!strcmp(t,"ptp_telemetry")) {
        char expected[170];
        assert(g_ptp.referenceSource==1u && g_ptp.encoderOrigin==g_enc.pos);
        ptp_send(1,epoch,-100,100,200,0);
        for(i=0;i<1000 && g_ptp.active;i++)cycle(1);
        stage_status();txq_flush_blocking();
        sprintf(expected,"p1=%ld p2=%ld e1=%ld err=%ld inputs=0 drops=0",
            (long)(g_ptp.origin[0]-g_arm1.usteps),(long)(g_ptp.origin[1]-g_arm2.usteps),
            (long)((g_enc.pos-g_ptp.encoderOrigin)*(g_encIncDir==ARM1_LEFT_DIR_LEVEL?1:-1)),(long)(cmd_lsb()-g_enc.pos));
        assert(strstr(simLog,expected));
        assert(g_ptp.origin[0]-g_arm1.usteps<0 && g_ptp.origin[1]-g_arm2.usteps>0);
        jog_handle_byte(JOG_CMD_STOP);txq_flush_blocking();
        assert(g_ptp.referenceSource==0u && strstr(simLog,"ref=0 source=0"));return;
    }
    if(!strcmp(t,"ptp_logged_retry")) {
        /* Latest field target: lose a complete triplicated nibble, then retry unchanged. */
        ptp_send(2,epoch,-2195,4437,200,2);txq_flush_blocking();
        assert(!g_ptp.active && g_ptp.sequence==0 && p1==simPulse1 && p2==simPulse2);
        assert(strstr(simLog,"PTP RX dropped: incomplete"));
        ptp_send(2,epoch,-2195,4437,200,0);assert(g_ptp.active && g_ptp.sequence==2);
        for(i=0;i<100;i++)cycle(1);
        { uint32 done=g_ptp.done; int32 a=g_arm1.usteps,b=g_arm2.usteps;
          ptp_send(2,epoch,-2195,4437,200,0);
          assert(g_ptp.done==done && g_arm1.usteps==a && g_arm2.usteps==b); }
        jog_handle_byte(JOG_CMD_STOP);
        ptp_send(2,epoch,-2195,4437,200,0);assert_stopped();return;
    }
    if(!strncmp(t,"ptp_uart_",9)) {
        if(!strcmp(t,"ptp_uart_paced"))simUartChunk=8;
        int logged=!strncmp(t,"ptp_uart_logged_",16);
        ptp_send(1,epoch,logged?-2195:125,logged?4437:-250,200,4);
        simRxActive=1;simRxOffset=(uint32)atoi(t+(logged?16:9))*24u;
        simLogN=0;simLog[0]=0;
        sim_run(0);
        if(!g_ptp.active||g_ptp.sequence!=1){fprintf(stderr,"%s\nRX:",simLog);for(i=0;i<simPtpReceivedN;i++)fprintf(stderr,"%02X ",simPtpReceived[i]);}
        assert(g_motionEnabled&&g_ptp.active&&g_ptp.sequence==1&&g_ptp.done>0);
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
        ptp_send(1,!strcmp(t,"ptp_stale_epoch")?epoch-1:epoch,!strcmp(t,"ptp_range")?7000:125,-250,!strcmp(t,"ptp_rate")?601:200,0);
        assert(!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    ptp_send(1,epoch,target1,target2,200,0);assert(g_ptp.active&&g_ptp.sequence==1);
    if(!strcmp(t,"ptp_home_busy")){jog_home_all();assert(g_home.phase==HOME_IDLE&&g_ptp.active);}
    if(!strcmp(t,"ptp_duplicate")){ptp_send(1,epoch,125,-250,200,0);assert(g_ptp.active&&g_ptp.done==0);}
    if(!strcmp(t,"ptp_busy")){ptp_send(2,epoch,250,250,200,0);assert(g_ptp.sequence==1&&g_ptp.target[0]==start1-ptp_offset(0,125));}
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
        if(g_ptp.active&&g_motionEnabled&&g_ptp.total) {
            assert(abs_i32(g_arm1.usteps-start1)==(int32)((int64)g_ptp.distance[0]*g_ptp.done/g_ptp.total));
            assert(abs_i32(g_arm2.usteps-start2)==(int32)((int64)g_ptp.distance[1]*g_ptp.done/g_ptp.total));
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
static void traj_test(const char *t) {
    unsigned i,p1,p2; int32 origin1,origin2; uint32 seq=1;
    if(!strcmp(t,"traj_manual_plan")) {traj_send(5,seq++,0,0,0);assert(!simPulse1&&!simPulse2);}
    else jog_home_all();
    for(i=0;i<250000&&g_home.phase!=HOME_IDLE&&g_motionEnabled;i++)cycle(1);
    assert(g_homed1&&g_homed2&&g_ptp.reference);cycle(1);sim_keys(0);
    p1=simPulse1;p2=simPulse2;origin1=g_arm1.usteps;origin2=g_arm2.usteps;
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
            uint16 index=g_traj.index;cycle(1);
            if(g_traj.index!=index&&g_motionEnabled) {
                assert(g_arm1.usteps==origin1-ptp_offset(0,nodes[index].q1));
                assert(g_arm2.usteps==origin2-ptp_offset(1,nodes[index].q2-nodes[index].q1));
            }
        }
        if(!g_motionEnabled)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==count);
        assert(g_arm1.usteps==origin1-ptp_offset(0,nodes[count-1].q1));
        assert(g_arm2.usteps==origin2-ptp_offset(1,nodes[count-1].q2-nodes[count-1].q1));
        if(!strcmp(t,"traj_host_plan_offset"))assert(abs_i32(cmd_lsb()-g_enc.pos)>=66&&abs_i32(cmd_lsb()-g_enc.pos)<=68&&g_ptp.reference);
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
        simRxActive=1;simRxOffset=137u*24u;sim_run(0);
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
        simRxActive=1;simRxOffset=499u*24u;sim_run(0);
        if(!g_motionEnabled||g_traj.state!=4)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_arm1.usteps==origin1&&g_arm2.usteps==origin2);
        assert(simPulse1-p1==72&&simPulse2-p2==90);return;
    }
    traj_send(4,++seq,0,0,0);
    assert(g_traj.state==3&&g_ptp.active==1);
    if(!strcmp(t,"traj_rx_motion")){HostUart_errorStatus=HostUart_RX_STS_STOP_ERROR;jog_rx_service();assert_stopped();return;}
    if(!strcmp(t,"traj_duplicate")) {traj_send(4,seq,0,0,0);assert(g_traj.index==1);}
    if(!strcmp(t,"traj_overrun"))simClock+=24u*300000u;
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
    packetType=5;
    ptp_send(1,!strcmp(t,"manual_stale")?epoch-1:epoch,!strcmp(t,"manual_payload")?1:0,0,0,!strcmp(t,"manual_crc")?1:(!strcmp(t,"manual_timeout")?5:0));
    packetType=1;
    assert(p1==simPulse1&&p2==simPulse2&&g_home.phase==HOME_IDLE);
    if(bad) {assert(!g_ptp.reference&&!g_homed1&&!g_homed2);return;}
    assert(g_ptp.reference&&g_homed1&&g_homed2&&g_directionKnown&&g_encIncDir==1&&g_ptp.epoch!=epoch);
    assert(g_ptp.origin[0]==12345&&g_ptp.origin[1]==-23456&&g_arm1SoftCenter==g_enc.pos);
    if(!strcmp(t,"manual_duplicate")) {
        uint32 newEpoch=g_ptp.epoch;packetType=5;ptp_send(1,epoch,0,0,0,0);packetType=1;
        assert(g_ptp.epoch==newEpoch&&g_ptp.sequence==1&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"manual_old_epoch")) {
        ptp_send(2,epoch,20,30,200,0);assert(!g_ptp.active&&g_ptp.sequence==1);return;
    }
    if(!strcmp(t,"manual_stop")) {
        jog_handle_byte(JOG_CMD_STOP);assert_stopped();assert(!g_ptp.reference);
        sim_keys(0);stage_enable();traj_send(5,2,0,0,0);
        assert(g_ptp.reference&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
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
    FZ(!g_ptp.reference || (g_homed1 && g_homed2 && g_directionKnown));
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
int main(int argc,char **argv) {
    const char *t=argv[1]; unsigned i, p1, p2;
    assert(argc==2);
    if(!strcmp(t,"boot_tx_backlog")) {
        hw_init();assert(enc_set_origin());
        simRxActive=1;simRxBase=simClock;simUartTxBusyUntil=simClock+120u*24000u;
        sim_run(1);
        assert(!g_motionEnabled && !simPulse1 && !simPulse2);
        assert(g_arm1.hasEnc && !g_enc.trackingLost && g_encMaxGapMs<10u);
        assert(simWdtFeeds>1000u && strstr(simLog,"STAGE2 v7 framed control"));
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
    if(!strcmp(t,"traj_manual_plan") || !strncmp(t,"fuzz_",5)) {simStep1=400-22577;simStep2=120-36990;}
    /* Logged circle endpoint: q1=-14.37 deg, q2=+29.04 deg from straight. */
    if(!strcmp(t,"home_from_drawing")) {simStep1=400-22577+2555;simStep2=120-36990-6590;}
    if(!strcmp(t,"home_active_at_start")) simStep1=450;
    if(!strcmp(t,"home_from_arm2_range") || !strcmp(t,"home_long_stop"))
        simStep2=120-36990-13616; /* +60 deg: farthest commissioned arm2 pose */
    setup();
    if(!strncmp(t,"fuzz_",5)){fuzz_test((uint32)atoi(t+5));puts(t);return 0;}
    if(!strcmp(t,"link_capabilities")) {
        /* The LINK line is the only carrier of the host's compatibility contract (docs/PROTOCOL.md section 6). */
        char proto[24]; simLogN=0; simLog[0]=0;
        stage_status(); txq_flush_blocking();
        sprintf(proto," proto=%u",(unsigned)PROTO_VERSION);
        assert(strstr(simLog,"STATE fw=stage2-v7 ") && strstr(simLog,"LINK uart=hw") && strstr(simLog,"fw=" FW_BUILD_ID) && strstr(simLog,proto));
        assert(strstr(simLog," short_hb=1") && strstr(simLog," joint_limits=1") && strstr(simLog," motion_rates=1") && strstr(simLog," q2_absolute=1"));
        puts(t);return 0;
    }
    if(!strncmp(t,"limits_",7)){limits_test(t);puts(t);return 0;}
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
        stage_status();txq_flush_blocking();assert(strstr(simLog,"last_rejected_byte=0x02"));
        g_uploadRxQuarantine=1u;jog_handle_byte(0x04);assert(g_rxRejected==++rejected);
        stage_status();txq_flush_blocking();assert(strstr(simLog,"last_rejected_byte=0x04"));
        g_uploadRxQuarantine=0u;jog_handle_byte(JOG_CMD_ENABLE);assert(g_rxRejected==++rejected);
        stage_status();txq_flush_blocking();assert(strstr(simLog,"last_rejected_byte=0x63"));
        packetType=6;ptp_send(g_controlSequence+1u,g_ptp.epoch-1u,0,0,0,0);packetType=1;
        assert(g_rxRejected==++rejected);
        simLogN=0;simLog[0]=0;stage_status();txq_flush_blocking();
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
                seq=g_controlSequence+1;epoch=g_ptp.epoch;ptp_send(seq,epoch,0,0,0,0);
                txq_flush_blocking();
                assert(g_motionEnabled && g_home.phase!=HOME_IDLE && g_ptp.epoch==epoch+1);
                assert(JOG_STATUS_MS>JOG_DEADMAN_MS && wall_ms()-requested<JOG_DEADMAN_MS);
                snprintf(expected,sizeof(expected),"PTP_STATE lock=%u ref=0 active=0 epoch=%lu ",
                         (unsigned)!PTP_COMMISSIONED,(unsigned long)(epoch+1));
                assert(strstr(simLog,expected)); /* no jog_tick() or periodic status after HOME */
            } else if(!strcmp(t,"control_replay_deadman")) {
                packetType=6;seq=g_controlSequence+1;ptp_send(seq,g_ptp.epoch,0,0,0,0);
                for(i=0;i<260;i++) {
                    simClock+=24000u;jog_update_time();
                    ptp_send(seq,g_ptp.epoch,0,0,0,0);
                }
                assert_stopped();
            } else if(!strcmp(t,"control_raw_deadman")) {
                for(i=0;i<260;i++){simClock+=24000u;jog_update_time();jog_handle_byte(0x41);}
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
                    simClock+=24000u;jog_update_time();
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
                    simClock+=24000u;jog_update_time();
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
                    simClock+=24000u;jog_update_time();
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
                    simClock+=24000u;   /* 1 ms per iteration: 300 ms total */
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
        if(!strcmp(t,"encoder_guard_gap")) simClock+=101u*24000u;
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
        uint32 before=wall_ms();
        if(!strcmp(t,"clock_wraps")) {
            simClock+=3u*24000000u;assert(wall_ms()-before==3000u);
            before=wall_ms();simInts=0;
            simClock+=0x1000000u; /* pending wrap must not be lost or counted twice */
            assert(wall_ms()-before>=699u);simInts=1;sim_tick_sync();
            assert(wall_ms()-before>=699u && wall_ms()-before<=700u);
        } else if(!strcmp(t,"clock_ms_wrap")) {
            simClock+=(uint64_t)0xfffffff0u*24000u;before=wall_ms();
            simClock+=40u*24000u;assert(wall_ms()-before==40u);
        } else if(!strcmp(t,"clock_deadman_stall")) {
            jog_home_all();simClock+=251u*24000u;
            jog_update_time();assert(!g_motionEnabled && g_jogMs-g_lastKeyMs>=251u);
            sim_keys(0);assert_stopped();
        } else if(!strcmp(t,"clock_home_timeout")) {
            jog_home_all();home_enter(HOME_RELEASE);g_home.sinceMs=wall_ms();
            simClock+=(uint64_t)HOME_PHASE_TIMEOUT_MS*24000u;
            g_jogMs=wall_ms();g_lastKeyMs=g_jogMs;
            home_service();assert_stopped();
        } else if(!strcmp(t,"clock_tx_timeout")) {
            g_txqHead=g_txqTail=0;JLOG("stuck TX");simUartTxFull=1;
            before=wall_ms();txq_flush_blocking();
            assert(!txq_pending() && wall_ms()-before>=100u && wall_ms()-before<110u);
            simUartTxFull=0;
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
            JLOG("ACK\r\n");simUartTxFull=1;txq_send_one();
            assert(txq_pending() && simLogN==0);
            simUartTxFull=0;txq_flush_blocking();assert(!strcmp(simLog,"ACK\r\n"));
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
        assert(g_home.ramp.cruising && g_home.periodUs==step_period_us(slow?HOME_SLOW_RATE_USTEP_S:HOME_FAST_RATE_USTEP_S));
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
            assert(simStep1==simHome1At-22577 && simStep2==simHome2At-36990);
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
        simClock+=(HOME_SETTLE_MS-(g_jogMs-g_home.sinceMs))*24000u;
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
        simRxActive=1;simRxBase=simClock;simRxOffset=(uint32)atoi(t+10)*24u;
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
        assert(home_straight_steps(1)==22577u);
        assert(home_straight_steps(2)==36990u);
        assert(g_arm2.usteps==-36990 && simStep2==120-36990 && g_arm1.usteps==-22577);
        assert(!ARM2_NPN_ACTIVE());
        assert(abs_i32(abs_i32(lsb_to_out_deg100(g_enc.pos))-12700)<10);
        assert(strstr(simLog,"HOME OK:"));
        p1=simPulse1;p2=simPulse2;for(i=0;i<200;i++)cycle(1);
        assert(simPulse1==p1 && simPulse2==p2);
        jog_home_all();for(i=0;i<250000&&g_home.phase!=HOME_IDLE&&g_motionEnabled;i++)cycle(1);
        assert(g_homed1 && g_homed2);
        assert(g_arm2.usteps==-36990 && simStep2==120-36990 && g_arm1.usteps==-22577);
    }
    puts(t);return 0;
}
'''

cases = ['boot','boot_bad','boot_limit','success','reverse_polarity','home_active_at_start',
         'missing_sensor','stuck_sensor','missing_npn','stalled_motor','reject_moving',
         'fault','limit','encoder_loss','jog_cancels','sensor_bounce']
cases += ['startup_stiction','home_low_speed_tracking','home_following_stall','jog_slip_follow','jog_slip_stall','link_capabilities']
cases += ['boot_tx_backlog','rejected_byte_status','operator_stop_reenable','operator_stop_after_protection','operator_stop_after_fault']
cases += ['limits_'+n for n in ['configure','retry','invalid','busy','enabled']]
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
ptp_cases += ['limits_ptp','limits_jog']
ptp_cases += ['rates_ptp','rates_traj']
ptp_cases += ['ptp_final_'+name for name in ['positive','negative','boundary','over_limit','following','counts']]
ptp_cases += ['ptp_logged_retry']
ptp_cases += ['traj_rx_'+name for name in ['retry','ready_retry','repeated','stop','timeout','overrun','overflow','break','parity','mixed','heartbeat','holding','fault','encoder','motion']]
ptp_cases += ['ptp_telemetry']
ptp_cases += ['traj_'+name for name in ['success','dwell','duplicate','capacity','wrong_start','partial','order','range','rate','time','accel',
    'upload_stop','upload_jog','upload_timeout','crc','ptp_busy','stop','fault','limit','home1','npn','encoder','timeout','overrun','uart_upload','uart_run']]
ptp_cases += ['manual_'+name for name in ['success','duplicate','stop','old_epoch','disabled','moving','busy','upload','sensor','fault','encoder','timeout','stale','payload','crc']]
ptp_cases += ['fuzz_%d' % n for n in range(1, 7)]   # the gate's share; long campaigns: --emit-c=DIR and run fuzz_<seed> in parallel
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
    for name in ['cytypes.h','cyfitter.h','cydevice_trm.h','cypins.h','CyLib.h','HostUart.h']:
        (tmp/name).write_text('',encoding='ascii')
    if emit_c is not None:
        emit_c.mkdir(parents=True, exist_ok=True)
        for name in ['cytypes.h','cyfitter.h','cydevice_trm.h','cypins.h','CyLib.h','HostUart.h']: (emit_c/name).write_text('',encoding='ascii')
        (emit_c/'check.c').write_text(pin_defs+'\n'+mock+'\n#define main firmware_main\n'+source+'\n#undef main\n'+harness,encoding='utf-8')
        print('wrote', emit_c/'check.c'); sys.exit(0)
    (tmp/'check.c').write_text(pin_defs+'\n'+mock+'\n#define main firmware_main\n'+source+'\n#undef main\n'+harness,encoding='utf-8')
    (tmp/'build.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /TC /std:c11 /W3 /DPTP_COMMISSIONED=0 /I"{tmp}" "{tmp / "check.c"}" /Fe:"{tmp / "check.exe"}" /Fo:"{tmp / "check.obj"}"\n',encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'{case}: {result.stdout}\n{result.stderr}'
        print('PASS',case)
    # Protocol regression profile uses artificial ranges only in the mock executable.
    defines='/DPTP_COMMISSIONED=1 /DPTP_MIN1_CD=-6000 /DPTP_MAX1_CD=6000 /DPTP_MIN2_CD=-9000 /DPTP_MAX2_CD=9000'
    (tmp/'build.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /TC /std:c11 /W3 {defines} /I"{tmp}" "{tmp / "check.c"}" /Fe:"{tmp / "check.exe"}" /Fo:"{tmp / "check.obj"}"\n',encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in ptp_cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'{case}: {result.stdout}\n{result.stderr}'
        fuzz_total('simulated-unlock',result.stdout)
        print('PASS simulated-unlock',case)
    # Build the exact delivery defaults as a third profile, with no macro overrides.
    build = replace_once((tmp/'build.cmd').read_text(encoding='utf-8'), defines, '')
    (tmp/'build.cmd').write_text(build, encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True,errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in ptp_cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,errors='replace',timeout=(120 if case.startswith('fuzz_') else 20))
        assert result.returncode==0, f'delivery {case}: {result.stdout}\n{result.stderr}'
        fuzz_total('delivery-default',result.stdout)
        print('PASS delivery-default',case)
for profile, sums in fuzz_sums.items():
    print('fuzz coverage', profile, ' '.join(f'{k}={v}' for k, v in sums.items()))
    for reason, count in sorted(fuzz_stops.get(profile, {}).items(), key=lambda kv: -kv[1]): print(f'  stop x{count}: {reason}')
    # a fuzz run that never reached a mode proves nothing about it
    if len(sys.argv)==1: assert all(v > 0 for v in sums.values()) and len(sums)==6, f'fuzz did not reach every mode: {sums}'
print(f'{len(cases)+2*len(ptp_cases)} actual-C simulation cases passed; hardware validation still required.')
