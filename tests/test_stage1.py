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
sys.argv = [a for a in sys.argv if not a.startswith('--host-plan=')]
source = (root / 'DRV8825.cydsn/main.c').read_text(encoding='utf-8')

def replace_body(text, signature, body):
    start = text.index('{', text.index(signature))
    depth = 1
    end = start + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[:start] + '{\n' + body + '\n}' + text[end:]

source = replace_body(source, 'static uint32 as5045_read_raw(void)', 'return mock_frame();')
source = source.replace('static void jog_run(uint8 encOk)\n{',
                        'static void jog_run(uint8 encOk)\n{\n if(!simRxActive){simBoot = encOk; return;}\n if(simPtpWireN)goto sim_loop_ready;')
loop_start = source.index('static void jog_run(uint8 encOk)\n{')
loop_end = source.index('/* ================================ demo', loop_start)
loop = source[loop_start:loop_end].replace('for (;;)\n    {',
    'for (;;)\n    {\n if(simRxActive && simClock-simRxBase>=24000000u*2u)return;')
loop = loop.replace('    tTick = SU_NOW();', 'sim_loop_ready:\n    if(simPtpWireN)simRxBase=simClock;\n    tTick = SU_NOW();')
source = source[:loop_start] + loop + source[loop_end:]
source = source.replace('static void home_service(void)\n{',
    'static void home_service(void)\n{\n if(simRxActive)simClock+=24u*20u; /* modeled work latency */')
source = source.replace('int main(void)', 'int firmware_main(void)')
source = source.replace('static void jog_handle_byte(uint8 b)\n{',
    'static void jog_handle_byte(uint8 b)\n{\n if(simPtpWireN && simPtpReceivedN<256)simPtpReceived[simPtpReceivedN++]=b;')
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
#define CY_PINS_DM_STRONG 0
#define CY_PINS_PC_DATAOUT 1
#define CY_PINS_DM_DIG_HIZ 0
#define CY_PINS_DM_RES_UP 0
#define CyGlobalIntEnable ((void)0)
static uint32 simClock;
static int simPins[128], simStep1, simStep2, simBoot, simBadEncoder;
static int simFault, simLimit, simHome1Mode, simHome2Mode, simStall, simPolarity=1, simStiction;
static int simRxActive; static uint32 simRxBase, simRxOffset;
static uint8 simPtpWire[36], simPtpWireN;
static uint8 simPtpReceived[256];static unsigned simPtpReceivedN;
static int simAngleOverride=-1;
static unsigned simPulse1, simPulse2;
static char simLog[262144]; static unsigned simLogN;
static uint32 mock_now(void) { simClock+=24; return (0xffffffu-simClock)&0xffffffu; }
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
static void sim_uart_push(uint8 b) {
    if(simUartCount==256){HostUart_rxBufferOverflow=1;return;}
    simUartRx[simUartHead++%256]=b;simUartCount++;
}
static void sim_uart_fill(void) {
    uint32 us;
    if(!simRxActive)return;
    us=(simClock-simRxBase+simRxOffset)/24u;
    while(us>=500u+simUartHeartbeat*40000u){sim_uart_push(0x41);simUartHeartbeat++;}
    while(simUartPacketIndex<simPtpWireN && us>=5000u+simUartPacketIndex*87u)
        sim_uart_push(simPtpWire[simUartPacketIndex++]);
}
static void HostUart_Start(void){simUartStarted=1;}
static unsigned HostUart_GetRxBufferSize(void){sim_uart_fill();return simUartCount;}
static uint16 HostUart_GetByte(void){
    uint16 b=(uint16)HostUart_errorStatus<<8;HostUart_errorStatus=0;
    if(simUartCount){b|=simUartRx[simUartTail++%256];simUartCount--;}
    return b;
}
static void HostUart_ClearRxBuffer(void){simUartHead=simUartTail=simUartCount=0;HostUart_rxBufferOverflow=0;}
static unsigned HostUart_GetTxBufferSize(void){return simUartTxFull?255:0;}
static void HostUart_PutChar(uint8 b){assert(!simUartTxFull);mock_log(b);}
static uint32 mock_frame(void) {
    uint32 angle, frame, bits, parity=0;
    if(simRxActive)simClock+=24u*115u;
    if(simBadEncoder) return 0;
    angle=(uint32)(1000+(int64)simStep1*simPolarity*256/100)&4095u;
    if(simAngleOverride>=0)angle=(uint32)simAngleOverride;
    frame=(angle<<6)|0x26u;
    for(bits=frame; bits; bits>>=1) parity^=bits&1;
    return frame|parity;
}
static int mock_read(int pin) {
    if(pin==CYREG_PRT12_PC5) return simFault?0:16;
    if(pin==CYREG_PRT0_PC4) return simLimit?16:0;
    if(pin==CYREG_PRT0_PC3) return (simHome1Mode<0?0:(simHome1Mode>0?1:simStep1>=400))?16:0;
    if(pin==CYREG_PRT1_PC0) return (simHome2Mode<0?0:(simHome2Mode>0?1:simStep2>=120))?0:16;
    return simPins[pin];
}
static void mock_set(int pin,int value) {
    if(value && !simPins[pin]) {
        if(pin==CYREG_PRT12_PC3 && !simPins[CYREG_PRT1_PC7] && simPins[CYREG_PRT12_PC2] && simPins[CYREG_PRT1_PC2]) {
            simPulse1++; if(!simStall && !(simStiction && simPulse1<=24)) simStep1+=simPins[CYREG_PRT12_PC4]?1:-1;
        }
        if(pin==CYREG_PRT2_PC1 && !simPins[CYREG_PRT2_PC7] && simPins[CYREG_PRT2_PC2] && simPins[CYREG_PRT2_PC3]) {
            simPulse2++; simStep2+=simPins[CYREG_PRT2_PC0]?1:-1;
        }
    }
    simPins[pin]=value;
}
#define CY_GET_REG32(p) mock_now()
#define CY_SET_REG32(p,v) ((void)0)
#define CY_SET_REG8(p,v) mock_set((p),((v)&1))
#define CyPins_ReadPin(p) mock_read(p)
#define CyPins_SetPin(p) mock_set((p),1)
#define CyPins_ClearPin(p) mock_set((p),0)
#define CyEnterCriticalSection() 0
#define CyExitCriticalSection(x) ((void)(x))
static void CyDelay(uint32 ms) { simClock+=ms*24000u; }
'''

harness = r'''
static void setup(void) {
    hw_init();
    assert(enc_set_origin());
    jog_axis_init(&g_arm1,"arm1",PIN_STEP2,PIN_DIR2,1,1,ARM1_LEFT_DIR_LEVEL,800,200,20000,40000);
    jog_axis_init(&g_arm2,"arm2",PIN_STEP1,PIN_DIR1,0,0,ARM2_LEFT_DIR_LEVEL,600,200,12000,24000);
    jog_handle_byte(0x41);
    stage_enable();
    assert(g_motionEnabled && !simPulse1 && !simPulse2 && !g_homed1 && !g_homed2);
}
static void cycle(int heartbeat) {
    simClock+=24000u;
    if(heartbeat && (g_jogMs%40u==0)) jog_handle_byte(0x41);
    if(g_arm1.hasEnc) {
        if(!enc_sample() && g_enc.badStreak>=ENC_BAD_LIMIT) {
            g_arm1.hasEnc=0; stage_stop("encoder lost",0);
        }
    }
    jog_tick();
    if(!jog_safety_blocked()) {
        if(g_home.phase!=HOME_IDLE) home_service();
        else if(g_ptp.active) ptp_service();
        else {jog_axis_service(&g_arm1); jog_axis_service(&g_arm2);}
    }
    txq_flush_blocking();
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
    assert(g_homed1&&g_homed2&&g_ptp.reference);cycle(1);jog_handle_byte(0x41);
    start1=g_arm1.usteps;start2=g_arm2.usteps;p1=simPulse1;p2=simPulse2;epoch=g_ptp.epoch;
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
        int logged=!strncmp(t,"ptp_uart_logged_",16);
        ptp_send(1,epoch,logged?-2195:125,logged?4437:-250,200,4);
        simRxActive=1;simRxOffset=(uint32)atoi(t+(logged?16:9))*24u;
        simLogN=0;simLog[0]=0;
        jog_run(1);
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
    if(!strcmp(t,"ptp_jog"))jog_handle_byte(0x45);
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
        assert_stopped();assert(!g_ptp.reference&&!g_ptp.active);return;
    }
    assert(!strcmp(t,"ptp_success")||!strcmp(t,"ptp_reverse")||!strcmp(t,"ptp_duplicate")||!strcmp(t,"ptp_busy")||!strcmp(t,"ptp_home_busy")||
        !strcmp(t,"ptp_zero")||!strcmp(t,"ptp_axis1")||!strcmp(t,"ptp_axis2"));
    assert(g_ptp.reference&&!g_ptp.active);
    assert(g_arm1.usteps==start1-ptp_offset(0,target1)&&g_arm2.usteps==start2-ptp_offset(1,target2));
    p1=simPulse1;p2=simPulse2;
    ptp_send(1,epoch,125,-250,200,0);for(i=0;i<200;i++)cycle(1);
    assert(!g_ptp.active&&p1==simPulse1&&p2==simPulse2); /* completed duplicate cannot restart */
    ptp_send(2,epoch,0,0,200,0);assert(g_ptp.active);
    for(i=0;i<30000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
    assert(g_motionEnabled&&g_ptp.reference&&!g_ptp.active&&g_arm1.usteps==start1&&g_arm2.usteps==start2);
    ptp_send(1,epoch,125,-250,200,0);assert(!g_ptp.active); /* old seq rejected */
    jog_handle_byte(0x45);assert(!g_ptp.reference); /* manual jog invalidates PTP reference */
}
static void traj_send(uint8 type, uint32 seq, int16 q1, int16 q2, uint16 value) {
    packetType=type; ptp_send(seq,g_ptp.epoch,q1,q2,value,0); packetType=1u; txq_flush_blocking();
}
static void traj_test(const char *t) {
    unsigned i,p1,p2; int32 origin1,origin2; uint32 seq=1;
    if(!strcmp(t,"traj_manual_plan")) {traj_send(5,seq++,0,0,0);assert(!simPulse1&&!simPulse2);}
    else jog_home_all();
    for(i=0;i<250000&&g_home.phase!=HOME_IDLE&&g_motionEnabled;i++)cycle(1);
    assert(g_homed1&&g_homed2&&g_ptp.reference);cycle(1);jog_handle_byte(0x41);
    p1=simPulse1;p2=simPulse2;origin1=g_arm1.usteps;origin2=g_arm2.usteps;
    if(!strcmp(t,"traj_host_plan")||!strcmp(t,"traj_manual_plan")) {
        FILE *file=fopen("host-plan.bin","rb"); uint16 count; static traj_node_t nodes[2048];
        assert(file && fread(&count,2,1,file)==1 && count>=2 && count<=2048);
        assert(fread(nodes,6,count,file)==count);fclose(file);
        ptp_send(seq,g_ptp.epoch,nodes[0].q1,nodes[0].q2,200,0);
        for(i=0;i<200000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
        assert(g_motionEnabled&&!g_ptp.active&&g_arm1.usteps==origin1-ptp_offset(0,nodes[0].q1));
        p1=simPulse1;p2=simPulse2;traj_send(2,++seq,nodes[0].q1,nodes[0].q2,count);
        for(i=1;i<count;i++) {traj_send(3,++seq,nodes[i].q1,nodes[i].q2,nodes[i].ms);cycle(1);assert(g_traj.count==i+1);}
        assert(p1==simPulse1&&p2==simPulse2&&g_traj.state==2);
        traj_send(4,++seq,0,0,0);assert(g_ptp.active);
        for(i=0;i<650000&&g_ptp.active&&g_motionEnabled;i++) {
            uint16 index=g_traj.index;cycle(1);
            if(g_traj.index!=index&&g_motionEnabled) {
                assert(g_arm1.usteps==origin1-ptp_offset(0,nodes[index].q1));
                assert(g_arm2.usteps==origin2-ptp_offset(1,nodes[index].q2));
            }
        }
        if(!g_motionEnabled)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_traj.index==count);
        assert(g_arm1.usteps==origin1-ptp_offset(0,nodes[count-1].q1));
        assert(g_arm2.usteps==origin2-ptp_offset(1,nodes[count-1].q2));return;
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
    if(!strcmp(t,"traj_uart_upload")) {
        packetType=3;ptp_send(2,g_ptp.epoch,10,-10,250,4);packetType=1;
        simRxActive=1;simRxOffset=137u*24u;jog_run(1);
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
    if(!strcmp(t,"traj_upload_jog")) {jog_handle_byte(0x45);assert(!g_traj.state&&!g_ptp.reference);return;}
    if(!strcmp(t,"traj_upload_timeout")) {for(i=0;i<260;i++)cycle(0);assert(!g_traj.state);assert_stopped();return;}
    if(!strcmp(t,"traj_crc")) {
        packetType=3;ptp_send(2,g_ptp.epoch,10,10,250,1);packetType=1;
        assert(g_traj.count==1&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    if(!strcmp(t,"traj_ptp_busy")) {ptp_send(2,g_ptp.epoch,10,10,200,0);assert(!g_ptp.active&&g_traj.count==1);return;}
    for(i=1;i<=4;i++) {
        int16 q=(i==1?10:(i==2?20:(i==3?10:0)));
        if(!strcmp(t,"traj_dwell"))q=0;
        traj_send(3,++seq,q,(int16)-q,250);
        assert(g_traj.count==i+1);
    }
    assert(g_traj.state==2&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);
    if(!strcmp(t,"traj_uart_run")) {
        packetType=4;ptp_send(++seq,g_ptp.epoch,0,0,0,4);packetType=1;
        simRxActive=1;simRxOffset=499u*24u;jog_run(1);
        if(!g_motionEnabled||g_traj.state!=4)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled&&!g_ptp.active&&g_traj.state==4&&g_arm1.usteps==origin1&&g_arm2.usteps==origin2);
        assert(simPulse1-p1==72&&simPulse2-p2==90);return;
    }
    traj_send(4,++seq,0,0,0);
    assert(g_traj.state==3&&g_ptp.active==1);
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
        jog_handle_byte(0x41);stage_enable();traj_send(5,2,0,0,0);
        assert(g_ptp.reference&&!g_ptp.active&&p1==simPulse1&&p2==simPulse2);return;
    }
    ptp_send(2,g_ptp.epoch,20,30,200,0);assert(g_ptp.active);
    for(i=0;i<5000&&g_ptp.active&&g_motionEnabled;i++)cycle(1);
    assert(g_motionEnabled&&!g_ptp.active&&g_ptp.reference);
    assert(g_arm1.usteps==12345-ptp_offset(0,20)&&g_arm2.usteps==-23456-ptp_offset(1,30));
}
int main(int argc,char **argv) {
    const char *t=argv[1]; unsigned i, p1, p2;
    assert(argc==2);
    if(!strcmp(t,"boot") || !strcmp(t,"boot_bad") || !strcmp(t,"boot_limit")) {
        simBadEncoder=!strcmp(t,"boot_bad"); simLimit=!strcmp(t,"boot_limit");
        firmware_main();
        assert(!simPulse1 && !simPulse2 && !g_motionEnabled);
        assert(simPins[PIN_EN1] && simPins[PIN_EN2]);
        assert(strstr(simLog,(simBadEncoder||simLimit)?"SELFTEST BLOCKED":"SELFTEST PASS"));
        puts(t);return 0;
    }
    if(!strcmp(t,"reverse_polarity")||!strcmp(t,"ptp_reverse")) simPolarity=-1;
    /* Operator aligned the links before power/enable; sensors remain at their real offsets. */
    if(!strcmp(t,"traj_manual_plan")) {simStep1=400-22577;simStep2=120-36990;}
    setup();
    if(!strncmp(t,"manual_",7)){manual_test(t);puts(t);return 0;}
    if(!strncmp(t,"traj_",5)){traj_test(t);puts(t);return 0;}
    if(!strncmp(t,"ptp_",4)){ptp_test(t);puts(t);return 0;}
    if(!strcmp(t,"startup_stiction"))simStiction=1;
    if(!strcmp(t,"home_active_at_start")) simStep1=450;
    if(!strcmp(t,"missing_sensor")) simHome1Mode=-1;
    if(!strcmp(t,"stuck_sensor")) simHome1Mode=1;
    if(!strcmp(t,"missing_npn")) simHome2Mode=-1;
    if(!strcmp(t,"stalled_motor")) simStall=1;
    if(!strcmp(t,"reject_moving")) {
        jog_handle_byte(0x45); jog_tick(); jog_home_all(); assert(g_home.phase==HOME_IDLE);
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
        }
        puts(t);return 0;
    }
    jog_home_all(); assert(g_home.phase!=HOME_IDLE);
    if(!strncmp(t,"average_",8)) {
        int negative=!strcmp(t,"average_reverse_wrap");
        run_until(HOME_SETTLE,1);
        for(i=0;i<HOME_SETTLE_MS-1;i++)cycle(1);
        assert(g_home.zeroSamples==0 && !g_homed1);
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
        jog_run(1);
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
            simHome1Mode=1;cycle(1);p1=simPulse1;cycle(1);
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
        if(!strcmp(t,"arm2_jog_cancels"))jog_handle_byte(0x45);
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
        if(!strcmp(t,"jog_cancels"))jog_handle_byte(0x45);
        for(i=0;i<8;i++)cycle(1);
        assert_stopped();
        if(simFault)assert(simPins[PIN_EN1]&&simPins[PIN_EN2]);
        if(simFault||simLimit||simBadEncoder){stage_enable();assert(!g_motionEnabled);}
        puts(t);return 0;
    }
    for(i=0;i<250000 && g_home.phase!=HOME_IDLE && g_motionEnabled;i++) cycle(1);
    assert(i<250000);
    if(!strcmp(t,"missing_sensor")||!strcmp(t,"stuck_sensor")||
       !strcmp(t,"missing_npn")||!strcmp(t,"stalled_motor")) {
        assert_stopped(); assert(!strstr(simLog,"HOME OK:"));
    } else {
        if(!g_homed2)fprintf(stderr,"%s\n",simLog);
        assert(g_motionEnabled && g_homed1 && g_homed2 && g_directionKnown);
        assert(home_straight_steps(1)==22577u && HOME1_SEEK_MAX_USTEPS>home_straight_steps(1));
        assert(home_straight_steps(2)==36990u && HOME_SEEK_MAX_USTEPS>home_straight_steps(2));
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
cases += ['startup_stiction']
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
ptp_cases += ['ptp_zero','ptp_axis1','ptp_axis2']
ptp_cases += ['ptp_logged_retry']
ptp_cases += ['ptp_telemetry']
ptp_cases += ['traj_'+name for name in ['success','dwell','duplicate','capacity','wrong_start','partial','order','range','rate','time','accel',
    'upload_stop','upload_jog','upload_timeout','crc','ptp_busy','stop','fault','limit','home1','npn','encoder','timeout','overrun','uart_upload','uart_run']]
ptp_cases += ['manual_'+name for name in ['success','duplicate','stop','old_epoch','disabled','moving','busy','upload','sensor','fault','encoder','timeout','stale','payload','crc']]
if host_plan is not None: ptp_cases += ['traj_host_plan','traj_manual_plan']
if len(sys.argv)>1:
    cases=[c for c in cases if c in sys.argv[1:]]
    ptp_cases=[c for c in ptp_cases if c in sys.argv[1:]]
    assert cases or ptp_cases, 'No matching test cases'
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat')
assert vcvars.exists(), f'Install Visual Studio C++ tools: {vcvars}'
with tempfile.TemporaryDirectory(prefix='arm-stage1-') as temp:
    tmp = Path(temp)
    if host_plan is not None: (tmp/'host-plan.bin').write_bytes(host_plan.read_bytes())
    for name in ['cytypes.h','cyfitter.h','cydevice_trm.h','cypins.h','CyLib.h','HostUart.h']:
        (tmp/name).write_text('',encoding='ascii')
    (tmp/'check.c').write_text(pin_defs+'\n'+mock+'\n'+source+'\n'+harness,encoding='utf-8')
    (tmp/'build.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /TC /std:c11 /W3 /DPTP_COMMISSIONED=0 /I"{tmp}" "{tmp / "check.c"}" /Fe:"{tmp / "check.exe"}" /Fo:"{tmp / "check.obj"}"\n',encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True)
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,timeout=20)
        assert result.returncode==0, f'{case}: {result.stdout}\n{result.stderr}'
        print('PASS',case)
    # Protocol regression profile uses artificial ranges only in the mock executable.
    defines='/DPTP_COMMISSIONED=1 /DPTP_MIN1_CD=-6000 /DPTP_MAX1_CD=6000 /DPTP_MIN2_CD=-9000 /DPTP_MAX2_CD=9000'
    (tmp/'build.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /TC /std:c11 /W3 {defines} /I"{tmp}" "{tmp / "check.c"}" /Fe:"{tmp / "check.exe"}" /Fo:"{tmp / "check.obj"}"\n',encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True)
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in ptp_cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,timeout=20)
        assert result.returncode==0, f'{case}: {result.stdout}\n{result.stderr}'
        print('PASS simulated-unlock',case)
    # Build the exact delivery defaults as a third profile, with no macro overrides.
    build = (tmp/'build.cmd').read_text(encoding='utf-8').replace(defines, '')
    (tmp/'build.cmd').write_text(build, encoding='utf-8')
    result=subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,capture_output=True,text=True)
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    for case in ptp_cases:
        result=subprocess.run([str(tmp/'check.exe'),case],cwd=tmp,capture_output=True,text=True,timeout=20)
        assert result.returncode==0, f'delivery {case}: {result.stdout}\n{result.stderr}'
        print('PASS delivery-default',case)
print(f'{len(cases)+2*len(ptp_cases)} actual-C simulation cases passed; hardware validation still required.')
