"""Exercise the actual RC capture callback with mocked timer/RTOS interfaces."""
from pathlib import Path
import argparse, re, subprocess, tempfile, os

parser=argparse.ArgumentParser()
parser.add_argument('--compiler',default='gcc')
args=parser.parse_args()
source=(Path(__file__).resolve().parents[2]/'LUCSYS_APP/rc_joystick.c').read_text(encoding='gbk')
source=re.sub(r'^#include[^\n]*\n','',source,flags=re.M)
pre=r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
typedef int BaseType_t;
typedef int HAL_TIM_ActiveChannel;
typedef struct { int Channel; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim4;
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 4
#define TIM_CHANNEL_3 8
#define TIM_CHANNEL_4 12
#define HAL_TIM_ACTIVE_CHANNEL_1 1
#define HAL_TIM_ACTIVE_CHANNEL_2 2
#define HAL_TIM_ACTIVE_CHANNEL_3 3
#define HAL_TIM_ACTIVE_CHANNEL_4 4
#define TIM_ICPOLARITY_FALLING 0
#define TIM_ICPOLARITY_RISING 1
#define TIM_RESET_CAPTUREPOLARITY(t,c) ((void)0)
#define TIM_SET_CAPTUREPOLARITY(t,c,p) ((void)0)
#define PI 3.14159265358979323846f
enum { LSR280, LSR300, RCJOY_CMD };
typedef struct { int LX,LY,RX,RY; uint8_t freq; } RC_REMOTE_t;
typedef struct { int cmdsource; float Vx,Vy,Vz; } RobotControlCMDType_t;
static struct { int CarType; } RobotHardWareParam={LSR280};
static struct { int defalutSpeed; } RobotControlParam={500};
typedef int RTOSDebugPrivateVar;
static uint8_t UpdateFreq(RTOSDebugPrivateVar *v){(void)v;return 50;}
typedef struct { uint8_t (*UpdateFreq)(RTOSDebugPrivateVar*); } RTOSDebugInterface_t;
typedef RTOSDebugInterface_t *pRTOSDebugInterface_t;
static RTOSDebugInterface_t RTOSDebugTimer={UpdateFreq};
static uint32_t tick, capture;
static uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *t,uint64_t c){(void)t;(void)c;return capture;}
static uint32_t xTaskGetTickCountFromISR(void){return tick;}
static int target_limit_int(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);}
static RobotControlCMDType_t packet;
static int writes, reject;
static int WriteRobotControlQueue(RobotControlCMDType_t *c,BaseType_t *w){(void)w;packet=*c;writes++;return reject;}
'''
post=r'''
static void pulse(int ch,int width){
 BaseType_t w=0; htim4.Channel=ch;
 capture=100;RCJoystick_IC_CaptureCallback(&htim4,&w);
 capture=100+width;RCJoystick_IC_CaptureCallback(&htim4,&w);
}
static void command(int lateral,int forward,int throttle,int rotation){
 pulse(4,lateral);pulse(2,forward);pulse(3,throttle);
 rc_remote.RX=rotation;tick+=20;int before=writes;
 pulse(1,rotation);assert(writes==before+1);assert(packet.cmdsource==RCJOY_CMD);
}
static void expect(float x,float y,float z){
 assert(fabsf(packet.Vx-x)<0.0001f);assert(fabsf(packet.Vy-y)<0.0001f);assert(fabsf(packet.Vz-z)<0.0001f);
}
int main(void){
 /* Hold forward for >2 seconds to exercise the original unlock condition. */
 rc_remote.LY=2000;rc_remote.RY=1500;
 for(int i=0;i<110;i++){tick+=20;pulse(1,1500);}
 assert(writes>0);
 command(1500,1500,1500,1500);expect(0,0,0);
 command(1000,1500,1500,1500);expect(0,0.5f,0);
 command(2000,1500,1500,1500);expect(0,-0.5f,0);
 command(1410,1500,1500,1500);expect(0,0,0);
 command(1590,1500,1500,1500);expect(0,0,0);
 command(1400,1500,1500,1500);expect(0,0.1f,0);
 command(1600,1500,1500,1500);expect(0,-0.1f,0);
 command(1000,1500,2000,1500);expect(0,1,0);
 command(1000,1500,1000,1500);expect(0,0,0);
 command(1000,2000,1500,1000);expect(0.5f,0.5f,PI/4);
 command(1000,1000,1500,1000);expect(-0.5f,0.5f,-PI/4);
 command(2500,1500,1500,1500);expect(0,-0.5f,0);
 RobotHardWareParam.CarType=LSR300;
 command(1000,2000,1500,1500);expect(0.5f,0,0);
 RobotHardWareParam.CarType=LSR280;
 /* Lateral-only commands must continue to be queued beyond idleCount's limit. */
 for(int i=0;i<20;i++){command(1000,1500,1500,1500);expect(0,0.5f,0);}
 /* Queue failure must revoke RC unlock, as it did before this change. */
 reject=1;command(1000,1500,1500,1500);reject=0;
 int before=writes;tick+=20;pulse(1,1500);assert(writes==before);
 puts("PASS: channel 4 capture, both lateral directions, neutral/deadzone, throttle, mixed motion, input limits, other vehicle, lateral command persistence, queue failure relock");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lsr280_rc_regression_') as folder:
    p=Path(folder); c=p/'rc_test.c';exe=p/'rc_test.exe'
    c.write_text(pre+source+post,encoding='utf-8')
    env=dict(os.environ);env['PATH']=str(Path(args.compiler).parent)+os.pathsep+env.get('PATH','')
    subprocess.run([args.compiler,'-std=c99','-Wall','-Wextra','-O2',str(c),'-o',str(exe)],check=True,env=env)
    subprocess.run([str(exe)],check=True,env=env)
