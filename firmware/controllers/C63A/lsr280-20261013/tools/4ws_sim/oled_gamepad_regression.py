"""Run actual gamepad BSP and OLED pages with mocked display and RTOS."""
from pathlib import Path
import argparse,os,re,subprocess,tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--compiler',default='gcc')
args=parser.parse_args()
base=Path(__file__).resolve().parents[2]
bsp=(base/'LUCSYS_BSP/bsp_gamepad.c').read_text(encoding='gbk')
bsp=re.sub(r'^#include[^\n]*\n','',bsp,flags=re.M)
show=(base/'LUCSYS_APP/show_task.c').read_text(encoding='gbk')
pad_header=(base/'USB_HOST/App/xbox360_gamepad.h').read_text(encoding='gbk')
key_enum=re.search(r'enum\s*\{\s*Xbox360KEY_UP.*?\};',pad_header,re.S).group(0)
def extract(name):
    m=re.search(r'static void '+name+r'\(void\)\s*\{',show)
    assert m,name
    i=m.end();depth=1
    while depth:
        if show[i]=='{':depth+=1
        elif show[i]=='}':depth-=1
        i+=1
    return show[m.start():i]
pre=r'''
#include <assert.h>
#include <stdio.h>
#include "bsp_gamepad.h"
KEY_ENUM
static uint32_t xTaskGetTickCount(void){return 0;}
static uint32_t numbers[128][64];
static unsigned refreshes, keycalls;
static void number(uint8_t x,uint8_t y,uint32_t n,uint8_t len,uint8_t size){(void)len;(void)size;numbers[x][y]=n;}
static void string(uint8_t x,uint8_t y,const char *s){(void)x;(void)y;(void)s;}
static void decimal(uint8_t x,uint8_t y,float n,uint8_t a,uint8_t b){(void)x;(void)y;(void)n;(void)a;(void)b;}
static void refresh(void){refreshes++;}
static struct {void (*ShowString)(uint8_t,uint8_t,const char*);
 void (*ShowNumber)(uint8_t,uint8_t,uint32_t,uint8_t,uint8_t);
 void (*ShowFloat)(uint8_t,uint8_t,float,uint8_t,uint8_t);
 void (*RefreshGram)(void);} display={string,number,decimal,refresh},*oled=&display;
static struct {int LX,LY,RX,RY,freq;} rc_remote={1000,1200,1800,2000,50};
static struct {float dis;unsigned timeoutCount;} RangerHAL_A,RangerHAL_B,RangerHAL_C,RangerHAL_D,RangerHAL_E,RangerHAL_F;
static int get_EnKeyState(void){return 1;}
static GamePadKeyStateType_t pressed(uint8_t key){assert(key==Xbox360KEY_Menu);keycalls++;return GamePadKeyState_Press;}
'''
pre=pre.replace('KEY_ENUM',key_enum)
post=r'''
static void neutral_page(void){
 JoyShow();
 assert(numbers[30][0]==127 && numbers[30][10]==127 && numbers[30][20]==127 && numbers[30][30]==127);
 assert(numbers[30][40]==GamePadKeyState_Release);
 assert(numbers[70][0]==1000 && numbers[70][10]==1200 && numbers[70][20]==1800 && numbers[70][30]==2000 && numbers[70][40]==50);
}
int main(void){
 assert(GamePadInterface==&GamePadDefalut);
 assert(GamePadInterface->getKeyState!=0);
 for(int k=0;k<256;k++)assert(GamePadInterface->getKeyState((uint8_t)k)==GamePadKeyState_Release);
 for(int i=0;i<100;i++){RangerSensorShow();neutral_page();}
 assert(refreshes==200);
 GamePadType_t connected={.LX=10,.LY=20,.RX=30,.RY=40,.LT=50,.RT=60,.getKeyState=pressed};
 GamePadInterface=&connected;gamepad_brand=Xbox360;GamePadDebug.ready=1;
 JoyShow();assert(keycalls==1 && numbers[30][0]==10 && numbers[30][40]==GamePadKeyState_Press);
 USB_GamePad_PullOutCallback();
 assert(GamePadInterface==&GamePadDefalut && GamePadDebug.ready==0);
 assert(connected.LX==127 && connected.LY==127 && connected.RX==127 && connected.RY==127 && connected.LT==0 && connected.RT==0);
 RangerSensorShow();neutral_page();assert(keycalls==1);
 /* Reconnect after unplugging: the real key callback still works. */
 GamePadInterface=&connected;JoyShow();assert(keycalls==2 && numbers[30][40]==GamePadKeyState_Press);
 puts("PASS: neutral callback for all key IDs; 100 ultrasound-to-joystick transitions without USB; connected keys; unplug/reset; RC values; reconnect");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lsr280_oled_regression_') as folder:
    p=Path(folder);c=p/'oled_test.c';exe=p/'oled_test.exe'
    c.write_text(pre+bsp+extract('RangerSensorShow')+extract('JoyShow')+post,encoding='utf-8')
    env=dict(os.environ);env['PATH']=str(Path(args.compiler).parent)+os.pathsep+env.get('PATH','')
    subprocess.run([args.compiler,'-std=c99','-Wall','-Wextra','-O2','-I'+str(base/'LUCSYS_BSP/Inc'),'-I'+str(base/'USB_HOST/App'),str(c),'-o',str(exe)],check=True,env=env)
    subprocess.run([str(exe)],check=True,env=env)
