param([string]$Compiler='gcc.exe')
$ErrorActionPreference='Stop'
$Compiler=(Get-Command $Compiler -ErrorAction Stop).Source
$src=[Text.Encoding]::GetEncoding(936).GetString([IO.File]::ReadAllBytes((Join-Path $PSScriptRoot '../../LUCSYS_APP/RobotControl_task_v2.c')))
function Extract([string]$name){
 $m=[regex]::Match($src,'static void '+$name+'\([^;]*?\)\s*\{'); if(-not $m.Success){throw $name}
 $start=$m.Index; $i=$src.IndexOf('{',$start); $depth=1; $i++
 while($depth -gt 0){if($src[$i] -eq '{'){$depth++}; if($src[$i] -eq '}'){$depth--}; $i++}
 return $src.Substring($start,$i-$start)
}
$pre=@"
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
typedef struct {float Vx,Vy,Vz; int extra;} RobotControlCMDType_t;
typedef int BaseType_t;
#define pdFAIL 0
#define pdPASS 1
#define ROBOT_CONTROL_FREQ_HZ 100u
#define WHEEL_COUNT 4
#define WHEEL_A 0
#define WHEEL_B 1
#define WHEEL_C 2
#define WHEEL_D 3
#define COUNTS_PER_DEG (16384.0f/360.0f)
enum {GamePad_CMD,RCJOY_CMD,Charger_CMD,AutoTest_CMD,ROS_CMD,CAN_CMD};
static int RobotControl_CMDsource=ROS_CMD;
static struct {int ImuAssistedFlag,SecurityLevel,en_flag,ParkingMode;} RobotControlParam;
static RobotControlCMDType_t LinearHelp_Target,packet;
static int available,Driver1,Driver2,sends1,sends2,g_xQueueRobotControl;
static struct {int reorientHold,maxProgress,lowfreCnt,progress[4],lastProgress[4];} s_steer;
static int xQueueReceive(int q,RobotControlCMDType_t *c,int wait){(void)q;(void)wait;if(!available)return pdFAIL;*c=packet;available=0;return pdPASS;}
static void Linear_IMUHelp(RobotControlCMDType_t *c){assert(c->Vz==0);LinearHelp_Target.Vz=0.2f;}
static void Steer_SendDriver(void *s,int *d,int a,int b){(void)s;(void)a;(void)b;if(d==&Driver1)sends1++;else sends2++;}
"@
$post=@"
int main(void){
 RobotControlCMDType_t cmd={0};uint8_t lost=0;
 RobotControlParam.ImuAssistedFlag=1;packet.Vx=1;available=1;
 Robot_CmdReceive(&cmd,&lost);assert(cmd.Vz==0.2f);
 for(int i=0;i<50;i++){Robot_CmdReceive(&cmd,&lost);assert(cmd.Vx==1 && cmd.Vz==0.2f);}
 RobotControlParam.ImuAssistedFlag=0;
 for(int i=0;i<300;i++)Robot_CmdReceive(&cmd,&lost);
 assert(cmd.Vx==0 && cmd.Vz==0 && lost==100);
 RobotControlParam.en_flag=1;s_steer.maxProgress=1000;
 s_steer.progress[1]=1000;
 for(int i=0;i<10;i++){s_steer.progress[1]-=20;Steer_Watchdog();}
 assert(sends1==0 && sends2==0);
 for(int i=0;i<10;i++)Steer_Watchdog();
 assert(sends1==1 && sends2==0);
 s_steer.lowfreCnt=8;s_steer.reorientHold=1;Steer_Watchdog();assert(s_steer.lowfreCnt==0);
 puts("PASS: raw command isolation, command expiry, per-wheel watchdog, watchdog reset");return 0;
}
"@
$out=Join-Path $env:TEMP 'lsr280_control_regression.c';$exe=Join-Path $env:TEMP 'lsr280_control_regression.exe'
[IO.File]::WriteAllText($out,$pre+(Extract 'Robot_FailSafe')+(Extract 'Robot_CmdReceive')+(Extract 'Steer_Watchdog')+$post,[Text.UTF8Encoding]::new($false))
$env:Path=(Split-Path $Compiler)+';'+$env:Path
& $Compiler -std=c99 -Wall -Wextra -O2 $out -o $exe
if($LASTEXITCODE -ne 0){throw 'Compile failed'}
& $exe
if($LASTEXITCODE -ne 0){throw 'Tests failed'}
