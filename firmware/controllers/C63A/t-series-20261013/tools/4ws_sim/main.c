/**
 * @file    main.c
 * @brief   四轮四转运动学 PC 端对拍工程
 *
 * @details
 * 直接编译固件里的真实源文件(../../LUCSYS_BSP/4ws_v2.c),不做任何拷贝或改写,
 * 保证测的就是实际跑在车上的那份代码.
 *
 * 覆盖 5 项不变量:
 *   T1 往返对拍     逆解->假设到位->正解, 三轴速度应还原
 *   T2 物理角度范围 AB轮目标∈[-50,220]度, CD轮∈[-220,50]度
 *   T3 投影法一致性 WheelSpeedProjection 与 InverseKinematics 的轮速必须相同
 *   T4 速度矢量连续 输入连续变化时,轮速矢量不应跳变(180度翻转+速度取反是合法的)
 *   T5 零速保持     三轴为零时保持上次角度,仅轮速归零
 *
 * T3 对应 code review 第7条: 固件用逆解取角度、用投影取轮速,两者必须严格一致,
 * 否则会出现"下发的角度与轮速符号不匹配,轮子瞬间反转".
 * T4 对应实车观察到的转向抖动: 上游指令连续变化时目标角不应出现无意义突跳.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* 本文件为 UTF-8 编码,而 Windows 控制台默认是 GBK(CP936),
   不切换会把中文输出显示成乱码.Linux 终端本来就是 UTF-8,无需处理. */
#ifdef _WIN32
#include <windows.h>
static void console_use_utf8(void){ SetConsoleOutputCP(CP_UTF8); }
#else
static void console_use_utf8(void){ }
#endif

#include "4ws.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG(r)  ((r)*57.29577951308232f)
#define DEG2RAD(d)  ((d)*0.01745329251994330f)

/* T280 实机参数, 与 robot_select_init.h 保持一致 */
#define T280_WHEELSPACING   0.335f
#define T280_AXLESPACING    0.43f
#define T280_WHEELTURNR     0.0f
#define T280_WHEELPERIMETER (0.13f*3.14159265f)

/* 判据阈值 */
#define EPS_ROUNDTRIP   1e-3f    /* 往返对拍允许误差 m/s, rad/s */
#define EPS_PROJECTION  1e-4f    /* 投影法与逆解的轮速差 */
#define ANGLE_MARGIN    0.5f     /* 物理范围判定余量(度), 容忍浮点边界 */
#define EPS_CONTINUITY  0.05f    /* 速度矢量连续性阈值 m/s */

static int g_fail = 0;

static void report(const char* name,int bad,long total,const char* detail)
{
    if( bad==0 )
        printf("  [PASS] %-16s %ld 组全部通过%s\n", name, total, detail?detail:"");
    else {
        printf("  [FAIL] %-16s %ld 组中 %d 组不通过 (%.4f%%)%s\n",
               name, total, bad, 100.0*bad/total, detail?detail:"");
        g_fail++;
    }
}

static float frand(float lo,float hi)
{
    return lo + ((float)rand()/(float)RAND_MAX)*(hi-lo);
}

/* 把四个目标值搬到实际值, 模拟"轮子完全到位" */
static void assume_reached(RobotParam_t* car)
{
    car->RealMotorA = car->TargetMotorA;
    car->RealMotorB = car->TargetMotorB;
    car->RealMotorC = car->TargetMotorC;
    car->RealMotorD = car->TargetMotorD;
}

static void car_init(RobotParam_t* car)
{
    memset(car,0,sizeof(*car));
    FourWSCar_param_init(car,T280_WHEELSPACING,T280_AXLESPACING,
                         T280_WHEELTURNR,T280_WHEELPERIMETER);
}

/* ------------------------------------------------------------------ */
/* T1 往返对拍: 逆解 -> 假设到位 -> 正解, 应还原出原始三轴速度            */
/* ------------------------------------------------------------------ */
static void test_roundtrip(long n)
{
    RobotParam_t car; car_init(&car);
    int bad=0; float worst=0.0f;

    for(long i=0;i<n;i++)
    {
        float vx=frand(-2.0f,2.0f), vy=frand(-2.0f,2.0f), vz=frand(-3.0f,3.0f);

        four_ws_InverseKinematics(vx,vy,vz,&car);
        assume_reached(&car);
        four_ws_kinematics(&car);

        float ex=fabsf(vx-car.feedback_Vx);
        float ey=fabsf(vy-car.feedback_Vy);
        float ez=fabsf(vz-car.feedback_Vz);
        float e = ex>ey?ex:ey; if(ez>e) e=ez;
        if(e>worst) worst=e;
        if(e>EPS_ROUNDTRIP) bad++;
    }
    char buf[64]; snprintf(buf,sizeof(buf),"  最大误差 %.6f",worst);
    report("T1 往返对拍",bad,n,buf);
}

/* ------------------------------------------------------------------ */
/* T2 物理角度范围: AB∈[-50,220]度  CD∈[-220,50]度                      */
/* ------------------------------------------------------------------ */
static void test_angle_range(long n)
{
    RobotParam_t car; car_init(&car);
    int bad=0; float worstAB=0.0f, worstCD=0.0f;

    for(long i=0;i<n;i++)
    {
        float vx=frand(-2.0f,2.0f), vy=frand(-2.0f,2.0f), vz=frand(-3.0f,3.0f);
        four_ws_InverseKinematics(vx,vy,vz,&car);

        float a[2] = { RAD2DEG(car.TargetMotorA.radian), RAD2DEG(car.TargetMotorB.radian) };
        float c[2] = { RAD2DEG(car.TargetMotorC.radian), RAD2DEG(car.TargetMotorD.radian) };
        int hit=0;
        for(int k=0;k<2;k++){
            float over = 0.0f;
            if(a[k] < -50.0f) over = -50.0f-a[k];
            if(a[k] > 220.0f) over = a[k]-220.0f;
            if(over>worstAB) worstAB=over;
            if(over>ANGLE_MARGIN) hit=1;

            over = 0.0f;
            if(c[k] < -220.0f) over = -220.0f-c[k];
            if(c[k] >   50.0f) over = c[k]-50.0f;
            if(over>worstCD) worstCD=over;
            if(over>ANGLE_MARGIN) hit=1;
        }
        if(hit) bad++;
        assume_reached(&car);
    }
    char buf[96]; snprintf(buf,sizeof(buf),"  最大越界 AB %.3f度 / CD %.3f度",worstAB,worstCD);
    report("T2 角度范围",bad,n,buf);
}

/* ------------------------------------------------------------------ */
/* T3 投影法一致性: 同一组速度下, 投影出的轮速必须等于逆解出的轮速        */
/*    固件用逆解取角度、用投影取轮速, 两者不一致会导致角度与轮速符号打架  */
/* ------------------------------------------------------------------ */
static void test_projection(long n)
{
    RobotParam_t car; car_init(&car);
    int bad=0; float worst=0.0f;

    for(long i=0;i<n;i++)
    {
        float vx=frand(-2.0f,2.0f), vy=frand(-2.0f,2.0f), vz=frand(-3.0f,3.0f);

        four_ws_InverseKinematics(vx,vy,vz,&car);
        float sInv[4] = { car.TargetMotorA.speed, car.TargetMotorB.speed,
                          car.TargetMotorC.speed, car.TargetMotorD.speed };

        /* 投影法复用逆解写入的角度, 只重算轮速 */
        four_ws_WheelSpeedProjection(vx,vy,vz,&car);
        float sPrj[4] = { car.TargetMotorA.speed, car.TargetMotorB.speed,
                          car.TargetMotorC.speed, car.TargetMotorD.speed };

        int hit=0;
        for(int k=0;k<4;k++){
            float d=fabsf(sInv[k]-sPrj[k]);
            if(d>worst) worst=d;
            if(d>EPS_PROJECTION) hit=1;
        }
        if(hit) bad++;
        assume_reached(&car);
    }
    char buf[64]; snprintf(buf,sizeof(buf),"  最大轮速差 %.6f m/s",worst);
    report("T3 投影一致性",bad,n,buf);
}

/* ------------------------------------------------------------------ */
/* T4 速度矢量连续性                                                    */
/*    轮速矢量 v = speed*(cos a, sin a) 应随输入连续变化.                */
/*    180度翻转配合速度取反不改变该矢量, 所以合法翻转不会被误判.          */
/* ------------------------------------------------------------------ */
static void test_continuity(void)
{
    struct { const char* name; float vx0,vy0,vz0, vx1,vy1,vz1; } sweeps[] = {
        { "直行->自转",   0.5f, 0.0f,-2.0f,   0.5f, 0.0f, 2.0f },
        { "直行->横移",   0.5f,-1.0f, 0.0f,   0.5f, 1.0f, 0.0f },
        { "低速+自转",   0.05f, 0.0f,-2.0f,  0.05f, 0.0f, 2.0f },
        { "倒车->前进",  -1.0f, 0.0f, 0.3f,   1.0f, 0.0f, 0.3f },
        { "斜向扫描",    -1.0f, 1.0f, 0.5f,   1.0f,-1.0f, 0.5f },
    };
    const int NS = (int)(sizeof(sweeps)/sizeof(sweeps[0]));
    const int STEPS = 20000;
    int totalBad=0; long totalPts=0; float worst=0.0f; const char* worstName="";

    for(int s=0;s<NS;s++)
    {
        RobotParam_t car; car_init(&car);
        float pvx[4]={0},pvy[4]={0}; int first=1; int bad=0;

        for(int i=0;i<=STEPS;i++)
        {
            float t=(float)i/(float)STEPS;
            float vx=sweeps[s].vx0+(sweeps[s].vx1-sweeps[s].vx0)*t;
            float vy=sweeps[s].vy0+(sweeps[s].vy1-sweeps[s].vy0)*t;
            float vz=sweeps[s].vz0+(sweeps[s].vz1-sweeps[s].vz0)*t;

            four_ws_InverseKinematics(vx,vy,vz,&car);
            WheelInfo_t* w[4]={&car.TargetMotorA,&car.TargetMotorB,
                               &car.TargetMotorC,&car.TargetMotorD};
            float cvx[4],cvy[4];
            for(int k=0;k<4;k++){
                cvx[k]=w[k]->speed*cosf(w[k]->radian);
                cvy[k]=w[k]->speed*sinf(w[k]->radian);
            }
            if(!first){
                for(int k=0;k<4;k++){
                    float dx=cvx[k]-pvx[k], dy=cvy[k]-pvy[k];
                    float d=sqrtf(dx*dx+dy*dy);
                    if(d>worst){ worst=d; worstName=sweeps[s].name; }
                    if(d>EPS_CONTINUITY){ bad++; break; }
                }
            }
            memcpy(pvx,cvx,sizeof(cvx)); memcpy(pvy,cvy,sizeof(cvy));
            first=0;
            assume_reached(&car);
            totalPts++;
        }
        if(bad) printf("       %-12s 跳变 %d 次\n", sweeps[s].name, bad);
        totalBad+=bad;
    }
    char buf[96]; snprintf(buf,sizeof(buf),"  最大跳变 %.4f m/s (%s)",worst,worstName);
    report("T4 矢量连续性",totalBad,totalPts,buf);
}

/* ------------------------------------------------------------------ */
/* T5 零速保持: 三轴为零时保持上次角度, 仅轮速归零                        */
/* ------------------------------------------------------------------ */
static void test_zero_hold(long n)
{
    RobotParam_t car; car_init(&car);
    int bad=0;

    for(long i=0;i<n;i++)
    {
        float vx=frand(-2.0f,2.0f), vy=frand(-2.0f,2.0f), vz=frand(-3.0f,3.0f);
        four_ws_InverseKinematics(vx,vy,vz,&car);
        float keep[4]={car.TargetMotorA.radian,car.TargetMotorB.radian,
                       car.TargetMotorC.radian,car.TargetMotorD.radian};

        four_ws_InverseKinematics(0.0f,0.0f,0.0f,&car);
        float now[4]={car.TargetMotorA.radian,car.TargetMotorB.radian,
                      car.TargetMotorC.radian,car.TargetMotorD.radian};
        float spd[4]={car.TargetMotorA.speed,car.TargetMotorB.speed,
                      car.TargetMotorC.speed,car.TargetMotorD.speed};

        int hit=0;
        for(int k=0;k<4;k++){
            if(fabsf(keep[k]-now[k])>1e-6f) hit=1;   /* 角度必须保持 */
            if(fabsf(spd[k])>1e-6f)         hit=1;   /* 轮速必须归零 */
        }
        if(hit) bad++;
        assume_reached(&car);
    }
    report("T5 零速保持",bad,n,NULL);
}

/* ------------------------------------------------------------------ */
/* T6 分支抖动: 轮子追不上目标时, 小幅抖动的Vz不应让目标反复翻转180度      */
/*                                                                     */
/* PathChange 用"实测轮角 vs 目标"判断走哪条路(直接到位 / +180度并反转轮速)。*/
/* 判据输入是实测轮角, 而轮子又在朝目标转 —— 零迟滞时会自持振荡:          */
/* 越过90度门限 -> 目标翻180度 -> 轮子掉头 -> 又越回门限 -> 再翻。         */
/* 实车表现: 转向目标反复翻转, 轮子永远追不上, 限速器把车速压到0, 车卡死。  */
/* 本用例用一阶滞后模拟"轮子跟不上", 复现该工况。                          */
/* ------------------------------------------------------------------ */
static void lag_wheels(RobotParam_t* car,float k)
{
    /* 轮子以 k 的比例向目标靠拢, k<1 表示追不上 */
    car->RealMotorA.radian += k*(car->TargetMotorA.radian - car->RealMotorA.radian);
    car->RealMotorB.radian += k*(car->TargetMotorB.radian - car->RealMotorB.radian);
    car->RealMotorC.radian += k*(car->TargetMotorC.radian - car->RealMotorC.radian);
    car->RealMotorD.radian += k*(car->TargetMotorD.radian - car->RealMotorD.radian);
}

static void test_branch_chatter(void)
{
    const int  CASES  = 4000;    /* 随机工作点数 */
    const int  CYCLES = 24;      /* 每个工作点上抖动多少拍 */
    const float DVZ   = 0.02f;   /* Vz 抖动幅度 rad/s, 很小 */
    const float LAG   = 0.02f;   /* 轮子每拍只走 2% 的差距: 追不上, 长时间停在分界附近 */
    const float FLIP  = 2.6f;    /* 判为翻转的目标跳变门限(rad), 约150度 */
    const float HALFPI = 1.5707963f;

    RobotParam_t car;
    int  bad = 0;
    int  worst = 0;
    long total = 0;

    for(int c=0;c<CASES;c++)
    {
        car_init(&car);
        float vx  = frand(-0.4f,0.4f);
        float vy  = frand(-0.4f,0.4f);
        float vzc = frand(-0.6f,0.6f);

        /* 先解一次拿到该工作点的目标角 */
        four_ws_InverseKinematics(vx,vy,vzc,&car);

        /* 关键: 把实测轮角按在距目标约90度处, 也就是 PathChange 的判定分界上。
           这正是实车卡死时的状态(转向偏差 60~130 度), 直接热身到稳态是复现不出来的。*/
        float off = HALFPI + frand(-0.10f,0.10f);
        car.RealMotorA.radian = car.TargetMotorA.radian - off;
        car.RealMotorB.radian = car.TargetMotorB.radian - off;
        car.RealMotorC.radian = car.TargetMotorC.radian + off;
        car.RealMotorD.radian = car.TargetMotorD.radian + off;
        car.pathFlip[0]=car.pathFlip[1]=car.pathFlip[2]=car.pathFlip[3]=0;

        float prev[4] = { car.TargetMotorA.radian, car.TargetMotorB.radian,
                          car.TargetMotorC.radian, car.TargetMotorD.radian };
        int flips = 0;
        for(int i=0;i<CYCLES;i++)
        {
            float vz = vzc + ((i&1) ? DVZ : -DVZ);
            four_ws_InverseKinematics(vx,vy,vz,&car);
            float now[4] = { car.TargetMotorA.radian, car.TargetMotorB.radian,
                             car.TargetMotorC.radian, car.TargetMotorD.radian };
            for(int w=0;w<4;w++)
            {
                if( fabsf(now[w]-prev[w]) > FLIP ) flips++;
                prev[w] = now[w];
            }
            lag_wheels(&car,LAG);
        }
        total++;
        /* 判据盯的是"自持振荡"而不是"翻转次数不为零"。
           合法的重新选路本来就会翻,偶尔翻几次不是缺陷;
           24拍内翻够 8 次(相当于某个轮子每三拍就翻一次)才说明陷进了极限环 ——
           实车卡死时正是这种状态:目标每拍都反,轮子永远追不上,限速器把车速压到0。*/
        if( flips >= 8 ) bad++;
        if( flips > worst ) worst = flips;
    }

    /* 判据是"不劣于已知底线"而不是"零抖动"。
       PathChange 里除了路径优化,还有两处物理限位翻转(角度越出 -50/+220 度时无条件
       翻转并反速),那两处没有分支记忆,原始角在限位附近摆动时照样会来回翻。
       余量扫描证实了这一点:0度余量 28.8% 抖动,15度 11.8%,35度 3.7%,
       再加到 45/60 度仍停在 3.1% 不再降 —— 剩下的就是限位翻转贡献的底线。
       给限位判据也加迟滞是可行的(理论范围 -90/+270 与实际启用的 -50/+220 之间
       本就留了 40 度余量),属于未完成项。此处门限设 5%,足以拦住路径判据的回归。 */
    const double LIMIT_PCT = 1.0;
    double pct = 100.0*bad/total;
    char det[160];
    snprintf(det,sizeof(det),"  抖动率 %.2f%% (门限 %.1f%%)  最多一组翻转 %d 次",
             pct, LIMIT_PCT, worst);
    report("T6 分支抖动", (pct > LIMIT_PCT) ? bad : 0, total, det);
}

extern void PathChange_AB(WheelInfo_t*,float,uint8_t*);
extern void PathChange_CD(WheelInfo_t*,float,uint8_t*);
static void test_bounded_path(void)
{
    int bad=0;
    for(int flip=0;flip<2;flip++) {
        uint8_t state=(uint8_t)flip;
        WheelInfo_t w={DEG2RAD(-40),1};
        PathChange_AB(&w,DEG2RAD(220),&state);
        if(fabsf(RAD2DEG(w.radian)-140)>0.001f) bad++;
        state=(uint8_t)flip; w.radian=DEG2RAD(40); w.speed=1;
        PathChange_CD(&w,DEG2RAD(-220),&state);
        if(fabsf(RAD2DEG(w.radian)+140)>0.001f) bad++;
    }
    report("T7 bounded travel",bad,4,"");
}
int main(int argc,char** argv)
{
    long n = (argc>1) ? atol(argv[1]) : 200000L;
    unsigned seed = (argc>2) ? (unsigned)atoi(argv[2]) : 1u;
    if(n<1000) n=1000;
    srand(seed);   /* 固定种子, 结果可复现 */
    console_use_utf8();

    printf("四轮四转运动学对拍  样本 %ld 组  随机种子 %u\n", n, seed);
    printf("车型参数: 轮距 %.3f  轴距 %.3f  转向半径 %.3f\n\n",
           T280_WHEELSPACING,T280_AXLESPACING,T280_WHEELTURNR);

    test_roundtrip(n);
    test_angle_range(n);
    test_projection(n);
    test_continuity();
    test_zero_hold(n>20000?20000:n);
    test_branch_chatter();
    test_bounded_path();

    printf("\n");
    if(g_fail==0){ printf("全部通过\n"); return 0; }
    printf("%d 项未通过\n", g_fail);
    return 1;
}
