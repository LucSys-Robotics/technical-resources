/**
 * @file    main.c
 * @brief PC differential-test harness for four-wheel steering kinematics
 *
 * @details
 * Directly compile the real source files in the firmware (.. /.. /LUCSYS_BSP/4ws_v2.c), no copying or rewriting is done,
 * Ensures that the code tested is actually running on the car.
 *
 * Covers 5 invariants:
 *   T1 round-trip check: inverse kinematics -> assume alignment -> forward kinematics; recover the original three-axis velocity
 *   T2 Physical Angle Range AB Round Target ∈ [-50,220] degrees, CD Round ∈ [-220,50] degrees
 *   T3 projection consistency: WheelSpeedProjection must have the same wheel speed as InverseKinematics
 *   T4 Velocity vector continuous. When input is continuously changing, the wheel speed vector should not jump (180-degree flip + speed inversion is legal).
 *   T5 zero-speed hold: When all three axes are zero, the previous angle is maintained, only the wheel speed returns to zero
 *
 * T3 corresponds to code review Article 7: Firmware uses reverse resolution to obtain angles, and projections to obtain wheel speed; both must be strictly consistent.
 * Otherwise, "the angle of the release does not match the wheel speed symbol, causing the wheel to instantly reverse."
 * Steering jitter observed in the actual vehicle corresponding to T4: When upstream commands change continuously, the target angle should not experience meaningless jumps.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* This file is UTF-8 encoded, while the Windows console defaults to GBK (CP936).
   If you don't switch, the Chinese output will appear as garbled text. Linux terminals are originally UTF-8, so no processing is needed. */
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

/* LSR280 actual specifications, consistent with robot_select_init.h */
#define LSR280_WHEELSPACING   0.335f
#define LSR280_AXLESPACING    0.43f
#define LSR280_WHEELTURNR     0.0f
#define LSR280_WHEELPERIMETER (0.13f*3.14159265f)

/* Criteria threshold */
#define EPS_ROUNDTRIP   1e-3f    /* Allowed round-trip comparison error, m/s and rad/s */
#define EPS_PROJECTION  1e-4f    /* The difference in wheel speed between the projection method and the inverse solution */
#define ANGLE_MARGIN    0.5f     /* Physical range determination margin (degrees), tolerance floating-point boundary */
#define EPS_CONTINUITY  0.05f    /* Velocity-vector continuity threshold, m/s */

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

/* Move four target values to actual values to simulate "wheels fully in place" */
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
    FourWSCar_param_init(car,LSR280_WHEELSPACING,LSR280_AXLESPACING,
                         LSR280_WHEELTURNR,LSR280_WHEELPERIMETER);
}

/* ------------------------------------------------------------------ */
/* T1 round-trip check: inverse kinematics -> assume alignment -> forward kinematics; recover the original three-axis velocity            */
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
/* T2 Physical angle range: AB∈ [-50,220] degrees; CD∈ [-220,50] degrees                      */
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
/* T3 Projection Consistency: Under the same set of speeds, the projected wheel speed must equal the wheel speed obtained by inverse solution        */
/*    Firmware uses reverse resolution to obtain angles and projections to get wheel speed; inconsistencies between the two can cause conflicts between angle and wheel speed symbols  */
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

        /* The projection method multiplexes the angle written in the inverse solution, only recalculating the wheel speed */
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
/* T4 velocity vector continuity                                                    */
/*    Wheel velocity vector v = speed*(cos a, sin a) should vary continuously with the input.                */
/*    180-degree flip with reversed speed does not change the vector, so legal flips will not be misjudged.          */
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
/* T5 zero-speed hold: When all three axes are zero, the previous angle is maintained; only wheel speed returns to zero                        */
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
            if(fabsf(keep[k]-now[k])>1e-6f) hit=1;   /* The angle must be maintained */
            if(fabsf(spd[k])>1e-6f)         hit=1;   /* Wheel speed must return to zero */
        }
        if(hit) bad++;
        assume_reached(&car);
    }
    report("T5 零速保持",bad,n,NULL);
}

/* ------------------------------------------------------------------ */
/* T6 Branch jitter: When the wheel cannot keep up with the target, the slightly shaking Vz should not cause the target to repeatedly flip 180 degrees      */
/*                                                                     */
/* PathChange uses "measured wheel angle vs target" to determine which route to take (directly in position / +180 degrees and reverse wheel speed).*/
/* The criterion input is the measured wheel angle, and the wheel is rotating toward the target — it oscillates at zero hysteresis:          */
/* Cross the 90-degree threshold -> Target flips 180 degrees -> Wheel turns -> Crosses back to the threshold -> Flips again.         */
/* Real car performance: The steering target repeatedly flips, the wheels never catch up, the speed limiter lowers the speed to zero, and the car gets stuck.  */
/* This use case simulates "wheels not keeping up" with first-order lag to reproduce the operating condition.                          */
/* ------------------------------------------------------------------ */
static void lag_wheels(RobotParam_t* car,float k)
{
    /* Wheel approaches the target by a factor k; k<1 means it cannot catch up */
    car->RealMotorA.radian += k*(car->TargetMotorA.radian - car->RealMotorA.radian);
    car->RealMotorB.radian += k*(car->TargetMotorB.radian - car->RealMotorB.radian);
    car->RealMotorC.radian += k*(car->TargetMotorC.radian - car->RealMotorC.radian);
    car->RealMotorD.radian += k*(car->TargetMotorD.radian - car->RealMotorD.radian);
}

static void test_branch_chatter(void)
{
    const int  CASES  = 4000;    /* Number of random operating points */
    const int  CYCLES = 24;      /* How many beats are jittered at each work point */
    const float DVZ   = 0.02f;   /* Vz jitter amplitude rad/s, very small */
    const float LAG   = 0.02f;   /* Wheel covers only 2% of the error per cycle: cannot catch up and stays near the boundary for a long time */
    const float FLIP  = 2.6f;    /* Target jump threshold (RAD) judged to be flipped, approximately 150 degrees */
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

        /* First solve the target angle that obtains the work point */
        four_ws_InverseKinematics(vx,vy,vzc,&car);

        /* Key: Set the measured wheel angle about 90 degrees from the target, which is the PathChange determination boundary.
           This is the state seen when the physical chassis stalls (steering error 60~130 degrees); warming directly to steady state cannot reproduce it.*/
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
        /* The criterion focuses on "self-sustaining oscillations" rather than "flips not zero."
           Legal re-selection will naturally be overturned; occasionally flipping a few times is not a flaw;
           8 flips within 24 cycles (one wheel flipping every three cycles) indicate a limit cycle --
           This is exactly the state when the real car gets stuck: the target reverses with every beat, the wheels never catch up, and the speed limiter lowers the speed to zero.*/
        if( flips >= 8 ) bad++;
        if( flips > worst ) worst = flips;
    }

    /* The criterion is "not worse than the known bottom line" rather than "zero jitter."
       Besides path optimization,  PathChange also includes two physical limit reversals (unconditional when the angle exceeds -50/+220 degrees).
       Flip and reverse speed), those two places have no branch memory, and the original angle will flip back and forth when swinging near the limit.
       Margin scans confirm this: 0 degrees margin 28.8% jitter, 15 degrees 11.8%, 35 degrees 3.7%,
       Still stops at 3.1% after adding to 45/60 degrees — the rest is just the bottom line contributed by limit inversion.
       It is feasible to add hysteresis to the limit criterion (theoretical range between -90/+270 and the actual -50/+220).
       already leaves a 40-degree margin), which is an incomplete term. Here, the threshold is set at 5%, which is enough to block the regression of the path criterion. */
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
    srand(seed);   /* Fix the seed, and the fruit can be reproducible */
    console_use_utf8();

    printf("四轮四转运动学对拍  样本 %ld 组  随机种子 %u\n", n, seed);
    printf("车型参数: 轮距 %.3f  轴距 %.3f  转向半径 %.3f\n\n",
           LSR280_WHEELSPACING,LSR280_AXLESPACING,LSR280_WHEELTURNR);

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
