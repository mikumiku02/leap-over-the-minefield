/*********************************************************************************************************************
 * Name:flycontrol.c
 *      飞控
 * Created on:
 * Author:  田深深深深深深
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"

PIDController hight;
PIDController Roll;
PIDController Yaw;
PIDController Pitch;
PIDController XX;
PIDController YY;
PIDController PMW_X;
PIDController PMW_Y;
float hhhh;
float yyyy;
float rrrr;
float pppp;
float offest_x;
float offest_y;
float pmw_xx;
float pmw_yy;
float target_xx;
float target_yy;
float highter;//高度
volatile float PPXX;
float PPYY;

void fly_star (void)
{   
    target_xx = PID_Update(&XX,94.0,offest_x,100);//摄像头
    target_yy = PID_Update(&YY,60.0,offest_y,100);
    
    hhhh= PID_Update(&hight,1200.0,highter,1200.0);//TOF
    
    pmw_xx=PID_Update(&PMW_X,0+target_xx,PPXX,100);//光流
    pmw_yy=PID_Update(&PMW_Y,0+target_yy,PPYY,100);
    
    yyyy= PID_Update(&Yaw,0,euler_angle.yaw,100);//电机要的
    rrrr= PID_Update(&Roll,0+pmw_xx,euler_angle.roll,100);
    pppp= PID_Update(&Pitch,0+pmw_yy,euler_angle.pitch,100);
    
}


