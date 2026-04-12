/*********************************************************************************************************************
 * Name:quaternion.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "kaerman.h"
#include "quaternion.h"
my_qu sc;
quat_angle s_angle;
siyuanshu s_shu;
chw_angle cl_angle;
float imu660rb_acc_x_1 =0;
float imu660rb_acc_y_1 =0;
float imu660rb_acc_z_1 =0;
float imu660rb_acc_x_0 = 0;
float imu660rb_acc_y_0 = 0;
float imu660rb_acc_z_0 = 0;

float gyro_offset_x = 0.0f;                                     // 存放X轴陀螺仪开机静止时的本底噪声偏差值
float gyro_offset_y = 0.0f;                                     // 存放Y轴陀螺仪开机静止时的本底噪声偏差值
float gyro_offset_z = 0.0f; 

#define GYRO_SCALE 14.3f
//16.4f
#define ACC_SCALE  4098.0f
void quaternion_Init(my_qu*liu,quat_angle*aksq,siyuanshu*siyaun ,float kkp,float kki,float pit_time)
{
  liu->kp = kkp;
  liu->ki = kki;
  liu->qdt = pit_time;
  aksq->agx=0;
  aksq->agy=0;
  aksq->agz=0;
  siyaun->q0 = 1;
  siyaun->q1 = 0;
  siyaun->q2 = 0;
  siyaun->q3 = 0;
}

void my_quaternion(my_qu*liu, quat_angle*aksq, siyuanshu*siyaun)
{
    // 1. 修复低通滤波器状态丢失的问题 (使用 static)
    float my_kp = liu->kp;
  
  
    static float last_acc_x = 0, last_acc_y = 0, last_acc_z = 0;
    static float last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
    static float exInt = 0,eyInt = 0,ezInt = 0;
    
    float ff_gx = (imu660rb_gyro_x-gyro_offset_x);
    float ff_gy = (imu660rb_gyro_y-gyro_offset_y);
    float ff_gz = -(imu660rb_gyro_z-gyro_offset_z);
    
    if(fabsf(ff_gx) < 3.0f) ff_gx = 0.0f;
    if(fabsf(ff_gy) < 3.0f) ff_gy = 0.0f;
    if(fabsf(ff_gz) < 3.0f) ff_gz = 0.0f;
    
    float kk = 0.05f;
    float kk1 = 0.1f; // 陀螺仪如果本身噪声不大，其实不需要太强的低通滤波
    
    float raw_ax = -(imu660rb_acc_x/ACC_SCALE);
    float raw_ay = -(imu660rb_acc_y/ACC_SCALE);
    float raw_az = -(imu660rb_acc_z/ACC_SCALE);
    
    //printf("%f\n",raw_ay);
    
    // 加速度低通滤波
    float ax = (kk * last_acc_x + (1.0f - kk) * raw_ax);
    float ay =  kk * last_acc_y + (1.0f - kk) * raw_ay; // 确认正负号是否符合你的坐标系
    float az = (kk * last_acc_z + (1.0f - kk) * raw_az);
    last_acc_x = ax; last_acc_y = ay; last_acc_z = az;
    //printf("%f,%f,%f\n",raw_ax,raw_ay,raw_az);
    
    // 2. 修复致命Bug：陀螺仪低通滤波绝对不能混入加速度ax, ay, az！
    float gx_deg = (kk1 * last_gyro_x + (1.0f - kk1) * ff_gx);
    float gy_deg = (kk1 * last_gyro_y + (1.0f - kk1) * ff_gy);
    float gz_deg = (kk1 * last_gyro_z + (1.0f - kk1) * ff_gz);
    last_gyro_x = gx_deg; last_gyro_y = gy_deg; last_gyro_z = gz_deg;

    // 转换为弧度制
    float gx = (gx_deg/GYRO_SCALE) * 0.01745329f;
    float gy = (gy_deg/GYRO_SCALE) * 0.01745329f;
    float gz = (gz_deg/GYRO_SCALE) * 0.01745329f;

    float ckx, cky, ckz;
    float ex, ey;//ez
    float norm;

    float q0 = siyaun->q0;
    float q1 = siyaun->q1;
    float q2 = siyaun->q2;
    float q3 = siyaun->q3;

       
        norm = sqrtf(ax*ax+ay*ay+az*az);
        //printf("%f\n",fabsf(norm - 1.0f));
     if (fabsf(norm - 1.0f) < 0.2f) 
    {
        ax /= norm; 
        ay /= norm; 
        az /= norm;
        //printf("%f,%f,%f,\n",ax,ay,az);
        
        
        // 提取重力估计向量
        ckx = 2.0f * (q1*q3 - q0*q2);
        cky = 2.0f * (q0*q1 + q2*q3);
        ckz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 叉积求误差
        ex = (ay*ckz - az*cky);
        ey = (az*ckx - ax*ckz);
        //ez = 0-ckz;// 4. 关键：六轴无磁力计，舍弃 Z 轴误差，防止加速度噪声污染 Yaw
        //ez = 0;
        
        // 误差积分 (加入权重抑制剧烈运动时的错误积分)
       
        exInt += ex * liu->ki * liu->qdt ;
        eyInt += ey * liu->ki * liu->qdt ;
        //ezInt += 0.05*ez * liu->ki * liu->qdt ; // 强制清零
        ezInt = 0;
        
        // 积分限幅
        if(exInt > 3.0f) exInt = 3.0f; else if(exInt < -3.0f) exInt = -3.0f;
        if(eyInt > 3.0f) eyInt = 3.0f; else if(eyInt < -3.0f) eyInt = -3.0f;
        if(ezInt > 3.0f) ezInt = 3.0f; else if(ezInt < -3.0f) ezInt = -3.0f;
    }
      else
      {
        // 剧烈晃动瞬间，完全屏蔽加速度计修正，只靠陀螺仪积分
        ex = 0.0f;
        ey = 0.0f;
      }
     
        float gyro_rate = sqrtf(gx*gx + gy*gy + gz*gz);
         if(gyro_rate > 0.8f) { // 约 45度/秒
            my_kp= 0.2f; 
        }
        
        // PI 补偿角速度
        gx = gx + (my_kp * ex) + exInt;
        gy = gy + (my_kp * ey) + eyInt;
        //gz = gz + (liu->kp * ez*0.05)+ezInt; // 纯靠陀螺仪积分
        gz = gz;
        

    // 四元数更新 (龙格库塔法)
    float halfT = liu->qdt * 0.5f;
    float dq0 = (-q1*gx - q2*gy - q3*gz) * halfT;
    float dq1 = ( q0*gx + q2*gz - q3*gy) * halfT;
    float dq2 = ( q0*gy - q1*gz + q3*gx) * halfT;
    float dq3 = ( q0*gz + q1*gy - q2*gx) * halfT;

    siyaun->q0 += dq0;
    siyaun->q1 += dq1;
    siyaun->q2 += dq2;
    siyaun->q3 += dq3;

    // 四元数归一化
    norm = sqrtf(siyaun->q0*siyaun->q0 + siyaun->q1*siyaun->q1 + siyaun->q2*siyaun->q2 + siyaun->q3*siyaun->q3);
    siyaun->q0 /= norm;
    siyaun->q1 /= norm;
    siyaun->q2 /= norm;
    siyaun->q3 /= norm;

    // 计算欧拉角
    q0 = siyaun->q0; q1 = siyaun->q1; q2 = siyaun->q2; q3 = siyaun->q3;
    aksq->agx = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.2957795f;
    aksq->agy = asinf(2.0f*(q0*q2 - q1*q3)) * 57.2957795f;
    aksq->agz = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.2957795f;
}


void imu_calibration(void)                                      
{                                                               
    // 防止单片机启动太快，读到全是无效的0导致校准失效                                       

    int32 sum_gx = 0, sum_gy = 0, sum_gz = 0;                   // 定义累加变量防溢出
    for(int i = 0; i <1000; i++)                                // 开启连续500次采样
    {                                                           
        imu660rb_get_gyro();                                    // 抓取底层陀螺仪数据
        sum_gx += imu660rb_gyro_x;                                  
        sum_gy += imu660rb_gyro_y;                                  
        sum_gz += imu660rb_gyro_z;                                  
        system_delay_ms(1);                                     // 每次延时2毫秒，总共耗时1秒钟
    }                                                           
    
    gyro_offset_x = (float)sum_gx / 1000.0f;                     // 计算真正的静止绝对零偏
    gyro_offset_y = (float)sum_gy / 1000.0f;                     
    gyro_offset_z = (float)sum_gz / 1000.0f;                     
}                                             

void ycl(void)
{
static float acc_alpha = 0.2f;    
cl_angle.ac_x=acc_alpha*cl_angle.ac_x+(1-acc_alpha)*imu660rb_acc_x;
cl_angle.ac_y=acc_alpha*cl_angle.ac_y+(1-acc_alpha)*imu660rb_acc_y;
cl_angle.ac_z=acc_alpha*cl_angle.ac_z+(1-acc_alpha)*imu660rb_acc_z;

//cl_angle.gy_x = imu660rb_gyro_x;
//cl_angle.gy_y = imu660rb_gyro_y;
cl_angle.gy_z = imu660rb_gyro_z;

static float gyro_filtered_z = 0; //gyro_filtered_x = 0, gyro_filtered_y = 0, 
float raw_gyro_z_dps = (imu660rb_gyro_z - gyro_offset_z) / 14.30f;
float FILTER_ALPHA = 0.2f;
if(raw_gyro_z_dps > -4.0f && raw_gyro_z_dps < 4.0f)
    {
        // 频率降回 5ms 后，把追踪阻尼恢复到 0.0005f，稳稳吃掉温漂！
        gyro_offset_z += (imu660rb_acc_z - gyro_offset_z) * 0.0005f;
    }

    // 【强制静区】：角速度小于 4.0 度/秒，直接一刀切判定为 0！
    if(raw_gyro_z_dps > -4.0f && raw_gyro_z_dps < 4.0f)
    {
        cl_angle.ac_z = 0.0f;                                              
        gyro_filtered_z = 0.0f;                                             
    }
    else
    {
        float gyro_z_rad = raw_gyro_z_dps * 0.01745329f;
        gyro_filtered_z = FILTER_ALPHA * gyro_filtered_z + (1.0f - FILTER_ALPHA) * gyro_z_rad;
        cl_angle.gy_z = gyro_filtered_z;
    }
    //gyro_filtered_x = FILTER_ALPHA * gyro_filtered_x + (1.0f - FILTER_ALPHA) * cl_angle.gy_x;   
    //gyro_filtered_y = FILTER_ALPHA * gyro_filtered_y + (1.0f - FILTER_ALPHA) * cl_angle.gy_y;  
    //cl_angle.gy_x=gyro_filtered_x;
    //cl_angle.gy_y=gyro_filtered_y;
}


