/*
 * PlatForm.h
 *
 *  Created on: 2017年7月13日
 *      Author: Administrator
 */

#ifndef INCLUDE_PLATFORM_H_
#define INCLUDE_PLATFORM_H_

typedef struct
    {
     double speed_rate; //速度比例，为运动的实时速度比例，格式为：0.01，表示1%的速度
     double limit_rate; //限制比例，在实时最大速度的基础上在进行限制，格式：0.95，表示最大速度的95%为此时最大速度
}S_Line_Fix_Type;

typedef struct
    {
     float x_pre;//起始点坐标
     float y_pre;
     float z_pre;
     float c_pre;//绕X轴旋转角度
     float x_now;//目标点坐标
     float y_now;
     float z_now;
     float c_now;
}POSE_DECARE_Type;

typedef struct
    {
     double Speed_J1_Arr[10000];      //一轴速度插补数组
     double Offset_J1_Arr[10000];     //一轴位移插补数组
     double Speed_J2_Arr[10000];
     double Offset_J2_Arr[10000];
     double Speed_J3_Arr[10000];
     double Offset_J3_Arr[10000];
     double Speed_J4_Arr[10000];
     double Offset_J4_Arr[10000];
}S_Line_J_Runin;

extern POSE_DECARE_Type Decare_Point;
extern S_Line_Fix_Type S_Line_Fix;
extern S_Line_J_Runin S_Line_Runin;



//extern int SCARA_S_Line(S_Line_Fix_Type *S_Line_Fix,POSE_DECARE_Type* Decare_Point,S_Line_J_Runin *S_Line_Runin);
extern int Delta_S_line(S_Line_Fix_Type *S_Line_Fix,POSE_DECARE_Type* Decare_Point,S_Line_J_Runin *S_Line_Runin);

#endif /* INCLUDE_PLATFORM_H_ */
