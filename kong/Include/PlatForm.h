/*
 * PlatForm.h
 *
 *  Created on: 2017��7��13��
 *      Author: Administrator
 */

#ifndef INCLUDE_PLATFORM_H_
#define INCLUDE_PLATFORM_H_

typedef struct
    {
     double speed_rate; //�ٶȱ�����Ϊ�˶���ʵʱ�ٶȱ�������ʽΪ��0.01����ʾ1%���ٶ�
     double limit_rate; //���Ʊ�������ʵʱ����ٶȵĻ������ڽ������ƣ���ʽ��0.95����ʾ����ٶȵ�95%Ϊ��ʱ����ٶ�
}S_Line_Fix_Type;

typedef struct
    {
     float x_pre;//��ʼ������
     float y_pre;
     float z_pre;
     float c_pre;//��X����ת�Ƕ�
     float x_now;//Ŀ�������
     float y_now;
     float z_now;
     float c_now;
}POSE_DECARE_Type;

typedef struct
    {
     double Speed_J1_Arr[10000];      //һ���ٶȲ岹����
     double Offset_J1_Arr[10000];     //һ��λ�Ʋ岹����
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
