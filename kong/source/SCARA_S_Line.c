/*
 * SCARA_S_Line.c
 *
 *  Created on: 2017��7��13��
 *      Author: Administrator
 */
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "PlatForm.h"
#include "math.h"

int SCARA_S_Line(S_Line_Fix_Type *S_Line_Fix,POSE_DECARE_Type* Decare_Point,S_Line_J_Runin *S_Line_Runin)
{

//  int S_Line_Fix->cnt1 = 0; //������������ı���
    unsigned int cnt1 = 0;
    unsigned int cnt2 = 0;                   //������������ı���
    unsigned int cnt3 = 0;                   //������������ı���
    unsigned int cnt4 = 0;                   //������������ı���
    double t_cnt = 0.01;        //�岹ֵ
    double L = 0.0;             //�λ�Ƴ���
    double Amax = 0.0;          //�λ�Ƴ�����ļ��ٶ�
    double Vs = 0.0;            //�λ�Ƴ��������ʼ�ٶ�
    double Ve = 0.0;            //�λ�Ƴ�����Ľ����ٶ�
    double J = 0.0;             //�λ�Ƴ�����ļӼ��ٶ�
    double Vc = 0.0;            //�λ�Ƴ����������ٶ�

    double tm = 0.0;            //���ٶȺͼӼ��ٶȵı�ֵ
    double T_total =0.0 ;
    //���߶�Ӧ��ʱ��ֵ
    double t1 = 0.0;
    long double t2 = 0.0;
    double t3 = 0.0;
    double t4 = 0.0;
    double t5 = 0.0;
    double t6 = 0.0;
    double t7 = 0.0;
    double T1 = 0.0;
    double T2 = 0.0;
    double T3 = 0.0;
    double T4 = 0.0;
    double T5 = 0.0;
    double T6 = 0.0;
    double T7 = 0.0;
//  double S_Line_Fix->Ttotal = 0.0;        //���ߵ���ʱ��
    float Stotal = 0.0;         //���κ��߶εķֽ�λ��

    double max2 = 0.0;          //���ڴ洢������������λ�Ƶı���
/*
    double S_Line_Fix->Speed_J1_end = 0;                //����ʱ���ٶ�
    double S_Line_Fix->Speed_J2_end = 0;                //����ʱ���ٶ�
    double S_Line_Fix->Speed_J3_end = 0;                //����ʱ���ٶ�
    double S_Line_Fix->Speed_J4_end = 0;                //����ʱ���ٶ�

    double S_Line_Fix->Speed_J1_start = 0;              //��ʼ�ٶ�
    double S_Line_Fix->Speed_J2_start = 0;              //��ʼ�ٶ�
    double S_Line_Fix->Speed_J3_start = 0;              //��ʼ�ٶ�
    double S_Line_Fix->Speed_J4_start = 0;              //��ʼ�ٶ�
*/
    //��������ڸ��ζ�Ӧ���ٶ�
    double V01 = 0.0;
    double V11 = 0.0;
    double V21 = 0.0;
    double V31 = 0.0;
    double V41 = 0.0;
    double V51 = 0.0;
    double V61 = 0.0;

    double V02 = 0.0;
    double V12 = 0.0;
    double V22 = 0.0;
    double V32 = 0.0;
    double V42 = 0.0;
    double V52 = 0.0;
    double V62 = 0.0;

    double V03 = 0.0;
    double V13 = 0.0;
    double V23 = 0.0;
    double V33 = 0.0;
    double V43 = 0.0;
    double V53 = 0.0;
    double V63 = 0.0;

    double V04 = 0.0;
    double V14 = 0.0;
    double V24 = 0.0;
    double V34 = 0.0;
    double V44 = 0.0;
    double V54 = 0.0;
    double V64 = 0.0;

    //��������ڸ��ζ�Ӧ��λ��
    double S11 = 0.0;
    double S21 = 0.0;
    double S31 = 0.0;
    double S41 = 0.0;
    double S51 = 0.0;
    double S61 = 0.0;

    double S12 = 0.0;
    double S22 = 0.0;
    double S32 = 0.0;
    double S42 = 0.0;
    double S52 = 0.0;
    double S62 = 0.0;

    double S13 = 0.0;
    double S23 = 0.0;
    double S33 = 0.0;
    double S43 = 0.0;
    double S53 = 0.0;
    double S63 = 0.0;

    double S14 = 0.0;
    double S24 = 0.0;
    double S34 = 0.0;
    double S44 = 0.0;
    double S54 = 0.0;
    double S64 = 0.0;

    double T=0.0;               //���в岹ʱ��ʱ��ֵ

    //�������ز���
    double J1=0.0;
    double Vc1=0.0;
    double Amax1=0.0;
    double L1=0.0;
    double Vs1 = 0.0;
    double Ve1 = 0.0;

    double J2=0.0;
    double Vc2=0.0;
    double Amax2=0.0;
    double L2=0.0;
    double Vs2 = 0.0;
    double Ve2 = 0.0;

    double J3=0.0;
    double Vc3=0.0;
    double Amax3=0.0;
    double L3=0.0;
    double Vs3 = 0.0;
    double Ve3 = 0.0;

    double J4=0.0;
    double Vc4=0.0;
    double Amax4=0.0;
    double L4=0.0;
    double Vs4 = 0.0;
    double Ve4 = 0.0;

    //һ����ز�������
    J1 = 1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;               //���Ӽ��ٶ�
    Vc1 = 450*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                                                                //����ٶ�       J1
    Amax1 = 2250*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                              //�����ٶ�
    L1 =fabsf(Decare_Point->x_now-Decare_Point->x_pre);                                                                                                                  //�ƶ���λ��
    Vs1=0;
    Ve1=0;
    //������ز�������
    J2 = 1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;               //���Ӽ��ٶ�
    Vc2 = 450*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                                                                //����ٶ�       J2
    Amax2 = 2250*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                              //�����ٶ�
    L2 = fabsf(Decare_Point->y_now-Decare_Point->y_pre);                       //�ƶ���λ��
    Vs1=0;
    Ve1=0;

    //������ز�������
    J3 = 1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;               //���Ӽ��ٶ�
    Vc3 = 2000*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                                                               //����ٶ�       J3
    Amax3 = 10000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                             //�����ٶ�
    L3 =fabsf(Decare_Point->z_now-Decare_Point->z_pre);                                                                                                                  //�ƶ���λ��
    Vs1=0;
    Ve1=0;

    //������ز�������
    J4 = 1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;               //���Ӽ��ٶ�
    Vc4 = 18000*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                                                              //����ٶ�       J4
    Amax4 = 90000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;                             //�����ٶ�
    L4 =fabsf(Decare_Point->c_now-Decare_Point->c_pre);                                                                                                                      //�ƶ���λ��
    Vs1=0;
    Ve1=0;

    //�Ե�һ�͵ڶ��������λ�Ƴ��̽����жϣ��жϳ�����е���
    if (fabsf(L1) >= fabsf(L2))
    {   L = fabsf(L1); }
    else
    {   L = fabsf(L2); }

    //�Ե����͵����������λ�Ƴ��̽����жϣ��жϳ�����е���
    if (fabsf(L3) >= fabsf(L4))
    {   max2 = fabsf(L3); }
    else
    {   max2 = fabsf(L4); }

    //�жϳ���˶��켣����֮�󣬽�������������
    if (L== fabsf(L1))
    {
        Vc= Vc1;
        Amax = Amax1;
        Vs = Vs1;
        Ve = Ve1;
        J = J1;
    }
    else if (L == fabsf(L2))
    {
        Vc = Vc2;
        Amax = Amax2;
        Vs = Vs2;
        Ve = Ve2;
        J = J2;
    }

    tm= Amax/J;                         //���ݼ��ٶȺͼӼ��ٶ�ֵ�������ʱ��ֵ

//  t1,t2,t3,t4,t5,t6,t7;               //����S�����ߵ��߶ηֱ��Ӧ��ʱ��ֵ
//  Stotal;                             //����S������������ȫ�����ڵ�λ��
//  Stotal_acc;                         //���ٶȶ��ܴﵽ�����ٶȵ�λ��
//  Sacc;                               //�����ȼ��ٶ��µ�λ��
//  Stotal_dec;                         //���ٶȶ��ܴﵽ�����ٶȵ�λ��
//  Sdec;                               //�����ȼ��ٶ��µ�λ��
//  Stotal_no_acc_dec;                  //���ٶȶκͼ��ٶȶζ����ܴﵽ�����ٶ�
//  Sdec1;                              //Vs<Veʱ�����ٶ�λ��
//  Sacc1;                              //Vs>Veʱ�����ٶ�λ��
//  Acc;                                //������ʹ�õļ��ٶȱ���
//  Stotaabs(L1);                       //�ڲ�����L>Stotalʱ���жϵ��ٽ�����

//  V0,V1,V2,V3,V4,V5,V6,V7,V;          //S�������и��ζ�Ӧ���ٶȺͼ�����ʹ�õ��ٶȱ���
//  S1,S2,S3,S4,S5,S6,S7,S;             //S�������и��ζ�Ӧ��λ�ƺͼ�����ʹ�õ�λ�Ʊ���
//

//  T ,Ttotal;                          //ʱ������������������Ҫ����ʱ��
//  T1,T2,T3,T4,T5,T6,T7;               //ʱ��������ں�������ж�

    //S�����ߵ��������������������������������
    if (Vs>=0 && Vs<Vc && Ve >= 0 && Ve < Vc)
    {
        //���һ�����ٶȺͼ��ٶ������ٶȶ��ܴﵽ
        if (Vs<Vc-Amax*tm && Ve< Vc-Amax*tm )
        {
            //��һ���λ�ƽ��в�ͬ����Ļ��֣�L�Ļ����Ǹ��ݴ����ݲ����ҵ����ֵ�
            if (L==fabsf(L1))
            {
                if (L1>0)
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))                                                                 //127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;                                                                                                                                                                                     //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/J;
                        t2=Vc/Amax-Amax/J;                                                                                                                                                                                                                                                          //����Vc=Amax*t2+Amax*t1����õ�
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J))) //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                                                                                                                                                                                             //(0.06008/L)^(1/2);     //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                        t1=Amax/J;
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                    }
                    else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);                                   //����L��t1�����߹�ϵ�������
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                    }
                }
                else
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/fabsf(J);
                        t2=Vc/Amax-Amax/fabsf(J);
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=-pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                        t1=Amax/fabsf(J);
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                    }
                    else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = -J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                    }
                }
                J1=J;
            }
            //�Զ����λ�ƽ��в�ͬ����Ļ���
            else if (L==fabsf(L2))
            {
                if (L2>0)
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;
                        t1=Amax/J;
                        t2=Vc/Amax-Amax/J;
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                             //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                        t1=Amax/J;//0.872*L^(1/2)
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                    }
                    else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                    }
                }
                else
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/fabsf(J);
                        t2=Vc/Amax-Amax/fabsf(J);
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=-pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                 //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                        t1=Amax/fabsf(J);//0.872*L^(1/2)
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                    }
                    else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = -J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                    }
                }
                J2=J;
            }
            //�������λ�ƽ��в�ͬ����Ļ���
/*          else if (L==fabsf(L3))
            {
                if (L3>0)
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;         //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/J;
                        t2=Vc/Amax-Amax/J;
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=pow((5.27477*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                 //J=((5.27477*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                        t1=Amax/J;//0.872*L^(1/2)
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                    }
                    else if (L<((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                    }
                }
                else
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;        //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/fabsf(J);
                        t2=Vc/Amax-Amax/fabsf(J);
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=-pow((5.27477*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                 //J=((5.27477*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                        t1=Amax/fabsf(J);//0.872*L^(1/2)
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                    }
                    else if (L<((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = -J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                    }
                }
                J3=J;
            }
            //�������λ�ƽ��в�ͬ����Ļ���
            else if (L==fabsf(L4))
            {
                if (L4>0)
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/J;
                        t2=Vc/Amax-Amax/J;
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=pow((650*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);
                        t1=Amax/J;//0.872*L^(1/2)
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                    }
                    else if (L<((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                    }
                }
                else
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                        t1=Amax/fabsf(J);
                        t2=Vc/Amax-Amax/fabsf(J);
                    }
                    else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                    {
                        J=-pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);
                        t1=Amax/fabsf(J);//0.872*L^(1/2)
                        t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                    }
                    else if (L<((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                    {
                        J = -J;
                        t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                        //S=L-S7���t2
                        t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                    }
                }
                J4=J;
            }
*/

            Stotal = ((Vc*Vc)/Amax)-(Vs*Vs+Ve*Ve)/(2*Amax)+(t1*(2*Vc+Vs+Ve))/2;         //�������������µ�λ��

            //����Ҫ���е�λ�ƺ�Stotal���бȽϣ��ж��Ƿ�������ٶ�
            if (L>=Stotal)
            {
                t4 =(L-Stotal)/Vc;                                                      //��������ٶ�ʱ��
                t3 = t1;
                t5 = t1;
                t6 = t2;
                t7 = t1;
            }
            //�ڲ�������������ʱ�����趨λ��С��Stotalʱ�����Ƿ�����ȼ��ٺ��ȼ���Ϊ�ж����ݽ����ж�
            else
            {
                t4 = 0;
                t3 = t1;
                t5 = t1;
                t6 = t2;
                t7 = t1;
            }
            T1 = t1;
            T2 = t1+t2;
            T3 = t1+t2+t3;
            T4 = t1+t2+t3+t4;
            T5 = t1+t2+t3+t4+t5;
            T6 = t1+t2+t3+t4+t5+t6;
            T7 = t1+t2+t3+t4+t5+t6+t7;
            T_total = t1+t2+t3+t4+t5+t6+t7;

            if (L==fabsf(L1))
            {
                if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                {
                    //�����߶ε�������мӼ��ٶȵ����
                    J2 = -(L2 - Vs2*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J3 = -(L3 - Vs3*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J4 = -(L4 - Vs4*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
                else
                {
                    //�������ε�������мӼ��ٶȵ����
                    J2 = -(L2 - Vs2*(T7) - Vs2*(T3) + Vs2*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J3 = -(L3 - Vs3*(T7) - Vs3*(T3) + Vs3*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J4 = -(L4 - Vs4*(T7) - Vs4*(T3) + Vs4*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
            }
            else if (L==fabsf(L2))
            {
                if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                {
                    J1 = -(L1 - Vs1*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J3 = -(L3 - Vs3*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J4 = -(L4 - Vs4*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
                else
                {
                    J1 = -(L1 - Vs1*(T7) - Vs1*(T3) + Vs1*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J3 = -(L3 - Vs3*(T7) - Vs3*(T3) + Vs3*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J4 = -(L4 - Vs4*(T7) - Vs4*(T3) + Vs4*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
            }

/*          else if (L==fabsf(L3))
            {
                if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                {
                    J1 = -(L1 - Vs1*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J2 = -(L2 - Vs2*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J4 = -(L4 - Vs4*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
                else
                {
                    J1 = -(L1 - Vs1*(T7) - Vs1*(T3) + Vs1*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J2 = -(L2 - Vs2*(T7) - Vs2*(T3) + Vs2*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J4 = -(L4 - Vs4*(T7) - Vs4*(T3) + Vs4*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
            }
            else if (L==fabsf(L4))
            {
                if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                {
                    J1 = -(L1 - Vs1*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J2 = -(L2 - Vs2*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J3 = -(L3 - Vs3*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
                else
                {
                    J1 = -(L1 - Vs1*(T7) - Vs1*(T3) + Vs1*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J2 = -(L2 - Vs2*(T7) - Vs2*(T3) + Vs2*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    J3 = -(L3 - Vs3*(T7) - Vs3*(T3) + Vs3*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                }
            }
*/
            //���������������£����������ļӼ��ٶȳ���������ֵ�����ʱ���ֵ���¼���
            if (J3>1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate || J3<-1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate|| J4>1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate ||  J4<-1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate)
            {
                J3 = 1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;
                J4 = 1000000*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->limit_rate;
                L = max2;
                if (L==fabsf(L3))
                {
                    Vc=Vc3;
                    Amax = Amax3;
                    Vs = Vs3;
                    Ve = Ve3;
                    J = J3;
                }
                else if (L==fabsf(L4))
                {
                    Vc=Vc4;
                    Amax = Amax4;
                    Vs = Vs4;
                    Ve = Ve4;
                    J = J4;
                }
                //��һ���λ�ƽ��в�ͬ����Ļ���
/*              if (L==fabsf(L1))
                {
                    if (L1>0)
                    {
                        if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                        {
                            J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                            t1=Amax/J;
                            t2=Vc/Amax-Amax/J;
                        }
                        else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                        {
                            J=pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                              //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                            t1=Amax/J;
                            t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                        }
                        else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                        {
                            J = J;
                            t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                            //S=L-S7���t2
                            t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                        }
                    }
                    else
                    {
                        if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                        {
                            J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                            t1=Amax/fabsf(J);
                            t2=Vc/Amax-Amax/fabsf(J);
                        }
                        else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                        {
                            J=-pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                               //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                            t1=Amax/fabsf(J);
                            t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                        }
                        else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                        {
                            J = -J;
                            t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                            //S=L-S7���t2
                            t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                        }
                    }
                    J1=J;
                }
                //�Զ����λ�ƽ��в�ͬ����Ļ���
                else if (L==fabsf(L2))
                {
                    if (L2>0)
                    {
                        if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                        {
                            J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                            t1=Amax/J;
                            t2=Vc/Amax-Amax/J;
                        }
                        else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                        {
                            J=pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                             //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                            t1=Amax/J;
                            t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                        }
                        else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                        {
                            J = J;
                            t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                            //S=L-S7���t2
                            t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                        }
                    }
                    else
                    {
                        if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                        {
                            J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                            t1=Amax/fabsf(J);
                            t2=Vc/Amax-Amax/fabsf(J);
                        }
                        else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                        {
                            J=-pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                            t1=Amax/fabsf(J);
                            t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                        }
                        else if (L<((0.0601*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                        {
                            J = -J;
                            t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                            //S=L-S7���t2
                            t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                        }
                    }
                    J2=J;
                }
                //�������λ�ƽ��в�ͬ����Ļ���
                else*/ if (L==fabsf(L3))
                {
                    if (L3>0)
                    {
                        if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                        {
                            J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                            t1=Amax/J;
                            t2=Vc/Amax-Amax/J;
                        }
                        else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                        {
                            J=pow((5.27477*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                             //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                            t1=Amax/J;
                            t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                        }
                        else if (L<((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                        {
                            J = J;
                            t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                            //S=L-S7���t2
                            t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                        }
                    }
                    else
                        {
                            if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                            {
                                J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                                t1=Amax/fabsf(J);
                                t2=Vc/Amax-Amax/fabsf(J);
                            }
                            else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                                {
                                    J=-pow((5.27477*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                            //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                                    t1=Amax/fabsf(J);
                                    t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                                }
                            else if (L<((5.27477*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                            {
                                    J = -J;
                                    t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                                    //S=L-S7���t2
                                    t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                            }
                        }
                        J3=J;
                    }
                    //�������λ�ƽ��в�ͬ����Ļ���
                    else if (L==fabsf(L4))
                    {
                        if (L4>0)
                        {
                            if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                            {
                                J=21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                                t1=Amax/J;
                                t2=Vc/Amax-Amax/J;
                            }
                            else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                            {
                                J=pow((650*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                 //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                                t1=Amax/J;
                                t2=(pow((Amax*(Amax*Amax*Amax + 4*L*J*J)),0.5) - 3*Amax*Amax)/(2*Amax*J);
                            }
                            else if (L<((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                            {
                                J = J;
                                t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                                //S=L-S7���t2
                                t2=(long double)(-((((long double )92440803)*J*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*J*((long double )171046299151*J*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*J*S_Line_Fix->speed_rate));
                            }
                        }
                        else
                        {
                            if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                            {
                                J=-21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate;       //J=0.0216986/0.01/0.01*�ٷֱ�*�ٷֱ�     0.0216986;
                                t1=Amax/fabsf(J);
                                t2=Vc/Amax-Amax/fabsf(J);
                            }
                            else if (L<(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)) && L>((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))   //L<127.61 && L>=0.066  L=(0.06008*���ٷֱ�*100�������η�)/J*J
                            {
                                J=-pow((0.06008*pow((S_Line_Fix->speed_rate*100),6)/L),0.5);                                //J=((0.06008*���ٷֱ�*100�������η�)/L)^(1/2)    0.0601
                                t1=Amax/fabsf(J);
                                t2=(pow((Amax*(Amax*Amax*Amax + 4*L*fabsf(J)*fabsf(J))),0.5) - 3*Amax*Amax)/(2*Amax*fabsf(J));
                            }
                            else if (L<((650*pow((S_Line_Fix->speed_rate*100),6))/(J*J)))
                            {
                                J = -J;
                                t1 = (0.5551*pow(L,0.3333))/(S_Line_Fix->speed_rate*100);
                                //S=L-S7���t2
                                t2=(long double)(-((((long double )92440803)*(-J)*pow(L,(3333.0/10000)))- ((long double)100000*(long double)10000000)*(sqrt(((long double )5551*(-J)*((long double )171046299151*(-J)*pow(L,(3333.0/5000))+(long double )4000000000000000000*pow(L,(6667.0/10000))*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate))/((long double )1000000000000*(long double)1000000000000))))/((long double )11102000000*(-J)*S_Line_Fix->speed_rate));
                            }
                        }
                        J4=J;
                    }


                Stotal = ((Vc*Vc)/Amax)-(Vs*Vs+Ve*Ve)/(2*Amax)+(t1*(2*Vc+Vs+Ve))/2;         //�������������µ�λ��

                //����Ҫ���е�λ�ƺ�Stotal���бȽϣ��ж��Ƿ�������ٶ�
                if (L>=Stotal)
                {
                    t4 =(L-Stotal)/Vc;                                                      //��������ٶ�ʱ��
                    t3 = t1;
                    t5 = t1;
                    t6 = t2;
                    t7 = t1;
                }
                //�ڲ�������������ʱ�����趨λ��С��Stotalʱ�����Ƿ�����ȼ��ٺ��ȼ���Ϊ�ж����ݽ����ж�
                else
                {
                    t4 = 0;
                    t3 = t1;
                    t5 = t1;
                    t6 = t2;
                    t7 = t1;
                }

                T1 = t1;
                T2 = t1+t2;
                T3 = t1+t2+t3;
                T4 = t1+t2+t3+t4;
                T5 = t1+t2+t3+t4+t5;
                T6 = t1+t2+t3+t4+t5+t6;
                T7 = t1+t2+t3+t4+t5+t6+t7;
                T_total = t1+t2+t3+t4+t5+t6+t7;
/*
                if (L==fabsf(L1))
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J2 = -(L2 - Vs2*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J3 = -(L3 - Vs3*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J4 = -(L4 - Vs4*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                    else
                    {
                        J2 = -(L2 - Vs2*(T7) - Vs2*(T3) + Vs2*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J3 = -(L3 - Vs3*(T7) - Vs3*(T3) + Vs3*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J4 = -(L4 - Vs4*(T7) - Vs4*(T3) + Vs4*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                }
                else if (L==fabsf(L2))
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J1 = -(L1 - Vs1*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J3 = -(L3 - Vs3*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J4 = -(L4 - Vs4*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                    else
                    {
                        J1 = -(L1 - Vs1*(T7) - Vs1*(T3) + Vs1*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J3 = -(L3 - Vs3*(T7) - Vs3*(T3) + Vs3*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J4 = -(L4 - Vs4*(T7) - Vs4*(T3) + Vs4*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                }
                else*/ if (L==fabsf(L3))
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J1 = -(L1 - Vs1*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J2 = -(L2 - Vs2*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J4 = -(L4 - Vs4*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                else
                    {
                        J1 = -(L1 - Vs1*(T7) - Vs1*(T3) + Vs1*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J2 = -(L2 - Vs2*(T7) - Vs2*(T3) + Vs2*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J4 = -(L4 - Vs4*(T7) - Vs4*(T3) + Vs4*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                }
                else if (L==fabsf(L4))
                {
                    if (L>=(Amax*Amax*Vc + (21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)*Vc*Vc)/(Amax*(21698.6*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate*S_Line_Fix->speed_rate)))//127.61    L=(Amax^2*Vc + J*Vc^2)/(Amax*J) ʹ�����ε�λ�ƽ��м���
                    {
                        J1 = -(L1 - Vs1*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J2 = -(L2 - Vs2*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J3 = -(L3 - Vs3*(T7))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 - (T3)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 - T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                    else
                    {
                        J1 = -(L1 - Vs1*(T7) - Vs1*(T3) + Vs1*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J2 = -(L2 - Vs2*(T7) - Vs2*(T3) + Vs2*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                        J3 = -(L3 - Vs3*(T7) - Vs3*(T3) + Vs3*(T4))/((T5*T5*T5)/6 + ((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2)*(T5) + (T3*T3*T3)/6 + T1*(T6*T6) + (T2)*(T1*(T2) - T1*T1/2) - (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) - ((T6*T6)*(T7))/2 + ((T6)*(T7*T7))/2 + (T6*T6*T6)/6 -T1*(T2*T2) + (T1*T1*(T2))/2 - (T5)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - (T4*T4*T4)/6 + ((T4*T4)*(T5))/2 - ((T4)*(T5*T5))/2 - (T4)*((T3*T3)/2 - (T2)*(T3) - T1*(T3) + (T2*T2)/2 + T1*T1/2) - (T2*T2*T2)/6 + (T1*(T7*T7))/2 - ((T2)*(T3*T3))/2 + ((T2*T2)*(T3))/2 - (T1*(T2) - T1*T1/2)*(T3) - T1*T1*T1/6 + (T7)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T5) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2 + T1*(T6)) + (T1*(T5*T5))/2 - (T7*T7*T7)/6 - (T1*(T3*T3))/2 + (T6)*((T5*T5)/2 + (T3*T3)/2 - (T4)*(T5) - (T2)*(T3) - T1*(T3) + (T4*T4)/2 + (T2*T2)/2 + T1*T1/2) - T1*(T6)*(T7) - T1*(T5)*(T6) + T1*(T2)*(T3));
                    }
                }
            }
        }
    }

/*
    //����������ٶ��ܴﵽ�����ٶ�
        elseif Vs<Vc-Amax*tm && Ve>=Vc-Amax*tm

            t2 = (Vc-Vs)/Amax-tm;
            t5 = ((Vc-Vs)/J)^(1/2);
            t7 = ((Vc-Vs)/J)^(1/2);
            t1 = tm;
            t3 = tm;
            t6 = 0;
            t4 = 0;
            Stotal_acc = 0.5*(Vs+Ve)*(tm+(Vc-Vs)/Amax)+2*Vc*t5-J*t5*t5*t5;
            if L >= Stotal_acc

                t4 = (L-Stotal_acc)/Vc;

            elseif L < Stotal_acc && Vs < Ve

                t1 = tm;
                t3 = tm;
                t5 = 0;
                t6 = 0;
                t7 = 0;
                t2 =(Ve-Vs)/Amax-tm;
                Sacc = Ve*Vs - Vs*Vs +Ve*tm +0.5*J*tm*tm*tm;
                if(L >= Sacc)

                    t4 = (L-Sacc)/Vc;

                else

                    t4 = 0;
                end
            end

    //����������ٶ��ܴﵽ�����ٶ�
        elseif Vs >= Vc - Amax*tm && Ve < Vc - Amax*tm

            t5 = tm;
            t7 = tm;
            t6 = (Vc-Ve)/Amax-tm;
            t1 = ((Vc-Vs)/J)^(1/2);
            t3 = ((Vc-Vs)/J)^(1/2);
            t2 = 0;
            t4 = 0;
            Stotal_dec = 2*Vs*t1 + J*t1*t1*t1 + 0.5*(Vc + Ve)*(tm + (Vc-Ve)/Amax);
            if L >= Stotal_dec

                t4= (L-Stotal_dec)/Vc;

            elseif L < Stotal_dec && Vs > Ve

                t1 = 0;
                t2 = 0;
                t3 = 0;
                t5 = tm;
                t7 = tm;
                t6 = (Vs-Ve)/Amax-tm;
                Sdec = 0.5*(Vs + Ve)*(tm+(Vs-Ve)/Amax);
                if L >= Sdec

                    t4 = (L-Sdec)/Vc;

                else

                    t4 = 0;
                end
            end

    //����ģ����ٶκͼ��ٶ������ٶȾ����ܴﵽ
        elseif Vs >= Vc - Amax*tm && Ve >= Vc-Amax*tm

            t1 = ((Vc - Vs) / J)^(1/2);
            t3 = ((Vc - Vs) / J)^(1/2);
            t5 = ((Vc - Ve) / J)^(1/2);
            t7 = ((Vc - Ve) / J)^(1/2);
            t2 = 0;
            t4 = 0;
            t6 = 0;
            Stotal_no_acc_dec = 2*Vs*t1 + J*t1*t1*t1 + 2*Vc*t5 - J*t5*t5*t5;
            if L >= Stotal_no_acc_dec

                t4 = (L - Stotal_no_acc_dec )/Vc;

            elseif L < Stotal_no_acc_dec && Vs < Ve

                t1 = 0;
                t2 = 0;
                t3 = 0;
                t4 = 0;
                t6 = 0;
                t5 = ((Vs-Ve)/J)^(1/2);
                t7 = ((Vs-Ve)/J)^(1/2);
                Sdec1 = 2*Vs*t5 -J*t5*t5*t5;
                if L >= Sdec1

                    t4 = (L - Sdec1)/Vs;

                else

                    t4 = 0;
                end

            elseif L < Stotal_no_acc_dec && Vs > Ve

                t5 = 0;
                t6 = 0;
                t7 = 0;
                t1 = ((Ve - Vs)/J)^(1/2);
                t3 = ((Ve - Vs)/J)^(1/2);
                t2 = 0;
                //t4 = 0;
                Sacc1 = Ve * t1 + Vs * t1;
                if L >= Sacc1
                    t4 = (L-Sacc1)/Ve;
                else
                    t4 = 0;
                end
           end
        end
*/

    //��L>Stotalʱ���������´���
    //�Լ��ٶȽ��м���
    Amax1 = J1 *T1;

    //������ζ�Ӧ���ٶ�ֵ
    V01 = Vs1;
    V11 = 0.5*J1*T1*T1 +V01;
    V21 = Amax1*T2+V11-Amax1*T1;
    V31 = Amax1*T3-(0.5*J1*T3*T3)+J1*T2*T3+V21-Amax1*T2-(0.5*J1*T2*T2);
    V41 = V31;
    V51 = (-0.5)*J1*T5*T5+J1*T4*T5+V41-(0.5)*J1*T4*T4;
    V61 = -Amax1*T6+V51+Amax1*T5;

    //������ζ�Ӧ��λ��ֵ
    S11 = (1.0/6)*J1*T1*T1*T1+Vs1*T1;
    S21 = 0.5*Amax1*T2*T2+V11*T2-Amax1*T1*T2+S11-V11*T1+0.5*Amax1*T1*T1;
    S31 = 0.5*Amax1*T3*T3-(1.0/6)*J1*T3*T3*T3+0.5*J1*T2*T3*T3+V21*T3-Amax1*T2*T3-0.5*J1*T2*T2*T3+S21-V21*T2+(1.0/6)*J1*T2*T2*T2+0.5*Amax1*T2*T2;
    S41 = V31*T4+S31-V31*T3;
    S51 = -(1.0/6)*J1*T5*T5*T5+0.5*J1*T4*T5*T5+V41*T5-0.5*J1*T4*T4*T5+S41-V41*T4+(1.0/6)*J1*T4*T4*T4;
    S61 = -0.5*Amax1*T6*T6+V51*T6+Amax1*T5*T6+S51-V51*T5-0.5*Amax1*T5*T5;

    //���ٶȡ����ٶȺ�λ�ƽ��в岹����
    for (cnt1 = 0; cnt1<=10000; cnt1++)
    {
        T = cnt1*t_cnt;
        if(T<=T_total)
        {
            if(T>=0 && T<T1)
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = J1*T;
                S_Line_Runin->Speed_J1_Arr[cnt1] = 0.5*J1*T*T+Vs1;
                S_Line_Runin->Offset_J1_Arr[cnt1] = ((1.0/6)*J1*T*T*T+Vs1*T);
            }
            else if (T>T1 && T<=T2)
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = Amax1;
                S_Line_Runin->Speed_J1_Arr[cnt1] = Amax1*T-Amax1*T1+V11;
                S_Line_Runin->Offset_J1_Arr[cnt1] = (0.5*Amax1*T*T+V11*T-Amax1*T1*T+S11-V11*T1+0.5*Amax1*T1*T1);
            }
            else if (T>=T2 && T<T3)
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = Amax1-J1*(T-T2);
                S_Line_Runin->Speed_J1_Arr[cnt1] = Amax1*T-(0.5*J1*T*T)+J1*T2*T+V21-Amax1*T2-0.5*J1*T2*T2;
                S_Line_Runin->Offset_J1_Arr[cnt1] = (0.5*Amax1*T*T-(1.0/6)*J1*T*T*T+0.5*J1*T2*T*T+V21*T-Amax1*T*T2-0.5*J1*T2*T2*T+S21-V21*T2+(1.0/6)*J1*T2*T2*T2+0.5*Amax1*T2*T2);
            }
            else if (T>(T3) && T<=(T4))
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = 0;
                S_Line_Runin->Speed_J1_Arr[cnt1] = V31;
                S_Line_Runin->Offset_J1_Arr[cnt1] = (V31*T+S31-V31*T3);
            }
                else if (T>=T4 && T<T5)
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = -J1*(T-T4);
                S_Line_Runin->Speed_J1_Arr[cnt1] = -0.5*J1*T*T+J1*T4*T+V41-0.5*J1*T4*T4;
                S_Line_Runin->Offset_J1_Arr[cnt1] = (-(1.0/6)*J1*T*T*T+0.5*J1*T4*T*T+V41*T-0.5*J1*T4*T4*T+S41-V41*T4+(1.0/6)*J1*T4*T4*T4);
            }
            else if (T>(T5) && T<=(T6))
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = -Amax1;
                S_Line_Runin->Speed_J1_Arr[cnt1] = -Amax1*T+Amax1*T5+V51;
                S_Line_Runin->Offset_J1_Arr[cnt1] = (-0.5*Amax1*T*T+V51*T+Amax1*T*T5+S51-V51*T5-0.5*Amax1*T5*T5);
            }
            else if (T>=T6 && T<T7)
            {
//              S_Line_Runin->Acc_J1_Arr[cnt1] = -Amax1+J1*(T-T6);
                S_Line_Runin->Speed_J1_Arr[cnt1] = -Amax1*T+Amax1*T6+(0.5*J1*T*T)-J1*T6*T+V61+0.5*J1*T6*T6;
                S_Line_Runin->Offset_J1_Arr[cnt1] = (-0.5*Amax1*T*T+(1.0/6)*J1*T*T*T-0.5*J1*T*T*T6+V61*T+Amax1*T*T6+0.5*J1*T6*T6*T+S61-V61*T6-0.5*Amax1*T6*T6-(1.0/6)*J1*T6*T6*T6);
            }
        }
        else
            break;
    }

    //����Ĳ岹����
    Amax2 = J2 *T1;

    V02 = Vs2;
    V12 = 0.5*J2*T1*T1+V02;
    V22 = Amax2*T2+V12-Amax2*T1;
    V32 = Amax2*T3-(0.5*J2*T3*T3)+J2*T2*T3+V22-Amax2*T2-(0.5*J2*T2*T2);
    V42 = V32;
    V52 = (-0.5)*J2*T5*T5+J2*T4*T5+V42-(0.5)*J2*T4*T4;
    V62 = -Amax2*T6+V52+Amax2*T5;

    S12 = (1.0/6)*J2*T1*T1*T1+Vs2*T1;
    S22 = 0.5*Amax2*T2*T2+V12*T2-Amax2*T1*T2+S12-V12*T1+0.5*Amax2*T1*T1;
    S32 = 0.5*Amax2*T3*T3-(1.0/6)*J2*T3*T3*T3+0.5*J2*T2*T3*T3+V22*T3-Amax2*T2*T3-0.5*J2*T2*T2*T3+S22-V22*T2+(1.0/6)*J2*T2*T2*T2+0.5*Amax2*T2*T2;
    S42 = V32*T4+S32-V32*T3;
    S52 = -(1.0/6)*J2*T5*T5*T5+0.5*J2*T4*T5*T5+V42*T5-0.5*J2*T4*T4*T5+S42-V42*T4+(1.0/6)*J2*T4*T4*T4;
    S62 = -0.5*Amax2*T6*T6+V52*T6+Amax2*T5*T6+S52-V52*T5-0.5*Amax2*T5*T5;

    for (cnt2 = 0; cnt2<=10000; cnt2++)
        {
            T = cnt2*t_cnt;
            if(T<=T_total)
            {
                if(T>=0 && T<T1)
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = J2*T;
                    S_Line_Runin->Speed_J2_Arr[cnt2] = 0.5*J2*T*T+Vs2;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = ((1.0/6)*J2*T*T*T+Vs2*T);
                }
                else if (T>T1 && T<=T2)
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = Amax2;
                    S_Line_Runin->Speed_J2_Arr[cnt2] = Amax2*T-Amax2*T1+V12;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = (0.5*Amax2*T*T+V12*T-Amax2*T1*T+S12-V12*T1+0.5*Amax2*T1*T1);
                }
                else if (T>=T2 && T<T3)
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = Amax2-J2*(T-T2);
                    S_Line_Runin->Speed_J2_Arr[cnt2] = Amax2*T-(0.5*J2*T*T)+J2*T2*T+V22-Amax2*T2-0.5*J2*T2*T2;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = (0.5*Amax2*T*T-(1.0/6)*J2*T*T*T+0.5*J2*T2*T*T+V22*T-Amax2*T*T2-0.5*J2*T2*T2*T+S22-V22*T2+(1.0/6)*J2*T2*T2*T2+0.5*Amax2*T2*T2);
                }
                else if (T>(T3) && T<=(T4))
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = 0;
                    S_Line_Runin->Speed_J2_Arr[cnt2] = V32;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = (V32*T+S32-V32*T3);
                }
                    else if (T>=T4 && T<T5)
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = -J2*(T-T4);
                    S_Line_Runin->Speed_J2_Arr[cnt2] = -0.5*J2*T*T+J2*T4*T+V42-0.5*J2*T4*T4;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = (-(1.0/6)*J2*T*T*T+0.5*J2*T4*T*T+V42*T-0.5*J2*T4*T4*T+S42-V42*T4+(1.0/6)*J2*T4*T4*T4);
                }
                else if (T>(T5) && T<=(T6))
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = -Amax2;
                    S_Line_Runin->Speed_J2_Arr[cnt2] = -Amax2*T+Amax2*T5+V52;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = (-0.5*Amax2*T*T+V52*T+Amax2*T*T5+S52-V52*T5-0.5*Amax2*T5*T5);
                }
                else if (T>=T6 && T<T7)
                {
//                  S_Line_Runin->Acc_J2_Arr[cnt2] = -Amax2+J2*(T-T6);
                    S_Line_Runin->Speed_J2_Arr[cnt2] = -Amax2*T+Amax2*T6+(0.5*J2*T*T)-J2*T6*T+V62+0.5*J2*T6*T6;
                    S_Line_Runin->Offset_J2_Arr[cnt2] = (-0.5*Amax2*T*T+(1.0/6)*J2*T*T*T-0.5*J2*T*T*T6+V62*T+Amax2*T*T6+0.5*J2*T6*T6*T+S62-V62*T6-0.5*Amax2*T6*T6-(1.0/6)*J2*T6*T6*T6);
                }
            }
            else
                break;
       }

    //����岹�ļ���
    Amax3 = J3 *T1;

    V03 = Vs3;
    V13 = 0.5*J3*T1*T1 +V03;
    V23 = Amax3*T2+V13-Amax3*T1;
    V33 = Amax3*T3-(0.5*J3*T3*T3)+J3*T2*T3+V23-Amax3*T2-(0.5*J3*T2*T2);
    V43 = V33;
    V53 = (-0.5)*J3*T5*T5+J3*T4*T5+V43-(0.5)*J3*T4*T4;
    V63 = -Amax3*T6+V53+Amax3*T5;

    S13 = (1.0/6)*J3*T1*T1*T1+Vs3*T1;
    S23 = 0.5*Amax3*T2*T2+V13*T2-Amax3*T1*T2+S13-V13*T1+0.5*Amax3*T1*T1;
    S33 = 0.5*Amax3*T3*T3-(1.0/6)*J3*T3*T3*T3+0.5*J3*T2*T3*T3+V23*T3-Amax3*T2*T3-0.5*J3*T2*T2*T3+S23-V23*T2+(1.0/6)*J3*T2*T2*T2+0.5*Amax3*T2*T2;
    S43 = V33*T4+S33-V33*T3;
    S53 = -(1.0/6)*J3*T5*T5*T5+0.5*J3*T4*T5*T5+V43*T5-0.5*J3*T4*T4*T5+S43-V43*T4+(1.0/6)*J3*T4*T4*T4;
    S63 = -0.5*Amax3*T6*T6+V53*T6+Amax3*T5*T6+S53-V53*T5-0.5*Amax3*T5*T5;

    for (cnt3 = 0; cnt3<=10000; cnt3++)
    {
        T = cnt3*t_cnt;
        if(T<=T_total)
        {
            if(T>=0 && T<T1)
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = J3*T;
                S_Line_Runin->Speed_J3_Arr[cnt3] = 0.5*J3*T*T+Vs3;
                S_Line_Runin->Offset_J3_Arr[cnt3] = ((1.0/6)*J3*T*T*T+Vs3*T);
            }
            else if (T>T1 && T<=T2)
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = Amax3;
                S_Line_Runin->Speed_J3_Arr[cnt3] = Amax3*T-Amax3*T1+V13;
                S_Line_Runin->Offset_J3_Arr[cnt3] = (0.5*Amax3*T*T+V13*T-Amax3*T1*T+S13-V13*T1+0.5*Amax3*T1*T1);
            }
            else if (T>=T2 && T<T3)
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = Amax3-J3*(T-T2);
                S_Line_Runin->Speed_J3_Arr[cnt3] = Amax3*T-(0.5*J3*T*T)+J3*T2*T+V23-Amax3*T2-0.5*J3*T2*T2;
                S_Line_Runin->Offset_J3_Arr[cnt3] = (0.5*Amax3*T*T-(1.0/6)*J3*T*T*T+0.5*J3*T2*T*T+V23*T-Amax3*T*T2-0.5*J3*T2*T2*T+S23-V23*T2+(1.0/6)*J3*T2*T2*T2+0.5*Amax3*T2*T2);
            }
            else if (T>(T3) && T<=(T4))
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = 0;
                S_Line_Runin->Speed_J3_Arr[cnt3] = V33;
                S_Line_Runin->Offset_J3_Arr[cnt3] = (V33*T+S33-V33*T3);
            }
            else if (T>=T4 && T<T5)
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = -J3*(T-T4);
                S_Line_Runin->Speed_J3_Arr[cnt3] = -0.5*J3*T*T+J3*T4*T+V43-0.5*J3*T4*T4;
                S_Line_Runin->Offset_J3_Arr[cnt3] = (-(1.0/6)*J3*T*T*T+0.5*J3*T4*T*T+V43*T-0.5*J3*T4*T4*T+S43-V43*T4+(1.0/6)*J3*T4*T4*T4);
            }
            else if (T>(T5) && T<=(T6))
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = -Amax3;
                S_Line_Runin->Speed_J3_Arr[cnt3] = -Amax3*T+Amax3*T5+V53;
                S_Line_Runin->Offset_J3_Arr[cnt3] = (-0.5*Amax3*T*T+V53*T+Amax3*T*T5+S53-V53*T5-0.5*Amax3*T5*T5);
            }
            else if (T>=T6 && T<T7)
            {
//              S_Line_Runin->Acc_J3_Arr[cnt3] = -Amax3+J3*(T-T6);
                S_Line_Runin->Speed_J3_Arr[cnt3] = -Amax3*T+Amax3*T6+(0.5*J3*T*T)-J3*T6*T+V63+0.5*J3*T6*T6;
                S_Line_Runin->Offset_J3_Arr[cnt3] = (-0.5*Amax3*T*T+(1.0/6)*J3*T*T*T-0.5*J3*T*T*T6+V63*T+Amax3*T*T6+0.5*J3*T6*T6*T+S63-V63*T6-0.5*Amax3*T6*T6-(1.0/6)*J3*T6*T6*T6);
            }
        }
        else
            break;

    }

    //����Ĳ岹����
    Amax4 = J4 *T1;

    V04 = Vs4;
    V14 = 0.5*J4*T1*T1+V04;
    V24 = Amax4*T2+V14-Amax4*T1;
    V34 = Amax4*T3-(0.5*J4*T3*T3)+J4*T2*T3+V24-Amax4*T2-(0.5*J4*T2*T2);
    V44 = V34;
    V54 = (-0.5)*J4*T5*T5+J4*T4*T5+V44-(0.5)*J4*T4*T4;
    V64 = -Amax4*T6+V54+Amax4*T5;

    S14 = (1.0/6)*J4*T1*T1*T1+Vs4*T1;
    S24 = 0.5*Amax4*T2*T2+V14*T2-Amax4*T1*T2+S14-V14*T1+0.5*Amax4*T1*T1;
    S34 = 0.5*Amax4*T3*T3-(1.0/6)*J4*T3*T3*T3+0.5*J4*T2*T3*T3+V24*T3-Amax4*T2*T3-0.5*J4*T2*T2*T3+S24-V24*T2+(1.0/6)*J4*T2*T2*T2+0.5*Amax4*T2*T2;
    S44 = V34*T4+S34-V34*T3;
    S54 = -(1.0/6)*J4*T5*T5*T5+0.5*J4*T4*T5*T5+V44*T5-0.5*J4*T4*T4*T5+S44-V44*T4+(1.0/6)*J4*T4*T4*T4;
    S64 = -0.5*Amax4*T6*T6+V54*T6+Amax4*T5*T6+S54-V54*T5-0.5*Amax4*T5*T5;

    for (cnt4 = 0; cnt4<=10000; cnt4++)
    {
        T = cnt4*t_cnt;
        if(T<=T_total)
        {
            if(T>=0 && T<T1)
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = J4*T;
                S_Line_Runin->Speed_J4_Arr[cnt4] = 0.5*J4*T*T+Vs4;
                S_Line_Runin->Offset_J4_Arr[cnt4] = ((1.0/6)*J4*T*T*T+Vs4*T);
            }
            else if (T>T1 && T<=T2)
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = Amax4;
                S_Line_Runin->Speed_J4_Arr[cnt4] = Amax4*T-Amax4*T1+V14;
                S_Line_Runin->Offset_J4_Arr[cnt4] = (0.5*Amax4*T*T+V14*T-Amax4*T1*T+S14-V14*T1+0.5*Amax4*T1*T1);
            }
            else if (T>=T2 && T<T3)
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = Amax4-J4*(T-T2);
                S_Line_Runin->Speed_J4_Arr[cnt4] = Amax4*T-(0.5*J4*T*T)+J4*T2*T+V24-Amax4*T2-0.5*J4*T2*T2;
                S_Line_Runin->Offset_J4_Arr[cnt4] = (0.5*Amax4*T*T-(1.0/6)*J4*T*T*T+0.5*J4*T2*T*T+V24*T-Amax4*T*T2-0.5*J4*T2*T2*T+S24-V24*T2+(1.0/6)*J4*T2*T2*T2+0.5*Amax4*T2*T2);
            }
            else if (T>(T3) && T<=(T4))
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = 0;
                S_Line_Runin->Speed_J4_Arr[cnt4] = V34;
                S_Line_Runin->Offset_J4_Arr[cnt4] = (V34*T+S34-V34*T3);
            }
            else if (T>=T4 && T<T5)
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = -J4*(T-T4);
                S_Line_Runin->Speed_J4_Arr[cnt4] = -0.5*J4*T*T+J4*T4*T+V44-0.5*J4*T4*T4;
                S_Line_Runin->Offset_J4_Arr[cnt4] = (-(1.0/6)*J4*T*T*T+0.5*J4*T4*T*T+V44*T-0.5*J4*T4*T4*T+S44-V44*T4+(1.0/6)*J4*T4*T4*T4);
            }
            else if (T>(T5) && T<=(T6))
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = -Amax4;
                S_Line_Runin->Speed_J4_Arr[cnt4] = -Amax4*T+Amax4*T5+V54;
                S_Line_Runin->Offset_J4_Arr[cnt4] = (-0.5*Amax4*T*T+V54*T+Amax4*T*T5+S54-V54*T5-0.5*Amax4*T5*T5);
            }
            else if (T>=T6 && T<T7)
            {
//              S_Line_Runin->Acc_J4_Arr[cnt4] = -Amax4+J4*(T-T6);
                S_Line_Runin->Speed_J4_Arr[cnt4] = -Amax4*T+Amax4*T6+(0.5*J4*T*T)-J4*T6*T+V64+0.5*J4*T6*T6;
                S_Line_Runin->Offset_J4_Arr[cnt4] = (-0.5*Amax4*T*T+(1.0/6)*J4*T*T*T-0.5*J4*T*T*T6+V64*T+Amax4*T*T6+0.5*J4*T6*T6*T+S64-V64*T6-0.5*Amax4*T6*T6-(1.0/6)*J4*T6*T6*T6);
            }
        }
        else
            break;

    }

    return 1;
//  asm("nop");      //break pointz
}




