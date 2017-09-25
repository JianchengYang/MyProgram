/*
 * main.c
 */
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "PlatForm.h"
#include "math.h"


#pragma DATA_SECTION(S_Line_Fix,"ZONE7DATA");
        S_Line_Fix_Type S_Line_Fix;
#pragma DATA_SECTION(S_Line_Runin,"ZONE7DATA");
        S_Line_J_Runin S_Line_Runin;
#pragma DATA_SECTION(Decare_Point,"ZONE7DATA");
        POSE_DECARE_Type Decare_Point;

int main(void) {
    S_Line_Fix.limit_rate=0.95;
    S_Line_Fix.speed_rate=0.1;

    Decare_Point.x_now=44.46;
    Decare_Point.y_now=15.57;
    Decare_Point.z_now=22.95;
    Decare_Point.c_now=10;

    Decare_Point.x_pre=44.81;
    Decare_Point.y_pre=53.52;
    Decare_Point.z_pre=9.42;
    Decare_Point.c_pre=60;

    Delta_S_Line(&S_Line_Fix,&Decare_Point,&S_Line_Runin);
    Decare_Point.c_pre=143.568;
	return 0;
}
