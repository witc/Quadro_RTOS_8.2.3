/*
 * Comp.c
 *
 * Created: 16.7.2014 16:05:41
 *  Author: JR
 */ 

#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "Mag.h"
#include "Mag-Hal.h"
#include "Main.h"
#include "FreeRTOS_V_8_Header.h"



void Mag_init(void)
{
	MY_MAG_CONF_STRUCT Mag_set;
	
	Mag_set.bCRA_REG=(7<<CRA_REG_M_OFFSET);	//frekvence - 220hz
	Mag_set.bCRB_REG=(1<<CRB_REG_M_OFFSET);	//citlivost +-4,7 gaus
	Mag_set.bMR_REG	=0;	//continous conversion
	
	Mag_send(ADRS_CRA_REG_M,&Mag_set.bCRA_REG,1);
	Mag_send(ADRS_CRB_REG_M,&Mag_set.bCRB_REG,1);
	Mag_send(ADRS_MR_REG_M,&Mag_set.bMR_REG,1);
	vTaskDelay(10/portTICK_RATE_MS);
	

}


/**********************************************************/

uint8_t Mag_get_b(uint8_t *XYZ)
{
	
	uint8_t temp=0;
	
 	Mag_read(SR_REG_M,&temp,1);
 		
 	if ((temp&0b1)!=0b1) 
 	{	
 		return 1;
 	}
 	else
 	{	
 		//Mag_read(OUT_XLM,&XYZ[0],6);
		 
		 Mag_read(OUT_XHM,&XYZ[1],1);
		 Mag_read(OUT_XLM,&XYZ[0],1);
		 
		 
		 
		 Mag_read(OUT_YHM,&XYZ[3],1);
		 Mag_read(OUT_YLM,&XYZ[2],1);
		 
		 Mag_read(OUT_ZHM,&XYZ[5],1);
		 Mag_read(OUT_ZLM,&XYZ[4],1);

 		return 0;
 	}
	
	
	
}

void Calibrate_Comp(MAG_XYZ * COMPAS)
 {		
	
	short MAX_X=-20000;
	short MAX_Y=-20000;
	short MAX_Z=-20000;
	
	short MIN_X=20000;
	short MIN_Y=20000;
	short MIN_Z=20000;
	 
	long long Kvadranty=0;
    LCD_Queue LCD;
	 
			 
	do 
	{
		while( Mag_get_b((uint8_t)COMPAS)!=0);
	 
		COMPAS->X=(short)((COMPAS->X_H<<8) | COMPAS->X_L);
		COMPAS->Y=(short)((COMPAS->Y_H<<8) | COMPAS->Y_L);
		COMPAS->Z=(short)((COMPAS->Z_H<<8) | COMPAS->Z_L);
		
		if (COMPAS->X==0)
   		{
   			if (COMPAS->Y>0)
   			{
   				COMPAS->fHeadingx=90;
   			}else
   			{
   				COMPAS->fHeadingx=270;
   			}
   		}
   		else
   		{	
			COMPAS->fHeadingx=(atan2f(COMPAS->Y,COMPAS->X)*(180/PI));//*(180/PI);
 		}
		
		 if (COMPAS->fHeadingx<0)
		 {
			 COMPAS->fHeadingx+=360;
		 }
		 		 		 
				 
		if (MAX_Y<COMPAS->Y)MAX_Y=COMPAS->Y;						//MIN_X=-146
		if (MAX_X<COMPAS->X)MAX_X=COMPAS->X;
		if (MAX_Z<COMPAS->Z)MAX_Z=COMPAS->Z;
	  
		if (MIN_X>COMPAS->X)MIN_X=COMPAS->X;
		if (MIN_Y>COMPAS->Y)MIN_Y=COMPAS->Y;
		if (MIN_Z>COMPAS->Z)MIN_Z=COMPAS->Z;
	  
		 	 		 
		  LCD.Mag.Heading=(short)COMPAS->fHeadingx;//(short)(Suma/5);
		  LCD.Mag.Min_X=(short)MIN_X;
		  LCD.Mag.Max_X=(short)MAX_X;
		  LCD.Mag.Calib=0x63;
 		  LCD.Task_id=COMPASS_i;
 		  vTaskDelay(50/portTICK_RATE_MS);
		   
		 COMPAS->fHeadingx/=10;
		 
		 Kvadranty|=(1<<(short)COMPAS->fHeadingx);
		  
	} while (Kvadranty!=-1);//0b111111111111111111111111111111111111
	 
	COMPAS->Nasobek1=(double)(abs(MAX_X)+abs(MIN_X))/(abs(MAX_Y)+abs(MIN_Y));
	COMPAS->Nasobek2=(double)(abs(MAX_X)+abs(MIN_X))/(abs(MAX_Z)+abs(MIN_Z));
	
	 COMPAS->X_Offset=MIN_X+MAX_X;	//posunuti
	 COMPAS->Y_Offset=MIN_Y+MAX_Y;
	 COMPAS->Z_Offset=MIN_Z+MAX_Z;
	 
	 COMPAS->X_Offset-=(COMPAS->X_Offset)/2;
	 COMPAS->Y_Offset-=(COMPAS->Y_Offset)/2;
	 COMPAS->Z_Offset-=(COMPAS->Z_Offset)/2;
		
 }
 
 void Get_Filtered_Heading(float Pitch, float Roll,MAG_XYZ *COMPAS,float *fHeading)
 {
	 static Calibrate CAL;
	 static char  counter_filter=0;
	 float headX;
	 float headY;
	 
// 	 CAL.SUM_X+=COMPAS->X;
// 	 CAL.Array_X[counter_filter]=COMPAS->X;
// 	 CAL.ASUM_X+=AKCE->X;
// 	 CAL.AArray_X[counter_filter]=AKCE->X;
// 	 
// 	 CAL.SUM_Y+=COMPAS->Y;
// 	 CAL.Array_Y[counter_filter]=COMPAS->Y;
// 	 CAL.ASUM_Y+=AKCE->Y;
// 	 CAL.AArray_Y[counter_filter]=AKCE->Y;
// 	 
// 	 CAL.SUM_Z+=COMPAS->Z;
// 	 CAL.Array_Z[counter_filter]=COMPAS->Z;
// 	 CAL.ASUM_Z+=AKCE->Z;
// 	 CAL.AArray_Z[counter_filter]=AKCE->Z;
// 	 //
// 	 counter_filter++;
// 	 if (counter_filter==COMP_FILTER) counter_filter=0;
// 	 
// 	 CAL.SUM_X-=CAL.Array_X[counter_filter];
// 	 CAL.SUM_Y-=CAL.Array_Y[counter_filter];
// 	 CAL.SUM_Z-=CAL.Array_Z[counter_filter];
// 	 
// 	 CAL.ASUM_X-=CAL.AArray_X[counter_filter];
// 	 CAL.ASUM_Y-=CAL.AArray_Y[counter_filter];
// 	 CAL.ASUM_Z-=CAL.AArray_Z[counter_filter];
// 	 
// 	 COMPAS->X=(short)(CAL.SUM_X)/COMP_FILTER;
// 	 COMPAS->Y=(short)(CAL.SUM_Y)/COMP_FILTER;
// 	 COMPAS->Z=(short)(CAL.SUM_Z)/COMP_FILTER;
// 	 
// 	 AKCE->X=(int)(CAL.ASUM_X)/COMP_FILTER;
// 	 AKCE->Y=(int)(CAL.ASUM_Y)/COMP_FILTER;
// 	 AKCE->Z=(int)(CAL.ASUM_Z)/COMP_FILTER;
	 
	 //
	 // 	Fir(&COMPAS->X,0);
	 // 	Fir(&COMPAS->Y,1);
	 // 	Fir(&COMPAS->Z,2);
	 //
	 //
	 // 	Fir(&AKCE->X,3);
	 // 	Fir(&AKCE->Y,4);
	 // 	Fir(&AKCE->Z,5);
	
	 COMPAS->Z*=-1;
		 
	 headX=(COMPAS->X )*cos(Pitch)+(COMPAS->Z)*sin(Pitch);
	 headY=((COMPAS->X) *sin(Roll)*sin(Pitch))+((COMPAS->Y)*cos(Roll))-((COMPAS->Z )* sin(Roll)*cos(Pitch));
	 
	 *fHeading=(atan2f((double)headY,(double)headX))*180/PI;
	 if ((*fHeading)< 0) *fHeading+=360;
	 
	 //Fir(&COMPAS->fHeadingx);
	 
	 
	 
 }

