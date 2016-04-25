/*
 * Comp.h
 *
 * Created: 16.7.2014 16:05:59
 *  Author: JR
 */ 


#ifndef COMP_H_
#define COMP_H_

#define COMP_FILTER	10


typedef struct{
	uint8_t X_L;
	uint8_t X_H;
	uint8_t Y_L;
	uint8_t Y_H;
	uint8_t Z_L;
	uint8_t Z_H;
	
	short X;
	short Y;
	short Z;
	
	float f_X;
	float f_Y;
	float f_Z;
	
	float fHeadingx;
	float fHeadingy;
	float fHeadingz;
	
	short X_Offset;
	short Y_Offset;
	short Z_Offset;
	
	 double Nasobek1;
	 double Nasobek2;
	 
}MAG_XYZ;



typedef struct{
	int SUM_X;
	int SUM_Y;
	int SUM_Z;
	short Array_X[COMP_FILTER];
	short Array_Y[COMP_FILTER];
	short Array_Z[COMP_FILTER];
	
	long long ASUM_X;
	long long ASUM_Y;
	long long ASUM_Z;
	short AArray_X[COMP_FILTER];
	short AArray_Y[COMP_FILTER];
	short AArray_Z[COMP_FILTER];
	
}Calibrate;


uint8_t Mag_get_b(uint8_t * XYZ);
void Calibrate_Comp(MAG_XYZ * COMP);
 void Get_Filtered_Heading(float Pitch, float Roll,MAG_XYZ *COMPAS,float *fHeading);


#define PI					3.14159265359


#endif /* COMP_H_ */