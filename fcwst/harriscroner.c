/**************************************************************************************************************
Copyright Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: harriscorner.cpp
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of Harris corner response.
	The following function types are included:
	+ mvCalculateHarrisResponseOptimize(): caculate the Harris response for input points, given the value of harris_response.val.
	+ CalculateHarrisResponse(): caculate the Harris response for input points, given the value of harris_response.val.
    + CornerResponseRestrain():  sort the corner in decreasing order by corner_pass.val and update corner_max.
		
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "harriscroner.h"
#include "Geometry.h"

/*
Function process:
	+ caculate the Harris response for input points
	Fan-in : 
	        + CalculateHarrisResponse()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void CalculateHarrisResponseOptimize(const uint8_t *srcImg, s32 width, s32 height, const s32 *tp,
									 AdasCorner *harris_response, s32 num)
{
	s32 i, j, count;
	s32 c0, c1, c2, cr;
	s32 dx0, dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8;
	s32 dy0, dy1, dy2, dy3, dy4, dy5, dy6, dy7, dy8;

	uint8_t ImgT2L2,ImgT2L1,ImgT2L0;
	uint8_t ImgT2R2,ImgT2R1;

	uint8_t ImgT1L2,ImgT1L1,ImgT1L0;
	uint8_t ImgT1R2,ImgT1R1;

	uint8_t ImgB1L2,ImgB1L1,ImgB1L0;
	uint8_t ImgB1R2,ImgB1R1;

	uint8_t ImgB2L2,ImgB2L1,ImgB2L0;
	uint8_t ImgB2R2,ImgB2R1;

	uint8_t ImgL2,ImgL1,ImgL0;
	uint8_t ImgR2,ImgR1;

	s32 CurHig;
	s32 CurHigUp2;
	s32 CurHigUp1;
	s32 CurHiglow1;
	s32 CurHiglow2;
	l32 nMidVal;

	for(count = 0; count < num; ++count)
	{
		j = harris_response[count].y;
		i = harris_response[count].x;

		CurHig = j * width + i;
		CurHigUp1 = CurHig - width;
		CurHigUp2 = CurHigUp1 - width;
		CurHiglow1 = CurHig + width;
		CurHiglow2 = CurHiglow1 + width;


		ImgT2L2 = *(srcImg+CurHigUp2 - 2);
		ImgT2L1 = *(srcImg+CurHigUp2 - 1);
		ImgT2L0 = *(srcImg+CurHigUp2 );
		ImgT2R2 = *(srcImg+CurHigUp2 + 2);
		ImgT2R1 = *(srcImg+CurHigUp2 + 1);
		

		ImgT1L2 = *(srcImg+CurHigUp1 - 2);
		ImgT1L1 = *(srcImg+CurHigUp1 - 1);
		ImgT1L0 = *(srcImg+CurHigUp1 );
		ImgT1R2 = *(srcImg+CurHigUp1 + 2);
		ImgT1R1 = *(srcImg+CurHigUp1  + 1);

		ImgB1L2 = *(srcImg+CurHiglow1 - 2);
		ImgB1L1 = *(srcImg+CurHiglow1 - 1);
		ImgB1L0 = *(srcImg+CurHiglow1 );
		ImgB1R2 = *(srcImg+CurHiglow1 + 2);
		ImgB1R1 = *(srcImg+CurHiglow1 + 1);

		ImgB2L2 = *(srcImg+CurHiglow2- 2);
		ImgB2L1 = *(srcImg+CurHiglow2 - 1);
		ImgB2L0 = *(srcImg+CurHiglow2 );
		ImgB2R2 = *(srcImg+CurHiglow2 + 2);
		ImgB2R1 = *(srcImg+CurHiglow2 + 1);

		ImgL2 = *(srcImg+CurHig - 2);
		ImgL1 = *(srcImg+CurHig- 1);
		ImgL0 = *(srcImg+CurHig );
		ImgR2 = *(srcImg+CurHig + 2);
	    ImgR1 = *(srcImg+CurHig  + 1);

		dx0   = -ImgT2L2 - (ImgT1L2<<1) - ImgL2 + ImgT2L0 + (ImgT1L0 << 1) + ImgL0;
		dx0   =  dx0 >> 2;

		/*dy0   =  -*(srcImg+width*(j-2)+i-2)-*(srcImg+width*(j-2)+i-1)*2-*(srcImg+width*(j-2)+i)
			+*(srcImg+width*j+i-2)+*(srcImg+width*j+i-1)*2+*(srcImg+width*j+i);*/

		dy0 = -ImgT2L2  - (ImgT2L1<<1) - ImgT2L0 + ImgL2 + (ImgL1<<1) +ImgL0;

		dy0   =  dy0 >> 2;

		nMidVal = dy0 * tp[0];
		c0    = (dx0*dx0*tp[0]) >> 20;
		c1    = (dy0*nMidVal) >> 20;
		c2    = (dx0*nMidVal) >> 20;

		/*dx1   = -*(srcImg+width*(j-2)+i-1)-*(srcImg+width*(j-1)+i-1)*2-*(srcImg+width*j+i-1)
			+*(srcImg+width*(j-2)+i+1)+*(srcImg+width*(j-1)+i+1)*2+*(srcImg+width*j+i+1);*/

		dx1 = -ImgT2L1 - (ImgT1L1<<1) -ImgL1 +ImgT2R1 + (ImgT1R1<<1) + ImgR1;

		dx1   =  dx1 >> 2;

		/*dy1   =  -*(srcImg+width*(j-2)+i-1)-*(srcImg+width*(j-2)+i)*2-*(srcImg+width*(j-2)+i+1)
			+*(srcImg+width*j+i-1)+*(srcImg+width*j+i)*2+*(srcImg+width*j+i+1);*/
		dy1 = -ImgT2L1 - (ImgT2L0<<1) -ImgT2R1 + ImgL1  + (ImgL0<<1) + ImgR1;

		dy1   =  dy1 >> 2;

		nMidVal = dy1 * tp[1];
		c0   += (dx1*dx1*tp[1]) >> 20;
		c1   += (dy1*nMidVal) >> 20;
		c2   += (dx1*nMidVal) >> 20;

		
		dx2 = -ImgT2L0 - (ImgT1L0<<1) -ImgL0 + ImgT2R2 + (ImgT1R2<<1) +ImgR2;

		dx2   =  dx2 >> 2;


		dy2 = -ImgT2L0 - (ImgT2R1<<1) -ImgT2R2 + ImgL0 + (ImgR1<<1) +ImgR2;

		dy2   =  dy2 >> 2;

		nMidVal = dy2 * tp[2];
		c0   += (dx2*dx2*tp[2]) >> 20;
		c1   += (dy2*nMidVal) >> 20;
		c2   += (dx2*nMidVal) >> 20;


		dx3 = -ImgT1L2 - (ImgL2<<1) -ImgB1L2 + ImgT1L0 + (ImgL0<<1) +ImgB1L0;

		dx3   =  dx3 >> 2;

	
		dy3 = -ImgT1L2 - (ImgT1L1<<1) -ImgT1L0 +ImgB1L2 + (ImgB1L1<<1) +ImgB1L0;

		dy3   =  dy3 >> 2;

		nMidVal = dy3 * tp[3];
		c0   += (dx3*dx3*tp[3]) >> 20;
		c1   += (dy3*nMidVal) >> 20;
		c2   += (dx3*nMidVal) >> 20;				


		dx4 = -ImgT1L1 -(ImgL1<<1) -ImgB1L1 + ImgT1R1 +(ImgR1<<1) + ImgB1R1;

		dx4   =  dx4 >> 2;
	
		dy4 = -ImgT1L1 -(ImgT1L0<<1) -ImgT1R1 + ImgB1L1 +(ImgB1L0 <<1) +ImgB1R1;
		dy4   =  dy4 >> 2;

		nMidVal = dy4 * tp[4];
		c0   += (dx4*dx4*tp[4]) >> 20;
		c1   += (dy4*nMidVal) >> 20;
		c2   += (dx4*nMidVal) >> 20;


		dx5 = -ImgT1L0 -(ImgL0<<1) -ImgB1L0 +ImgT1R2 +(ImgR2<<1) + ImgB1R2;

		dx5   =  dx5 >> 2;


		dy5 =  -ImgT1L0 -(ImgT1R1<<1) -ImgT1R2 + ImgB1L0 +( ImgB1R1<<1) +ImgB1R2;
		dy5   =  dy5 >> 2;

		nMidVal = dy5 * tp[5];
		c0   += (dx5*dx5*tp[5]) >> 20;
		c1   += (dy5*nMidVal) >> 20;
		c2   += (dx5*nMidVal) >> 20;

	
		dx6 = -ImgL2 - (ImgB1L2<<1)- ImgB2L2 +ImgL0 +(ImgB1L0<<1) +ImgB2L0;

		dx6   =  dx6 >> 2;
		dy6 = -ImgL2 -(ImgL1<<1) -ImgL0 + ImgB2L2 + (ImgB2L1<<1) +ImgB2L0;
		dy6   =  dy6 >> 2;

		nMidVal = dy6 * tp[6];
		c0   += (dx6*dx6*tp[6]) >> 20;
		c1   += (dy6*nMidVal) >> 20;
		c2   += (dx6*nMidVal) >> 20;

		
		dx7   = -ImgL1 -(ImgB1L1<<1) -ImgB2L1 + ImgR1 +(ImgB1R1<<1) + ImgB2R1;

		dx7   =  dx7 >> 2;
		dy7 = -ImgL1 -( ImgL0<<1) -ImgR1 +ImgB2L1 +(ImgB2L0<<1) +ImgB2R1;
		dy7   =  dy7 >> 2;

		nMidVal = dy7 * tp[7];
		c0   += (dx7*dx7*tp[7]) >> 20;
		c1   += (dy7*nMidVal) >> 20;
		c2   += (dx7*nMidVal) >> 20;			

		dx8 = -ImgL0 -(ImgB1L0<<1) -ImgB2L0 + ImgR2 +(ImgB1R2<<1) +ImgB2R2;

		dx8   =  dx8 >> 2;				
		dy8 = -ImgL0 -(-ImgR1<<1) - ImgR2 +ImgB2L0 +(ImgB2R1<<1) +ImgB2R2; 
		dy8   =  dy8 >> 2;

		nMidVal = dy8 * tp[8];
		c0   += (dx8*dx8*tp[8]) >> 20;
		c1   += (dy8*nMidVal) >> 20;
		c2   += (dx8*nMidVal) >> 20;

		cr    = (s32)(c0*c1-c2*c2-(c0+c1)*(((c0+c1)*1311) >> 15));//0.04

		harris_response[count].val = cr;
	}
}

/*
Function process:
	+ caculate the Harris response for input points, given the value of harris_response.val
	Fan-in : 
	        + mvCornerDetct()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void CalculateHarrisResponse(const uint8_t *srcImg, AdaSize size, AdasCorner *harris_response, s32 num)
{
	s32 gaussian_kernel[9] = {115731, 116895, 115731, 116895, 118070, 116895, 115731, 116895, 115731};

	CalculateHarrisResponseOptimize(srcImg, size.width, size.height, gaussian_kernel, harris_response, num);

}

/*
Function process:
	+ sort the corner in decreasing order by corner_pass.val and update corner_max
	Fan-in : 
	        + binSort_Corner()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void CornerResponseRestrain(AdasCorner *corner_pass,AdasCorner *corner_max, s32 num_pass,\
	                             s32 *num_max,s32 max_num)
{
	s32 i;

	if(num_pass <= max_num)
	{
		for(i = 0; i < num_pass; ++i)
		{
			corner_max[i] = corner_pass[i];
			corner_max[i].State.nMacthNum = 0;
		}
		*num_max = num_pass;
	}
	else
	{

		binSort_Corner(corner_pass, num_pass);
		for(i = 0; i < max_num; ++i)
		{
			corner_max[i] = corner_pass[i];
			corner_max[i].State.nMacthNum = 0;
		}

		*num_max = max_num;
	}
}


