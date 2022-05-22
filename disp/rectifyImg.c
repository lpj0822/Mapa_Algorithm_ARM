/*************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: computeDisp.c
Version: 1.1		Date: 2017-04-01		Author: Alfred Wan		ID: 1051932

Description:
The functions are used for the main function of DISP_Interface.c.

Deviation: N/A

History:
+ Version: 1.1		Date: 2017-04-01		Author: Alfred Wan		ID: 1051932
Modification: Fixed irregularized sentences.
*************************************************************************************************/

#include "rectifyImg.h"

/*
*Function name:  extractLUT
*Parameter list: Name      I/O      Type	    Description
*                ptr       I 	     void*       input poniter
*                n         I        int         Aligned pointer
*                ctype     I        int         Alignment size that must be a power of two  
*Return value:   Type    Description
*                void*   The func tion returns the aligned pointer of the same type as the input pointer
*Function description: brief Aligns a pointer to the specified number of bytes.
*/
void extractLUT(cfgRectify params)
{
	int L_xd, L_yd, R_xd, R_yd;
	int i, j, x = 0;

	if (0 == params.isUseInterpolation)
	{
		for (j = params.cutSy; j < params.cutSy + params.cutHeight; j++)
		{
			for (i = params.cutSx; i < params.cutSx + params.cutWidth; i++)
			{
				L_xd = (int)params.RTF_Lx[j * params.orgWidth + i];
				L_yd = (int)params.RTF_Ly[j * params.orgWidth + i];
				//L_pt1[x] = min(max(L_yd * params.orgWidth * 4 + L_xd * 2, 0), params.orgHeight * params.orgWidth * 4 - 1);
				L_pt1[x] = WS_MIN(WS_MAX(L_yd * params.orgWidth + L_xd, 0), params.orgHeight * params.orgWidth - 1);

				//printf("%d \n", L_xd);

				R_xd = WS_MIN(WS_MAX((int)params.RTF_Rx[j * params.orgWidth + i], 0), params.orgWidth);
				R_yd = WS_MIN(WS_MAX((int)params.RTF_Ry[j * params.orgWidth + i], 0), params.orgHeight);
				//R_pt1[x] = min(max(R_yd * params.orgWidth * 4 + R_xd * 2, 0), params.orgHeight * params.orgWidth * 4  - 1);
				R_pt1[x] = WS_MIN(WS_MAX(R_yd * params.orgWidth + R_xd, 0), params.orgHeight * params.orgWidth - 1);
				x++;
			}
		}
	}
	else if (1 == params.isUseInterpolation)
	{
		float L_xf, L_yf, R_xf, R_yf, w_x, w_y, sub_x, sub_y;
		for (j = params.cutSy; j < params.cutSy + params.cutHeight; j++)
		{
			for (i = params.cutSx; i < params.cutSx + params.cutWidth; i++)
			{
				L_xf = params.RTF_Lx[j * params.orgWidth + i];
				L_yf = params.RTF_Ly[j * params.orgWidth + i];
				L_xd = (int)L_xf;
				L_yd = (int)L_yf;
				w_x = L_xf - L_xd;
				w_y = L_yf - L_yd;
				sub_x = 1.0 - w_x;
				sub_y = 1.0 - w_y;
				L_w1[x] = (int)(sub_x * sub_y * 0xFFFF);
				L_w2[x] = (int)(w_x * sub_y * 0xFFFF);
				L_w3[x] = (int)(sub_x * w_y * 0xFFFF);
				L_w4[x] = (int)(w_x * w_y * 0xFFFF);
				//L_pt1[x] = WS_MIN(WS_MAX(L_yd * params.orgWidth + L_xd, 0), params.orgHeight * params.orgWidth - 1);
				//L_pt2[x] = WS_MIN(WS_MAX(L_yd * params.orgWidth + L_xd + params.orgWidth, 0), params.orgHeight * params.orgWidth - 1);
				L_pt1[x] = WS_MIN(WS_MAX(L_yd * params.orgWidth * 4 + L_xd * 2, 0), params.orgHeight * params.orgWidth * 4 - 1);
				L_pt2[x] = WS_MIN(WS_MAX(L_yd * params.orgWidth * 4 + L_xd * 2 + params.orgWidth * 2, 0), params.orgHeight * params.orgWidth * 4  - 1);

				R_xf = params.RTF_Rx[j * params.orgWidth + i];
				R_yf = params.RTF_Ry[j * params.orgWidth + i];
				R_xd = (int)R_xf;
				R_yd = (int)R_yf;
				w_x = R_xf - R_xd;
				w_y = R_yf - R_yd;
				sub_x = 1.0 - w_x;
				sub_y = 1.0 - w_y;
				R_w1[x] = (int)(sub_x * sub_y * 0xFFFF);
				R_w2[x] = (int)(w_x * sub_y * 0xFFFF);
				R_w3[x] = (int)(sub_x * w_y * 0xFFFF);
				R_w4[x] = (int)(w_x * w_y * 0xFFFF);
				//R_pt1[x] = WS_MIN(WS_MAX(R_yd * params.orgWidth + R_xd, 0), params.orgHeight * params.orgWidth - 1);
				//R_pt2[x] = WS_MIN(WS_MAX(R_yd * params.orgWidth + R_xd + params.orgWidth, 0), params.orgHeight * params.orgWidth - 1);
				R_pt1[x] = WS_MIN(WS_MAX(R_yd * params.orgWidth * 4 + R_xd * 2, 0), params.orgHeight * params.orgWidth * 4  - 1);
				R_pt2[x] = WS_MIN(WS_MAX(R_yd * params.orgWidth * 4 + R_xd * 2 + params.orgWidth * 2, 0), params.orgHeight * params.orgWidth * 4  - 1);
				x++;
			}
		}
	}
}

/*
*Function name:  rectifyImg
*Parameter list: Name      I/O      Type	    Description
*                ptr       I 	     void*       input poniter
*                n         I        int         Aligned pointer
*                ctype     I        int         Alignment size that must be a power of two  
*Return value:   Type    Description
*                void*   The func tion returns the aligned pointer of the same type as the input pointer
*Function description: brief Aligns a pointer to the specified number of bytes.
*/
void rectifyImg(const unsigned char* srcL, const unsigned char* srcR, unsigned char* dstL, unsigned char* dstR, cfgRectify params)
{
	int i; 
	if (0 == params.isUseInterpolation)
	{
		int LR_d;
		i = 0;
		do 
		{		
			LR_d = L_pt1[i];
			*(dstL++) = (unsigned char)(srcL[LR_d]);

			LR_d = R_pt1[i];
			*(dstR++) = (unsigned char)(srcR[LR_d]);
			i++;
		}while (i < params.cutHeight * params.cutWidth);
	}
	else if(1 == params.isUseInterpolation)
	{
		int LR_d1, LR_d2, LR_d3, LR_d4;
		unsigned short LR_w1, LR_w2, LR_w3, LR_w4;
	    i = 0;
		do
		{
			LR_d1 = L_pt1[i];
			LR_d2 = LR_d1 + 1;
			LR_d3 = L_pt2[i];
			LR_d4 = LR_d3 + 1;		
			LR_w1 = L_w1[i];
			LR_w2 = L_w2[i];
			LR_w3 = L_w3[i];
			LR_w4 = L_w4[i];
			*(dstL++) = (unsigned char)((srcL[LR_d1] * LR_w1 + srcL[LR_d2] * LR_w2 +\
				srcL[LR_d3] * LR_w3 + srcL[LR_d4] * LR_w4 + 32768) >> 16);

			LR_d1 = R_pt1[i];
			LR_d2 = LR_d1 + 1;
			LR_d3 = R_pt2[i];
			LR_d4 = LR_d3 + 1;
			LR_w1 = R_w1[i];
			LR_w2 = R_w2[i];
			LR_w3 = R_w3[i];
			LR_w4 = R_w4[i];
		   	*(dstR++) = (unsigned char)((srcR[LR_d1] * LR_w1 + srcR[LR_d2] * LR_w2 +\
				srcR[LR_d3] * LR_w3 + srcR[LR_d4] * LR_w4 + 32768) >> 16);
			i++;
		}while (i < params.cutHeight * params.cutWidth);
	}
}

/*
*Function name:  INVRectifyImg
*Parameter list: Name      I/O      Type	    Description
*                ptr       I 	     void*       input poniter
*                n         I        int         Aligned pointer
*                ctype     I        int         Alignment size that must be a power of two  
*Return value:   Type    Description
*                void*   The func tion returns the aligned pointer of the same type as the input pointer
*Function description: brief Aligns a pointer to the specified number of bytes.
*/
void INVRectifyImg( cfgRectify params )
{
	int i,j;
	for( j = params.cutSy; j < params.cutSy + params.cutHeight; j++)
	{
		for (  i = params.cutSx; i < params.cutSx + params.cutWidth; i++)
		{
			int R_x = params.RTF_Rx[j * params.orgWidth + i];
			int R_y = params.RTF_Ry[j * params.orgWidth + i];
			int L_x = params.RTF_Lx[j * params.orgWidth + i];
			int L_y = params.RTF_Ly[j * params.orgWidth + i];

			if( R_x >= 1 && R_y >= 1 && R_x < params.orgWidth - 1 && R_y < params.orgHeight - 1)
			{
				int index_xy_WN = (R_y - 1) * params.orgWidth + R_x -1;
				int index_xy_N  = (R_y - 1) * params.orgWidth + R_x;
				int index_xy_EN = (R_y - 1) * params.orgWidth + R_x + 1;
				int index_xy_W  = (R_y ) * params.orgWidth + R_x -1;
				int index_xy_C  = (R_y ) * params.orgWidth + R_x;
				int index_xy_E  = (R_y ) * params.orgWidth + R_x + 1;
				int index_xy_WS = (R_y + 1) * params.orgWidth + R_x -1;
				int index_xy_S  = (R_y + 1) * params.orgWidth + R_x;
				int index_xy_ES = (R_y + 1) * params.orgWidth + R_x + 1;

				//
				*(params.INV_Rx + index_xy_WN) = i - params.cutSx;
				*(params.INV_Rx + index_xy_N)  = i - params.cutSx;
				*(params.INV_Rx + index_xy_EN) = i - params.cutSx;
				*(params.INV_Rx + index_xy_W)  = i - params.cutSx;
				*(params.INV_Rx + index_xy_C)  = i - params.cutSx;
				*(params.INV_Rx + index_xy_E)  = i - params.cutSx;
				*(params.INV_Rx + index_xy_WS) = i - params.cutSx;
				*(params.INV_Rx + index_xy_S)  = i - params.cutSx;
				*(params.INV_Rx + index_xy_ES) = i - params.cutSx;

				*(params.INV_Ry + index_xy_WN) = j - params.cutSy;
				*(params.INV_Ry + index_xy_N)  = j - params.cutSy;
				*(params.INV_Ry + index_xy_EN) = j - params.cutSy;
				*(params.INV_Ry + index_xy_W)  = j - params.cutSy;
				*(params.INV_Ry + index_xy_C)  = j - params.cutSy;
				*(params.INV_Ry + index_xy_E)  = j - params.cutSy;
				*(params.INV_Ry + index_xy_WS) = j - params.cutSy;
				*(params.INV_Ry + index_xy_S)  = j - params.cutSy;
				*(params.INV_Ry + index_xy_ES) = j - params.cutSy;
			}

			if( L_x >= 1 && L_y >= 1 && L_x < params.orgWidth - 1 && L_y < params.orgHeight - 1)
			{
				int index_xy_WN = (L_y - 1) * params.orgWidth + L_x -1;
				int index_xy_N  = (L_y - 1) * params.orgWidth + L_x;
				int index_xy_EN = (L_y - 1) * params.orgWidth + L_x + 1;
				int index_xy_W  = (L_y ) * params.orgWidth + L_x -1;
				int index_xy_C  = (L_y ) * params.orgWidth + L_x;
				int index_xy_E  = (L_y ) * params.orgWidth + L_x + 1;
				int index_xy_WS = (L_y + 1) * params.orgWidth + L_x -1;
				int index_xy_S  = (L_y + 1) * params.orgWidth + L_x;
				int index_xy_ES = (L_y + 1) * params.orgWidth + L_x + 1;

				//赋值
				*(params.INV_Lx + index_xy_WN) = i - params.cutSx;
				*(params.INV_Lx + index_xy_N)  = i - params.cutSx;
				*(params.INV_Lx + index_xy_EN) = i - params.cutSx;
				*(params.INV_Lx + index_xy_W)  = i - params.cutSx;
				*(params.INV_Lx + index_xy_C)  = i - params.cutSx;
				*(params.INV_Lx + index_xy_E)  = i - params.cutSx;
				*(params.INV_Lx + index_xy_WS) = i - params.cutSx;
				*(params.INV_Lx + index_xy_S)  = i - params.cutSx;
				*(params.INV_Lx + index_xy_ES) = i - params.cutSx;

				*(params.INV_Ly + index_xy_WN) = j - params.cutSy;
				*(params.INV_Ly + index_xy_N)  = j - params.cutSy;
				*(params.INV_Ly + index_xy_EN) = j - params.cutSy;
				*(params.INV_Ly + index_xy_W)  = j - params.cutSy;
				*(params.INV_Ly + index_xy_C)  = j - params.cutSy;
				*(params.INV_Ly + index_xy_E)  = j - params.cutSy;
				*(params.INV_Ly + index_xy_WS) = j - params.cutSy;
				*(params.INV_Ly + index_xy_S)  = j - params.cutSy;
				*(params.INV_Ly + index_xy_ES) = j - params.cutSy;
			}
		}
	}
}

/*
*Function name:  resizeLUT
*Parameter list: Name      I/O      Type	    Description
*                ptr       I 	     void*       input poniter
*                n         I        int         Aligned pointer
*                ctype     I        int         Alignment size that must be a power of two  
*Return value:   Type    Description
*                void*   The func tion returns the aligned pointer of the same type as the input pointer
*Function description: brief Aligns a pointer to the specified number of bytes.
*/
void resizeLUT(cfgRectify params)
{
	int i, j, k, x = 0;

	for (j = 0; j < params.orgHeight; j++)
	{
		for (i = 0; i < params.orgWidth; i++)
		{
			k = (j * 2 * params.orgWidth * 2) + i * 2;
			params.RTF_Lx[x] = WS_MIN(WS_MAX(m11[k] / 2, 0), params.orgWidth - 1);
			params.RTF_Ly[x] = WS_MIN(WS_MAX(m12[k] / 2, 0), params.orgHeight - 1);
			params.RTF_Rx[x] = WS_MIN(WS_MAX(m21[k] / 2, 0), params.orgWidth - 1);
			params.RTF_Ry[x] = WS_MIN(WS_MAX(m22[k] / 2, 0), params.orgHeight - 1);
			x++;
		}
	}

	//for (j = 0; j < params.orgHeight; j++)
	//{
	//	for ( i = 0; i < params.orgWidth; i++)
	//	{
	//		k = j * params.orgWidth + i;
	//		params.RTF_Lx[x] = WS_MIN(WS_MAX(m11[k], 0), params.orgWidth);
	//		params.RTF_Ly[x] = WS_MIN(WS_MAX(m12[k], 0), params.orgWidth);
	//		params.RTF_Rx[x] = WS_MIN(WS_MAX(m21[k], 0), params.orgWidth);
	//		params.RTF_Ry[x] = WS_MIN(WS_MAX(m22[k], 0), params.orgWidth);
	//		x++;
	//	}
	//}
}
