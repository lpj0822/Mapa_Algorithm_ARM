/*************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: disparity_calculation_module.h
Version: 1.1		Date: 2017-04-05		Author: wan li 		ID: 1051932

Description:
	The functions are used for the main function of disparity_calculation_test.cpp.
	The following function types are included:
	+ Rectification function for stereo camera images using LUTs.
	+ Histogram equalization function for improving the quality of disparity calculation results.

Deviation: N/A

History:
	+ Version: 1.1		Date: 2017-04-05		Author: wan li		ID: 1051932
	  Modification: #define self-protection added.
*************************************************************************************************/

#ifndef _RECTIFY_IMG_H_
#define _RECTIFY_IMG_H_

#include "common_type_def.h"
#include "cfg_disparity.h"


extern int *L_pt1, *L_pt2;
extern int *R_pt1, *R_pt2;
extern unsigned short *L_w1, *L_w2, *L_w3, *L_w4;
extern unsigned short *R_w1, *R_w2, *R_w3, *R_w4;
extern float m11[], m12[], m21[], m22[];

/*
/*Function name:  rectifyImg
*Parameter list: Name      I/O      Type	    Description
*                ptr       I 	     void*       input poniter
*                n         I        int         Aligned pointer
*                ctype     I        int         Alignment size that must be a power of two  
*Return value:   Type    Description
*                void*   The func tion returns the aligned pointer of the same type as the input pointer
*Function description: brief Aligns a pointer to the specified number of bytes.
*/
extern void rectifyImg(const unsigned char* srcL, const unsigned char* srcR, unsigned char* dstL, unsigned char* dstR, cfgRectify params);

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
extern void INVRectifyImg(cfgRectify params);

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
extern void resizeLUT(cfgRectify params);

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
extern void extractLUT(cfgRectify params);

#endif