/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: preprocess.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate for image processing.
	The following function types are included:
	+ GammaCorrect(): Do the Gamma correct for img.
	+ GammaCorrectPixel(): Do the Gamma correct for one pixel.
    + EnhanceContrast(): Do the histogram equalization for img.
			
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef PROCESS_H
#define PROCESS_H

#include "type_def.h"

/*
I/O:	    Name		      Type	     		  Content
					    						  
[in/out]	img		          uint8_t*		      image buffer
[in]	    w		          s32		          width of img;
[in]        Rio               AdasRect            ROI of img to do the Gamma Correct

Realized function:
    + Do the Gamma correct for img.
*/
void GammaCorrect(uint8_t *img, s32 w, AdasRect *Rio);

/*
I/O:	    Name		      Type	     		  Content
					    						  
[in/out]	pixel		      uint8_t*		      one pixel value

Realized function:
    + Do the Gamma correct for one pixel.
*/
void GammaCorrectPixel(uint8_t *pixel);

/*
I/O:	    Name		      Type	     		  Content
					    						  
[in/out]	pData		      uint8_t*		      image buffer
[in]	    iWidth	          s32		          width of img;
[in]	    pfspace	          float*		      public free space;
[in]        Rio               AdasRect            ROI of img to do the histogram equalization

Realized function:
    + Do the histogram equalization for img.
*/
void EnhanceContrast(uint8_t *pData, s32 iWidth,float *pfspace,AdasRect Rect);

#endif