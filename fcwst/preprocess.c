/**************************************************************************************************************
Copyright Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: preprocess.cpp
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

#include "preprocess.h"
#include "math.h"

extern uint8_t uGammLut[256];//Lut for Gamma correct

/*
Function process:
	+ Do the Gamma correct for img
	Fan-in : 
	        + ()
	Fan-out:
	        + 
	ATTENTION: __________
*/
void GammaCorrect(uint8_t *img, s32 w,AdasRect *Rio)
{
	s32 i,j;
	uint8_t *Ptr =0;

	for (j = Rio->y; j < Rio->y + Rio->height; j++)
	{
		Ptr = img +  w *j;

		for (i = Rio->x; i < Rio->x + Rio->width; i++)
		{
            Ptr[i] = uGammLut[Ptr[i]];
		}
	}
	
}

/*
Function process:
	+ Do the Gamma correct for one pixel
	Fan-in : 
	        + ()
	Fan-out:
	        + 
	ATTENTION: __________
*/
void GammaCorrectPixel(uint8_t *pixel)
{
	*pixel = uGammLut[*pixel];
}

/*
Function process:
	+ Do the histogram equalization for image
	Fan-in : 
	        + ()
	Fan-out:
	        + 
	ATTENTION: __________
*/
void EnhanceContrast(uint8_t *pData, int iWidth,float *pfspace,AdasRect Rect)  
{  
	s32 i,j;  
	uint8_t *pTr = (uint8_t*)pfspace;
	float32_t *probability =0 ;
	float32_t* grayNum = 0;
	float32_t* newgray = 0;
	uint8_t MaxGry = 0;
	uint8_t MinGray = 255;
	s32 sizeRec = Rect.width * Rect.height;
	

	probability = (float32_t *)pTr;
	pTr = ADAS_ALIGN_16BYTE(probability + 256);
	grayNum = (float32_t *)pTr;
	pTr = ADAS_ALIGN_16BYTE(grayNum + 256);
	newgray = (float32_t *)pTr;
	pTr = ADAS_ALIGN_16BYTE(newgray + 256);
	memset(grayNum,0,sizeof(float32_t) * 256);
	
	for(i = Rect.y ;i < Rect.y + Rect.height; i++)  
	{  
		pTr = pData + i * iWidth;

		for(j = Rect.x;j < Rect.x + Rect.width;j++)  
		{  
			grayNum[pTr[j]]++; 
			MaxGry = WS_MAX(MaxGry,pTr[j]);
			MinGray = WS_MAX(MinGray,pTr[j]);
		}  
	}  

	for(i= 0;i < 256;i++)  
	{  
		probability[i]= grayNum[i]/sizeRec;  
	}  

	newgray[0] = probability[0];
	for(i = 1;i < 256;i++)  
	{   
	   newgray[i] = newgray[i-1] + probability[i];
	}  


	for(i = Rect.y ;i < Rect.y + Rect.height; i++)  
	{  
		pTr = pData + i * iWidth;

		for(j = Rect.x;j < Rect.x + Rect.width;j++)  
		{  
			pTr[j] = (uint8_t)(newgray[pTr[j]] * 255  + 0.5);  
		}  
	}  


	return;  
}  

