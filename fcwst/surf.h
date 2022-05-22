/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: surf.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate of surf features.
	The following function types are included:
	+ mvComputeSurfDescriptor(): caculate the surf feature of input point.
	+ mvComputeSingleSurfDescriptor(): caculate the surf feature of one input point.
    + mvSurfDist(): caculate the distance between surf features.
			
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef SURF_H
#define SURF_H

#include "type_def.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    puImgData		      const uint8_t*	  input image buffer
[in]	    szImg		          AdaSize		      image size
[in]	    pPoint		          const AdasCorner*   input fast points
[in]	    nPtCnt		          s32		          input fast points Num

[in/out]	pSurfFeature		  uint8_t*		      surf feature.

Realized function:
    + caculate the surf feature of input point
*/
void mvComputeSurfDescriptor(uint8_t *puImgData, AdaSize szImg, const AdasCorner *pPoint, s32 nPtCnt,\
							 uint8_t *pSurfFeature);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    puImgData		      const uint8_t*	  input image buffer
[in]	    szImg		          AdaSize		      image size
[in]	    pPoint		          const AdasCorner*   input fast points

[in/out]	pSurfFeature		  uint8_t*		      surf feature.

Realized function:
    + caculate the surf feature of input point
*/
void mvComputeSingleSurfDescriptor(uint8_t *puImgGray, AdaSize szImg, const AdasCorner *pPoint, \
								   uint8_t *pSurfFeature);

/*
I/O:	    Name		  Type	     		  Content
					    				  
[in]	    p1		      const uint8_t*	  input surf feature
[in]	    p2		      const uint8_t*	  input surf feature
   
[out]	    returned	  s32*		          surf feature distance.

Realized function:
    + caculate the distance between surf features
*/
s32 mvSurfDist(const uint8_t *p1, const uint8_t *p2);

#endif