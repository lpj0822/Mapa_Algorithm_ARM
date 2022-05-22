/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: harriscorner.h
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

#ifndef HARRIS_CORNER_H
#define HARRIS_CORNER_H

#include "type_def.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    srcImg		          const uint8_t*	  input image buffer
[in]	    width		          s32		          image width
[in]	    height		          s32		          image height
[in]	    tp		              const s32*		  gaussian_kernel
[in/out]	harris_response		  AdasCorner*		  points.
[in]	    num		              s32		          Num of points.

Realized function:
    + caculate the Harris response for input points, given the value of harris_response.val
*/
void mvCalculateHarrisResponseOptimize(const uint8_t *srcImg, s32 width, s32 height, const s32 *tp, \
									   AdasCorner *harris_response, s32 num);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    srcImg		          const uint8_t*	  input image buffer
[in]	    size		          AdaSize		      image size
[in/out]	harris_response		  AdasCorner*		  points.
[in]	    num		              s32		          Num of points.

Realized function:
    + caculate the Harris response for input points, given the value of harris_response.val
*/
void CalculateHarrisResponse(const uint8_t *srcImg, AdaSize size, AdasCorner *harris_response, s32 num);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	corner_pass		      AdasCorner*		  input points.
[in/out]	corner_max		      AdasCorner*		  output points.
[in]	    num_pass		      s32		          Num of input points.
[in/out]	num_max		          s32*		          Num of output points.
[in]	    max_num		      s32		              The max output Num.

Realized function:
    + sort the corner in decreasing order by corner_pass.val and update corner_max
*/
void CornerResponseRestrain(AdasCorner *corner_pass, AdasCorner *corner_max, s32 num_pass, s32 *num_max, s32 max_num);

#endif