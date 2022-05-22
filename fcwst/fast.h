/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: fast.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of fast points detection and nonmax suppress.
	The following function types are included:
	+ oast9_16(): fast corner detect.
	+ corner_score(): caculate the score of fast corner.
    + fast_nonmax(): Do the nonmax suppress for corner points.
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef   FAST_H
#define   FAST_H

#include "type_def.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    im		              const uint8_t*	  input image buffer
[in]	    xsize		          s32	              width od image
[in]	    ysize		          s32	              height od image
[in]	    corners		          xy*	              the input fast corner
[in]	    num_corners		      s32	              Num of input fast points
[in]	    barrier		          s32	              Corner Thresh 
[in/out]	numnx		          s32*	              Output points Num after nonmax suppress 
[in/out]	pRowStart		      s32*	              the point index of each row 
[in/out]	pScore		          s32*	              score of points
[in/out]	pXYNoMax		      xy*	              the output fast corner after nonmax suppress 

[out]       returned              xy*	              the output fast corner after nonmax suppress 

Realized function:
    + Do the nonmax suppress for corner points
*/
xy*  fast_nonmax(const uint8_t* im, s32 xsize, s32 ysize, xy* corners, s32 numcorners, s32 barrier, s32* numnx,
	             s32 *pRowStart,s32 * pScore,xy *pXYNoMax);

xy * fast_corner_detect_9(const uint8_t* im, s32 xsize, s32 ysize, s32 barrier, s32* num,xy *pXyCorner);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    imp		              const uint8_t*	  ptr for the point
[in]	    pointer_dir		      const s32*	      integer pointer offstes
[in]	    barrier		          s32	              threshold

[out]	    returned		      s32	              score

Realized function:
    + caculate the score of fast corner
*/
s32 corner_score(const uint8_t*  imp, const s32 *pointer_dir, s32 barrier);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    im		              const uint8_t*	  input image buffer
[in]	    pMask		          const uint8_t*	  input mask image buffer
[in]	    xsize		          s32	              width od image
[in]	    ysize		          s32	              height od image
[in]	    b		              s32	              Corner Thresh 
[in/out]	num_corners		      s32*	              Num of fast points
[in/out]	pXyCorner		      s32*	              location fast points
[in]	    nId		              s32	              sacle index
[in]	    nStart_y		      s32	              y start region of ROI
[in]	    nEnd_y		          s32	              y end region of ROI
[in]	    nStart_x		      s32	              x start region of ROI
[in]	    nEnd_x		          s32	              x end region of ROI

Realized function:
    + fast corner detect
*/
void oast9_16(const uint8_t* im, const uint8_t* pMask, s32 xsize, s32 ysize, s32 b, s32* num_corners, xy *pXyCorner,\
			  s32 nId, s32 nStart_y, s32 nEnd_y, s32 nStart_x, s32 nEnd_x);

#endif
