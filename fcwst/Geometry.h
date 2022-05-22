/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Geometry.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of commonly used operates for points and rects.
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "type_def.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    srcRec		          AdasRect		      input rect.
[in]	    pDstRec		          AdasRect		      output rect.
[in]	    fShrinkRate		      float32_t		      shrinking factor.

Realized function:
    + do the shrinking for srcRec and get pDstRec
*/
void mvShinkRect(AdasRect srcRec,AdasRect *pDstRec,float32_t fShrinkRate);


/*
I/O:	    Name		          Type	     		          Content
					    						  
[in]	    pDetRecw		      const AdasRect*		      Rect1.
[in]	    pGroupRec		      const   AdasRect*		      Rect2.
[in/out]	LapRec		          AdasRect*		              overlap rect.

[out]	    returned              uint8_t		              if overlapped return 1;else return 0.

Realized function:
    + if psrc and pdst are overlapped, return 1 and LapRec is the overlapped region; else return 0;
*/
uint8_t mvLapDetbyGoup( const AdasRect *pDetRecw,const AdasRect *pGroupRec, AdasRect *LapRec);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    psrc		          AdasRect*		      Rect1.
[in]	    pdst		          AdasRect*		      Rect2.
[in/out]	    flaprioTrd		      float32_t		      ratio of overlap.

[out]	    returned              uint8_t		      if overlapped return 1;else return 0.

Realized function:
    + if psrc and pdst are overlapped, return 1; else return 0;
*/
uint8_t mvLapTrd(const AdasRect *psrc, const AdasRect *pdst, float32_t flaprioTrd);

/*
I/O:	    Name		    Type	     		  Content
					    					  
[in]	    pCroner		    const AdasCorner*	  corner point.
[in]	    rec		        const AdasRect*	      rect.

[out]	    returned        uint8_t		          if point in rect return 1; else return 0.

Realized function:
    + if point in rect return 1; else return 0.
*/
uint8_t mvAdasCornerInRect(const AdasCorner *pCroner, const AdasRect *rec);

/*
I/O:	    Name		    Type	     		  Content
					    					  
[in]	    point		    const AdasPoint*	  corner point.
[in]	    rec		        const AdasRect*	      rect.

[out]	    returned        uint8_t		          if point out of rect return 1; else return 0.

Realized function:
    + if point out of rect return 1; else return 0.
*/
uint8_t mvAdaspointOutRect(const AdasPoint *point, const AdasRect *rec);

/*
I/O:	    Name		    Type	     		  Content
					    					  
[in]	    point		    const AdasPoint*	  corner point.
[in]	    rec		        const AdasRect*	      rect.

[out]	    returned        uint8_t		          if point in rect return 1; else return 0.

Realized function:
    + if point in rect return 1; else return 0.
*/
uint8_t mvAdaspointInRect(const AdasPoint *point, const AdasRect *rec);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    psrc		          const AdasRect*	  Rect1.
[in]	    Scope		          s32	              2*Scope is the width of new rect.
[in/out]	pdst		          AdasRect*	          new Rect.

Realized function:
    + extend the rect by scope with fixed center
*/
void mvCenRecByScope(const AdasRect *psrc,s32 Scope, AdasRect *pdst);

/*
I/O:	    Name		    Type	     		  Content
					    				  
[in]	    pVec		    AdasPoint*		      Vector.
[out]       returned        float32_t             the norm of Vector

Realized function:
    + caculate the norm of Vector;
*/
float32_t mvNormVec(AdasPoint *pVec);

 /*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	DstRec		          AdasRect*		      input Rect1.
[in]	    ImgRec		          AdasRect*		      Rect2.

Realized function:
    + make sure DstRec inside of ImgRec;
*/
void mvScopeImg(AdasRect *DstRec,const AdasRect ImgRec);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in/out]	data		    s32*		          input data.
[in]	    n		        s32*		          data num.

Realized function:
    + sort the int in decreasing order;
*/
void binSort_INT(s32 *data, s32 n);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in/out]	data		    float32_t*		      input data.
[in]	    n		        s32*		          data num.

Realized function:
    + sort the float in decreasing order;
*/
void binSort_FLOAT(float32_t *data, s32 n);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in]	    A		        AdasPoint*		      point A.
[in]	    B		        AdasPoint*		      point B.
[out]       returned        float32_t             the norm of point A and B

Realized function:
    + caculate the norm of point A and B;
*/
float32_t mvNormPoint(AdasPoint *A, AdasPoint *B);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in]	    A		        AdasPoint*		      point A.
[in]	    B		        AdasPoint*		      point B.
[out]       returned        float32_t             the pow of norm of point A and B

Realized function:
    + caculate the pow of norm of point A and B;
*/
s32 mvDisPow(AdasPoint *A, AdasPoint *B);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in/out]	data		    AdasCorner*		      input corner.
[in]	    n		        s32*		          corner num.

Realized function:
    + sort the corner in decreasing order by data.val;
*/
void binSort_Corner(AdasCorner *data, s32 n);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in]	    x		        s32		              point.x
[in]	    y		        s32		              point.y.
[out]	    returned		AdasPoint		      AdasPoint.

Realized function:
    + return AdasPoint.
*/
AdasPoint adaspoint(s32 x,s32 y);

 /*
I/O:	    Name		    Type	     		  Content
					    				  
[in]	    width		    s32		              width
[in]	    height		    s32		              height
[out]	    returned		AdaSize		          AdaSize.

Realized function:
    + return AdaSize.
*/
AdaSize adasize(s32 width,s32 height);

 /*
I/O:	    Name		    Type	     		  Content
		
[in]	    x		        s32		              point.x
[in]	    y		        s32		              point.y.
[in]	    width		    s32		              width
[in]	    height		    s32		              height
[out]	    returned		adasrect		      adasrect.

Realized function:
    + return adasrect.
*/
AdasRect adasrect(s32 x,s32 y,s32 width,s32 height);

#endif
