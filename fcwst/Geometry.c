/**************************************************************************************************************
Copyright Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Geometry.cpp
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of commonly used operates for points and rects.
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "Geometry.h"

/*

Function process:over
	+ caculate the over-lap of pDetRecw and pGroupRec, if overlap excit return 1 and LapRec is the overlapped region, else return 0.
	Fan-in : 
	        + 
	Fan-out:
	        + 
	ATTENTION: __________

*/
uint8_t mvLapDetbyGoup( const AdasRect *pDetRecw,const AdasRect *pGroupRec, AdasRect *LapRec )
{
	s32 Lapwidth =  pDetRecw->width + pGroupRec->width - (WS_MAX( pDetRecw->x + pDetRecw->width, pGroupRec->x + pGroupRec->width) \
		- WS_MIN( pDetRecw->x,pGroupRec->x));
	s32 Lapheigt=   pDetRecw->height + pGroupRec->height - (WS_MAX( pDetRecw->y + pDetRecw->height, pGroupRec->y + pGroupRec->height) \
		- WS_MIN( pDetRecw->y,pGroupRec->y));

	if (Lapwidth > 0 && Lapheigt > 0)
	{
		LapRec->width  = Lapwidth;
		LapRec->height = Lapheigt;
		LapRec->x = WS_MAX(pGroupRec->x,pDetRecw->x);
		LapRec->y = WS_MAX(pGroupRec->y,pDetRecw->y);
		return 1;
	}

	return 0;

}

/*
Function process:
	+ if psrc and pdst are overlapped, return 1; else return 0;
	Fan-in : 
	        + mvReInput()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 uint8_t mvLapTrd(const AdasRect *psrc, const AdasRect *pdst, float32_t flaprioTrd)
 {
	 s32 Lapwidth =  psrc->width + pdst->width - (WS_MAX( pdst->x + pdst->width, psrc->x + psrc->width) \
		 - WS_MIN( pdst->x,psrc->x));

	 s32 Lapheigt =  psrc->height + pdst->height - ( WS_MAX( psrc->y + psrc->height, pdst->y + pdst->height) \
		 - WS_MIN( pdst->y,psrc->y));

	 if (Lapwidth < 0 || Lapheigt < 0 || Lapwidth * Lapheigt < psrc->width * psrc->height * flaprioTrd)
	 {
		 return 0;
	 }

	 return 1;
 }

/*
Function process:
	+ if point in rect return 1; else return 0
	Fan-in : 
	        + mvMatchTrackPoint()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 uint8_t mvAdasCornerInRect(const AdasCorner *pCroner, const AdasRect *rec)
 {

	 if (pCroner->x > rec->x && pCroner->x < rec->x + rec->width \
		 && pCroner->y > rec->y && pCroner->y < rec->y + rec->height)
	 {
		 return 1;
	 }

	 return 0;
 }

 /*
Function process:
	+ if point in rect return 1; else return 0
	Fan-in : 
	        + mvMatchTrackPoint()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 uint8_t mvAdaspointInRect(const AdasPoint *point, const AdasRect *rec)
  {
	  if (point->x > rec->x && point->x < rec->x + rec->width \
		  && point->y > rec->y && point->y < rec->y + rec->height)
	  {
		  return 1;
	  }

	  return 0;

  }

  /*
Function process:
	+ if point out of rect return 1; else return 0
	Fan-in : 
	        + mvMatchTrackPoint()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 uint8_t mvAdaspointOutRect(const AdasPoint *point, const AdasRect *rec)
 {

	 if (point->x < rec->x || point->x > rec->x + rec->width \
		 || point->y < rec->y || point->y > rec->y + rec->height)
	 {
		 return 1;
	 }

	 return 0;
 }

/*
Function process:
	+ make sure DstRec inside of ImgRec
	Fan-in : 
	        + mvReDetcorByRec()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 void mvScopeImg( AdasRect *DstRec,const AdasRect ImgRec )
 {
	 if (DstRec->x < ImgRec.x)
	 {
		 DstRec->x = ImgRec.x;
	 }

	 if (DstRec->y < ImgRec.y)
	 {
		 DstRec->y = ImgRec.y;
	 }

	 if (DstRec->x + DstRec->width > ImgRec.x + ImgRec.width)
	 {
		 DstRec->width = ImgRec.x + ImgRec.width -1 - DstRec->x ;
	 }

	 if (DstRec->y + DstRec->height > ImgRec.y + ImgRec.height)
	 {
		 DstRec->height = ImgRec.y + ImgRec.height - 1 - DstRec->y ;
	 }
	 
 }

/*
Function process:
	+ sort the corner in decreasing order by data.val
	Fan-in : 
	        + mvReDetcorByRec()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 void binSort_Corner(AdasCorner *data, s32 n)
  {
	  s32 i,j,left,mid,right;
	  AdasCorner temp;

	  for( i = 1; i < n; i++ )
	  {
		  temp = data[i];
		  left = 0; right = i -1;

		  while( left <= right )
		  {
			  mid = ((left + right)>>1);
			  if( temp.val > data[mid].val )
			  {
				  right = mid - 1;
			  } else {
				  left  = mid + 1;
			  }
		  }

		  for( j = i-1; j >= left; j-- )
		  {
			  *(data + j+1) = *(data + j);
		  }

		  if( left != i )
		  {
			  *(data + left) = temp;
		  }
	  }

  }

 /*
Function process:
	+ sort the int in decreasing order
	Fan-in : 
	        + mvReDetcorByRec()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 void binSort_INT(s32 *data, s32 n)
 {
	 s32 i,j,left,mid,right;
	 s32 temp;

	 for( i = 1; i < n; i++ )
	 {
		 temp = data[i];
		 left = 0; right = i -1;

		 while( left <= right )
		 {
			 mid = ((left + right)>>1);
			 if( temp < data[mid] )
			 {
				 right = mid - 1;
			 } else {
				 left  = mid + 1;
			 }
		 }

		 for( j = i-1; j >= left; j-- )
		 {
			 *(data + j+1) = *(data + j);
		 }

		 if( left != i )
		 {
			 *(data + left) = temp;
		 }
	 }
 }

/*
Function process:
	+ sort the float in decreasing order
	Fan-in : 
	        + mvReDetcorByRec()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 void binSort_FLOAT(float32_t *data, s32 n)
 {
	 s32 i,j,left,mid,right;
	 float32_t temp;

	 for( i = 1; i < n; i++ )
	 {
		 temp = data[i];
		 left = 0; right = i -1;

		 while( left <= right )
		 {
			 mid = ((left + right)>>1);
			 if( temp < data[mid] )
			 {
				 right = mid - 1;
			 } else {
				 left  = mid + 1;
			 }
		 }

		 for( j = i-1; j >= left; j-- )
		 {
			 *(data + j+1) = *(data + j);
		 }

		 if( left != i )
		 {
			 *(data + left) = temp;
		 }
	 }
    }

 /*
Function process:
	+ caculate the norm of point A and B
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 float32_t mvNormPoint(AdasPoint *A, AdasPoint *B)
 {
	 float32_t fdis;
	
	 fdis = (float32_t)sqrt((float32_t)(A->x - B->x) * (A->x - B->x) + (A->y - B->y) * (A->y - B->y));
     
	 return fdis;
 }

 /*
Function process:
	+ caculate the norm of vector
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 float32_t mvNormVec(AdasPoint *pVec)
 {
	 float32_t fdis;

	 fdis = (float32_t)sqrt( (float32_t)pVec->x * pVec->x + pVec->y * pVec->y );

	 return fdis;
 }

 /*
Function process:
	+ caculate the pow of norm of point A and B
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
s32 mvDisPow(AdasPoint *A, AdasPoint *B)

{
	s32 ndis;

	ndis = (A->x - B->x) * (A->x - B->x) + (A->y - B->y) * (A->y - B->y);

	return ndis;
}


/*
Function process:
	+ extend the rect by scope with fixed center
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 void mvCenRecByScope(const AdasRect *psrc, s32 Scope, AdasRect *pdst)
{
     pdst->x = psrc->x  + (psrc->width>>1) - (Scope>>1);
     pdst->y = psrc->y  + (psrc->height>>1) - (Scope>>1);
	 pdst->width = Scope;
	 pdst->height = Scope;

}

/*
Function process:
	+ return AdasPoint
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
AdasPoint adaspoint(s32 x,s32 y)
{
	AdasPoint point;
	point.x = x;
	point.y = y;
	return point;
}

/*
Function process:
	+ return adasize
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
 AdaSize adasize(s32 width,s32 height)
{
	AdaSize size;
	size.width = width;
	size.height = height;
	return size;
 }

/*
Function process:
	+ return rect
	Fan-in : 
	        + main()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
AdasRect adasrect(s32 x,s32 y,s32 width,s32 height)
{
	AdasRect rec;
	rec.x = x;
	rec.y = y;
	rec.width = width;
	rec.height = height;
	return rec;
} 

/*
Function process:
	+ do the shrinking for srcRec
	Fan-in : 
	        + main()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvShinkRect(AdasRect srcRec,AdasRect *pDstRec,float32_t fShrinkRate)
 {
	 pDstRec->x = srcRec.x + (int16_t)(fShrinkRate * srcRec.width);
	 pDstRec->y = srcRec.y +  (int16_t)(fShrinkRate * srcRec.height);
	 pDstRec->width = srcRec.width - (int16_t)(fShrinkRate * (srcRec.width<<1));
	 pDstRec->height = srcRec.height - (int16_t)(fShrinkRate * (srcRec.height<<1));
 }
