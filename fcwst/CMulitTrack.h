/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: CMulitTrack.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930
Version: 2.0		Date: 2018-05-08		Author: Li Wan		        ID: 1047931

Description:	
	The functions in this file are defined as the realized of Multi-target tracking.
	The following function types are included:
	+ FCW_TRACK_Init_adas_global_param(): malloc the memory for Tracking.
	+ FCW_TRACK_MultieTrack(): The main realize of multi-tracking.
    + FCW_TRACK_GetResult():  Get the multi-tracking result.
	+ FCW_TRACK_mvClearWholeGroup(): Clear m_globlparam[nId].m_pGroupSets
	+ FCW_TRACK_Unit_adas_global_param(): Free memory.
	+ FCW_TRACK_SimpleTrack(): Do the sample tracking based on temple tracking. if tracked, return 1; else return 0;
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
	+ Version: 2.0		Date: 2018-05-08		Author: Li Wan		    ID: 1047931
**************************************************************************************************************/

#ifndef CMULITTRACK_H
#define CMULITTRACK_H

#include "adastype.h"
#include "Geometry.h"
#include "type_def.h"
#include "kalmanfilter.h"

#define DETCOR_STAR
#define TRACK_DEBUG

#define scale_shink_1_id_W  40
#define scale_shink_2_id_W  120
#define inner_global_param_num 16


#ifdef TRACK_DEBUG
	#ifdef WIN32

		#define _WIN32_TRACK_DEBUG_
	#else

		#define _ANDROID_TRACK_DEBUG_
	#endif
#endif


/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    ImgSize		          AdaSize		     Size of input image.

Realized function:
    + malloc the memory for Tracking
	+ ATTENTION: ImgSize is the size of resized img (1/2 of orig image)
*/
void FCW_TRACK_Init_adas_global_param(AdaSize ImgSize);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    pInPutParam		      PortInput*		  input stract of multi-tracking.

Realized function:
    + The main realize of multi-tracking
*/
void FCW_TRACK_MultieTrack( PortInput *pInPutParam);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    pTrackOutput		  MuliTracker**		  output stract of multi-tracking.

Realized function:
    + Get the multi-tracking result
*/
void FCW_TRACK_GetResult(MuliTracker **pTrackOutput);

/*
Realized function:
    + Clear m_globlparam[nId].m_pGroupSets
*/
void FCW_TRACK_mvClearWholeGroup(void);

/*
Realized function:
    + Free memory
*/
void FCW_TRACK_Unit_adas_global_param(void);

/*
I/O:	    Name		        Type	     	  Content
					    						  
[in]	    SrcRec		        const AdasRect	  Target rect in last frame.
[in]	    pSrcImg		        imgage*		      image date in last frame.
[in]	    pDstImg		        const imgage*	  image date in this frame.
[in]	    MotionVec		    AdasPoint	      offset of SrcRec in this frame.
[in/out]	    pDstRec		        AdasRect*    	  Target rect in this frame.

Realized function:
    + Do the sample tracking based on temple tracking. if tracked, return 1; else return 0;
*/
s32 FCW_TRACK_SimpleTrack(const AdasRect SrcRec, imgage *pSrcImg, const imgage *pDstImg, AdasPoint MotionVec,\
						  AdasRect *pDstRec);


#ifdef DETCOR_STAR
//	void mvCopyToObjMat(const imgage *pGray,AdasRect Rio,imgage *RioMat);
#endif
	

#endif
