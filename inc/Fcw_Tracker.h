/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: CMulitTrack.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

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
**************************************************************************************************************/

#ifndef FCW_TRACK_H
#define FCW_TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "type_def.h"


//extern MuliTracker   g_MuliTracker; //每帧的输出结果

/**************多目标外部调用的函数*************************/
extern void FCW_TRACK_MultieTrack( PortInput *pInPutParam); //多目标跟踪接口

extern void FCW_TRACK_Init_adas_global_param(AdaSize ImgSize); //内存分配，用于算法内部的数据处理

extern void FCW_TRACK_Unit_adas_global_param(void);//释放内存

extern void FCW_TRACK_mvClearWholeGroup(void);

extern void FCW_TRACK_GetResult(MuliTracker **pTrackOutput);

extern s32 FCW_TRACK_SimpleTrack(const AdasRect SrcRec, imgage *pSrcImg,const imgage *pDstImg,AdasPoint MotionVec,AdasRect *pDstRec);

#ifdef __cplusplus
}
#endif

#endif // !FCW_TRACK_H
