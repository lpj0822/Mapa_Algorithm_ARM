/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: trajectory.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate of trajects of object.
	The following function types are included:
	+ mvPredictTrack(): Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint).
	+ mvClearTrajIfo(): Clear the trajecy of groups.
    + mvMatchTrackPoint(): Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint).
	+ mvInitVote(): caculate the InitVote (offset with the center point) of traject point based on InitRect
	+ mvProcessVote(): caculate the InitVote (offset with the center point) of traject point based on InitRect
	+ mvTrajecyVote(): caculate the center vote point of pTrajecy
			
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef TRAJECTORY_H
#define  TRAJECTORY_H

#include "type_def.h"


typedef struct TRAGECY
{
	AdasRect pRectPredict;

	TrackPoint *pTrackPoint;
	uint8_t *pfeature;
	uint8_t *processfeature;
	s32 nTrackId;
	s32 nMapGroupId;

	int16_t nTrackLen;
	int16_t nEstTimes;
	AdasPoint ptPredict;
	Vote  InitVote;
	Vote  OrInitVote;
	Vote  ProcessVote;
	AdasPoint InitPoint;
	AdasPoint ProcessPoint;
	AdasPoint OrinitPoint;

	uint8_t bInitTracked;
	uint8_t bOrInitVoteTracked;
	uint8_t bProcessTracked;

#ifdef SHOW_RESULT
	Scalar col;
#endif

}trajecy;

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pTrajecy		      trajecy*		      trajecy struct.
[in]	    size		          const AdaSize	      Max predict region (image size)
[in]	    nFramSeq		      const int64_t		  frame num.

Realized function:
    + Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint);
*/
void mvPredictTrack( trajecy *pTrajecy, const AdaSize size, const int64_t nFramSeq);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pTrajecy		      trajecy*		      trajecy struct.

Realized function:
    + Clear the trajecy of groups.
*/
void mvClearTrajIfo(trajecy *pTrajecy);

void mvCopyTrajecy(trajecy *pDstTrajecy,trajecy *pSrcTrajecy);

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in]	    pTrajecy		      const trajecy*		 trajecy struct.
[in]	    pfeature		      const uint8_t*	     input fast points features
[in]	    pFastCorner		      AdasCorner*	         input fast points corner
[in]	    FastCornerNum		  const s32	             input fast points Num
[in/out]	pCornerPos		      s32*                   output matched fast point ID
[in/out]	pCornerTrackDis		  s32*                   output matched fast point distance
[in]	    nId		              s32	                 Scale factor

[out]	    returned		      s32		             if find matched point return 1; else return 0.

Realized function:
    + Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint);
*/
s32 mvMatchTrackPoint( const trajecy *pTrack, const uint8_t *pfeature,\
	                   AdasCorner *pFastCorner,const s32 FastCornerNum,\
	                  s32 *pCornerPos, s32 *pCornerTrackDis,s32 nId);

s32 mvMatchProcessPoint( const trajecy *pTrack, AdasRect ProcessContour ,const uint8_t *pfeature,\
	AdasCorner *pFastCorner,const s32 FastCornerNum,\
	s32 *pCornerPos, s32 *pCornerTrackDis);

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in/out]	pTrajecy		      trajecy*		         trajecy struct.
[in]	    initRec		          const AdasRect	     targect rect
[in]	    VotePointIndex		  int	                 index of vote point

Realized function:
    + caculate the InitVote (offset with the center point) of traject point based on InitRect
*/
void mvInitVote(trajecy *pTrajecy, const AdasRect initRec, int VotePointIndex );

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in/out]	pTrajecy		      trajecy*		         trajecy struct.
[in]	    ProcessRec		      const AdasRect	     process targect rect

Realized function:
    + caculate the InitVote (offset with the center point) of traject point based on InitRect
*/
void mvProcessVote( trajecy *pTrajecy, const AdasRect ProcessRec );

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in]	    pTrajecy		      trajecy*		         trajecy struct.
[in]	    fscale		          float	                 sacle ratio of pTrajecy
[in]	    VoteStyle		      VoteSort	             vote style

Realized function:
    + caculate the center vote point of pTrajecy
*/
Vote mvTrajecyVote(trajecy *pTrajecy, float fscale, VoteSort VoteStyle);

s32  mvDisPowToIndexPoin(trajecy *pTrajecy,s32 poinIndex);

AdasPoint mvMovecToIndexPoin( trajecy *pTrajecy,s32 poinIndex );

void mvCorrectProcessVote( trajecy *pTrajecy, const VoteErorrVal voteCorrectVal );

#endif
