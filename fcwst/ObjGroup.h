/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: ObjGroup.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate for tracking groups.
	The following function types are included:
	+ mvClearGroupIfo(): clear Group information.
	+ mvMidGroupRec(): Do the midle filter for pGroup->rtContour based on pGroup->histoyRec.
    + mvScaleVal(): Caculate the scale factor of pGroup.
	+ mvDelUnReasonTrack(): judge if the tracked point is suitable;
	+ mvEstablInitVote(): Init vote for traject that inside pGroup->rtContour.
	+ mvclearInitState(): make pGroup->pObjtr->bInitTracked=0, if pGroup->pObjtr in Rec.
	+ mvUpdataGroupCenTraj(): update pGroup->Centr by pGroup->rtContour.
	+ mvclearProcesState(): make pGroup->pObjtr->bProcessTracked=0, if pGroup->pObjtr inside of Rec.
		
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef OBJGROUP_H
#define OBJGROUP_H
#include "type_def.h"
#include "trajectory.h"
#include "kalmanfilter.h"
#include "declare.h"


typedef struct CAR_BOTTOM
{
	int64_t nDetBottomFam;

	s32  nDetYPos;
	s32  nDistoBottom;

	int16_t nDetBottomNum;
}CarBot;

typedef struct OBj_GROUP
{
	int64_t nFramseq;
	int64_t nPreProssVotFram;
	int64_t nPerLastDetFramseq;
	int64_t updataOriFramSeq;
	int64_t updataFrambyCar;
	AdasRect rtContour;
	AdasRect InitContour;
	AdasRect OriInitContour;
	AdasRect ProcessContour;
	AdasRect PreditRec;
	trajecy Centr;
	CarBot   CarBottom;

	s32 nGroupId;
	s32  nLastupdataCarWidth;
	s32  ntrajecyNum;
	s32  nCurrentCarBottom;
	s32  nMotionLeng;
	s32 nBottomWidth[GROUP_BOTTOM_NUM];
	s32 nBottomNum;
	s32 nUpdataIndex;
	objtype ntype;
	kalman_rect_state KalmState;
	sysKalmanState sysKalmState;
	Motion  *pMotion;
	uint8_t *pOrInitGray;
	trajecy *pObjtr;
	histrec histoyRec;
	imgage UpdatedImg[UPDATA_IMAGE_NUM];
	imgage Templat;

	systime    nTime;

	uint8_t nDetState[GROUP_DETER_STATE_NUM];
	uint8_t nStateNum;
	uint8_t nTruelyObj;
    uint8_t SerpreditNum;
	int16_t  nMinCarWid;

	int64_t nDetDiscardFam;
	s32  nDisCardNum;

#ifdef SHOW_RESULT
	Scalar Groupcol;
	uint8_t binitvote;
#endif

}obj_group;

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]	    bCleaTrajecy		  int8_t		      if 1 clear traject;

Realized function:
    + clear Group information;
*/
void mvClearGroupIfo(obj_group*pGroup,uint8_t bCleaTrajecy );//bCleaTrajecy =1

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group

Realized function:
    + Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
*/
void  mvMidGroupRec(obj_group*pGroup);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    pGroup		          obj_group*		  tracking Target group
[in/out]    pSclarSpace           float32_t*          scale ratio of different points
[in/out]    fscale                float32_t*          middle point of pSclarSpace
[in]        votestyle             VoteSort            vote style
[in]        nId                   s32                 tracking image sacle factor

[out]       returned              uint8_t             if can get suitable sacle factor return 1; else return 0

Realized function:
    + Caculate the scale factor of pGroup;
*/
uint8_t mvScaleVal(obj_group*pGroup, float32_t *pSclarSpace, float32_t *fscale, VoteSort votestyle, s32 nId);

void mvElimiNateInitTrack(obj_group*pGroup); 

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group

Realized function:
    + judge if the tracked point is suitable;
*/
void mvDelUnReasonTrack(obj_group*pGroup); 

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]        fShirkRat             float32_t           if >0, do shirking for pGroup->InitContour
[in]        nId                   s32                 sacle factor
[in]        nFramSeq              int16_t             frame Num

Realized function:
    + Init vote for traject that inside pGroup->rtContour;
*/
void mvEstablInitVote(obj_group*pGroup, float32_t fShirkRat, s32 nId, int64_t nFramSeq);//float32_t fShirkRat = 0.15f

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]        Rec                   AdasRect            rect reion

Realized function:
    + make pGroup->pObjtr->bInitTracked=0, if pGroup->pObjtr in Rec
*/
void mvclearInitState(obj_group*pGroup, AdasRect Rec);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group

Realized function:
    + update pGroup->Centr by pGroup->rtContour;
*/
void mvUpdataGroupCenTraj(obj_group*pGroup);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]        Rec                   AdasRect            rect reion

Realized function:
    + make pGroup->pObjtr->bProcessTracked=0, if pGroup->pObjtr inside of Rec
*/
void mvclearProcesState(obj_group*pGroup,AdasRect Rec);

void mvSetTrajLenthOne(obj_group*pGroup);



#endif
