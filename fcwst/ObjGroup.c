/**************************************************************************************************************
Copyright Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: ObjGroup.cpp
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

#include "ObjGroup.h"
#include "declare.h"
#include "trajectory.h"
#include "Geometry.h"

/*
Function process:
	+ clear Group information
	Fan-in : 
	        + mvPreditGroup()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvClearGroupIfo(obj_group*pGroup,uint8_t bCleaTrajecy)
{
	s32 i;

	pGroup->nGroupId = -1;
	pGroup->ntrajecyNum = 0;
	pGroup->Centr.nTrackLen = 0;
	pGroup->histoyRec.nSizNum = 0;
	pGroup->nPreProssVotFram = 0;
	pGroup->nTruelyObj = 0;
	pGroup->nStateNum = 0;

	//initial discard target
	pGroup->nDisCardNum = 0;

	pGroup->Templat.nWid = 0;
	pGroup->Templat.nHig = 0;
	pGroup->updataFrambyCar = -1;
	pGroup->SerpreditNum = 0;
    //TODO change for liding, maybe 9999
	pGroup->nMinCarWid = 255;
	pGroup->CarBottom.nDetBottomNum = 0;
	pGroup->CarBottom.nDetYPos = 0;
	pGroup->nMotionLeng = 0;
	pGroup->CarBottom.nDistoBottom = 0;
	pGroup->nBottomNum = 0;
	pGroup->nUpdataIndex = 0;
	pGroup->nLastupdataCarWidth = 0;

	if (bCleaTrajecy)
	{
		for (i = 0 ; i < MAX_TRACKS_NUM_OF_GROUP; i++)
		{
			mvClearTrajIfo(pGroup->pObjtr + i);
		}
	}
	
#ifdef SHOW_RESULT
	pGroup->Groupcol = Scalar(rand()%255,rand()%255,rand()%255);
#endif


}

/*
Function process:
	+ Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
	Fan-in : 
	        + mvGroupFilter()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvMidGroupRec(obj_group*pGroup)
{	
	 AdasPoint CenPoin;
	 s32 hisrec_wid[MAX_HISRORY_GROUP_REC_NUM];
	 s32 hisrec_Hei[MAX_HISRORY_GROUP_REC_NUM];
	 s32 SortNum = WS_MIN(pGroup->histoyRec.nSizNum,MAX_HISRORY_GROUP_REC_NUM);

	 CenPoin.x = pGroup->rtContour.x + (pGroup->rtContour.width >> 1);
	 CenPoin.y = pGroup->rtContour.y + (pGroup->rtContour.height >> 1);

	 memcpy(hisrec_wid,pGroup->histoyRec.pGroupSiz,sizeof(s32) * SortNum);
	 memcpy(hisrec_Hei,pGroup->histoyRec.pGroupSiz + MAX_HISRORY_GROUP_REC_NUM,sizeof(s32) * SortNum);

	 binSort_INT(hisrec_wid,SortNum);
	 binSort_INT(hisrec_Hei,SortNum);

	 pGroup->rtContour.width = hisrec_wid[SortNum>>1];
	 pGroup->rtContour.height = hisrec_Hei[SortNum>>1];
	 pGroup->rtContour.x =  CenPoin.x - (pGroup->rtContour.width>>1);
	 pGroup->rtContour.y =  CenPoin.y - (pGroup->rtContour.height>>1);
	
}

/*
Function process:
	+ Caculate the scale factor of pGroup
	Fan-in : 
	        + mvInitConsensVote()
	Fan-out:
	        + mvDisPow()
			+ binSort_FLOAT()
	ATTENTION: __________
*/
uint8_t mvScaleVal( obj_group*pGroup, float32_t *pSclarSpace, float32_t *fscale, VoteSort votestyle, s32 nId)
{
	s32 m,n;
	trajecy *pTrajecy = 0;
	trajecy *pOtherTrajecy = 0;
	TrackPoint *pA = 0;
	TrackPoint *pB = 0;
	uint8_t bJoin;
	s32 nSclaleNum = 0;
	float32_t fDis ;
	AdasRect shrinkRec;
	float32_t fShikRio = (scale_shink_4_id == nId) ? 0.2f: 0.1f;
	*fscale = 1.0f;

	shrinkRec.x = pGroup->InitContour.x + (s32)(pGroup->InitContour.width * fShikRio);
	shrinkRec.y = pGroup->InitContour.y + (s32)(pGroup->InitContour.height * fShikRio);
	shrinkRec.width = (s32)(pGroup->InitContour.width * (1 - fShikRio *2));
	shrinkRec.height = (s32)(pGroup->InitContour.height * (1 - fShikRio *2));

	if(InitVote == votestyle)
	{
		for ( m = 0; m < pGroup->ntrajecyNum -1; m++)
		{
			pTrajecy = pGroup->pObjtr + m;

			for (n = m + 1; n < pGroup->ntrajecyNum; n++)
			{

				pOtherTrajecy = pGroup->pObjtr + n;
				bJoin = pTrajecy->bInitTracked && !pTrajecy->nEstTimes;
				bJoin = bJoin && pOtherTrajecy->bInitTracked &&  !pOtherTrajecy->nEstTimes;

				pA = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT );
				pB = pOtherTrajecy->pTrackPoint + ( (pOtherTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT );

				if (bJoin && !mvAdaspointOutRect(&pA->point,&shrinkRec) && !mvAdaspointOutRect(&pB->point,&shrinkRec))
				{
					pSclarSpace[nSclaleNum] = (float32_t)mvDisPow(&pA->point,&pB->point );

					fDis = (float32_t)mvDisPow(&pTrajecy->InitPoint, &pOtherTrajecy->InitPoint);

					if (fDis > 0.0001f) 
					{
						pSclarSpace[nSclaleNum] = (pSclarSpace[nSclaleNum]/fDis);
						nSclaleNum++;
					}

				}

			}
		}

		if (nSclaleNum > 2 )
		{
			binSort_FLOAT(pSclarSpace, nSclaleNum); 
			*fscale = sqrtf(pSclarSpace[nSclaleNum>>1]);

			return 1;
		}
	}
	else if (ProcessVote == votestyle)
	{
		for ( m = 0; m < pGroup->ntrajecyNum -1; m++)
		{
			pTrajecy = pGroup->pObjtr + m;

			for (n = m + 1; n < pGroup->ntrajecyNum; n++)
			{
				pOtherTrajecy = pGroup->pObjtr + n;
				bJoin = pTrajecy->bProcessTracked && pTrajecy->nEstTimes < 2;
				bJoin = bJoin && pOtherTrajecy->bProcessTracked &&  pOtherTrajecy->nEstTimes < 2;

				if (bJoin)
				{
					pA = pTrajecy->pTrackPoint + ((pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT);
					pB = pOtherTrajecy->pTrackPoint + ((pOtherTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT);
					pSclarSpace[nSclaleNum] = (float32_t)mvDisPow(&pA->point,&pB->point );

					fDis = (float32_t)mvDisPow(&pTrajecy->ProcessPoint,&pOtherTrajecy->ProcessPoint);

					if (fDis > 0.0001f) 
					{
						pSclarSpace[nSclaleNum] = (pSclarSpace[nSclaleNum]/fDis);
						nSclaleNum++;
					}

				}

			}
		}

		if (nSclaleNum > 2 )
		{
			binSort_FLOAT(pSclarSpace,nSclaleNum); 
			*fscale = sqrtf(pSclarSpace[nSclaleNum>>1]);

			return 1;
		}
	}

	else if (OrinitVote == votestyle)
	{
		for ( m = 0; m < pGroup->ntrajecyNum -1; m++)
		{
			pTrajecy = pGroup->pObjtr + m;

			for (n = m + 1; n < pGroup->ntrajecyNum; n++)
			{
				pOtherTrajecy = pGroup->pObjtr + n;
				bJoin = pTrajecy->bOrInitVoteTracked ;
				bJoin = bJoin && pOtherTrajecy->bOrInitVoteTracked;

				if (bJoin)
				{
					pA = pTrajecy->pTrackPoint + ((pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT);
					pB = pOtherTrajecy->pTrackPoint + ((pOtherTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT);
					pSclarSpace[nSclaleNum] = (float32_t)mvDisPow(&pA->point,&pB->point );

					fDis = (float32_t)mvDisPow(&pTrajecy->OrinitPoint,&pOtherTrajecy->OrinitPoint);

					if (fDis > 0.0001f) 
					{
						pSclarSpace[nSclaleNum] = sqrtf(pSclarSpace[nSclaleNum]/fDis);
						nSclaleNum++;
					}

				}

			}
		}

		if (nSclaleNum > 2 )
		{
			binSort_FLOAT(pSclarSpace,nSclaleNum); 
			*fscale = pSclarSpace[nSclaleNum>>1];

			return 1;
		}
	}
	
	
	return 0;
}

void mvElimiNateInitTrack( obj_group*pGroup )
{
	s32 i;
	trajecy *pTrajecy = 0;
	TrackPoint *pTrackPoin;

	for ( i = 0; i < pGroup->ntrajecyNum;i++)
	{
		pTrajecy = pGroup->pObjtr + i;

		pTrackPoin = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT );

		if (pTrajecy->bInitTracked && mvAdaspointOutRect(&pTrackPoin->point,&pGroup->rtContour))
		{
			pTrajecy->bInitTracked = 0;
		}
	}

}

/*
Function process:
	+ judge if the tracked point is suitable
	Fan-in : 
	        + mvTrajectorymatch()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvDelUnReasonTrack( obj_group*pGroup )
{

	s32 m,n;
	trajecy *pTrajecy = 0;
	TrackPoint *pTrackPoin = 0;
	trajecy *pOtherTrajecy = 0;
	uint8_t bTrackFilter,bFind;

	for ( m = 0;  m < pGroup->ntrajecyNum;m++)
	{
		pTrajecy = pGroup->pObjtr + m;
		pTrackPoin = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT );

		bTrackFilter = ( mvAdaspointOutRect(&pTrackPoin->point,&pGroup->rtContour) ) || \
			(pTrajecy->nEstTimes > 3 );

		if (bTrackFilter)
		{
			bFind = 0;

			for ( n = pGroup->ntrajecyNum -1; n > m; n--)
			{
				pOtherTrajecy = pGroup->pObjtr + n;
				pTrackPoin =  pOtherTrajecy->pTrackPoint + ( ( pOtherTrajecy->nTrackLen - 1 ) & MAX_CORNER_OF_TRACK_BIT );

				bTrackFilter = ( mvAdaspointOutRect(&pTrackPoin->point,&pGroup->rtContour) ) || \
					(pOtherTrajecy->nEstTimes > 3 );

				if (!bTrackFilter)
				{
					bFind = 1;
					break;
				}

				mvClearTrajIfo(pOtherTrajecy);
				pGroup->ntrajecyNum--;
			}

			if (bFind)
			{
				mvCopyTrajecy(pTrajecy,pOtherTrajecy);
				mvClearTrajIfo(pOtherTrajecy);
				pGroup->ntrajecyNum--;
			}

		}

	}


}

/*
Function process:
	+ make pGroup->pObjtr->bInitTracked=0, if pGroup->pObjtr in Rec
	Fan-in : 
	        + mvPreditGroup()
	Fan-out:
	        + mvAdaspointInRect()
	ATTENTION: __________
*/
void mvclearInitState(obj_group*pGroup,AdasRect Rec)
{
	s32 m;
	trajecy *pTrajecy = 0;
	TrackPoint *Trackpin = 0;

	for ( m = 0;  m < pGroup->ntrajecyNum;m++)
	{
		pTrajecy = pGroup->pObjtr + m;

		Trackpin = pTrajecy->pTrackPoint + (( pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT);

		if (pTrajecy->bInitTracked && mvAdaspointInRect(&Trackpin->point,&Rec) )
		{
			pTrajecy->bInitTracked = 0;

		}	
	}
}

void mvSetTrajLenthOne(obj_group*pGroup)
{
	s32 m;
	trajecy *pTrajecy = 0;

	for ( m = 0;  m < pGroup->ntrajecyNum;m++)
	{
		pTrajecy = pGroup->pObjtr + m;
	
		if(pTrajecy->nTrackLen > 1)
		{ 
			pTrajecy->pTrackPoint[0] = pTrajecy->pTrackPoint[(pTrajecy->nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT];
		    pTrajecy->nTrackLen  =1;
		}
	}
}

/*
Function process:
	+ Init vote for traject that inside pGroup->rtContour
	Fan-in : 
	        + mvUpdataGroupByDet()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvEstablInitVote(obj_group* pGroup, float32_t fShirkRat, s32 nId, int64_t nFramSeq)
{
	s32 m;
	trajecy *pTrajecy = 0;
	TrackPoint *Trackpin = 0;
	AdasRect shrinkRec;
	
	pGroup->InitContour = pGroup->rtContour;
	
	if (fabs(fShirkRat)  < 0.0001f)
	{
		shrinkRec = pGroup->InitContour;

		for ( m = 0; m < pGroup->ntrajecyNum; m++)
		{
			pTrajecy = pGroup->pObjtr + m;


			Trackpin = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen - 1 ) &MAX_CORNER_OF_TRACK_BIT);

			if (mvAdaspointOutRect (&Trackpin->point,&shrinkRec) )
			{
				pTrajecy->bInitTracked = 0;

			}
			else
			{
				pTrajecy->bInitTracked = 1;
				
				mvInitVote(pTrajecy, pGroup->InitContour, (pTrajecy->nTrackLen-1)&MAX_CORNER_OF_TRACK_BIT );

			}

			pTrajecy->bProcessTracked = 0;

		}

	}
	else
	{
		shrinkRec.x = pGroup->InitContour.x + (s32)(pGroup->InitContour.width * fShirkRat);
		shrinkRec.y = pGroup->InitContour.y + (s32)(pGroup->InitContour.height * fShirkRat);
		shrinkRec.width =  (s32)(pGroup->InitContour.width * (1 - fShirkRat - fShirkRat) );
		shrinkRec.height = (s32)(pGroup->InitContour.height * (1 - fShirkRat -fShirkRat) );

		if ( nFramSeq == pGroup->CarBottom.nDetBottomFam )
		{
			shrinkRec.height = (s32)(pGroup->InitContour.height * (1 - fShirkRat) );
		}

		for ( m = 0;  m < pGroup->ntrajecyNum;m++)
		{
			pTrajecy = pGroup->pObjtr + m;


			Trackpin = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen -1) & MAX_CORNER_OF_TRACK_BIT );

			if (mvAdaspointOutRect (&Trackpin->point,&shrinkRec) )
			{
				pTrajecy->bInitTracked = 0;

			}
			else
			{
				pTrajecy->bInitTracked = 1;
				mvInitVote(pTrajecy,pGroup->InitContour,(pTrajecy->nTrackLen-1) & MAX_CORNER_OF_TRACK_BIT);

			}

			pTrajecy->bProcessTracked = 0;

		}

	}
	

#ifdef SHOW_RESULT
	pGroup->binitvote = 1;
#endif

}

/*
Function process:
	+ update pGroup->Centr by pGroup->rtContour
	Fan-in : 
	        + mvPreditGroup()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvUpdataGroupCenTraj( obj_group *pGroup)
{
	TrackPoint *point = 0;

	point = pGroup->Centr.pTrackPoint + (pGroup->Centr.nTrackLen&MAX_CORNER_OF_TRACK_BIT);
	point->point.x = pGroup->rtContour.x + (pGroup->rtContour.width>>1);
	point->point.y = pGroup->rtContour.y + (pGroup->rtContour.height>>1);
	point->nFramseq  = pGroup->nFramseq;
	pGroup->Centr.nTrackLen++;
}

/*
Function process:
	+ Set pTrajecy->bProcessTracked = 0 if traject point inside Rec
	Fan-in : 
	        + mvPreditGroup()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvclearProcesState(obj_group*pGroup,AdasRect Rec)
{
	s32 m;
	trajecy *pTrajecy = 0;
	TrackPoint *Trackpin = 0;

	for ( m = 0;  m < pGroup->ntrajecyNum;m++)
	{
		pTrajecy = pGroup->pObjtr + m;

		Trackpin = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen -1 ) & MAX_CORNER_OF_TRACK_BIT );

		if (pTrajecy->bProcessTracked && mvAdaspointInRect(&Trackpin->point,&Rec) )
		{
			pTrajecy->bProcessTracked = 0;

		}
	}
}
