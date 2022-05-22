#ifndef TYPE_DEF_H 
#define TYPE_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_type_def.h"

#ifdef TIME_TEST
#define  TIME_TAKE(a,b,c) Time_Take(a,b,c)
#endif


#define scale_shink_1_id 0  
#define scale_shink_2_id 1 
#define scale_shink_4_id 2 

typedef enum VoteSort
{
	ProcessVote =0,
	InitVote = 1,
	OrinitVote
}VoteSort;

typedef struct  MATCHSTATE
{
	s32 nGroupIndex[2];
	s32 nTrjIndexofGroup[2];
	s32 nMatchDis;
	uint8_t nMacthNum;
	
}MatchState;

typedef struct  ADASCORNER
{
	s32 val;
	s32 nCornerIndex;
	MatchState  State;
	int16_t x;
	int16_t y;

} AdasCorner;

typedef struct HISTREC
{
    s32 *pGroupSiz;
	int nSizNum;
}histrec;

typedef struct { s32 x, y; } xy; 

typedef struct SYSTIME
{
	u16 wHour;
	u16 wMin;
	u16 wSec;
	unsigned int wMilSec;
}systime;


typedef struct PROT_INPUT
{
	int64_t nFramSeq;
	float32_t fzoom;
	uint8_t *pGrayfram;
	AdasRect *objRec;
	s32 nRecNum;
	AdaSize imgSize;
	systime objTime;
	imgage pOriGrayfram;
}PortInput;


typedef struct TRACK_POINT
{
	int64_t nFramseq;
	int16_t nMatchStatus;
	systime    nTime;
	AdasPoint point;

} TrackPoint;


typedef struct MOTION
{

	AdasRect groupRec;

	float32_t  x_fdis;
	float32_t x_fSpeed;
	float32_t x_fAccelSpeed;

	float32_t  z_fdis;
	float32_t z_fSpeed;
	float32_t z_fAccelSpeed;

	float32_t delVanish;
	float32_t fTTC;
	float32_t dTTC;


	systime ts;
	uint8_t  bInCollishionPath;

} Motion;



typedef struct TRACK_OBJ
{
	int64_t nFramSeq;
	int64_t nId;
	AdasRect objRec;
	s32 DisToBottom;
	TrackPoint *pCenTrajecy;
	Motion  *pMotion;
	s32  nMotionLeng;
	float32_t fTTC;
	float32_t dTTC;
	s32 nTrackLen;
	uint8_t bTrue;
}trakobj;


typedef struct MULI_TRACKER
{
	trakobj *pTrackerset;
    s32  nTrackeNum;

}MuliTracker;


typedef struct VERF_OBJ
{
	AdasRect objRec;
	float* objData;
	int64_t nGroupID;
	uint8_t nObjState;
}verfobj;


typedef struct MULI_VERF_OBJ
{
	verfobj *objInput;
	int64_t nObjNum;
	imgage pOriGrayfram;
}MuliVerfobj;

typedef struct VERF_OBJ_RELT
{
	int64_t nGroupID;
	uint8_t nObjState;
}verfobjrelt;

typedef struct MULI_VERF_OBJ_RELT
{
	verfobjrelt *objInput;
	int64_t nObjNum;
}MuliVerfobjRelt;


typedef  AdasPoint VoteErorrVal;

#define DETECTOR_FRAM_STEP 15

#ifdef __cplusplus
}
#endif

#endif
