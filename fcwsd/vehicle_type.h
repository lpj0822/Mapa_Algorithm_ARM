/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: vehicle_type.h
Version: 1.0		Date: 2017-02-22		Author: Yanming Wang		ID: 1407930

Description:
	This header file defines the structures and macros used in vehicle detection.
	
Deviation: N/A

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef VEHICLE_TYPE_H
#define VEHICLE_TYPE_H

#include <FCWSD_Interface.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Used for debug */
//#define FCWS_DETECTOR_DEBUG

/* If defined, use the result of LDWS for vehicle detection */
#define USE_LDWS

#define USE_PROPOSALS

#define ACC_DETECTOR 1

/* If defined, use GPU to speed up vehicle detection */
//#define USE_GPU

/* If defined, use openmp to speed up vehicle detection */
#define OPENMP

/* If defined, use NEON to speed up vehicle detection */
//#define NEON

#ifdef FCWS_DETECTOR_DEBUG
	#ifdef WIN32
		#define WIN32_FCWS_DETECTOR_DEBUG
	#else
		#define ANDROID_FCWS_DETECTOR_DEBUG
	#endif
#endif

#ifdef WIN32
	#undef NEON
	#undef NE10
#endif

#ifdef USE_LDWS
	#include <LDWS_Interface.h>
#endif

#ifdef USE_GPU
#ifdef WIN32
	#define FCWS_GPU_KERNEL_BFILE_PATH ("fcw.bin")
	#define FCWS_GPU_KERNEL_CFILE_PATH ("fcw.txt")
#elif defined __arm__
	#define FCWS_GPU_KERNEL_BFILE_PATH ("/data/fcw.bin")
	#define FCWS_GPU_KERNEL_CFILE_PATH ("/data/fcw.txt")
#endif
#endif

#ifdef NE10
	#include "NE10.h"
#endif

#ifdef NEON
	#include <arm_neon.h>
#endif

#ifndef uint8_t
	typedef  unsigned char         uint8_t;
#endif

#ifndef char_t
	typedef  char                  char_t;
#endif

#ifndef int16_t
	typedef  short                 int16_t;
#endif

#ifndef u32
	typedef  unsigned int          u32;
#endif

#ifndef s32
	typedef  int                   s32;
#endif

#ifndef u64
	typedef  unsigned long long    u64;
#endif

//#ifndef int64_t
#if 0
	typedef  long long             int64_t;
#endif

#ifndef float32_t
	typedef  float                 float32_t;
#endif

#ifndef float64_t
	typedef  double                float64_t;
#endif

#ifndef l32
	typedef  long                  l32;
#endif

typedef  long                  GroupId;

typedef						   unsigned char Bool;

#define MAX_IMAGE_WIDTH		(pVehicleDetor->srcWidth)
#define MAX_IMAGE_HIGHT		(pVehicleDetor->srcHeight)

#define MAX_TASKS_NUM		((MAX_IMAGE_WIDTH * MAX_IMAGE_HIGHT) >> 2)
//#define MAX_TASKS_NUM		(15000)
#define MAX_DETECT_NUM		(MAX_TASKS_NUM >> 3)
#define MAX_PUBLIC_BUFFER	((MAX_DETECT_NUM << 3) + 102400)
#define MAX_TREE_NODE		(MAX_DETECT_NUM)

/* The minimum size of width can be detected, decided by detector */
//#define MIN_DET_SIZE_WIDTH	(20)
/* The minimum size of height can be detected, decided by detector */
//#define MIN_DET_SIZE_HEIGHT	(20)

#define ALIGN_16BYTE(x)		((uint8_t*)(((unsigned long long)(x) + 15) >> 4 << 4))

/* Used in GPU speed up */
#define LOCKA_X_NUM			(16)
#define LOCKA_Y_NUM			(16)

/* Num of segments of detected region */
#define ROI_SEGMENT_NUM		(10)
#define ROI_SEGMENT_FACTOR	(0.1f) /* 1 / ROI_SEGMENT_NUM */

#ifndef SWAP
	#define SWAP(a,b,t)			((t) = (a), (a) = (b), (b) = (t))
#endif

#ifndef MIN
	#define MIN(a, b)			((a) > (b) ? (b) : (a))
#endif
#ifndef MAX
	#define MAX(a, b)			((a) > (b) ? (a) : (b))
#endif

#ifndef ABS
	#define ABS(x)				((x) >= 0 ? (x) : (-(x)))
#endif

#define myRound(value)		(s32)((value) + ( (value) >= 0 ? 0.5 : -0.5))

typedef struct LbpTaskCar 
{
	s32 x;
	s32 y;
}lbpTaskCar;

typedef struct LbpRectCar 
{
	s32 x;
	s32 y;
	s32 width;
	s32 height;
	s32 confidence;
}lbpRectCar;

typedef struct WeakClassifierCar
{
    s32*		lbpmap; /* Loading need to be singed */
	s32*		rectIdx;
	float32_t*	posiblity;
    s32*	    leafIdx;
}weakClassifierCar;

typedef struct LbpParaCar 
{
	float32_t	scalingFactor;
	float32_t	eps;
	s32			minSizeWidth;
}lbpParaCar;

typedef struct StageCar 
{
    weakClassifierCar   *classifiers;
    float32_t			stageThreshold;
	s32					weakClassifiersNum;
}stageCar;

typedef struct LbpDataCar
{
    stageCar		*s;
    lbpRectCar	    *r;
    s32				featureWidth;
    s32				featureHeight;
	s32             maxDepth;
	s32             leafNum;
    s32				stagesNum;
    s32				rectsNum;
	s32             totalWeakNum;
}lbpDataCar;

typedef struct lbpCar 
{
	lbpDataCar	    data;
    lbpParaCar	    para;
	lbpRectCar	    *pdetectedRect; /* Must use new/delete instead of malloc/free because of this */
    lbpTaskCar	    *ptasks;		/* Task to scan all size and positions  */

    s32				width;
    s32				height;
	s32				ndetdNum;
	s32				ntaskNum;
}lbpCar;

typedef struct ObjectDetCar 
{
	lbpCar  *l;
    u32		*integralImg;
    s32		width;
    s32		height;
}objectDetCar;

typedef struct DetectorGlobalPara
{
	objectSetsCar	objSets;

	uint8_t			*pAlloc;

#ifdef USE_GPU
	uint8_t			*pGpuBuff;
#endif
	uint8_t			*pbulicBuff;

	s32				*pTreeNode;  /* for grouped windows */
	s32				*pLabNode;   /* for grouped windows */

	objectDetCar	*pdetorModel;


	s32				vanishY;            /* Vanishing line Y coordinate */

	s32				srcWidth;
	s32				srcHeight;
	s32				fixedTaskNum;		/* Max num of tasks to be processed if useFixedTaskNum is set */
	s32				useFixedTaskNum;
	s32				currentWindowSize;	/* if useFixedTaskNum is set, detector works in this scale size  */
	float32_t		srcROIYFactor;
	float32_t		startMFactor;
	float32_t		aspectRatio;

#ifdef FCWS_DETECTOR_DEBUG
	s32		detframesUsed;				/* if useFixedTaskNum is set, how many frames can detect all the scales */
	s32		showFCWDebug;				/* show debug result */
#endif

#ifdef USE_LDWS
	LDWS_Point		*roiPoint;
	LDWS_Point		*roiPointCopy;
	s32				posInfor[ROI_SEGMENT_NUM];
#endif
	FCWSDRect       roi;
	uint8_t			initFlg;
	uint8_t         index;
}FCWSDetectorGlobalPara;

/* chained list for tasks */
typedef struct LbpTaskCarNode
{
	lbpTaskCar				task;
	struct LbpTaskCarNode	*next;
}LBPTaskCarList;

typedef struct NROWCOL
{
	s32 nrow[3];
	s32 ncol[3];
	s32 ncolAdd[3];
}nrowCol;

/* chained list for Multiscale tasks */
typedef struct DetectorMultiscaleNode
{
	FCWSDRect	roiRec;
	FCWSDSize	windowSize;
	float32_t	factor;
	s32			*arr_y; /* for integral image calculation */
	s32			*arr_x;

	nrowCol     *arrRowCol;

//#ifdef FCWS_DETECTOR_DEBUG
	s32           ntask;
//#endif

	LBPTaskCarList					*pLBPTCLhead;
	struct DetectorMultiscaleNode   *next;
}FCWSDetectorMultiscaleList;



typedef struct FCWSDetectorROI
{
	s32	     posInfor;
	FCWSDetectorMultiscaleList *pDMLhead;
}FCWSDetectorROI;

#ifdef USE_GPU

#define MAX_DET_NUM			(10240)
#define WEAK_CLASSIFER_NUM	(512)
#define MAX_NUM_STAGES		(64)
#define MAX_NUM_RECS		(40960)

typedef struct reduce_stage
{
	s32		num_weak_classifiers;
	float32_t	stage_threshold;
}reduce_stage;

typedef struct reduce_lbp_data 
{
	s32 num_stages;
	s32 num_rects;
	s32 feature_width;
	s32 feature_height;
}reduce_lbp_data;

typedef struct reduce_lbp 
{
	s32 width;
	s32 height;

	reduce_lbp_data data;
	lbp_para_car	para;
}reduce_lbp;
#endif

#endif
