/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: common_type_def.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:
	This file define the commonly used types for tracking.

ATTENTION: 

Deviation: 

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef COMMON_TYPE_DEF_H 
#define COMMON_TYPE_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include<math.h>
#include <sys/types.h>


#define  INPUT_YUV


#ifndef uint8_t
typedef  unsigned char         uint8_t;
#endif

#ifndef char_t
typedef  char                  char_t; 
#endif

#ifndef int16_t
typedef  short                 int16_t;
#endif

#ifndef u16
typedef unsigned short          u16;
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

typedef struct ADASPOINT
{
	int16_t x;
	int16_t y;//int16_t
}AdasPoint;

typedef struct ADASPOINT_2F
{
	float32_t x;
	float32_t y;
}AdasPoint2f;

typedef  AdasPoint Vote;
typedef  unsigned char Bool;

typedef  enum  OBJTYPE
{
	Car =0,
	People,
	Unknow
}objtype;

typedef struct  ADASRECT
{
	objtype nType;
	int16_t x;
	int16_t y;
	int16_t width;
	int16_t height;
	int16_t confidence;
	int64_t nFramSeq;
} AdasRect;

typedef struct  ADASRECT_FLOAT
{
	int64_t nFramSeq;
	objtype nType;
	float32_t x;
	float32_t y;
	float32_t width;
	float32_t height;
	int16_t confidence;
} AdasRect_f;

typedef struct ADASIZE
{
	int16_t width;
	int16_t height;
}AdaSize;

typedef struct CAN_INFO
{
	float32_t fSpeed;
	float32_t fPretendingAngle;
}CANInfo;

typedef struct IMAGE
{
	uint8_t *ptr;
#ifdef INPUT_YUV
	uint8_t *putr;
	uint8_t *pvtr;
#endif
	int16_t nWid;
	int16_t nHig;
	uint8_t nChannle;
	uint8_t groundValue; 
	uint8_t dayOrNight;
	CANInfo CANData;
}imgage;

#define SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))

#ifndef WS_MIN
#define WS_MIN(a, b)   ((a) > (b) ? (b) : (a))

#endif
#ifndef WS_MAX
#define WS_MAX(a, b)   ((a) > (b) ? (a) : (b))
#endif
#ifndef WS_ABS
#define WS_ABS(x) ((x) >= 0 ? (x) : (-(x)))
#endif

#define    CMEMCPY(dst,src,len) memcpy((dst),(src),(len))
#define    CMEMSET(dst,val,len) memset((dst),(val),(len))

//#if __x86_64
#define ADAS_ALIGN_16BYTE(x) ((uint8_t*)(((unsigned long long)(x) + 15) >> 4 << 4))

#define ADAS_ALIGN_16BYTE_SIZE(x)   ((((unsigned long long)(x) + 15) >> 4) << 4)
//#else
//#define ADAS_ALIGN_16BYTE(x) ((uint8_t*)(((unsigned int)(x) + 15) >> 4 << 4))
//#define ADAS_ALIGN_16BYTE_SIZE(x)   ((((unsigned int)(x) + 15) >> 4) << 4)
//#endif

#ifdef __cplusplus
}
#endif

#endif
