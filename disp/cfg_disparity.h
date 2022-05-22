#ifndef _CFG_DISPARITY_H_
#define _CFG_DISPARITY_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CFG_RECTIFY
{
	unsigned short orgWidth;
	unsigned short orgHeight;
	unsigned short cutSx;
	unsigned short cutSy;
	unsigned short cutWidth;
	unsigned short cutHeight;

	unsigned char *m_pAlloc;
	float* RTF_Lx;
	float* RTF_Ly;
	float* RTF_Rx;
	float* RTF_Ry;

	float* INV_Lx;
	float* INV_Ly;
	float* INV_Rx;
	float* INV_Ry;

	int isUseInterpolation;
	int isExtract;
}cfgRectify;

typedef struct CFG_BM
{
	int preFilterCap;
	int SADWindowSize;
	int minDisparity;
	int numDisparities;
	int uniquenessRatio;
	int textureThreshold;
	int speckleRange;
	int speckleWindowSize;
	int width;
	int height;
	int ompNum;
}cfgBM;

typedef struct RES_DISP
{
	int  isDone;
	unsigned char *imgL;
	unsigned char *imgR;
	short *disp;
	int width;
	int height;
	unsigned short *dis_Z;
}RES_DISP;

typedef struct WS_POINT
{
	short x;
	short y;
}wsPoint;


#define  WS_INT_MAX   2147483647    /* maximum (signed) int value */

#define    TABSZ       256
#define    TABSK       2304
#define    OFS         1024





#ifdef __cplusplus
}
#endif

#endif 
