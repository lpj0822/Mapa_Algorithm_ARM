#include "DISP_Interface.h"
#include "LDWS_Interface.h"
#include "rectifyImg.h"
#include "computeDisp.h"
#include "utils.h"


static RES_DISP outPutParams;
static cfgBM BMCfg;
static LDWS_InitGuid *pLDWSInit = NULL;
cfgRectify inPutParams;
int *L_pt1, *L_pt2;
int *R_pt1, *R_pt2;
unsigned short *L_w1, *L_w2, *L_w3, *L_w4;
unsigned short *R_w1, *R_w2, *R_w3, *R_w4;
unsigned char tab0[TABSZ];
unsigned char tab1[TABSK];

LDWS_Point ptVanish, ptOrg, ptTel;
float32_t slpOrg, slpTel;
extern float camera_height, limit_height, limit_dist;
extern int skyline_x;
extern int skyline_y;

int setDispParams()
{
	//参数初始化
	int x, nalloclSize;
	uint8_t *pTr = 0;
	
	//inPutParams.orgWidth = 960; 
	//inPutParams.orgHeight = 540; 
	//inPutParams.orgWidth = inPutParams.orgWidth * 2; 
	//inPutParams.orgHeight = inPutParams.orgHeight * 2; 
	//inPutParams.cutWidth = 576;
	//inPutParams.cutHeight = 324;
	//inPutParams.cutWidth = 640;
	//inPutParams.cutHeight = 256;
	//inPutParams.cutSx = 320; 
	//inPutParams.cutSy = 384;

	int i, j, k = 0;
	int CarWidth, CameraPosx, LeftDeviation, RoadWidth, cutSx;
	int nMinVah, nMinOrg, nMinTel;
	float32_t coeff;

	LDWS_Getinit(&pLDWSInit);
	
	CarWidth = (int)(LDWS_GetCarWidth() * 100);
	CameraPosx = (int)(pLDWSInit->Param[1] * 100);
	LeftDeviation = (int)(LDWS_GetLeftDeviation() * 100);
	RoadWidth = (int)(pLDWSInit->Param[0] * 100);
	camera_height = (float)LDWS_GetCameraHeight();

	limit_height = 2.5;
	limit_dist = 100;
	skyline_y = 10;
	nMinVah = WS_INT_MAX;
	nMinOrg = WS_INT_MAX;
    inPutParams.orgWidth = 640;
	inPutParams.orgHeight = 360;
	inPutParams.cutWidth = 320;
	inPutParams.cutHeight = 128;
	inPutParams.isUseInterpolation = 1;

	coeff = (CameraPosx - (CarWidth / 2 + LeftDeviation)) * 1.0f / RoadWidth;

	ptOrg.x = pLDWSInit->pBoundPoint[3].x + (pLDWSInit->pBoundPoint[2].x - pLDWSInit->pBoundPoint[3].x) * coeff;
	ptOrg.y = inPutParams.orgHeight * 2;
	ptTel.x = ptOrg.x + (pLDWSInit->pBoundPoint[2].x - pLDWSInit->pBoundPoint[3].x) * (CarWidth * 1.0f / RoadWidth);
	ptTel.y = inPutParams.orgHeight * 2;
	LDWS_GetVanishPointSet(&ptVanish);
	slpOrg = (ptVanish.x - ptOrg.x) * 1.0f / (ptVanish.y - ptOrg.y);
	slpTel = (ptVanish.x - ptTel.x) * 1.0f / (ptVanish.y - ptTel.y);

	for (j = 0; j < inPutParams.orgHeight * 2; j++)
	{
		for (i = 0; i < inPutParams.orgWidth * 2; i++)
		{
			int L_x = WS_MIN(WS_MAX(m11[k], 0), inPutParams.orgWidth * 2 - 1);
			int L_y = WS_MIN(WS_MAX(m12[k], 0), inPutParams.orgHeight * 2 - 1);
			int index_xy_C = (L_y) * inPutParams.orgWidth * 2 + L_x;
			int indexTop = ptVanish.y * inPutParams.orgWidth * 2 + ptVanish.x;

			if (WS_ABS(index_xy_C - indexTop) < nMinVah)
			{
				nMinVah = WS_ABS(index_xy_C - indexTop);
				inPutParams.cutSy = WS_MIN(WS_MAX(j / 2 - skyline_y, 0), inPutParams.orgHeight - inPutParams.cutHeight - 1);
				cutSx = WS_MIN(WS_MAX(i / 2, 0), inPutParams.orgWidth - 1);
			}

			k++;
		}
	}
	inPutParams.cutSx = WS_MIN(WS_MAX(cutSx - inPutParams.cutWidth / 2, 0), inPutParams.orgWidth - inPutParams.cutWidth - 1);
	skyline_x = inPutParams.cutWidth / 2;

	nalloclSize = inPutParams.orgWidth * inPutParams.orgHeight * sizeof(float) * 4 +\
		          inPutParams.orgWidth * inPutParams.orgHeight * sizeof(int) * 4 +\
				  inPutParams.orgWidth * inPutParams.orgHeight * sizeof(unsigned short) * 8 +\
				  inPutParams.orgWidth * inPutParams.orgHeight * sizeof(float) * 4 +\
				  inPutParams.cutWidth * inPutParams.cutHeight * sizeof(unsigned char) * 2 +\
				  inPutParams.cutWidth * inPutParams.cutHeight * sizeof(short);

	nalloclSize += (1024 << 8);
	inPutParams.m_pAlloc = (unsigned char* )my_malloc(nalloclSize);
	pTr = inPutParams.m_pAlloc;


	//LUT抽行抽列
	inPutParams.RTF_Lx = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.RTF_Lx + inPutParams.orgWidth * inPutParams.orgHeight);

	inPutParams.RTF_Ly = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.RTF_Ly + inPutParams.orgWidth * inPutParams.orgHeight);

	inPutParams.RTF_Rx = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.RTF_Rx + inPutParams.orgWidth * inPutParams.orgHeight);

	inPutParams.RTF_Ry = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.RTF_Ry + inPutParams.orgWidth * inPutParams.orgHeight);

	resizeLUT(inPutParams);

	//LUT申请内存
	L_pt1 = (int* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(L_pt1 + inPutParams.orgWidth * inPutParams.orgHeight);

	L_pt2 = (int* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(L_pt2 + inPutParams.orgWidth * inPutParams.orgHeight);

	L_w1 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(L_w1 + inPutParams.orgWidth * inPutParams.orgHeight);

	L_w2 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(L_w2 + inPutParams.orgWidth * inPutParams.orgHeight);

	L_w3 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(L_w3 + inPutParams.orgWidth * inPutParams.orgHeight);

	L_w4 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(L_w4 + inPutParams.orgWidth * inPutParams.orgHeight);

	R_pt1 = (int* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(R_pt1 + inPutParams.orgWidth * inPutParams.orgHeight);

	R_pt2 = (int* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(R_pt2 + inPutParams.orgWidth * inPutParams.orgHeight);

	R_w1 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(R_w1 + inPutParams.orgWidth * inPutParams.orgHeight);

	R_w2 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(R_w2 + inPutParams.orgWidth * inPutParams.orgHeight);

	R_w3 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(R_w3 + inPutParams.orgWidth * inPutParams.orgHeight);

	R_w4 = (unsigned short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(R_w4 + inPutParams.orgWidth * inPutParams.orgHeight);

	extractLUT(inPutParams);

	//反投影
	inPutParams.INV_Rx = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.INV_Rx + inPutParams.orgWidth * inPutParams.orgHeight);

	inPutParams.INV_Ry = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.INV_Ry + inPutParams.orgWidth * inPutParams.orgHeight);

	inPutParams.INV_Lx = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.INV_Lx + inPutParams.orgWidth * inPutParams.orgHeight);

	inPutParams.INV_Ly = (float* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(inPutParams.INV_Ly + inPutParams.orgWidth * inPutParams.orgHeight);

	INVRectifyImg(inPutParams);

	//计算视差参数
	BMCfg.preFilterCap = 60; 
	BMCfg.SADWindowSize = 11; 
	BMCfg.minDisparity = 0;
	BMCfg.numDisparities = 32;
	BMCfg.uniquenessRatio = 30;
	BMCfg.textureThreshold = 200;
	BMCfg.speckleRange = 32;
	BMCfg.speckleWindowSize = 200;
	BMCfg.width = inPutParams.cutWidth;
	BMCfg.height = inPutParams.cutHeight;
	BMCfg.ompNum = 2;

	for (x = 0; x < TABSZ; x++)
		tab0[x] = (unsigned char)abs(x - BMCfg.preFilterCap);
	for (x = 0; x < TABSK; x++)
		tab1[x] = (unsigned char)(x - OFS < -BMCfg.preFilterCap ? 0 : x - OFS > BMCfg.preFilterCap ? BMCfg.preFilterCap * 2 : x - OFS + BMCfg.preFilterCap);

	//
	outPutParams.isDone = 0;
	outPutParams.width = inPutParams.cutWidth;
	outPutParams.height = inPutParams.cutHeight;
	outPutParams.dis_Z = NULL;

	outPutParams.imgL = (unsigned char* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(outPutParams.imgL + inPutParams.cutWidth * inPutParams.cutHeight);

	outPutParams.imgR = (unsigned char* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(outPutParams.imgR + inPutParams.cutWidth * inPutParams.cutHeight);

	outPutParams.disp = (short* ) pTr;
	pTr = ADAS_ALIGN_16BYTE(outPutParams.disp + inPutParams.cutWidth * inPutParams.cutHeight);

	return 1;
}

void setereoMatch(unsigned char* left, unsigned char* right)
{
	//int i, j;

	//step1: distortion correction
	rectifyImg(left, right, outPutParams.imgL, outPutParams.imgR, inPutParams);

	//step2: right image translation
	//for (i = 0; i < BMCfg.height; i++)
	//	for (j = 0; j < BMCfg.width - 2; j++)
	//		outPutParams.imgR[i * BMCfg.width + j] = outPutParams.imgR[i * BMCfg.width + j + 2];

	//step3: calculate disparity
	computeDisp(outPutParams.imgL,  outPutParams.imgR, outPutParams.disp, BMCfg);

	//step4: calculate distance
	//for (i = 0; i < BMCfg.height; i++)
	//{
	//	for (j = 0; j < BMCfg.width; j++)
	//	{
	//		if (outPutParams.disp[i * BMCfg.width + j] < 48)
	//			outPutParams.disp[i * BMCfg.width + j] /= 3;
	//		else
	//			outPutParams.disp[i * BMCfg.width + j] -= 32;
	//	}
	//}
}

void getDispResult( RES_DISP* params )
{
	// 获取结果
	*params = outPutParams;
}

void releaseDispBuff()
{
	my_free(inPutParams.m_pAlloc);
}

