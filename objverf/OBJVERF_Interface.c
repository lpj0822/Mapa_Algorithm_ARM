#include "OBJVERF_Interface.h"
#include "stdlib.h"
#include "utils.h"

#define  MAX_PUBLIC_SPACE_SIZE  314572

static MuliVerfobjRelt pMuliVerfobj;
MuliVerfobjRelt pVerfobjOutput;

//resize
static int *resizeArrayX[2] = { NULL };
static int *resizeArrayY[2] = { NULL };

/*
Function process:
+ make sure DstRec inside of ImgRec
Fan-in : 
+ mvReDetcorByRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
static void mvScopeImg( AdasRect *DstRec,const AdasRect ImgRec)
{
	if (DstRec->x < ImgRec.x)
	{
		DstRec->x = ImgRec.x;
	}

	if (DstRec->y < ImgRec.y)
	{
		DstRec->y = ImgRec.y;
	}

	if (DstRec->x + DstRec->width > ImgRec.x + ImgRec.width)
	{
		DstRec->width = ImgRec.x + ImgRec.width -1 - DstRec->x ;
	}

	if (DstRec->y + DstRec->height > ImgRec.y + ImgRec.height)
	{
		DstRec->height = ImgRec.y + ImgRec.height - 1 - DstRec->y ;
	} 
}

static void resizeObjectData(const imgage pOriGrayfram, const AdasRect objRec, const int resizeWidth, const int resizeHeight, int *tempX, int *tempY, float* objData)
{
	const int imageWidth = pOriGrayfram.nWid;
	int row = 0;
	int col = 0;
	float factorWidth = (float)objRec.width / resizeWidth;
	float factorHeight = (float)objRec.height / resizeHeight;
	const uint8_t* grayImage = pOriGrayfram.ptr + imageWidth * objRec.y;
	const uint8_t* tempSrc = NULL;
	float* tempDst = NULL;
	for (col = 0; col < resizeWidth; col++)
	{
		tempX[col] = (int)(col * factorWidth);
	}
	for (row = 0; row < resizeHeight; row++)
	{
		tempY[row] = (int)(row * factorHeight);
	}
	for (row = 0; row < resizeHeight; row++)
	{
		tempSrc = grayImage + imageWidth * tempY[row] + objRec.x;
		tempDst = objData + resizeWidth * row;
		for (col = 0; col < resizeWidth; col++)
		{
			tempDst[col] = *(tempSrc + tempX[col]) - 99.0f;
		}
	}
}

//flag 0-->ObjVerf 1-->CNN
void MuliObjVerfInit(const int resizeWidth, const int resizeHeight, const int flag)
{
	if (flag == 0)
	{
		pMuliVerfobj.objInput = (verfobjrelt *)malloc(sizeof(verfobjrelt)* MAX_TRACK_MUM);
		pVerfobjOutput.objInput = (verfobjrelt *)malloc(sizeof(verfobjrelt)* MAX_TRACK_MUM);
		if (resizeArrayX[0] == NULL)
		{
			resizeArrayX[0] = (int*)malloc(sizeof(int)* resizeWidth);
		}
		if (resizeArrayY[0] == NULL)
		{
			resizeArrayY[0] = (int*)malloc(sizeof(int)* resizeHeight);
		}
	}
	else if (flag == 1)
	{
		if (resizeArrayX[1] == NULL)
		{
			resizeArrayX[1] = (int*)malloc(sizeof(int)* resizeWidth);
		}
		if (resizeArrayY[1] == NULL)
		{
			resizeArrayY[1] = (int*)malloc(sizeof(int)* resizeHeight);
		}
	}
}

/*
Function process:
+ verify the history target in m_globlparam[nId].m_pGroupSets
Fan-in :
+ mvSingledScaleTrack()
Fan-out:
+ FCWSD_Processor_ROI()
+ mvSureTrueGroup()
ATTENTION: __________
*/
void verfObjByDetcor(MuliVerfobj *pVerfobjInput) {

	s32 i;
	AdasRect RioRec;
	uint8_t nDetCarNum = 0;
	FCWSDSize fcwsDetMinSize;
	FCWSDSize fcwsDetMaxSize;
	FCWSDRect roi;
	FCWSDImage imgRioTemp;

	for (i = 0; i < pVerfobjInput->nObjNum; i++) 
	{
		RioRec = pVerfobjInput->objInput[i].objRec;

		if (RioRec.width * RioRec.height < MAX_PUBLIC_SPACE_SIZE)
		{
			AdasRect dstRect;
			RioRec.x -= RioRec.width * 0.2f;
			RioRec.y -= RioRec.height * 0.2f;
			RioRec.width += RioRec.width * 0.4f;
			RioRec.height += RioRec.height * 0.4f;

			dstRect.x = 0;
			dstRect.y = 0;
			dstRect.width = pVerfobjInput->pOriGrayfram.nWid - 1;
			dstRect.height = pVerfobjInput->pOriGrayfram.nHig - 1;
			mvScopeImg(&RioRec, dstRect);

			fcwsDetMinSize.width = (s32) (pVerfobjInput->objInput[i].objRec.width
				/ 1.2f - 1);
			fcwsDetMinSize.height = (s32) (pVerfobjInput->objInput[i].objRec.height
				/ 1.2f - 1);

			fcwsDetMaxSize.width = (s32) (pVerfobjInput->objInput[i].objRec.width
				* 1.2f + 1);

			fcwsDetMaxSize.height = (s32) (pVerfobjInput->objInput[i].objRec.height
				* 1.2f + 1);

			roi.point.x = RioRec.x;
			roi.point.y = RioRec.y;
			roi.size.width = RioRec.width;
			roi.size.height = RioRec.height;

			imgRioTemp.ptr = pVerfobjInput->pOriGrayfram.ptr;
			imgRioTemp.nWid = pVerfobjInput->objInput[i].objRec.width;
			imgRioTemp.nHig = pVerfobjInput->objInput[i].objRec.height;


			if (pVerfobjInput->pOriGrayfram.dayOrNight == 0)
			{
				nDetCarNum = FCWSD_Processor_ROI(1, 0, &imgRioTemp, &roi,
					&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			}
			else if (pVerfobjInput->pOriGrayfram.dayOrNight == 1)
			{
				nDetCarNum = FCWSD_Processor_ROI(5, 0, &imgRioTemp, &roi,
					&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			}
		}
		else
		{
			nDetCarNum = 0;
		}

		pVerfobjInput->objInput[i].nObjState = nDetCarNum ? 1 : 0;
	}
}


void updataMuliObjVerf(MuliVerfobj *pVerfobjInput )
{
	int i;
	pMuliVerfobj.nObjNum = pVerfobjInput->nObjNum;
	for (i = 0; i < pMuliVerfobj.nObjNum; i++)
	{
		pMuliVerfobj.objInput[i].nGroupID = pVerfobjInput->objInput[i].nGroupID;
		pMuliVerfobj.objInput[i].nObjState = pVerfobjInput->objInput[i].nObjState;
	}
}

void getMuliObjVerf( )
{
	int i;
	pVerfobjOutput.nObjNum = pMuliVerfobj.nObjNum;
	for ( i = 0; i < pMuliVerfobj.nObjNum; i++)
	{
		pVerfobjOutput.objInput[i].nGroupID = pMuliVerfobj.objInput[i].nGroupID;
		pVerfobjOutput.objInput[i].nObjState = pMuliVerfobj.objInput[i].nObjState;
	}
}

//flag 0-->ObjVerf 1-->CNN
void MuliObjVerfRelease(const int flag)
{
	if (flag == 0)
	{
		if (pMuliVerfobj.objInput != NULL)
		{
			free(pMuliVerfobj.objInput);
			pMuliVerfobj.objInput = NULL;
		}
		if (pVerfobjOutput.objInput != NULL)
		{
			free(pVerfobjOutput.objInput);
			pVerfobjOutput.objInput = NULL;
		}
		if (resizeArrayX[0] != NULL)
		{
			free(resizeArrayX[0]);
			resizeArrayX[0] = NULL;
		}
		if (resizeArrayY[0] != NULL)
		{
			free(resizeArrayY[0]);
			resizeArrayY[0] = NULL;
		}
	}
	else if (flag == 1)
	{
		if (resizeArrayX[1] != NULL)
		{
			free(resizeArrayX[1]);
			resizeArrayX[1] = NULL;
		}
		if (resizeArrayY[1] != NULL)
		{
			free(resizeArrayY[1]);
			resizeArrayY[1] = NULL;
		}
	}
}

void resizeVeryObject(const int resizeWidth, const int resizeHeight, MuliVerfobj *pVerfobjInput)
{
	int objectIndex = 0;
	for (objectIndex = 0; objectIndex < pVerfobjInput->nObjNum; objectIndex++)
	{
		resizeObjectData(pVerfobjInput->pOriGrayfram, pVerfobjInput->objInput[objectIndex].objRec, 
			resizeWidth, resizeHeight, resizeArrayX[0], resizeArrayY[0], pVerfobjInput->objInput[objectIndex].objData);
	}
}

void resizeFCWSDObject(const int resizeWidth, const int resizeHeight, const FCWSDImage *pOriGrayImg, const objectCar rect, float* objectData)
{
	const int imageWidth = pOriGrayImg->nWid;
	int row = 0;
	int col = 0;
	float factorWidth = (float)rect.objectRect.size.width / resizeWidth;
	float factorHeight = (float)rect.objectRect.size.height / resizeHeight;
	const uint8_t* grayImage = pOriGrayImg->ptr + imageWidth * rect.objectRect.point.y;
	const uint8_t* tempSrc = NULL;
	float* tempDst = NULL;
	int* tempX = resizeArrayX[1];
	int* tempY = resizeArrayY[1];
	for (col = 0; col < resizeWidth; col++)
	{
		tempX[col] = (int)(col * factorWidth);
	}
	for (row = 0; row < resizeHeight; row++)
	{
		tempY[row] = (int)(row * factorHeight);
	}
	for (row = 0; row < resizeHeight; row++)
	{
		tempSrc = grayImage + imageWidth * tempY[row] + rect.objectRect.point.x;
		tempDst = objectData + resizeWidth * row;
		for (col = 0; col < resizeWidth; col++)
		{
			tempDst[col] = *(tempSrc + tempX[col]) - 99.0f;
		}
	}
}

