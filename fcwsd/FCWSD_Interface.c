/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.c
Version: 1.0		Date: 2017-02-22		Author: Yanming Wang		ID: 1407930

Description:
	FCWS is the abbreviation of Forward Collision Warning System, FCWSD discribs the detection function of FCWS.
	The functions in this file are defined as the interface functions of vehicle detection.
	The following function types are included:
	+ FCWSD_Init(): The initialization of variables and structures to be used in FCWSD functions.
	+ FCWSD_Processor(): The main procedure of vehicle detection. Given the buff of input image and detection 
	                     parameters, the locations of vehicle will be given as the output.
	+ FCWSD_GetResult(): Get the result of detected vehicle.
	+ FCWSD_Free(): Free the memory space of variables.

Deviation: 'FCWSD_' is used as the prefix of vehicle detection interface functions

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "utils.h"
#include "FCWSD_Interface.h"
#include "vehicle_type.h"
#include "vehicle_det.h"
#include "vehicle_proposals.h"
#include <stdlib.h>

/* Global parameters for detector */
static FCWSDetectorGlobalPara pVehicleDetor[DETECT_HANDLE_NUM] = {0};
static FCWSDetectorROI		  g_DetectDefaultROI[DETECT_HANDLE_NUM][ROI_SEGMENT_NUM];

/*
Function process:
	+ Init variables and structures to be used in FCWSD functions
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_InitVehicle()
	        + FCWD_InitVehicleDetprocess_Rio()
	ATTENTION: __________
*/
int FCWSD_Init(const int index, const FCWSDParam *pParam, const void *pLDWSOutput, const void *p3DMapOutput, const char *file)
{
	int ret = 0;
	FCWSDSize defaultMinObjectSize;
	FCWSDSize defaultMaxObjectSize;

	if (index < DETECT_HANDLE_NUM && index >= 0 
		&& pParam != NULL 
		&& pParam->startMFactor >= 1.0
		&& pParam->srcROIYFactor >= 0.0
		&& pParam->srcWidth != 0
		&& pParam->srcHeight != 0
		&& pParam->useFixedTaskNum >= 0
		&& pParam->useFixedTaskNum <= 1
		&& pParam->fixedTaskNum >=0)
	{
		if( pParam->useFixedTaskNum == 1 && pParam->fixedTaskNum ==0)
		{
			my_printf("Init failed, please check the pParam->fixedTaskNum! \n");
		    return 0;
		}
	
		pVehicleDetor[index].srcWidth			= pParam->srcWidth;
		pVehicleDetor[index].srcHeight			= pParam->srcHeight;
		pVehicleDetor[index].srcROIYFactor		= pParam->srcROIYFactor;
		pVehicleDetor[index].fixedTaskNum		= pParam->fixedTaskNum;
		pVehicleDetor[index].startMFactor		= pParam->startMFactor;
		pVehicleDetor[index].useFixedTaskNum	= pParam->useFixedTaskNum;
		pVehicleDetor[index].roi                = pParam->roi;

		pVehicleDetor[index].index = index;//detector index




#ifdef FCWS_DETECTOR_DEBUG
		pVehicleDetor[index].detframesUsed	= 1;
		pVehicleDetor[index].showFCWDebug	= 1;
#endif

		ret = FCWD_InitVehicle(&pVehicleDetor[index], pLDWSOutput, p3DMapOutput, file, pParam->scalingFactor, pParam->eps);

		pVehicleDetor[index].currentWindowSize  = pVehicleDetor[index].pdetorModel->l->data.featureWidth;

		if (ret == 0)
		{
			my_printf("Init failed, FCWD_InitVehicle, please check the param \n");
			return 0;
		}

		defaultMinObjectSize.width	= pVehicleDetor[index].pdetorModel->l->data.featureWidth;
		defaultMinObjectSize.height = pVehicleDetor[index].pdetorModel->l->data.featureHeight;
		pVehicleDetor[index].aspectRatio = (double) defaultMinObjectSize.width / defaultMinObjectSize.height;

		defaultMaxObjectSize.width	= (int) MIN((int)(pVehicleDetor[index].aspectRatio * (1 - pParam->srcROIYFactor) * pParam->srcHeight),pParam->srcWidth);
		defaultMaxObjectSize.height = (int) ((1 - pParam->srcROIYFactor) * pParam->srcHeight);

		ret = FCWD_InitVehicleDetProcess_Rio(&pVehicleDetor[index], index, &g_DetectDefaultROI[index][0], \
			                                 pLDWSOutput, NULL, &defaultMinObjectSize, &defaultMaxObjectSize);

		if (ret == 0)
		{
			my_printf("Init failed, FCWD_InitVehicleDetProcess_Rio, please check the param \n");
			return 0;
		}

#ifdef USE_PROPOSALS
		if (index == 0)
		{
			ret = initProposals(pParam->srcWidth, pParam->srcHeight, pParam->srcROIYFactor);

			if (ret == 0)
			{
				my_printf("Init failed, initProposals, please check the param \n");
				return 0;
			}
		}
#endif
	}
	else
	{
		my_printf("Init failed, please check the param\n");
		return 0;
	}

	return 1;
}

/*
Function process:
	+ The main procedure of vehicle detection.
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_VehicleDetProcess_Rio()
	ATTENTION: __________
*/
int FCWSD_Processor(const int index, const FCWSDImage *pOriGrayImg, const void *pLDWSOutput, const void *p3DMapOutput,
					FCWSDRect *roi, const FCWSDSize *minObjectSize, const FCWSDSize *maxObjectSize, const int groupThreshold, const int maxRT)
{
	int ret;
	if(roi != NULL)
	{
		/* roi should inside of pVehicleDetor[index].roi */
		roi->point.x = MAX(pVehicleDetor[index].roi.point.x, roi->point.x);
		roi->point.y = MAX(pVehicleDetor[index].roi.point.y, roi->point.y);
		roi->size.width = MIN(pVehicleDetor[index].roi.point.x + pVehicleDetor[index].roi.size.width, 
							  roi->point.x + roi->size.width) - roi->point.x;
		roi->size.height = MIN(pVehicleDetor[index].roi.point.y + pVehicleDetor[index].roi.size.height, 
							  roi->point.y + roi->size.height) - roi->point.y;
	}

	ret = FCWD_VehicleDetProcess_Rio(&pVehicleDetor[index], &g_DetectDefaultROI[index][0], pOriGrayImg, pLDWSOutput, NULL, roi, minObjectSize, maxObjectSize, groupThreshold, maxRT);

	return ret;
}

int  FCWSD_Processor_ROI(const int index, const int VerFlg, const FCWSDImage *pGrayImg, FCWSDRect *roi, \
								const FCWSDSize *minObjectSize, const FCWSDSize *maxObjectSize, const int groupThreshold)
{
	int ret;
	if(roi != NULL)
	{
		/* roi should inside of pVehicleDetor[index].roi */
		roi->point.x = MAX(pVehicleDetor[index].roi.point.x, roi->point.x);
		roi->point.y = MAX(pVehicleDetor[index].roi.point.y, roi->point.y);
		roi->size.width = MIN(pVehicleDetor[index].roi.point.x + pVehicleDetor[index].roi.size.width,
							  roi->point.x + roi->size.width) - roi->point.x;
		roi->size.height = MIN(pVehicleDetor[index].roi.point.y + pVehicleDetor[index].roi.size.height,
							  roi->point.y + roi->size.height) - roi->point.y;
	}

	ret = FCW_DETCOR_Vehicle_Rio(&pVehicleDetor[index], &g_DetectDefaultROI[index][0], pGrayImg,minObjectSize,maxObjectSize,roi,groupThreshold, VerFlg);
	return ret;
}


/*
Function process:
+ make sure DstRec inside of ImgRec
Fan-in :
+ mvReDetcorByRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
static void mvScopeImg(FCWSDRect *DstRec, const FCWSDRect ImgRec)
{
	if (DstRec->point.x < ImgRec.point.x)
	{
		DstRec->point.x = ImgRec.point.x;
	}

	if (DstRec->point.y < ImgRec.point.y)
	{
		DstRec->point.y = ImgRec.point.y;
	}

	if (DstRec->point.x + DstRec->size.width > ImgRec.point.x + ImgRec.size.width)
	{
		DstRec->size.width = ImgRec.point.x + ImgRec.size.width - 1 - DstRec->point.x;
	}

	if (DstRec->point.y + DstRec->size.height > ImgRec.point.y + ImgRec.size.height)
	{
		DstRec->size.height = ImgRec.point.y + ImgRec.size.height - 1 - DstRec->point.y;
	}
}


/*
Function process:
+ Do the re-det around the pSrcRect and get the new location as pDetRec
Fan-in :
+ mvReLoaclDetRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
void detcorByRec(const FCWSDImage GrayImg, const unsigned char dayOrNight)
{

	s32 i;
	objectCar pSrcRect;
	FCWSDRect  OriRioRec;
	s32 nExtend_x, nExtend_y;
	FCWSDSize fcwsDetMinSize;
	FCWSDSize fcwsDetMaxSize;
	FCWSDRect roi;

	/* get the global det result */
	objectSetsCar fcwsdInput;
	if (dayOrNight == 0)
	{
		fcwsdInput = pVehicleDetor[0].objSets;
	}
	else if (dayOrNight == 1)
	{
		fcwsdInput = pVehicleDetor[4].objSets;
	}

	for (i = 0; i < fcwsdInput.nObjectNum; ++i)
	{
		pSrcRect = fcwsdInput.objects[i];

		if (pSrcRect.objectRect.size.width > 0)
		{
			FCWSDRect dstRect;
			nExtend_x = (s32)(pSrcRect.objectRect.size.width * 0.2f);
			nExtend_y = (s32)(pSrcRect.objectRect.size.height * 0.2f);
			OriRioRec.point.x = (pSrcRect.objectRect.point.x - nExtend_x);
			OriRioRec.point.y = (pSrcRect.objectRect.point.y - nExtend_y);
			OriRioRec.size.width = (pSrcRect.objectRect.size.width + (nExtend_x << 1));
			OriRioRec.size.height = (pSrcRect.objectRect.size.height + (nExtend_y << 1));

			dstRect.point.x = 0;
			dstRect.point.y = 0;
			dstRect.size.width = GrayImg.nWid - 1;
			dstRect.size.height = GrayImg.nHig - 1;
			mvScopeImg(&OriRioRec, dstRect);

			fcwsDetMinSize.width = (s32)(pSrcRect.objectRect.size.width
				/ 1.2f - 1);
			fcwsDetMinSize.height = (s32)(pSrcRect.objectRect.size.height
				/ 1.2f - 1);

			fcwsDetMaxSize.width = (s32)(pSrcRect.objectRect.size.width
				* 1.2f + 1);
			fcwsDetMaxSize.height = (s32)(pSrcRect.objectRect.size.height
				* 1.2f + 1);

			roi.point.x = OriRioRec.point.x;
			roi.point.y = OriRioRec.point.y;
			roi.size.width = OriRioRec.size.width;
			roi.size.height = OriRioRec.size.height;

			if (dayOrNight == 0)
			{
				FCWSD_Processor_ROI(2, 1, &GrayImg, &roi, &fcwsDetMinSize,
					&fcwsDetMaxSize, 1);
				if (1 == pVehicleDetor[2].objSets.nObjectNum)
				{
					pVehicleDetor[0].objSets.objects[i].objectRect = pVehicleDetor[2].objSets.objects[0].objectRect;
				}
				else
				{
					pVehicleDetor[0].objSets.objects[i].confidence = -255;
				}
			}
			else if (dayOrNight == 1)
			{
				FCWSD_Processor_ROI(6, 1, &GrayImg, &roi, &fcwsDetMinSize,
					&fcwsDetMaxSize, 1);
				if (1 == pVehicleDetor[6].objSets.nObjectNum)
				{
					pVehicleDetor[4].objSets.objects[i].objectRect = pVehicleDetor[6].objSets.objects[0].objectRect;
				}
				else
				{
					pVehicleDetor[4].objSets.objects[i].confidence = -255;
				}
			}	

			
		}
	}
	
}

/*
Function process:
	+ Get the result of detected vehicle.
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_GetDetResult()
	ATTENTION: __________
*/
int FCWSD_GetResult(const int index, objectSetsCar **pFCWSDOutput)
{
	int ret;

	ret = FCWD_GetDetResult(&pVehicleDetor[index], pFCWSDOutput);
	
	return ret;
}

void getDetectObject(const int index, objectSetsCar **pFCWSOutput)
{
	*pFCWSOutput = &pVehicleDetor[index].objSets;
}

int FCWSD_RefineResult(objectSetsCar *pFCWSOutput, int imgWidth, int imgheight)
{
	double carHeight = 0, carWidth = 0;
	int j, heightNew, yNew1, widthNew, xNew1;
	if (pFCWSOutput != NULL)
	{
		for (j = 0 ;j < pFCWSOutput->nObjectNum; j++)
		{
			carHeight = LDWS_GetImageY((pFCWSOutput->objects[j].objectRect.point.y + pFCWSOutput->objects[j].objectRect.size.height), pFCWSOutput->objects[j].objectRect.point.y);

			carWidth = LDWS_GetDetaXofWorld(pFCWSOutput->objects[j].objectRect.size.width, pFCWSOutput->objects[j].objectRect.point.y+ pFCWSOutput->objects[j].objectRect.size.height);
			
   			heightNew = LDWS_GetCarY(pFCWSOutput->objects[j].objectRect.point.y+ pFCWSOutput->objects[j].objectRect.size.height, 1.8);
			yNew1 = pFCWSOutput->objects[j].objectRect.point.y + pFCWSOutput->objects[j].objectRect.size.height - heightNew;
			widthNew = pFCWSOutput->objects[j].objectRect.size.width * 1.8 / carWidth;
			xNew1 = 0.5 * ((2 * pFCWSOutput->objects[j].objectRect.point.x + pFCWSOutput->objects[j].objectRect.size.width) - widthNew);

			if(xNew1>0 && yNew1>0 && (xNew1 + widthNew)< imgWidth && (yNew1 + heightNew)< imgheight)
			{
				pFCWSOutput->objects[j].objectRect.point.x = xNew1;
				pFCWSOutput->objects[j].objectRect.point.y = yNew1;
				pFCWSOutput->objects[j].objectRect.size.width = MAX(widthNew,heightNew);
				pFCWSOutput->objects[j].objectRect.size.height = pFCWSOutput->objects[j].objectRect.size.width;
			}
			else
			{
				pFCWSOutput->objects[j].confidence = 0;
			}
		}
	}

	return(1);
}

/*
Function process:
	+ Free the result of detected vehicle.
	Fan-in : 
	        + main()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int FCWSD_FreeResult(objectSetsCar **pFCWSDOutput)
{
	if ((*pFCWSDOutput)->objects != NULL)
	{
		my_free((*pFCWSDOutput)->objects);
		(*pFCWSDOutput)->objects = NULL;
	}
	my_free(*pFCWSDOutput);
	*pFCWSDOutput = NULL;


    return 0;
}

/*
Function process:
	+ Free the memory space of variables.
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_UnitVehicle()
	ATTENTION: __________
*/
int FCWSD_Free(const int index)
{
	int ret;

	ret = FCWD_UnitVehicle(&pVehicleDetor[index], &g_DetectDefaultROI[index][0]);

#ifdef USE_PROPOSALS
	if (index == 0)
	{
		ret = freeProposals();
	}
#endif // USE_PROPOSALS

	return ret;
}

