/**************************************************************************************************************
Copyright 漏 Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: vehicle_det.h
Version: 1.0		Date: 2017-02-22		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are the realization of vehicle detection.
	The following function types are included:
	+ FCWD_InitVehicle(): The initialization of variables and structures to be used in FCWSD functions.
	+ FCWD_UnitVehicle(): Free the memory space of variables.

	+ FCWD_InitVehicleDetprocess_Rio():Init tasks for detector
	+ FCWD_VehicleDetprocess_Rio(): Main realization for vehicle detection
	+ FCWD_GetDetResult(): Get the detector results

	static function:
	+ initTaskWindowsCar(): Assign the scanning windows (tasks) for chained list
	+ lbpDetectWindowsCar(): Detect scanning windows for one single scaling factor
	+ getResizeIntegralImage(): Calculate the integral image
	+ userGetTime(): Return system time (ms)
	+ lbpGroupWindowsCar(): Merge overlapped detect rectangles
	+ freeTaskWindowsCar(): Free the memory of single scale tasks chained list
	+ freeVehicleDetProcess_Rio(): Free the memory of multi-scale detector chained list

Deviation: 'FCWSD_' is used as the prefix of vehicle detection functions in FCWS

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "utils.h"
#include "vehicle_type.h"
#include "vehicle_det.h"
#include "lbp_detect.h"
#include "group_rect.h"
#include "vehicle_proposals.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef USE_GPU
	//#include "GPULbpDetect.h"
     #include "gpu.h"
#endif

nrowCol arrRowCol[15 * 1024];
/*
I/O:	    Name		    Type	                       Size			  	                      Content
					    								  
[in/out]	p		        FCWSDetectorMultiscaleList*	   sizeof(FCWSDetectorMultiscaleList*)	  chained list for Multiscale tasks.
[in]	    step		    const s32	                   4-Byte                                 step for scanning window.
[in]	    minObjectSize	const FCWSDSize*		       sizeof(FCWSDSize*)                     Minimum size for detector.
					    								  
[out]	returned value  int          4-Byte	              If 0, initialized failed.

Realized function:
	+ Assign the scanning windows (tasks) for chained list.
*/
static int	initTaskWindowsCar( FCWSDetectorMultiscaleList *p, const s32 step, const FCWSDSize *minObjectSize, const int sizeLimit);

/*
I/O:	 Name		    Type	       Size			  	    Content
		 			    			  					    
[in/out] l		        lbpCar*	       sizeof(lbpCar*)	    Structural for detector
[in]     img            u32*           sizeof(u32*)         integral image
[in]     factor         float32_t      sizeof(float32_t)    Scaling factor for the detect window
[in]     roiFlg         unsigned char  sizeof(uchar)        if roiFlg ==1, detector is used for ROI region
					    								    
[out]	returned value  s32            4-Byte	            Number of detected objects.

Realized function:
	+ Detect scanning windows for one single scaling factor.
*/
static u32  lbpDetectWindowsCar( lbpCar *l, u32 *img, const float32_t factor, const unsigned char roiFlg, const nrowCol *arrRowColDet);

/*
I/O:	  Name		    Type	                    Size			  	                  Content
		 			    			  					    
[in]      pOriGrayImg	FCWSDImage*	                sizeof(FCWSDImage*)   	              origin gray image
[in/out]  iData         u32*                        sizeof(u32*)                          integral image
[in]      detROI        FCWSDRect*                  sizeof(FCWSDRect*)                    ROI region for calculate integral image
[in]      p             FCWSDetectorMultiscaleList  sizeof(FCWSDetectorMultiscaleList*)   chained list for Multiscale tasks.
					    								    
[out]	returned value  s32                         4-Byte	                              if 0, function failed.

Realized function:
	+ Calculate the integral image.
*/
static u32	getResizeIntegralImage(const FCWSDImage *pOriGrayImg, u32 *iData, const FCWSDRect *detROI, const FCWSDetectorMultiscaleList *p);

/*
I/O:	  Name		    Type	                    Size			  	                  Content
		 			    			  					    					    								    
[out]	returned value  double                      8-Byte	                              Return system time.

Realized function:
	+ Return system time (ms).
*/
static double userGetTime(void);

/*
I/O:	   Name		        Type	                 Size			  	      Content

[in]       pVehicleDetor    FCWSDetectorGlobalPara*                           Global parameter for detector
[in]       l		        lbpCar*	                 sizeof(lbpCar*)          LBP structure for vehicle detector 
[in/out]   pobjSets		    objectSetsCar*	         sizeof(objectSetsCar*)	  The detected objects
[in]       roiStartPoint	FCWSDPoint*	             sizeof(FCWSDPoint*)	  Offset of the detected ROI region
[in]       groupThreshold	s32	                     4-Byte	                  Threshold for a rect group being a cluster
					     			  					  
[out]	   returned value   s32                      4-Byte	                  Number of detected objects.

Realized function:
	+ Merge overlapped detect rectangles.
*/
static int lbpGroupWindowsCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjSets, FCWSDPoint *roiStartPoint, int groupThreshold);

static void object_detector_lbp_group_window_car_ROI(int index, FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjsets, FCWSDPoint *RioStartPoint, int group_threshold);

/*
I/O:	    Name		Type	                      Size			  	                   Content
					   						  
[in/out]	p		    FCWSDetectorMultiscaleList*	  sizeof(FCWSDetectorMultiscaleList*)  chained list for single scale tasks.
					   						  
[out]	returned value  int                           4-Byte	                            If 0, free failed.

Realized function:
	+ Free the memory of single scale tasks chained list.
*/
static int freeTaskWindowsCar(FCWSDetectorMultiscaleList *p);

/*
I/O:	    Name		    Type	               Size			  	            Content
					    								  
[in/out]	p		        FCWSDetectorROI*	   sizeof(FCWSDetectorROI*)	    chained list for multi-scale tasks.
					    								  
[out]	returned value      int                    4-Byte	                    If 0, free failed.

Realized function:
	+ Free the memory of multi-scale detector chained list.
*/
static int freeVehicleDetProcess_Rio(FCWSDetectorROI *p);



/*
Function process:
	+ Detect scanning windows for one single scaling factor.
	Fan-in :
	        + FCWD_InitVehicleDetProcess_Rio()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int initTaskWindowsCar( FCWSDetectorMultiscaleList *p, const s32 step, const FCWSDSize *minObjectSize, const int sizeLimit)
{
	s32 x, y,taskNum = 0;
	float vehicleHeight = 0;

	LBPTaskCarList *t1 = NULL;
	LBPTaskCarList *t2 = NULL;

	/* free in freeTaskWindowsCar */
	p->pLBPTCLhead = (LBPTaskCarList *) my_malloc(sizeof(LBPTaskCarList));
	memset(p->pLBPTCLhead, 0, sizeof(LBPTaskCarList));

	if (p->pLBPTCLhead == NULL)
	{
		return 0;
	}

	/* free at the end of this function */
	t1 = (LBPTaskCarList *) my_malloc(sizeof(LBPTaskCarList));
	memset(t1, 0, sizeof(LBPTaskCarList));

	if (t1 == NULL)
	{ 
		return 0;
	}
	else 
	{
		t2 = t1;
		p->pLBPTCLhead->next = NULL; 
	}

	for ( y = 0; y < p->roiRec.size.height - minObjectSize->height - 1; y += step)
	{
		for ( x = 0; x < p->roiRec.size.width - minObjectSize->width - 1 ;x += step) 
		{	
#ifdef USE_LDWS
			if(sizeLimit)
			{
				vehicleHeight = LDWS_GetDetaXofWorld(minObjectSize->height * p->factor, (y + minObjectSize->height) * p->factor + p->roiRec.point.y);

				if (vehicleHeight < 1 || vehicleHeight > 4)
					continue;
			}
#endif // USE_LDWS
			t1->task.x = x;
			t1->task.y = y;

			taskNum++;
			if (taskNum == 1)
			{
				p->pLBPTCLhead = t1;
				t2->next	   = NULL;
			}
			else
			{
				t2->next = t1;
			}
			t2 = t1;
			t1 = (LBPTaskCarList *)malloc(sizeof(LBPTaskCarList));
			memset(t1, 0, sizeof(LBPTaskCarList));

		}
	}

	if(taskNum == 0)
	{
		free(p->pLBPTCLhead);
		p->pLBPTCLhead = NULL;
	}

	t2->next = NULL;

	if(t1 != NULL)
	    free(t1);
	t1 = NULL; 
	
//#ifdef FCWS_DETECTOR_DEBUG

	p->ntask = taskNum;
//#endif

	return 1;

 }

/*
Function process:
	+ Assign the scanning windows (tasks) for chained list
	Fan-in :
	        + FCWD_VehicleDetProcess_Rio()
	Fan-out:
	        + lbpDetectCar()
	ATTENTION: __________
*/
static u32 lbpDetectWindowsCar( lbpCar *l, u32 *img, const float32_t factor, const unsigned char roiFlg, const nrowCol *arrRowColDet)
{
#ifdef USE_GPU
	gpu_speed(l, img, factor);
#else

	s32 t = 0;
	s32 found = 0;
	lbpRectCar rectCar; /* detector result */

	if (roiFlg == 0)
	{
#ifdef OPENMP
#pragma omp parallel for private(found) schedule(guided) num_threads(2)
#endif // OPENMP
		for (t = 0; t < l->ntaskNum; t++)
		{
#if ACC_DETECTOR
			s32 arrNrow = l->ptasks[t].x + l->ptasks[t].y * l->width;
#else
			s32 arrNrow = 0;
#endif

			found = lbpDetectCar(l, img, l->ptasks[t].x, l->ptasks[t].y, arrNrow, arrRowColDet);
			if (found)
			{
#ifdef OPENMP
#pragma omp critical
#endif // OPENMP
				{
					rectCar.x = (s32)(l->ptasks[t].x * factor);
					rectCar.y = (s32)(l->ptasks[t].y * factor);
					rectCar.width = (s32)(l->data.featureWidth * factor);
					rectCar.height = (s32)(l->data.featureHeight * factor);
					l->pdetectedRect[l->ndetdNum++] = rectCar;
				}
			}
		}
	}
	else if (roiFlg == 1) /* for ROI object detect, Openmp is not used */
	{
		for (t = 0; t < l->ntaskNum; t++)
		{
#if ACC_DETECTOR
			s32 arrNrow = l->ptasks[t].x + l->ptasks[t].y * l->width;
#else
			s32 arrNrow = 0;
#endif
			found = lbpDetectCar(l, img, l->ptasks[t].x, l->ptasks[t].y, arrNrow,arrRowColDet);
			if (found) 
			{
				{
					rectCar.x = (s32)(l->ptasks[t].x * factor);
					rectCar.y = (s32)(l->ptasks[t].y * factor);
					rectCar.width = (s32)(l->data.featureWidth * factor);
					rectCar.height = (s32)(l->data.featureHeight * factor);
					l->pdetectedRect[l->ndetdNum++] = rectCar;
				}
			}
		} 
	}
#endif
	return l->ndetdNum;

}

/*
Function process:
	+ Calculate the integral image.
	Fan-in :
	        + FCWD_VehicleDetProcess_Rio()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static u32 getResizeIntegralImage(const FCWSDImage *pOriGrayImg, u32 *iData, const FCWSDRect *detROI, const FCWSDetectorMultiscaleList *p)
{
	s32 i, j;
	u32 rs; 
	uint8_t *PSrc  = NULL;
	u32		*PData = NULL; 

	//PSrc = pOriGrayImg->ptr + p->arr_y[0];
	PSrc = pOriGrayImg->ptr + p->arr_y[detROI->point.y];// according to original image (0,0)
	PData = iData + detROI->point.y * p->roiRec.size.width;// according to p->roiRec (p->roiRec.point.x, p->roiRec.point.y)
	rs = 0;
	for (i = detROI->point.x ;i < detROI->size.width + detROI->point.x; i++)
	{
        rs		 +=  PSrc[p->arr_x[i]];
        PData[i] = rs;
	}

	for ( j = detROI->point.y + 1 ; j < detROI->size.height + detROI->point.y; j++)
	{
		PSrc = pOriGrayImg->ptr + p->arr_y[j];

		PData = iData + (j - 1) * p->roiRec.size.width;

		rs = 0;

		for (i = detROI->point.x; i < detROI->size.width + detROI->point.x; i++)
		{
            rs		+= PSrc[p->arr_x[i]];
            PData[i + p->roiRec.size.width] = rs + PData[i];
		}
	}

	return 1;
}

/*
Function process:
	+ Return system time (ms).
	Fan-in :
	        + FCWD_VehicleDetProcess_Rio()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static double userGetTime(void) 
{
	return 0;
}

/*
Function process:
	+ Merge overlapped detect rectangles.
	Fan-in :
	        + getGroupedRectanglesCar()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int lbpGroupWindowsCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjSets, FCWSDPoint *roiStartPoint, int groupThreshold)
{
	s32 i;
	s32 index = 0;
	unsigned char dayOrNight = 0;
	if (pVehicleDetor->index >= 4)
	{
		dayOrNight = 1;
	}
	/* merge overlapped rectangles */
	getGroupedRectanglesCar(pVehicleDetor, l->pdetectedRect, &l->ndetdNum, groupThreshold, l->para.eps);
	
	if (dayOrNight == 0)
	{
		for (i = 0; i < l->ndetdNum; i++)
		{
			pobjSets->objects[i].objectRect.point.x = l->pdetectedRect[i].x + roiStartPoint->x;
			pobjSets->objects[i].objectRect.point.y = l->pdetectedRect[i].y + roiStartPoint->y;
			pobjSets->objects[i].objectRect.size.width = l->pdetectedRect[i].width;
			pobjSets->objects[i].objectRect.size.height = l->pdetectedRect[i].height;
			pobjSets->objects[i].confidence = l->pdetectedRect[i].confidence;
		}
		pobjSets->nObjectNum = l->ndetdNum;
	}
	else if (dayOrNight == 1)
	{
		for (i = 0, index = 0; i < l->ndetdNum; i++)
		{
			if (l->pdetectedRect[i].width > 140)
			{
				if (l->pdetectedRect[i].confidence > groupThreshold * 2)
				{
					pobjSets->objects[index].objectRect.point.x = l->pdetectedRect[i].x + roiStartPoint->x;
					pobjSets->objects[index].objectRect.point.y = l->pdetectedRect[i].y + roiStartPoint->y;
					pobjSets->objects[index].objectRect.size.width = l->pdetectedRect[i].width;
					pobjSets->objects[index].objectRect.size.height = l->pdetectedRect[i].height;
					pobjSets->objects[index].confidence = l->pdetectedRect[i].confidence;
					index++;
				}
			}
			else
			{
				pobjSets->objects[index].objectRect.point.x = l->pdetectedRect[i].x + roiStartPoint->x;
				pobjSets->objects[index].objectRect.point.y = l->pdetectedRect[i].y + roiStartPoint->y;
				pobjSets->objects[index].objectRect.size.width = l->pdetectedRect[i].width;
				pobjSets->objects[index].objectRect.size.height = l->pdetectedRect[i].height;
				pobjSets->objects[index].confidence = l->pdetectedRect[i].confidence;
				index++;
			}

		}
		pobjSets->nObjectNum = index;
	}
	
	l->ndetdNum = 0;

	return pobjSets->nObjectNum;
}

static void object_detector_lbp_group_window_car_ROI(int index,FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjsets, FCWSDPoint *RioStartPoint, int group_threshold)
{
	s32 i = 0, maxId = 0, maxConf = -1;
	lbpRectCar r;
	//lbpRectCar r, r1;
	s32 Weight;
	float32_t s; 
	if (index == 1)/* location */
	{
		/* merge overlapped rectangles */
		getGroupedRectanglesCar(pVehicleDetor, l->pdetectedRect, &l->ndetdNum, group_threshold, l->para.eps*2);
	
		for ( i = 0; i < l->ndetdNum; i++)
		{
			if (l->pdetectedRect[i].confidence > maxConf)
			{
				maxConf = l->pdetectedRect[i].confidence;
				maxId = i;
			}			
		}

		pobjsets->objects[0].objectRect.point.x = l->pdetectedRect[maxId].x + RioStartPoint->x;
		pobjsets->objects[0].objectRect.point.y = l->pdetectedRect[maxId].y + RioStartPoint->y;
		pobjsets->objects[0].objectRect.size.width = l->pdetectedRect[maxId].width;
		pobjsets->objects[0].objectRect.size.height = l->pdetectedRect[maxId].height;
		pobjsets->objects[0].confidence = l->pdetectedRect[maxId].confidence;

		pobjsets->nObjectNum = 1;

		l->ndetdNum = 0;
	}
	else
	{
		r.x = 0;
		r.y = 0;
		r.width = 0;
		r.height = 0;
		Weight = 0;
		for ( i = 0; i < l->ndetdNum; i++)
		{
			r.x += l->pdetectedRect[i].x;
			r.y += l->pdetectedRect[i].y;
			r.width += l->pdetectedRect[i].width;
			r.height += l->pdetectedRect[i].height;
			Weight ++;
		}
		s= 1.f / Weight;
		pobjsets->objects[0].objectRect.point.x		= myRound(r.x*s) + RioStartPoint->x;
		pobjsets->objects[0].objectRect.point.y		= myRound(r.y*s) + RioStartPoint->y;
		pobjsets->objects[0].objectRect.size.width	= myRound(r.width*s);
		pobjsets->objects[0].objectRect.size.height	= myRound(r.height*s);
		pobjsets->objects[0].confidence	= l->ndetdNum;
		pobjsets->nObjectNum = 1;

		l->ndetdNum = 0;
	}
}

/*
Function process:
	+ Free the memory of single scale tasks chained list..
	Fan-in :
	        + FCWD_UnitVehicle()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int freeTaskWindowsCar(FCWSDetectorMultiscaleList *p)
{
	LBPTaskCarList *node, *temp;
	FCWSDetectorMultiscaleList *nodeFCWS,*tempFCWS;

	if (!p)
	{
		return 0;
	}

	nodeFCWS = p;

	while(nodeFCWS != NULL)
	{
		tempFCWS = nodeFCWS;

	    node = tempFCWS->pLBPTCLhead;
	
		while (node != NULL)
		{
			temp = node;
			node = node->next;
			my_free(temp);
		}

	    tempFCWS->pLBPTCLhead = NULL;

		my_free(tempFCWS->arr_x);
		tempFCWS->arr_x = NULL;

		my_free(tempFCWS->arr_y);
		tempFCWS->arr_y = NULL;

		my_free(tempFCWS->arrRowCol);
		tempFCWS->arrRowCol = NULL;
		nodeFCWS = nodeFCWS->next;
	}

	p->next = NULL;

	return 1;
}

/*
Function process:
	+ Free the memory of multi-scale tasks chained list..
	Fan-in :
	        + FCWD_UnitVehicle()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int freeVehicleDetProcess_Rio(FCWSDetectorROI *p)
{
	FCWSDetectorMultiscaleList *node, *temp;

	if (!p)
	{
		return 0;
	}

	node = p->pDMLhead;
	
	while (node != NULL)
	{
		temp = node;
		node = node->next;
		my_free(temp);
	}

	p->pDMLhead = NULL;

	return 1;
}


/*
Function process:
	+ Init variables and structures to be used in FCWSD functions
	Fan-in :
	        + FCWSD_Init()
	Fan-out:
	        + loadLbpDataCar()
	ATTENTION: __________
*/
int FCWD_InitVehicle(FCWSDetectorGlobalPara *pVehicleDetor, const void *pLDWSOutput,const void *p3DMapOutput, const char *file, const double scalingFactor, const double eps)
{
	uint8_t *pTr = NULL;
	s32		mallcoSize = 0;
	s32		bLoadSuccess = 0;

#ifdef USE_LDWS
	LDWS_Output *pLDWS = NULL;
	if(pLDWSOutput != NULL)
	{
	    pLDWS	 = (LDWS_Output *)pLDWSOutput;
	}
#endif

	mallcoSize = MAX_PUBLIC_BUFFER					               /* pbulicBuff */
				+ sizeof(objectDetCar)			                   /* object_det */
				+ sizeof(lbpCar)					               /* object_det->l */
				+ MAX_IMAGE_WIDTH *  MAX_IMAGE_HIGHT * sizeof(u32) /* object_det->integral_img */
				+ MAX_TASKS_NUM * sizeof(lbpTaskCar)			   /* object_det->l->ptasks */
				+ MAX_DETECT_NUM * sizeof(lbpRectCar)			   /* object_det->l->pdetected_r */
				+ MAX_DETECT_NUM * sizeof(objectCar)			   /* ObjectSets_car.objects */
				+ (MAX_TREE_NODE<<1) * sizeof(s32)				   /* PARENT + RANK */
				+ (MAX_TREE_NODE) * sizeof(s32);				   /* labels */
 
#ifdef USE_LDWS	
	if (pLDWS != NULL)
	{
		mallcoSize += pLDWS->NB_INTERVALLES * pLDWS->NB_BANDES * sizeof(LDWS_Point) << 1;
	}
#endif


	mallcoSize += (1024 << 3) ; 

	pVehicleDetor->pAlloc = (uint8_t*) my_malloc(mallcoSize);
	memset(pVehicleDetor->pAlloc, 0, mallcoSize);

	if (pVehicleDetor->pAlloc == NULL) 
	{
		my_printf("FCWSD Malloc Error.\n");
	}

	pTr = pVehicleDetor->pAlloc;

	pVehicleDetor->pbulicBuff = pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pbulicBuff + MAX_PUBLIC_BUFFER);

	pVehicleDetor->pdetorModel = (objectDetCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel + 1); 

	pVehicleDetor->pdetorModel->l = (lbpCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->l + 1); 

	pVehicleDetor->pdetorModel->integralImg = (u32 *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->integralImg + MAX_IMAGE_WIDTH * MAX_IMAGE_HIGHT); 

	pVehicleDetor->pdetorModel->l->ptasks = (lbpTaskCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->l->ptasks + MAX_TASKS_NUM); 
	
	pVehicleDetor->pdetorModel->l->pdetectedRect = (lbpRectCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->l->pdetectedRect + MAX_DETECT_NUM); 

	pVehicleDetor->objSets.objects = (objectCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->objSets.objects  + MAX_DETECT_NUM);

	pVehicleDetor->pTreeNode = (s32 *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pTreeNode + (MAX_TREE_NODE << 1)); 

	pVehicleDetor->pLabNode = (s32 *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pLabNode + MAX_TREE_NODE);

#ifdef USE_LDWS
	if (pLDWS != NULL)
	{
		pVehicleDetor->roiPoint = (LDWS_Point *)pTr; 
		pTr = ALIGN_16BYTE(pVehicleDetor->roiPoint + pLDWS->NB_INTERVALLES * pLDWS->NB_BANDES);		

		pVehicleDetor->roiPointCopy = (LDWS_Point *)pTr; 
		pTr = ALIGN_16BYTE(pVehicleDetor->roiPointCopy + pLDWS->NB_INTERVALLES * pLDWS->NB_BANDES);				
	}
#endif
	
	pVehicleDetor->pdetorModel->l->ntaskNum = 0;
	pVehicleDetor->pdetorModel->l->ndetdNum = 0;
	pVehicleDetor->objSets.nObjectNum		= 0;

	pVehicleDetor->pdetorModel->l->para.scalingFactor		= scalingFactor;
	pVehicleDetor->pdetorModel->l->para.eps					= eps;
	
	bLoadSuccess = loadLbpDataCar(pVehicleDetor->pdetorModel->l, file);

	pVehicleDetor->pdetorModel->l->para.minSizeWidth		= pVehicleDetor->pdetorModel->l->data.featureWidth;
	pVehicleDetor->initFlg = 0;

  
	return bLoadSuccess;
}

/*
Function process:
	+ Init chained list for Multiscale tasks
	Fan-in :
	        + FCWSD_Init()
	Fan-out:
	        + initTaskWindowsCar()
	ATTENTION: In caculation of integral image, Width is supposed to be larger then height
*/
int FCWD_InitVehicleDetProcess_Rio(FCWSDetectorGlobalPara *pVehicleDetor, const int index, 
								   FCWSDetectorROI	*pDetectotDefaultROI, const void *pLDWSOutput, 
								   const void *p3DMapOutput, const FCWSDSize *minObjectSize, 
								   const FCWSDSize	*maxObjectSize)
{
	s32 j, k, step, iScaleCnt, returnVal;

	float32_t factor, fSum, startFactor, scaleFactor;
	
	FCWSDSize windowSize;
	
	FCWSDRect roiRec;

	s32 fstepHight;

	FCWSDetectorMultiscaleList *p1 = NULL;
	FCWSDetectorMultiscaleList *p2 = NULL;

#ifdef FCWS_DETECTOR_DEBUG
	s32 totalTaskNum;
#endif

	/* Vanish line is set to be a constant value */
	pVehicleDetor->vanishY = pVehicleDetor->srcROIYFactor * MAX_IMAGE_HIGHT;

	//if(index == 0)
	   fstepHight = (MAX_IMAGE_HIGHT - pVehicleDetor->vanishY) * ROI_SEGMENT_FACTOR;
	//else
    //   fstepHight = (MAX_IMAGE_HIGHT) * ROI_SEGMENT_FACTOR;

	for ( j = 0; j < ROI_SEGMENT_NUM; ++j )
	{
		//if(index == 0)
		    pDetectotDefaultROI[j].posInfor = (s32)( pVehicleDetor->vanishY + (j + 1) * fstepHight - 1);
		//else
		//	pDetectotDefaultROI[j].posInfor = (s32)((j + 1) * fstepHight - 1);

		//LOGI("In %d Section, %d is end at %d \n", index, j, pDetectotDefaultROI[j].posInfor);

		if ( j == ROI_SEGMENT_NUM - 1)
		{
			iScaleCnt = 0;

#ifdef FCWS_DETECTOR_DEBUG

			totalTaskNum = 0;
#endif
			/* free at the end of this function */
			p1 = (FCWSDetectorMultiscaleList *)malloc(sizeof(FCWSDetectorMultiscaleList));
			if (p1 == NULL)
			{ 
				return 0;
			} 
			else 
			{
				p2 = p1;
				pDetectotDefaultROI[j].pDMLhead = NULL; 
			}

			//if(index == 0)
			{
				scaleFactor = pVehicleDetor->pdetorModel->l->para.scalingFactor;//*1.2;
			    startFactor = pVehicleDetor->startMFactor;
			}
			//else
			//{
			//	scaleFactor = 1.2;
			//	startFactor = pVehicleDetor->startMFactor;
			//}

			for ( factor = startFactor; ; factor *=scaleFactor )
			{
				windowSize.height = (s32)( minObjectSize->height * factor );
				windowSize.width  = (s32)(pVehicleDetor->aspectRatio * windowSize.height);

				if ( windowSize.width > maxObjectSize->width || windowSize.height > maxObjectSize->height ||\
					 windowSize.height > fstepHight*ROI_SEGMENT_NUM || windowSize.width > pVehicleDetor->roi.size.width ||
					 windowSize.height > pVehicleDetor->roi.size.height)
				{
					break;
				}

				roiRec.point.x = pVehicleDetor->roi.point.x;
				roiRec.point.y = pVehicleDetor->roi.point.y;
				roiRec.size.width	= (s32)(pVehicleDetor->roi.size.width / factor);
				roiRec.size.height	= (s32)(pVehicleDetor->roi.size.height / factor);

				step = factor > 4. ? 1 : 2;//1 : 2;

				p1->windowSize = windowSize;
				p1->factor	   = factor;
				p1->roiRec	   = roiRec;

				/* free at freeTaskWindowsCar */
				p1->arr_y = (s32 *) my_malloc(sizeof(s32) * roiRec.size.height);
				memset(p1->arr_y, 0, sizeof(s32) * roiRec.size.height);
				p1->arr_x = (s32 *) my_malloc(sizeof(s32) * roiRec.size.width);
				memset(p1->arr_x, 0, sizeof(s32) * roiRec.size.width);

				/* ATTENTION: In caculation of integral image, Width is supposed to be larger then height */
				fSum = -factor;
				for (k = 0 ; k < roiRec.size.height; ++k)
				{
					fSum += factor;
					p1->arr_x[k] = (s32)(fSum + roiRec.point.x );
					p1->arr_y[k] = (s32)(fSum + roiRec.point.y ) * MAX_IMAGE_WIDTH;
				}
				for (; k < roiRec.size.width; k++)
				{
					fSum += factor;
					p1->arr_x[k] = (s32)(fSum + roiRec.point.x );
				}

				//if(index == 0)
				{
					if(pLDWSOutput != NULL)
					{
				        returnVal = initTaskWindowsCar( p1, step, minObjectSize, 1);
					}
					else
					{
						returnVal = initTaskWindowsCar( p1, step, minObjectSize, 0);
					}

				   if(returnVal == 0)
					   return 0;

				}
				//else
				//{
				//	p1->pLBPTCLhead = NULL;
				//}

				/*acculate part*/
				p1->arrRowCol = NULL;
#if ACC_DETECTOR
				if (1)
				{
					s32 s, w, n, t = 0;
					lbpCar *l;
					l = pVehicleDetor->pdetorModel->l;
					p1->arrRowCol = (nrowCol *)malloc(sizeof(nrowCol) * l->data.totalWeakNum);
					for (s = 0; s < l->data.stagesNum; s++) 
					{
						for (w = 0; w < l->data.s[s].weakClassifiersNum; w++) 
						{
							const lbpRectCar *r = l->data.r;
							const weakClassifierCar *c = &l->data.s[s].classifiers[w];

							for (n = 0; n < l->data.leafNum - 1; n++)
							{
								p1->arrRowCol[t].nrow[n] = r[c->rectIdx[n]].x + r[c->rectIdx[n]].y * p1->roiRec.size.width;

								p1->arrRowCol[t].ncol[n] =  r[c->rectIdx[n]].height * p1->roiRec.size.width;

								p1->arrRowCol[t].ncolAdd[n] =  r[c->rectIdx[n]].width;	
							}
							t++;
						}
					}
				}
#endif    
	
				iScaleCnt++;

#ifdef FCWS_DETECTOR_DEBUG

				totalTaskNum += p1->ntask;
				printf("ScaleLevel=%d, width=%d, height=%d, LevelTaskNum=%d \n", iScaleCnt, windowSize.width, windowSize.height, p1->ntask);
#endif
				if (iScaleCnt == 1)
				{
					pDetectotDefaultROI[j].pDMLhead = p1;
					p2->next = NULL;
				}
				else
				{
					p2->next = p1;
				}
				p2 = p1; 

				/* free at the end of this function */
				p1 = (FCWSDetectorMultiscaleList *) my_malloc(
						sizeof(FCWSDetectorMultiscaleList));
						
				memset(p1, 0, sizeof(FCWSDetectorMultiscaleList));

			}
			p2->next = NULL;
			my_free(p1);
			p1 = NULL;

#ifdef FCWS_DETECTOR_DEBUG
			printf("In all of %d Scales,totalTaskNum = %d \n", iScaleCnt,totalTaskNum);
#endif

			return 1;
		}
	}

    return 0;
}

/*
Function process:
	+ The main function for multiScale vehicle detection, can be used for ROI region detection.
	Fan-in :
	        + FCWSD_Processor()
	Fan-out:
	        + getResizeIntegralImager()
			+ lbpDetectWindowsCar()
			+ lbpGroupWindowsCar()
	ATTENTION: In caculation of integral image, Width is supposed to be larger then height
*/
int	FCWD_VehicleDetProcess_Rio(FCWSDetectorGlobalPara *pVehicleDetor, 
							   FCWSDetectorROI		  *pDetectotDefaultROI, 
							   const FCWSDImage			  *pOriGrayImg,
							   const void				  *pLDWSOutput, 
							   const void				  *p3DMapOutput,
							   const FCWSDRect			  *roi,
							   const FCWSDSize			  *minObjectSize,
							   const FCWSDSize			  *maxObjectSize,
							   const int				  groupThreshold, 
							   const int				  maxRT)
{
	s32 taskNum = 0;
	s32 sectionIndex = 0;
	s32 roiFlg = 0;
	s32 flgPosInfo		= 0;
	s32 totalTaskNum	= 0;
	s32 detNum =0;
	s32 detROIHeight = 0;
	//s32 minx = 10000;
	//s32 maxx = -1;
	//s32 miny = 10000;
	//s32 maxy = -1;

	FCWSDPoint	roiStartPoint = {0};
	FCWSDRect	detROI = {0};

#ifdef USE_LDWS
	s32 posInfor[ROI_SEGMENT_NUM];
#endif

	FCWSDetectorMultiscaleList	*p  = NULL;
	LBPTaskCarList				*t1 = NULL;

	float32_t tStart = 0;
	float32_t tEnd = 0;

#ifdef USE_PROPOSALS
	unsigned char dayOrNight = 0;
#endif

#ifdef USE_LDWS
	/* Vanishing line position in scale image */
	s32 vyScale = 0;
	/* end line position in scale image */
	s32 endScale = pVehicleDetor->srcHeight;
#endif

#ifdef USE_LDWS

	s32 i;
	LDWS_Output *pLDWS	 = (LDWS_Output *)pLDWSOutput;

	if (roi != NULL)
	{
		pLDWS = NULL;
	}
#endif

#ifdef USE_PROPOSALS
	if (pVehicleDetor->index >= 4)
	{
		dayOrNight = 1;
	}
#endif // USE_PROPOSALS


	if (roi != NULL)
	{
		roiFlg = 1;
	}

	if (maxRT != 0)
	{
		tStart = userGetTime();
	}

	pVehicleDetor->objSets.nObjectNum = 0;

	p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;

	if (pVehicleDetor->useFixedTaskNum)
	{
		while (p != NULL && p->windowSize.width < pVehicleDetor->currentWindowSize)
		{
			p = p->next;
		}

		if(p == NULL)
		    p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;
	}

	if (p != NULL)
	{
		roiStartPoint.x = p->roiRec.point.x;
		roiStartPoint.y = p->roiRec.point.y;
	}

#ifdef USE_LDWS

	/*  LDWS--> Providing the ROI, if there is no result, provided the initial value */
	if (pLDWS != NULL )
	{
		s32 laneWidth = 0;

		for ( i = 0; i < pLDWS->NB_INTERVALLES; i++ )
		{
			pVehicleDetor->roiPoint[i].y = pLDWS->pCaPoint[i].y - roiStartPoint.y;
			if (pVehicleDetor->roiPoint[i].y < 0)
			{
				pVehicleDetor->roiPoint[i].y = 0;
			}
			pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].y = pVehicleDetor->roiPoint[i].y;

			/* Extended the ROI region for vehicle detection */
			laneWidth = (pLDWS->pCaPoint[i + pLDWS->NB_INTERVALLES].x - pLDWS->pCaPoint[i].x + 1) >> 3;// ;// << 1;
			
			pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x = pLDWS->pCaPoint[i + pLDWS->NB_INTERVALLES].x + laneWidth - 1 - roiStartPoint.x;
			pVehicleDetor->roiPoint[i].x						 = pLDWS->pCaPoint[i].x - laneWidth + 1 - roiStartPoint.x;
			
			if (pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x >= pOriGrayImg->nWid)
			{
				pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x = pOriGrayImg->nWid - 1;
			}
			else if (pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x < 0)
			{
				pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x = 0;
			}

			if (pVehicleDetor->roiPoint[i].x  >= pOriGrayImg->nWid)
			{
				pVehicleDetor->roiPoint[i].x = pOriGrayImg->nWid - 1;
			}
			else if (pVehicleDetor->roiPoint[i].x < 0)
			{
				pVehicleDetor->roiPoint[i].x = 0;
			}
//			printf("roiPoint[%d].x= %d;  roiPoint[%d].y= %d \n",i,pVehicleDetor->roiPoint[i].x,i,pVehicleDetor->roiPoint[i].y);
//			printf("roiPoint[%d].x= %d;  roiPoint[%d].y= %d \n",i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x,i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].y);

 		}

		for( i = 0; i < ROI_SEGMENT_NUM; i++ )
		{
			//posInfor[i] = LDWS_GetCarY( pDetectotDefaultROI[i].posInfor, 1.5);/* if the car is 1.5m and at images position y, calculate the image height */
			posInfor[i] = LDWS_GetXLengthofImage( 1.5, pDetectotDefaultROI[i].posInfor);
			//LOGI("posInfor[%d] = %d \n",i,posInfor[i]);			
		}

	}
#endif

#ifdef USE_PROPOSALS
	computeProposals(dayOrNight, pOriGrayImg);
#endif // USE_PROPOSALS

	/* main function for chain list */
	while (p != NULL)
	{
		sectionIndex = -1;

#ifdef USE_LDWS

		if (pLDWS != NULL )
		{
			for(i = 0; i < ROI_SEGMENT_NUM; i++)
			{
				if( posInfor[i] > (s32)(p->windowSize.width * 1.2) && \
					((s32)((pDetectotDefaultROI[i].posInfor - p->roiRec.point.y)/ p->factor) >  (pVehicleDetor->pdetorModel->l->data.featureHeight+1)))
				{
					sectionIndex = i;
					break;
				}
			}  

			for ( i = 0; i < pLDWS->NB_INTERVALLES; i++ )
			{

				pVehicleDetor->roiPointCopy[i].x = (s32)((pVehicleDetor->roiPoint[i].x - p->windowSize.width + 1) / p->factor); 
				if (pVehicleDetor->roiPointCopy[i].x < 0)
				{
					pVehicleDetor->roiPointCopy[i].x = 0;
				}
				
				pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x = (s32)((pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x) / p->factor);
				
				if (pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x >= pOriGrayImg->nWid)
				{
					pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x = pOriGrayImg->nWid - 1;
				}
				
				pVehicleDetor->roiPointCopy[i].y = (s32)((pVehicleDetor->roiPoint[i].y) / p->factor);
				
				if (pVehicleDetor->roiPointCopy[i].y < 0)
				{
					pVehicleDetor->roiPointCopy[i].y = 0;
				}
				
				pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].y = pVehicleDetor->roiPointCopy[i].y;
				//my_printf("roiPointCopy[%d].x= %d;  roiPointCopy[%d].y= %d \n",i,pVehicleDetor->roiPointCopy[i].x,i,pVehicleDetor->roiPointCopy[i].y);
			    //my_printf("roiPointCopy[%d].x= %d;  roiPointCopy[%d].y= %d \n",i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x,i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].y);

			}

			vyScale = ( pOriGrayImg->nHig/2 - roiStartPoint.y) / p->factor;//pLDWS->vy
			endScale = (pLDWS->pCaPoint[pLDWS->NB_INTERVALLES-1].y - roiStartPoint.y)/ p->factor;
		}

#endif

		if (sectionIndex < 0)
		{
			flgPosInfo	  = 1;
			sectionIndex  = ROI_SEGMENT_NUM - 1;
		}
		else
		{
			flgPosInfo = 0;
		}

		detROIHeight = MIN((pDetectotDefaultROI[sectionIndex].posInfor - p->roiRec.point.y + 1) / p->factor,p->roiRec.size.height);

		if( ((s32)(p->roiRec.size.width * p->factor) < minObjectSize->width
			|| (s32)(p->roiRec.size.height * p->factor) < minObjectSize->height ) && flgPosInfo == 1 )
		{
			pVehicleDetor->currentWindowSize = 0;
			break;
		}

		if (roi != NULL)
		{
			if (roi->size.width < p->windowSize.width || roi->size.height < p->windowSize.height)
			{
				pVehicleDetor->currentWindowSize = 0;
				break;
			}

			if (p->windowSize.width > maxObjectSize->width || p->windowSize.height > maxObjectSize->height)
			{
				pVehicleDetor->currentWindowSize = 0;
				break;
			}
			if (p->windowSize.width < minObjectSize->width || p->windowSize.height < minObjectSize->height)
			{
				p = p->next;
				continue;
			}
			
			/* roi is for original image, according to 1920*1080 image size */
			detROI.size.width  = MIN((s32)(roi->size.width / p->factor), p->roiRec.size.width);
			detROI.size.height = MIN((s32)(roi->size.height / p->factor),detROIHeight);

			detROI.point.x = (s32)((roi->point.x - p->roiRec.point.x) / p->factor);
			if (detROI.point.x < 0)
			{
				detROI.point.x = 0;
			}
			detROI.point.y = (s32)((roi->point.y - p->roiRec.point.y) / p->factor);
			if (detROI.point.y < 0)
			{  
				detROI.point.y = 0;
			}
		}
		else
		{
			detROI.size.width  = p->roiRec.size.width;
			detROI.size.height = detROIHeight;
			detROI.point.x = 0;
			detROI.point.y = 0;
		}

		pVehicleDetor->pdetorModel->width		= p->roiRec.size.width;
		pVehicleDetor->pdetorModel->height		= detROIHeight;
		pVehicleDetor->pdetorModel->l->width	= p->roiRec.size.width;
		pVehicleDetor->pdetorModel->l->height	= detROIHeight;

		/* chain list for tasks */
		t1 = p->pLBPTCLhead;
		taskNum = 0;

#ifdef USE_LDWS

		if (pLDWS != NULL )
		{
			while (t1 != NULL)
			{
				if(t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight > detROI.point.y + detROIHeight)
					break;
				if (t1->task.y > detROI.point.y 
				 && t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight < detROI.point.y + detROIHeight)
				{
					for (i = 0; i < pLDWS->NB_INTERVALLES; ++i)
					{
						if (t1->task.y + (pVehicleDetor->pdetorModel->l->data.featureHeight)< pVehicleDetor->roiPointCopy[i].y)
						{
							if (t1->task.x > pVehicleDetor->roiPointCopy[i].x && t1->task.x < pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x)
							{
								//if(t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight > vyScale-15
								//	&& t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight < endScale+15)
								{
#ifdef USE_PROPOSALS
									if (filterCarTask(dayOrNight, &t1->task, pVehicleDetor, p->factor) == 1)
#endif
										pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
								}
							}
							break;
						}
						else
						{
							if(i == pLDWS->NB_INTERVALLES-1)
							{
								if (t1->task.y + (pVehicleDetor->pdetorModel->l->data.featureHeight>>1)< pVehicleDetor->roiPointCopy[i].y)
								{
									if (t1->task.x > pVehicleDetor->roiPointCopy[i].x && t1->task.x < pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x)
									{
										//if(t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight > vyScale-15)
										{
#ifdef USE_PROPOSALS
											if (filterCarTask(dayOrNight, &t1->task, pVehicleDetor, p->factor) == 1)
#endif
												pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
										}
									}
									break;
								}
							}
						}
					}
				}
				t1 = t1->next;
			}
		}
		else
		{
			if (roi != NULL)
			{
				while (t1 != NULL)
				{
					if (t1->task.x > detROI.point.x 
					 && t1->task.y > detROI.point.y
					 && t1->task.x < detROI.point.x + detROI.size.width  - pVehicleDetor->pdetorModel->l->data.featureWidth
					 && t1->task.y < detROI.point.y + detROI.size.height - pVehicleDetor->pdetorModel->l->data.featureHeight)
					{
						pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
					}
					t1 = t1->next;
				}
			}
			else
			{
				while (t1 != NULL)
				{
					pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
					t1 = t1->next;
				}
			}
		}
#else

		if (roi != NULL)
		{
			while (t1 != NULL)
			{
				if (t1->task.x >= detROI.point.x
				 && t1->task.y >= detROI.point.y
				 && t1->task.x <= detROI.point.x + detROI.size.width  - pVehicleDetor->pdetorModel->l->data.featureWidth
				 && t1->task.y <= detROI.point.y + detROI.size.height - pVehicleDetor->pdetorModel->l->data.featureHeight)
				{
					pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
				}
				t1 = t1->next;
			}
		}
		else
		{
			while (t1 != NULL)
			{
				pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
				t1 = t1->next;
			}
		}
#endif

		pVehicleDetor->pdetorModel->l->ntaskNum = taskNum;
		
#ifdef FCWS_DETECTOR_DEBUG

		/*if (roi == NULL && pVehicleDetor->showFCWDebug == 1)
		{
		my_printf("D: DetetWindowSize = %d, DetectSection = %d, DetcWinsize = %d, vySacle = %d, endSacle = %d, taskNum = %d \n", p->windowSize.height, sectionIndex, p->roiRec.size.height, vyScale, endScale, taskNum);
		} else*/
		//	else if (pVehicleDetor->showFCWDebug == 1)
		{
			my_printf("T: DetetWindowSize = %d, DetectSection = %d : taskNum = %d \n", p->windowSize.height, sectionIndex, taskNum);

		}
#endif
		totalTaskNum += taskNum;

		getResizeIntegralImage(pOriGrayImg, pVehicleDetor->pdetorModel->integralImg, &detROI, p);

		lbpDetectWindowsCar(pVehicleDetor->pdetorModel->l, pVehicleDetor->pdetorModel->integralImg, p->factor, roiFlg, p->arrRowCol);

#ifdef FCWS_DETECTOR_DEBUG
		my_printf(" l->detNum = %d \n",pVehicleDetor->pdetorModel->l->ndetdNum);
#endif

		if (maxRT != 0)
		{
			tEnd = userGetTime();
			if (tEnd > tStart + maxRT)
			{
				/* object size in last detected size */
				s32 temp = (s32)(p->windowSize.width)*1.1;// / pVehicleDetor->pdetorModel->l->para.scalingFactor);

				if (temp > pVehicleDetor->currentWindowSize)
				{
					pVehicleDetor->currentWindowSize = temp - 1;
				}
				else
				{
					pVehicleDetor->currentWindowSize = p->windowSize.width + 1;
				}

				//my_printf("FCWS Detector timeout,and exit!\n");

				p = p->next;
				if (pVehicleDetor->useFixedTaskNum && p == NULL)
				{
					pVehicleDetor->currentWindowSize = 0;

#ifdef FCWS_DETECTOR_DEBUG

					if (pVehicleDetor->showFCWDebug == 1)
					{
						//my_printf("totalTaskNum = %d \n", totalTaskNum);
					}
					//my_printf("Finish all Scales in %d Detected Frames! \n",	pVehicleDetor->detframesUsed);

					pVehicleDetor->detframesUsed = 1;
#endif
				}

				break;
			}
		}

		if (totalTaskNum > pVehicleDetor->fixedTaskNum && pVehicleDetor->useFixedTaskNum )
		{
			s32 temp = (s32)(p->windowSize.width)*1.1;// / pVehicleDetor->pdetorModel->l->para.scalingFactor);

#ifdef FCWS_DETECTOR_DEBUG

			if (pVehicleDetor->showFCWDebug)
			{
				//my_printf("totalTaskNum = %d \n",totalTaskNum);
			}
#endif
			if (temp > pVehicleDetor->currentWindowSize)
			{
			    pVehicleDetor->currentWindowSize = temp - 1;
			}
			else
			{
				pVehicleDetor->currentWindowSize = p->windowSize.width + 1;
			}

#ifdef FCWS_DETECTOR_DEBUG

			pVehicleDetor->detframesUsed++;
#endif
			p = p->next;
			if (pVehicleDetor->useFixedTaskNum && p == NULL)
			{
				pVehicleDetor->currentWindowSize = 0;

#ifdef FCWS_DETECTOR_DEBUG

				if (pVehicleDetor->showFCWDebug == 1)
				 {
					//my_printf("totalTaskNum = %d \n", totalTaskNum);
				}
				//my_printf("Finish all Scales in %d Detected Frames! \n", pVehicleDetor->detframesUsed);

				pVehicleDetor->detframesUsed = 1;
#endif
			}
			break;
		}

		p = p->next;

		if (pVehicleDetor->useFixedTaskNum && p == NULL)
		{
			pVehicleDetor->currentWindowSize = 0;

#ifdef FCWS_DETECTOR_DEBUG

			if (pVehicleDetor->showFCWDebug == 1)
			{
				//my_printf("totalTaskNum = %d \n", totalTaskNum);
			}
			//my_printf("Finish all Scales in %d Detected Frames! \n", pVehicleDetor->detframesUsed);

			pVehicleDetor->detframesUsed = 1;
#endif
	/*		if (totalTaskNum < (s32)(0.3 * pVehicleDetor->fixedTaskNum))
			{
				p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;
			}
	*/
		}
	}

	if (pVehicleDetor->pdetorModel->l->ndetdNum)
	{
		/* merge overlapped rectangles */
		detNum = lbpGroupWindowsCar(pVehicleDetor, pVehicleDetor->pdetorModel->l, &pVehicleDetor->objSets, &roiStartPoint, groupThreshold);
	}

#ifdef FCWS_DETECTOR_DEBUG
	my_printf("final object count: %d \n", detNum);
#endif

	return detNum;
}

int	FCW_DETCOR_Vehicle_Rio(FCWSDetectorGlobalPara *pVehicleDetor, 
						   FCWSDetectorROI		*pDetectotDefaultROI, 
						   const FCWSDImage			  *pGrayImg,
						   const FCWSDSize			  *minObjectSize,
						   const  FCWSDSize			  *maxObjectSize,
						   const FCWSDRect			  *roi, 
						   const int				  group_threshold, 
						   const int				  index)
{	
    s32 taskNum = 0;
	s32 roi_flg = 1;
	s32 totalTaskNum	= 0;
	s32 x, y, step;
	lbpTaskCar task;
	s32 ScaleNum = 0;

	FCWSDPoint	RioStartPoint = {0};
	FCWSDRect	detROI = {0};

	FCWSDetectorMultiscaleList	*p  = NULL;

	pVehicleDetor->objSets.nObjectNum = 0;

	p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;
	if (p != NULL)
	{
		RioStartPoint.x = p->roiRec.point.x;
		RioStartPoint.y = p->roiRec.point.y;
	}

	while (p != NULL)
	{
		p->roiRec.size.height = (pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].posInfor - p->roiRec.point.y + 1) / p->factor;

		if( ((s32)(p->roiRec.size.width * p->factor) < minObjectSize->width
			|| (s32)(p->roiRec.size.height * p->factor) < minObjectSize->height ))
		{
			break;
		}

		if (roi != NULL)
		{
			if (roi->size.width < p->windowSize.width || roi->size.height < p->windowSize.height)
			{
				break;
			}

			if (p->windowSize.width > maxObjectSize->width || p->windowSize.height > maxObjectSize->height)
			{
				break;
			}
			if (p->windowSize.width < minObjectSize->width || p->windowSize.height < minObjectSize->height)
			{
				p = p->next;
				continue;
			}


			/* roi is for original image, according to 1920*1080 image size */
			detROI.size.width  = MIN((s32)(roi->size.width / p->factor), p->roiRec.size.width);;
			detROI.size.height = MIN((s32)(roi->size.height / p->factor), p->roiRec.size.height);
			detROI.point.x = (s32)((roi->point.x - p->roiRec.point.x) / p->factor);/* according to p->roiRec.point.x */
			if (detROI.point.x < 0)
			{
				detROI.point.x = 0;
			}
			detROI.point.y = (s32)((roi->point.y - p->roiRec.point.y) / p->factor);
			if (detROI.point.y < 0)
			{  
				detROI.point.y = 0;
			}
			if(detROI.point.y + detROI.size.height > p->roiRec.size.height)
			{
					detROI.size.height = p->roiRec.size.height - detROI.point.y;
			}
		}
		pVehicleDetor->pdetorModel->width		= p->roiRec.size.width;
		pVehicleDetor->pdetorModel->height		= p->roiRec.size.height;
		pVehicleDetor->pdetorModel->l->width	= p->roiRec.size.width;
		pVehicleDetor->pdetorModel->l->height	= p->roiRec.size.height;

		//t1 = p->pLBPTCLhead;
		taskNum = 0;

		if (roi != NULL)
		{
			//while (t1 != NULL)
			//{
			//	if (t1->task.x > detROI.point.x
			//	 && t1->task.y > detROI.point.y
			//	 && t1->task.x < detROI.point.x + detROI.size.width  - pVehicleDetor->pdetorModel->l->data.feature_width
			//	 && t1->task.y < detROI.point.y + detROI.size.height - pVehicleDetor->pdetorModel->l->data.feature_height)
			//	{
			//		/* 閲囩敤LDW鐨勮溅閬撶嚎淇℃伅锛孡DW鏃犺緭鍑猴紝濡傛灉鏈塺oi鍒欒缃畆oi鍖哄煙 */
			//		pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
			//	}
			//	t1 = t1->next;
			//}
			//if( p->factor > 4 )
			//    step = 2;
			//else
			//	step = 1;
			if( index ==1)
			   step = (s32) ((detROI.size.width  - pVehicleDetor->pdetorModel->l->data.featureWidth)>>3);
			else
			   step = (s32) ((detROI.size.width  - pVehicleDetor->pdetorModel->l->data.featureWidth)>>3);

			if( step < 1)
				step = 1;

			//step = 2;

			for (x = detROI.point.x; x<detROI.point.x + detROI.size.width  - pVehicleDetor->pdetorModel->l->data.featureWidth; x += step)
			{
				for(y = detROI.point.y; y<detROI.point.y + detROI.size.height  - pVehicleDetor->pdetorModel->l->data.featureHeight; y += step)
				{
					task.x = x;
					task.y = y;
					pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = task;
				}
			}
		}

		pVehicleDetor->pdetorModel->l->ntaskNum = taskNum;
		
#ifdef FCWS_DETECTOR_DEBUG

		//LOGI("p ROI: x %d, y %d, W %d, H %d \n",p->roiRec.point.x, p->roiRec.point.y, p->roiRec.size.width, p->roiRec.size.height);
		//LOGI("Det ROI: x %d, y %d, W %d, H %d \n",detROI.point.x, detROI.point.y, detROI.size.width, detROI.size.height);
		if (roi == NULL && pVehicleDetor->showFCWDebug == 1)
		{
			//my_printf("D: DetetWindowSize = %d, taskNum = %d \n",p->windowSize.height,taskNum);
		}
		else
		{
			//my_printf("T: DetetWindowSize = %d, taskNum = %d, MinSIze(%d, %d), MaxSIze(%d, %d)\n",p->windowSize.height,taskNum, minObjectSize->height, minObjectSize->width,maxObjectSize->height,maxObjectSize->width);

		}
#endif
		totalTaskNum += taskNum;
		
		if(taskNum!=0)
		{
			getResizeIntegralImage(pGrayImg, pVehicleDetor->pdetorModel->integralImg, &detROI, p);
		 
			lbpDetectWindowsCar(pVehicleDetor->pdetorModel->l, pVehicleDetor->pdetorModel->integralImg, p->factor, roi_flg, p->arrRowCol);
		}		
		ScaleNum++;
									
		//if( index == 0 )
		//{
		//	if(ScaleNum > 0)
		//		break;
		//}
		//else
		//{
		//	if(ScaleNum > 1)
		//		break;
		//}

		
		//if(pVehicleDetor->pdetorModel->l->ndetdNum > 5)
		//	break;


		p = p->next;
		
	}

	//printf("index=%d, group_threshold=%d, ScaleNum = %d , TotalTaskNum = %d, l->detNum = %d ", index, group_threshold,ScaleNum, totalTaskNum,pVehicleDetor->pdetorModel->l->ndetdNum);

	if (pVehicleDetor->pdetorModel->l->ndetdNum)
	{
		/* merge overlapped rectangles */
		object_detector_lbp_group_window_car_ROI(index,pVehicleDetor, pVehicleDetor->pdetorModel->l,&pVehicleDetor->objSets, &RioStartPoint, group_threshold);
	}

	//printf("objsets.nObjectNum = %d \n", pVehicleDetor->objSets.nObjectNum);
	//printf("Local Detection\n");
	
	return (pVehicleDetor->objSets.nObjectNum);


}

/*
Function process:
	+ Get the detector results.
	Fan-in :
	        + FCWSD_GetResult()
	Fan-out: N/A

	ATTENTION: 
*/
int FCWD_GetDetResult(const FCWSDetectorGlobalPara *pVehicleDetor, objectSetsCar **pFCWSDOutput)
{
	s32 i = 0;

	if (*pFCWSDOutput == NULL)
	{
		/* free in ???? */
		*pFCWSDOutput = (objectSetsCar *)malloc(sizeof(objectSetsCar));
	}
	else
	{
		if ((*pFCWSDOutput)->objects != NULL)
		{
			free((*pFCWSDOutput)->objects);
			(*pFCWSDOutput)->objects = NULL;
		}
	}

	(*pFCWSDOutput)->nObjectNum = pVehicleDetor->objSets.nObjectNum;

	/* free in the begin of this function */
	(*pFCWSDOutput)->objects = (objectCar *)malloc(sizeof(objectCar) * pVehicleDetor->objSets.nObjectNum);

	for (i = 0; i < pVehicleDetor->objSets.nObjectNum; ++i)
	{
		(*pFCWSDOutput)->objects[i] = pVehicleDetor->objSets.objects[i];
	}

	(*pFCWSDOutput)->frameNumber = 0;

	return pVehicleDetor->objSets.nObjectNum;
}

/*
Function process:
	+ Free the memory space of variables.
	Fan-in :
	        + FCWSD_Free()
	Fan-out: N/A

	ATTENTION: 
*/
int FCWD_UnitVehicle(FCWSDetectorGlobalPara *pVehicleDetor, FCWSDetectorROI *pDetectotDefaultROI)
{
	s32 j, ret;

	for( j = 0; j < ROI_SEGMENT_NUM; ++j )
	{
 		ret = freeTaskWindowsCar(pDetectotDefaultROI[j].pDMLhead);

		ret = freeVehicleDetProcess_Rio(&pDetectotDefaultROI[j]);

	}

	if (pVehicleDetor->pAlloc != NULL)
	{
		free(pVehicleDetor->pAlloc);
		pVehicleDetor->pAlloc = NULL;
	}

	return 1;
}
