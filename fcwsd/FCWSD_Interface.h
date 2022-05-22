/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.h
Version: 1.0		Date: 2017-02-21		Author: Yanming Wang		ID: 1047930

Description:
	FCWS is the abbreviation of Forward Collision Warning System, FCWSD describes the detect function of FCWS.
	The functions in this file are defined as the interface functions of vehicle detection.
	The following function types are included:
	+ FCWSD_Init(): The initialization of variables and structures to be used in FCWSD functions.
	+ FCWSD_Processor(): The main procedure of vehicle detection. Given the buff of input image and detection 
	                     parameters, the locations of vehicle will be calculated.
    + FCWSD_Processor_ROI(): A faster method to detect ROI region vehicles
	+ FCWSD_GetResult(): Get the result of detected vehicle.
	+ FCWSD_Free(): Free the memory space of variables.

Deviation: 'FCWSD_' is used as the prefix of vehicle detection interface functions

History:
	+ Version: 1.0		Date: 2017-02-21		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef FCWSD_INTERFACE_H
#define FCWSD_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* The num of detector can be called simultaneously in multi-threading */
#define DETECT_HANDLE_NUM   (8)

typedef struct FCWSDPOINT
{
	int x;
	int y;
}FCWSDPoint;

typedef struct FCWSDSIZE
{
	int width;
	int height;
}FCWSDSize;

typedef struct  FCWSDRECT
{
	FCWSDPoint point;
	FCWSDSize  size;
}FCWSDRect;

typedef struct OBJECTCAR
{
	FCWSDRect	objectRect;
	int			confidence;
}objectCar;

typedef struct ObjectSetsCar
{
	objectCar	*objects;
	int			nObjectNum;
	int         frameNumber;
}objectSetsCar; /* for FCWSD output */

typedef struct FCWSDIMAGE
{
	unsigned char	*ptr;
	int				nWid;
	int				nHig;
}FCWSDImage;

typedef struct FCWSDPARAM
{
	double	srcROIYFactor;   /* The zone below srcROIYFactor*srcHeight of detected image will be processed. */
	double	startMFactor;    /* The minimum size of deteced vehicle is defiend by startMFactor*minObjectSize, \
						      minObjectSize is pre-defined in the training process. */
	double  scalingFactor;
	double  eps;
	int		useFixedTaskNum; /* If fixed task number is used (useFixedTaskNum == 1), the detector will classify \
							    a fixed number of scanning windows in each frame. */
	int		fixedTaskNum;    /* The number of fixed scanning windows to be processed in each frame. */
	int		srcWidth;       
	int		srcHeight;
	FCWSDRect roi;
	int		reserve;		 /* To be reserved */
}FCWSDParam;

typedef objectSetsCar FCWSD_Output;

/*
I/O:	Name		    Type	     Size			  	  Content
					    								  
[in]	index		    int		     4-Byte	              Index num of detector to be initialized, range between 0 - (DETECT_HANDLE_NUM-1).
[in]	pParam		    FCWSDParam*	 sizeof(FCWSDParam*)  Parameters of FCWSD.
[in]	pLDWSOutput	    void*		 sizeof(void*)        The result of LDWS (Lane Departure Warning System).
[in]	p3DMapOutput    void*	     sizeof(void*) 	      The result of 3D obstacle detection.
[in]	file		    char		 < 256	              The path of detector model.
					    								  
[out]	returned value  int          4-Byte	              If 0, initialization failed.

Realized function:
    + Declaration of function
	+ The initialization of variables and structures to be used in FCWSD functions.
*/
extern int  FCWSD_Init(const int index, const FCWSDParam *pParam, const void *pLDWSOutput, const void *p3DMapOutput, const char *file);

/*
I/O:	Name		    Type	     Size			  	  Content
					    								  
[in]	index		    int		     4-Byte	              Index num of detector to be used for vehicle detect.
[in]	pOriGrayImg		FCWSDImage*	 sizeof(FCWSDImage*)  Gray image buff to be detected.
[in]	pLDWSOutput	    void*		 sizeof(void*)        The result of LDWS (Lane Departure Warning System).
[in]	p3DMapOutput    void*	     sizeof(void*) 	      The result of 3D obstacle detection.
[in]	roi             FCWSDRect*	 sizeof(FCWSDRect*)   The ROI region of image to be detected.
[in]	minObjectSize   FCWSDSize*	 sizeof(FCWSDSize*)   The minimum size of vehicle can be detected.
[in]	maxObjectSize   FCWSDSize*	 sizeof(FCWSDSize*)   The maximum size of vehicle can be detected.
[in]	groupThreshold	int		     4-Byte	              The threshold of group rect.
[in]	maxRT		    int		     4-Byte	              If the processing time > maxRT(ms), stoped and return the result.
					    								  
[out]	returned value  int          4-Byte	              If 0, detected failed.

Realized function:
    + Declaration of function
	+ The main procedure of vehicle detection.
*/
extern int  FCWSD_Processor(const int index, const FCWSDImage *pOriGrayImg, const void *pLDWSOutput, const void *p3DMapOutput,
	        FCWSDRect *roi, const FCWSDSize *minObjectSize, const FCWSDSize *maxObjectSize, const int groupThreshold, const int maxRT);

/*
I/O:	Name		    Type	     Size			  	  Content
					    								  
[in]	VerFlg		    int		     4-Byte	              0-verified,1-relocation.
[in]	pGrayImg		FCWSDImage*	 sizeof(FCWSDImage*)  Gray image buff to be detected.
[in]	roi             FCWSDRect*	 sizeof(FCWSDRect*)   The ROI region of image to be detected.
[in]	minObjectSize   FCWSDSize*	 sizeof(FCWSDSize*)   The minimum size of vehicle can be detected.
[in]	maxObjectSize   FCWSDSize*	 sizeof(FCWSDSize*)   The maximum size of vehicle can be detected.
[in]	groupThreshold	int		     4-Byte	              The threshold of group rect.
					    								  
[out]	returned value  int          4-Byte	              If 0, detected failed.

Realized function:
    + Declaration of function
	+ A faster method to detect ROI region objects.
*/
extern int  FCWSD_Processor_ROI(const int index, const int VerFlg, const FCWSDImage *pGrayImg, FCWSDRect *roi, \
								const FCWSDSize *minObjectSize, const FCWSDSize *maxObjectSize, const int groupThreshold);

/*
I/O:	  Name		   Type	             Size			  	       Content
					    								  
[in]	  index		   int		         4-Byte	                   Index num of detector to be used for vehicle detect.
[in/out]  pFCWSOutput  ObjectSets_car**	 sizeof(ObjectSets_car**)  The results of vehicle detector.
					    								  
[out]	returned value  int              4-Byte	                   The detected object number.

Realized function:
    + Declaration of function
	+ Get the result of detected vehicle.
*/
extern int FCWSD_GetResult(const int index, objectSetsCar **pFCWSOutput);


extern int FCWSD_RefineResult(objectSetsCar *pFCWSOutput, int imgWidth, int imgheight);

/*
I/O:	  Name		   Type	             Size			  	       Content
					    								  
[in/out]  pFCWSOutput  ObjectSets_car**	 sizeof(ObjectSets_car**)  The results of vehicle detector.
					    								  
[out]	returned value  int              4-Byte	                   If 0, free failed.

Realized function:
    + Declaration of function
	+ Free the result of detected vehicle.
*/
extern int FCWSD_FreeResult(objectSetsCar **pFCWSDOutput);

/*
I/O:	  Name		    Type	   Size			   Content
					    								  
[in]	  index		    int		   4-Byte	       Index num of detector to be freed.
				    								  
[out]	returned value  int        4-Byte	       If 0, free failed.

Realized function:
    + Declaration of function
	+ Free the memory space of variables.
*/
extern int FCWSD_Free(const int index);

extern void getDetectObject(const int index, objectSetsCar **pFCWSOutput);

extern void detcorByRec(const FCWSDImage GrayImg, const unsigned char dayOrNight);

#ifdef __cplusplus
}
#endif

#endif
