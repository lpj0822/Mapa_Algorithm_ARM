/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
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

Deviation: 'FCWSD_' is used as the prefix of vehicle detection functions in FCWS

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef VEHICLEDET_H
#define VEHICLEDET_H

/*
I/O:	Name		    Type	                 Size			  	               Content
					    								  
[in/out]pVehicleDetor	FCWSDetectorGlobalPara*	 sizeof(FCWSDetectorGlobalPara*)   Global Parameter for FCWSD.
[in]	pLDWSOutput	    void*		             sizeof(void*)                     The result of LDWS (Lane Departure Warning System).
[in]	p3DMapOutput    void*	                 sizeof(void*) 	                   The result of 3D obstacle detection.
[in]	file		    char		             < 256	                           The path of detector model.
					    								  
[out]	returned value  int                      4-Byte	                           If 0, initialization failed.

Realized function:
	+ The initialization of variables and structures to be used in FCWSD functions.
*/
int FCWD_InitVehicle(FCWSDetectorGlobalPara *pVehicleDetor, const void *pLDWSOutput,const void *p3DMapOutput, const char *file, const double scalingFactor, const double eps);

/*
I/O:	Name		        Type	                 Size			  	               Content
					    								  
[in/out]pVehicleDetor	    FCWSDetectorGlobalPara*	 sizeof(FCWSDetectorGlobalPara*)   Global Parameter for FCWSD.
[in]	index		        int		                 4-Byte	                           Index num of detector to be initialized, range between 0 - (DETECT_HANDLE_NUM-1).
[in/out]pDetectotDefaultROI	FCWSDetectorROI*	     sizeof(FCWSDetectorROI*)          Chained list for Multiscale tasks
[in]	pLDWSOutput	        void*		             sizeof(void*)                     The result of LDWS (Lane Departure Warning System).
[in]	p3DMapOutput        void*	                 sizeof(void*) 	                   The result of 3D obstacle detection.
[in]	minObjectSize       FCWSDSize*	             sizeof(FCWSDSize*)                The minimum size of vehicle can be detected.
[in]	maxObjectSize       FCWSDSize*	             sizeof(FCWSDSize*)                The maximum size of vehicle can be detected.
					    								  
[out]	returned value      int                      4-Byte	                           If 0, initialization failed.

Realized function:
	+ The initialization of Global Parameters and Chained lists for Multiscale tasks.
*/
int FCWD_InitVehicleDetProcess_Rio(FCWSDetectorGlobalPara *pVehicleDetor, const int index, 
								   FCWSDetectorROI	*pDetectotDefaultROI, const void *pLDWSOutput, 
								   const void *p3DMapOutput, const FCWSDSize *minObjectSize, 
								   const FCWSDSize	*maxObjectSize);

/*
I/O:	Name		        Type	                 Size			  	               Content
					    								  
[in/out]pVehicleDetor	    FCWSDetectorGlobalPara*	 sizeof(FCWSDetectorGlobalPara*)   Global Parameter for FCWSD.
[in]    pDetectotDefaultROI	FCWSDetectorROI*	     sizeof(FCWSDetectorROI*)          Chained list for Multiscale tasks
[in]    pOriGrayImg         FCWSDImage*              sizeof(FCWSDImage*)               The original gray image to be detected 
[in]	pLDWSOutput	        void*		             sizeof(void*)                     The result of LDWS (Lane Departure Warning System).
[in]	p3DMapOutput        void*	                 sizeof(void*) 	                   The result of 3D obstacle detection.
[in]    roi                 FCWSDRect*               sizeof(FCWSDRect*)                The ROI region to be detected
[in]	minObjectSize       FCWSDSize*	             sizeof(FCWSDSize*)                The minimum size of vehicle can be detected.
[in]	maxObjectSize       FCWSDSize*	             sizeof(FCWSDSize*)                The maximum size of vehicle can be detected.
[in]	groupThreshold      int	                     4-Byte	                           The threshold for rect group.
[in]	maxRT               int	                     4-Byte	                           The maximum reponse time for FCWSD.
				    								  
[out]	returned value      int                      4-Byte	                           The detected object number.

Realized function:
	+ The main function for vehicle detection, can be used for ROI region detection.
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
							   const int				  maxRT);

int	FCW_DETCOR_Vehicle_Rio(FCWSDetectorGlobalPara *pVehicleDetor, 
						   FCWSDetectorROI		*pDetectotDefaultROI, 
						   const FCWSDImage			  *pGrayImg,
						   const FCWSDSize			  *minObjectSize,
						   const  FCWSDSize			  *maxObjectSize,
						   const FCWSDRect			  *roi, 
						   const int				  group_threshold, 
						   const int				  index);

/*
I/O:	Name		    Type	                 Size			  	               Content
					    								  
[in]    pVehicleDetor	FCWSDetectorGlobalPara*	 sizeof(FCWSDetectorGlobalPara*)   Global Parameter for FCWSD.
[out]	pFCWSDOutput	objectSetsCar **		 sizeof(objectSetsCar **)          The result of object detect.
					    								  
[out]	returned value  int                      4-Byte	                           The detected object number.

Realized function:
	+ Get the detector results.
*/
int FCWD_GetDetResult(const FCWSDetectorGlobalPara *pVehicleDetor, objectSetsCar **pFCWSDOutput);

/*
I/O:	Name		        Type	                 Size			  	               Content
					    								  
[in/out]pVehicleDetor	    FCWSDetectorGlobalPara*	 sizeof(FCWSDetectorGlobalPara*)   Global Parameter for FCWSD.
[in/out]pDetectotDefaultROI	FCWSDetectorROI*	     sizeof(FCWSDetectorROI*)          Chained list for Multiscale tasks
				    								  
[out]	returned value      int                      4-Byte	                           if 0, free failed.

Realized function:
	+ Free the memory space of variables.
*/
int FCWD_UnitVehicle(FCWSDetectorGlobalPara *pVehicleDetor, FCWSDetectorROI *pDetectotDefaultROI);


#endif
