/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: declare.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:
	This file define the commonly used constants for tracking.

ATTENTION: 

Deviation: 

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef DECLARE_H
#define DECLARE_H

#define  ALLOC_SUCCESS_STATE   3
#define MAX_XY_CORNER		2048
#define MAX_IMG_HEIGHT      720
#define MAX_IMG_WIDTH      720

#define MAX_CORNERS_OF_NONMAX 1024
#define MAX_CORNERS_PER_FRAME  512
#define MAX_OBJ_CONRER_SCOPE  120

#define SURF_DESC_DIMENTION 64
#define MAX_NEW_GROUP_PER_FRAM  8


#define  MAX_LAYER_OBJ_GROUP_NUMS    8
#define  MAX_OBJ_GROUP_NUMS    (MAX_LAYER_OBJ_GROUP_NUMS<<2)

#define MAX_TRACKS_NUM_OF_GROUP 128
#define  MAX_CORNER_OF_TRACK 16
#define  MAX_CORNER_OF_TRACK_BIT  15

#define  MAX_MOTION_NUM 64
#define  MAX_MOTION_BIT 63

#define  FRAM_RATIO 30

#define  MAX_HISRORY_GROUP_REC_NUM  8
#define  MAX_HISRORY_GROUP_REC_NUM_BIT  7

#define  RADIAN_TO_ANGLE            57.29
#define  VOTE_CONSENSE_TRD         3


#define GROUP_TEMPLATE_SHINK_RATE 0.15f
#define MV_ADAS_USE_FAST_STATIC  
#define FAST_OAST9_16

#define MAX_IMG_TEMPLATE_WIDTH      45
#define MAX_IMG_TEMPLATE_HEIGHT      45

#define MAX_TEMPLAT_TRACK_SIZE   256
#define  MAX_PUBLIC_SPACE_SIZE  314572
#define MAX_PREDIT_TRACK_SIZE   80

#define MAX_TEMPLAT_NCC_TRACK_SIZE   20

#define  MAX_GROUP_IMG_BYTE      (MAX_TEMPLAT_TRACK_SIZE * MAX_TEMPLAT_TRACK_SIZE)

#define  UPDATA_IMAGE_NUM 3
#define  UPDATA_IMAGE_SIZE 32

#define  MAX_EXTERN_SPACE_SIZE  314572


#define GROUP_DETER_STATE_NUM   6
#define GROUP_BOTTOM_NUM   2
#define TTC_THRESH  2.7
#define HALF_CAR_WIDTH   0.8
#define TTC_MAX -1
#define URBAN_SPEED 30

#endif
