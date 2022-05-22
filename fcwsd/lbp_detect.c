/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.h
Version: 1.0		Date: 2017-02-28		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are defined to use the LBP feature for classification.
	The following function types are included:
	+ loadLbpDataCar(): Init the structures of vehicle LBP detector.
	+ lbpDetectCar(): Classify for a single scanning window.
	+ lbpClassifyCar(): Extract LBP feature and go though weak classifier.

Deviation:

History:
	+ Version: 1.0		Date: 2017-02-28		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/
#include "utils.h"
#include "vehicle_type.h"
#include "lbp_detect.h"

#include <stdio.h>
#include <stdlib.h> 

extern nrowCol arrRowCol[15 * 1024];
/*
I/O:	Name		     Type	             Size			  	        Content
					          								  
[in]    r		         lbpRectCar*	     sizeof(lbpRectCar*)	    Rect location in scanning window
[in]	c		         weakClassifierCar*	 sizeof(weakClassifierCar*)	weak classifier.
[in]    img              u32*                sizeof(u32*)               integral image
[in]    x                u32                 4-Byte                     point x position  for scanning window
[in]    y                u32                 4-Byte                     point y position  for scanning window
[in]    width            u32                 4-Byte                     width for integral image
[in]    height           u32                 4-Byte                     height for integral image

[out]	returned value   float32_t           8-Byte	                    Threshold value of detector .

Realized function:
	+ Extract LBP feature and go though weak classifier.
*/
static int lbpClassifyCar( const lbpRectCar *r,  const weakClassifierCar *c, const u32 *img, const s32 x, const s32 y, const s32 width, const s32 height, const int idx, s32 t, s32 arrNrow, const nrowCol *arrRowColDet);

static  float32_t lbpClassifyCarTrees( const lbpRectCar *r,  const weakClassifierCar *c, const u32 *img, const s32 x, const s32 y, const s32 width, const s32 height, s32 t, s32 arrNrow, const nrowCol *arrRowColDet);


/*
Function process:
	+ Load parameters from detector model
	Fan-in : 
	        + FCWD_InitVehicle()
	Fan-out: N/A

	ATTENTION: In this realization detector model file is encrypted by XOR operation
*/
s32 loadLbpDataCar( lbpCar *l, const char *file)
{
    void *foi = NULL;
    s32 weakClassierLayer, strongClassierLayer, recNum; 
    char key[] = "Wissenstar"; /* secret key for XOR operation */
    s32 parNum = 0;
	s32 weakNum = 0;
	s32 keyLength = strlen(key);
	int i;

    foi = my_fopen(file, "rb");

    if (!foi)   
    {
        //my_printf("Can not open file : Det.txt, %s",file);
        return 0;    
    }

    my_fread(&l->data.featureHeight, sizeof(s32), 1, foi);
    l->data.featureHeight = l->data.featureHeight ^ key[parNum % keyLength];
    parNum++;
    //my_printf("l->data.featureHeight:  %d\n", l->data.featureHeight);

    my_fread(&l->data.featureWidth, sizeof(s32), 1, foi);
    l->data.featureWidth = l->data.featureWidth ^ key[parNum % keyLength];
    parNum++;
	//my_printf("l->data.featureWidth:  %d\n", l->data.featureWidth);
	fread(&l->data.maxDepth, sizeof(s32), 1, foi);
	l->data.maxDepth = l->data.maxDepth ^ key[parNum % keyLength];
	l->data.leafNum = pow(2,l->data.maxDepth);
	parNum++;
	//LOGI("%d\n", l->data.maxDepth);
	//LOGI("%d\n", l->data.leafNum);

    my_fread(&l->data.stagesNum, sizeof(s32), 1, foi);
    l->data.stagesNum = l->data.stagesNum ^ key[parNum % keyLength];
    parNum++;
    //my_printf("l->data.stagesNum:  %d\n", l->data.stagesNum);

    l->data.s = ( stageCar *)my_calloc(l->data.stagesNum, sizeof(stageCar));
    memset(l->data.s, 0, l->data.stagesNum*sizeof(stageCar));

    for (strongClassierLayer = 0; strongClassierLayer < l->data.stagesNum; strongClassierLayer++ )
    {	
        my_fread(&l->data.s[strongClassierLayer].weakClassifiersNum, sizeof(s32), 1, foi);
        l->data.s[strongClassierLayer].weakClassifiersNum = \
                                                            l->data.s[strongClassierLayer].weakClassifiersNum ^ key[parNum % keyLength];
        parNum++;
		//LOGI("%d\n", l->data.s[strongClassierLayer].weakClassifiersNum); 
		weakNum += l->data.s[strongClassierLayer].weakClassifiersNum;

        /* float32 - stageThreshold is not encrypted */
        my_fread(&l->data.s[strongClassierLayer].stageThreshold, sizeof(float32_t), 1, foi);
        //my_printf("l->data.s[%d].stageThreshold:  %f\n",strongClassierLayer,l->data.s[strongClassierLayer].stageThreshold);

		l->data.s[strongClassierLayer].classifiers = \
			 (weakClassifierCar *)calloc(l->data.s[strongClassierLayer].weakClassifiersNum, sizeof( weakClassifierCar));

		for (weakClassierLayer = 0; weakClassierLayer < l->data.s[strongClassierLayer].weakClassifiersNum; weakClassierLayer++ )
		{
			l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap = \
				(s32 *)calloc(8 * (l->data.leafNum - 1), sizeof(s32));
			l->data.s[strongClassierLayer].classifiers[weakClassierLayer].rectIdx = \
				(s32 *)calloc(l->data.leafNum - 1, sizeof(s32));
			l->data.s[strongClassierLayer].classifiers[weakClassierLayer].posiblity = \
				(float32_t *)calloc(l->data.leafNum, sizeof(float32_t));
			l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx = \
				(s32 *)calloc(2 * (l->data.leafNum - 1), sizeof(s32));

			for(i = 0; i < l->data.leafNum - 1; i++)
			{
			    fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i],   sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i + 1],   sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].rectIdx[i],   sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 0], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 1], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 2], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 3], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 4], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 5], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 6], sizeof(s32),1,foi);
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 7], sizeof(s32),1,foi);

				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i]   = \
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i] ^ key[parNum % keyLength];
	            parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i + 1]   = \
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i + 1] ^ key[parNum % keyLength];
	            parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].rectIdx[i]   = \
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].rectIdx[i] ^ key[parNum % keyLength];
	            parNum++;

				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 0] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 0] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 1] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 1] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 2] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 2] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 3] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 3] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 4] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 4] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 5] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 5] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 6] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 6] ^ key[parNum % keyLength];
				parNum++;
				l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 7] = \
					l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i + 7] ^ key[parNum % keyLength];
				parNum++;

				/*LOGI("%d %d %d %d %d %d %d %d %d %d %d\n",l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i],
					                              l->data.s[strongClassierLayer].classifiers[weakClassierLayer].leafIdx[2*i+1],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].rectIdx[i],
				                                  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +0],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +1],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +2],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +3],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +4],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +5],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +6],
												  l->data.s[strongClassierLayer].classifiers[weakClassierLayer].lbpmap[8*i +7]);*/

			}
		
			
			for(i = 0; i < l->data.leafNum; i++)
			{
				/* float32 - neg is not encrypted */
				fread(&l->data.s[strongClassierLayer].classifiers[weakClassierLayer].posiblity[i], sizeof(float32_t), 1, foi);
				//LOGI("%f\n", l->data.s[strongClassierLayer].classifiers[weakClassierLayer].posiblity[i]);
			}
			
			
		}
	}

	l->data.totalWeakNum = weakNum;
  	//my_printf("total weak num %d\n", weakNum); 
    my_fread(&l->data.rectsNum, sizeof(s32), 1, foi);
    l->data.rectsNum = l->data.rectsNum ^ key[parNum % keyLength];
    parNum++;
    //my_printf("%d \n",l->data.rectsNum);

    l->data.r = (lbpRectCar *)my_calloc(l->data.rectsNum, sizeof(lbpRectCar));
    memset(l->data.r, 0, l->data.rectsNum*sizeof(lbpRectCar));

    for (recNum = 0; recNum < l->data.rectsNum; recNum++ )
    {
        my_fread(&l->data.r[recNum].x, sizeof(s32), 1, foi);
        my_fread(&l->data.r[recNum].y, sizeof(s32), 1, foi);
        my_fread(&l->data.r[recNum].width, sizeof(s32), 1, foi);
        my_fread(&l->data.r[recNum].height, sizeof(s32), 1, foi);

		l->data.r[recNum].x = l->data.r[recNum].x ^ key[parNum % keyLength];
	    parNum++;
		l->data.r[recNum].y = l->data.r[recNum].y ^ key[parNum % keyLength];
	    parNum++;
		l->data.r[recNum].width = l->data.r[recNum].width ^ key[parNum % keyLength];
	    parNum++;
		l->data.r[recNum].height = l->data.r[recNum].height ^ key[parNum % keyLength];
	    parNum++;

        //my_printf("%d %d %d %d \n",l->data.r[recNum].x, l->data.r[recNum].y, l->data.r[recNum].width, l->data.r[recNum].height);
    }

    my_fclose(foi);

    //my_printf("%d \n",parNum);

	return 1;

}

/*
Function process:
	+ Classify a single scanning window to be 0 or 1
	Fan-in : 
	        + lbpDetectWindowsCar()
	Fan-out:
	        + lbpClassifyCar()

	ATTENTION: 
*/
s32 lbpDetectCar( const lbpCar *l, const u32 *img, const s32 x, const s32 y, s32 arrNrow,const nrowCol *arrRowColDet)
{
    /* loop for all stages */
    float32_t threshold = 0;
	s32 s = 0, w = 0, t = 0;


    for (s = 0; s < l->data.stagesNum; s++) 
	{
        /* loop all weak classifiers */
        threshold = 0;
        for (w = 0; w < l->data.s[s].weakClassifiersNum; w++) 
		{
            threshold += lbpClassifyCarTrees(l->data.r, &l->data.s[s].classifiers[w], img, x, y, l->width, l->height, t++, arrNrow,arrRowColDet);
        }

        if (threshold < l->data.s[s].stageThreshold) 
		{
            /* not matched */
            return 0;
        }
    }

    /* here we pass all the stages and found a match */
    return 1;
}

/*
Function process:
	+ Extract LBP feature and go though weak classifier
	Fan-in : 
	        + lbpDetectCar()
	Fan-out: N/A

	ATTENTION: 
    i0---i1---i2---i3
	| v0 | v1 | v2 | 
	i4---i5---i6---i7
	| v3 | v4 | v5 |
	i8---i9--i10--i11
	| v6 | v7 | v8 |
   i12--i13--i14--i15
*/
static  int lbpClassifyCar( const lbpRectCar *r,  const weakClassifierCar *c, const u32 *img, const s32 x, const s32 y, const s32 width, const s32 height, const int idx, s32 t, s32 arrNrow,const nrowCol *arrRowColDet)
{

#if ACC_DETECTOR

	s32 p0, p1, p2,
		p3,     p5,
		p6, p7, p8;	

	s32 i0,  i1,  i2,  i3,
		i4,  i5,  i6,  i7,
		i8,  i9,  i10, i11,
		i12, i13, i14, i15;

	s32  d5,  d6,  d7,
		d9,  d10, d11, d610;

	s32 nrow, ncolAdd, ncol, temp, leafFront, leafBehind;

	uint8_t bidx = 1;
	uint8_t lbp_code1 = 0; 
	uint8_t lbp_code2 = 0; 

	nrow = arrNrow + arrRowColDet[t].nrow[idx];

	ncol = arrRowColDet[t].ncol[idx];

	ncolAdd = arrRowColDet[t].ncolAdd[idx];
	
	temp = nrow;
	i0 = img[temp];
	temp += ncolAdd;
	i1 = img[temp];
	temp += ncolAdd;
	i2 = img[temp];
	temp += ncolAdd;
	i3 = img[temp];

	nrow += ncol;
	temp = nrow;
	i4 = img[temp];
	temp += ncolAdd;
	i5 = img[temp];
	temp += ncolAdd;
	i6 = img[temp];
	temp += ncolAdd;
	i7 = img[temp];

	nrow += ncol;
	temp = nrow;
	i8 = img[temp];
	temp += ncolAdd;
	i9 = img[temp];
	temp += ncolAdd;
	i10 = img[temp];

	temp += ncolAdd;
	i11 = img[temp];
	nrow += ncol;
	temp = nrow;
	i12 = img[temp];
	temp += ncolAdd;
	i13 = img[temp];
	temp += ncolAdd;
	i14 = img[temp];
	temp += ncolAdd;
	i15 = img[temp];

	d5 = i4 - i5;
	d6 = i5 - i6;
	d7 = i6 - i7;
	d9 = i8 - i9;
	d10 = i9 - i10;
	d11 = i10 - i11;
	d610  = d6 - d10;

	p0 = i0 - i1 - d5 - d610;         
	p1 = i1 - i2 - d6 - d610;           
	p2 = i2 - i3 - d7 - d610; 

	p3 = d5 - d9 - d610; 
	p5 = d7 - d11 - d610;   
	p6 = d9 - i12 + i13 - d610;     
	p7 = d10 - i13 + i14 - d610;    
	p8 = d11 - i14 + i15 - d610;  

	lbp_code1 |= (1 & ~(p0 >> 31)) << 2;
	lbp_code1 |= (1 & ~(p1 >> 31)) << 1;
	lbp_code1 |= (1 & ~(p2 >> 31));
	lbp_code2 |= (1 & ~(p5 >> 31)) << 4;
	lbp_code2 |= (1 & ~(p8 >> 31)) << 3;
	lbp_code2 |= (1 & ~(p7 >> 31)) << 2;
	lbp_code2 |= (1 & ~(p6 >> 31)) << 1;
	lbp_code2 |= (1 & ~(p3 >> 31));

	leafFront = c->lbpmap[(idx << 3) + lbp_code1];
	leafBehind = 1 << lbp_code2;

    bidx = ( leafFront & leafBehind) == 0;
	return c->leafIdx[(idx << 1) + bidx];
#else
	s32 p0, p1, p2,
		p3,     p5,
		p6, p7, p8;	

	/* value of integral image */
	s32 i0,  i1,  i2,  i3,
		i4,  i5,  i6,  i7,
		i8,  i9,  i10, i11,
		i12, i13, i14, i15;

	s32 i910, i56, i56910;
	s32 fx, fy;
	s32 nrow, ncolAdd, ncol, temp;

	uint8_t lbp_code = 0; 


	fx = x + r[c->rectIdx[idx]].x; 
	fy = y + r[c->rectIdx[idx]].y;

	nrow = fy * width + fx;

	/* offset for different row */
	ncol = r[c->rectIdx[idx]].height * width;

	/* offset for different column */
	ncolAdd = r[c->rectIdx[idx]].width;

	temp = nrow;
	i0 = img[temp];
	temp += ncolAdd;
	i1 = img[temp];
	temp += ncolAdd;
	i2 = img[temp];
	temp += ncolAdd;
	i3 = img[temp];

	nrow += ncol;
	temp = nrow;
	i4 = img[temp];
	temp += ncolAdd;
	i5 = img[temp];
	temp += ncolAdd;
	i6 = img[temp];
	temp += ncolAdd;
	i7 = img[temp];

	nrow += ncol;
	temp = nrow;
	i8 = img[temp];
	temp += ncolAdd;
	i9 = img[temp];
	temp += ncolAdd;
	i10 = img[temp];
	temp += ncolAdd;
	i11 = img[temp];

	nrow += ncol;
	temp = nrow;
	i12 = img[temp];
	temp += ncolAdd;
	i13 = img[temp];
	temp += ncolAdd;
	i14 = img[temp];
	temp += ncolAdd;
	i15 = img[temp];

	i910   = i9 - i10;
	i56	   = i6 - i5;
	i56910 = i56 + i910;

	p0 = i0 - i1 - i4 + i6 + i910;        /* v0-v4 =  (i[0] - i[1] - i[4] + i[5]) -(i[5] - i[6] - i[9] + i[10]) */
	p1 = i1 - i2 + i56 + i56910;          /* v1-v4 =  (i[1] - i[2] - i[5] + i[6]) -(i[5] - i[6] - i[9] + i[10]) */
	p2 = i2 - i3 - i5 + i7 + i910;        /* v2-v4 =  (i[2] - i[3] - i[6] + i[7]) -(i[5] - i[6] - i[9] + i[10]) */
	p3 = i4 - i5 - i8 + i9 + i56910;      /* v3-v4 =  (i[4] - i[5] - i[8] + i[9]) -(i[5] - i[6] - i[9] + i[10]) */
	p5 = i6 - i7 + i11 - i10 + i56910;    /* v5-v4 =  (i[6] - i[7] - i[10] + i[11]) -(i[5] - i[6] - i[9] + i[10]) */
	p6 = i8 - i10 - i12 + i13 + i56;      /* v6-v4 =  (i[8] - i[9] - i[12] + i[13]) -(i[5] - i[6] - i[9] + i[10]) */
	p7 = i14 - i13 + i56910 + i910;       /* v7-v4 =  (i[9] - i[10] - i[13] + i[14]) -(i[5] - i[6] - i[9] + i[10]) */
	p8 = i9 - i11 - i14 + i15 + i56;      /* v8-v4 =  (i[10] - i[11] - i[14] + i[15]) -(i[5] - i[6] - i[9] + i[10]) */

	/* lbp coding */
	lbp_code |= (1 & ~(p0 >> 31)) << 7;
	lbp_code |= (1 & ~(p1 >> 31)) << 6;
	lbp_code |= (1 & ~(p2 >> 31)) << 5 ;
	lbp_code |= (1 & ~(p5 >> 31)) << 4;
	lbp_code |= (1 & ~(p8 >> 31)) << 3;
	lbp_code |= (1 & ~(p7 >> 31)) << 2;
	lbp_code |= (1 & ~(p6 >> 31)) << 1;
	lbp_code |= (1 & ~(p3 >> 31));

	if (c->lbpmap[8*idx + (lbp_code >> 5)] & (1 << (lbp_code & 31))) 
	{
		return c->leafIdx[2*idx];
	}
	else
	{
		return c->leafIdx[2*idx+1];
	}
#endif

}


static  float32_t lbpClassifyCarTrees( const lbpRectCar *r,  const weakClassifierCar *c, const u32 *img, const s32 x, const s32 y, const s32 width, const s32 height, s32 t, s32 arrNrow,const nrowCol *arrRowColDet)
{
	int Idx = 0;

	do
	{
      Idx = lbpClassifyCar( r,  c, img, x, y, width, height, Idx, t, arrNrow,arrRowColDet);

	}while( Idx > 0);

	return(c->posiblity[-Idx]);
}
