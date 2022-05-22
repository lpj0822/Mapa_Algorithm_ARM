/**************************************************************************************************************
Copyright Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: fast.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of fast points detection and nonmax suppress.
	The following function types are included:
	+ oast9_16(): fast corner detect.
	+ corner_score(): caculate the score of fast corner.
    + fast_nonmax(): Do the nonmax suppress for corner points.
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "utils.h"
#include "fast.h"
#include "adastype.h"
#include "declare.h"


/*
Function process:
	+ caculate the score of points
	Fan-in : 
	        + fast_nonmax()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
s32 corner_score(const uint8_t*  imp, const s32 *pointer_dir, s32 barrier)
{
	/*The score for a positive feature is sum of the difference between the pixels
	  and the barrier if the difference is positive. Negative is similar.
	  The score is the max of those two.
	  
	   B = {x | x = points on the Bresenham circle around c}
	   Sp = { I(x) - t | x E B , I(x) - t > 0 }
	   Sn = { t - I(x) | x E B, t - I(x) > 0}
	   Score = max sum(Sp), sum(Sn)*/

	s32 cb = *imp + barrier;
	s32 c_b = *imp - barrier;
	s32 sp=0, sn = 0;

	s32 i=0;

	for(i=0; i<16; i++)
	{
		s32 p = imp[pointer_dir[i]];

		if(p > cb)
			sp += p-cb;
		else if(p < c_b)
			sn += c_b-p;
	}
	
	if(sp > sn)
		return sp;
	else 
		return sn;
}

/*
Function process:
	+ Do the nonmax suppress for corner points
	Fan-in : 
	        + mvCornerDetct()
	Fan-out:
	        + corner_score()
	ATTENTION: __________
*/
xy*  fast_nonmax(const uint8_t* im, s32 xsize, s32 ysize, xy* corners, s32 numcorners, s32 barrier, s32* numnx,
	  s32 *pRowStart, s32 * pScore, xy *pXYNoMax)
{
  
	/*Create a list of integer pointer offstes, corresponding to the */
	/*direction offsets in dir[]*/
	s32	pointer_dir[16];

#ifdef MV_ADAS_USE_FAST_STATIC
	s32 *row_start = pRowStart;
	s32 *scores = pScore;
	xy*  nonmax_corners = pXYNoMax;
#else
	s32* row_start = (s32*) my_malloc(ysize * sizeof(s32));
	s32* scores    = (s32*) my_malloc(numcorners * sizeof(s32));
	xy*  nonmax_corners = (xy*)my_malloc(numcorners* sizeof(xy));
#endif

	s32 num_nonmax=0;
	s32 prev_row = -1;
	s32 i, j;
	s32 xisze2,xisze3;
	s32 point_above = 0;
	s32 point_below = 0;
	*numnx = 0;

	xisze2 = xsize<<1;
	xisze3 = xisze2 + xsize ;
	pointer_dir[0] = xisze3;		
	pointer_dir[1] = 1 + xisze3;		
	pointer_dir[2] = 2 + xisze2;		
	pointer_dir[3] = 3 +  xsize;		
	pointer_dir[4] = 3  ;		
	pointer_dir[5] = 3 - xsize;		
	pointer_dir[6] = 2 - xisze2;		
	pointer_dir[7] = 1  -xisze3;		
	pointer_dir[8] =  -xisze3;		
	pointer_dir[9] = -1 - xisze3;		
	pointer_dir[10] = -2  - xisze2;		
	pointer_dir[11] = -3 - xsize;		
	pointer_dir[12] = -3 ;		
	pointer_dir[13] = -3 + xsize;		
	pointer_dir[14] = -2 + xisze2;		
	pointer_dir[15] = -1 + xisze3;

	if(numcorners < 5)
	{
#ifndef MV_ADAS_USE_FAST_STATIC
		my_free(row_start, ysize * sizeof(s32));
		my_free(scores, numcorners * sizeof(s32));
		my_ree(nonmax_corners);
#endif
		return 0;
	}

	/*xsize ysize numcorners corners*/

	/*Compute the score for each detected corner, and find where each row begins*/
	/* (the corners are output in raster scan order). A beginning of -1 signifies*/
	/* that there are no corners on that row.*/

  
	for(i=0; i <ysize; i++)
		row_start[i] = -1;
	  
	  
	for(i=0; i< numcorners; i++)
	{
		if(corners[i].y != prev_row)
		{
			row_start[corners[i].y] = i;
			prev_row = corners[i].y;
		}
		  
		scores[i] = corner_score(im + corners[i].x + corners[i].y * xsize, pointer_dir, barrier);
	}
  
  
	/*Point above points (roughly) to the pixel above the one of interest, if there*/
	/*is a feature there.*/
  
	for(i=0; i < numcorners; i++)
	{
		s32 score = scores[i];
		xy pos = corners[i];
			
		//Check left 
		if(i > 0)
			if(corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y && scores[i-1] > score)
				continue;
			
		//Check right
		if(i < (numcorners - 1))
			if(corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y && scores[i+1] > score)
				continue;
			
		//Check above (if there is a valid row above)
		if(pos.y != 0 && row_start[pos.y - 1] != -1) 
		{
			//Make sure that current point_above is one
			//row above.
			if(corners[point_above].y < pos.y - 1)
				point_above = row_start[pos.y-1];
			
			//Make point_above point to the first of the pixels above the current point,
			//if it exists.
			for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
			{}
			
			
			for(j=point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++)
			{
				s32 x = corners[j].x;
				if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && (scores[j] > score))
					goto cont;
			}
			
		}
			
		//Check below (if there is anything below)
		if(pos.y != ysize-1 && row_start[pos.y + 1] != -1 && point_below < numcorners) //Nothing below
		{
			if(corners[point_below].y < pos.y + 1)
				point_below = row_start[pos.y+1];
			
			// Make point below point to one of the pixels belowthe current point, if it
			// exists.
			for(; point_below < numcorners && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x - 1; point_below++)
			{}

			for(j=point_below; j < numcorners && corners[j].y == pos.y+1 && corners[j].x <= pos.x + 1; j++)
			{
				s32 x = corners[j].x;
				if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && (scores[j] >score))
					goto cont;
			}
		}
			
		nonmax_corners[num_nonmax].x = corners[i].x;
		nonmax_corners[num_nonmax].y = corners[i].y;

		num_nonmax++;

		cont:
			;
	}

	*numnx = num_nonmax;

#ifndef MV_ADAS_USE_FAST_STATIC
	my_free(row_start,ysize * sizeof(s32));
	my_free(scores,numcorners * sizeof(s32));
#endif
	return nonmax_corners;
}

xy * fast_corner_detect_9(const uint8_t* im, s32 xsize, s32 ysize, s32 barrier, s32* num,xy *pXyCorner)				
{																								
	s32 boundary = 3, y, cb, c_b;																
	const uint8_t  *line_max, *line_min;															
	//s32			rsize=512, total=0, nOldSize = 0;		
	s32			total=0;		

#ifdef MV_ADAS_USE_FAST_STATIC
	xy			*ret = pXyCorner;
#else
	xy	 		*ret = (xy*)my_malloc(rsize*sizeof(xy));											
#endif

	const uint8_t* cache_0;
	const uint8_t* cache_1;
	const uint8_t* cache_2;
	s32	pixel[16];	
	s32 xsize3;
	s32 xsize2;
	s32 nIndex1,nIndex2;
	xsize2 = xsize << 1;
	xsize3 = xsize + xsize2;


	pixel[0] = xsize3;		
	pixel[1] = 1 + xsize3;		
	pixel[2] = 2 + xsize2;		
	pixel[3] = 3 +  xsize;		
	pixel[4] = 3;		
	pixel[5] = 3 - xsize;		
	pixel[6] = 2 - xsize2;		
	pixel[7] = 1 - xsize3;		
	pixel[8] =  -xsize3;		
	pixel[9] = -1 - xsize3;		
	pixel[10] = -2 - xsize2;		
	pixel[11] = -3  - xsize;		
	pixel[12] = -3 ;		
	pixel[13] = -3 + xsize;		
	pixel[14] = -2 + xsize2;		
	pixel[15] = -1 + xsize3;

	nIndex1 =  boundary + (boundary -1) * xsize;
	nIndex2 =  xsize - boundary + (boundary -1) * xsize;
	for(y = boundary ; y < ysize - boundary; y++)												
	{																							
		/*cache_0 = im + boundary + y*xsize;														
		line_min = cache_0 - boundary;															
		line_max = im + xsize - boundary + y * xsize;	*/

		nIndex1 += xsize;
		cache_0 = nIndex1 + im;

		line_min = cache_0 - boundary;															
		nIndex2 +=  xsize;
		line_max =  nIndex2 + im;
																								
		cache_1 = cache_0 + pixel[5];
		cache_2 = cache_0 + pixel[14];
																								
		for(; cache_0 < line_max;cache_0++, cache_1++, cache_2++)
		{																						
			cb = *cache_0 + barrier;															
			c_b = *cache_0 - barrier;	
			
            if(*cache_1 > cb)
                if(*cache_2 > cb)
                    if(*(cache_0+3) > cb)
                        if(*(cache_0 + pixel[0]) > cb)
                            if(*(cache_0 + pixel[3]) > cb)
                                if(*(cache_0 + pixel[6]) > cb)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[15]) > cb)
                                            if(*(cache_0 + pixel[1]) > cb)
                                                goto success;
                                            else if(*(cache_0 + pixel[1]) < c_b)
                                                continue;
                                            else
                                                if(*(cache_0 + pixel[10]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        if(*(cache_0 + pixel[7]) > cb)
                                                            if(*(cache_0 + pixel[9]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else if(*(cache_0 + pixel[15]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[8]) > cb)
                                                if(*(cache_0 + pixel[7]) > cb)
                                                    if(*(cache_0 + pixel[1]) > cb)
                                                        goto success;
                                                    else if(*(cache_0 + pixel[1]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[10]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else if(*(cache_2+4) < c_b)
                                        continue;
                                    else
                                        if(*(cache_1+-6) > cb)
                                            if(*(cache_0 + pixel[9]) > cb)
                                                if(*(cache_0 + pixel[10]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        if(*(cache_0 + pixel[7]) > cb)
                                                            goto success;
                                                        else if(*(cache_0 + pixel[7]) < c_b)
                                                            continue;
                                                        else
                                                            if(*(cache_0+-3) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                    else if(*(cache_0 + pixel[8]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0+-3) > cb)
                                                            if(*(cache_0 + pixel[1]) > cb)
                                                                if(*(cache_0 + pixel[13]) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(*(cache_0 + pixel[6]) < c_b)
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*(cache_1+-6) > cb)
                                                continue;
                                            else if(*(cache_1+-6) < c_b)
                                                if(*(cache_0 + pixel[15]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[15]) > cb)
                                            if(*(cache_2+4) > cb)
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    goto success;
                                                else if(*(cache_0 + pixel[1]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        if(*(cache_1+-6) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else if(*(cache_2+4) < c_b)
                                                continue;
                                            else
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    if(*(cache_0+-3) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            if(*(cache_0 + pixel[10]) > cb)
                                                                if(*(cache_1+-6) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else if(*(cache_0 + pixel[1]) < c_b)
                                                            continue;
                                                        else
                                                            if(*(cache_0 + pixel[8]) > cb)
                                                                if(*(cache_0 + pixel[10]) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else if(*(cache_0 + pixel[3]) < c_b)
                                continue;
                            else
                                if(*(cache_0+-3) > cb)
                                    if(*(cache_0 + pixel[10]) > cb)
                                        if(*(cache_1+-6) > cb)
                                            if(*(cache_0 + pixel[8]) > cb)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    goto success;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_2+4) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                            else if(*(cache_0 + pixel[8]) < c_b)
                                                if(*(cache_0 + pixel[7]) > cb || *(cache_0 + pixel[7]) < c_b)
                                                    continue;
                                                else
                                                    goto success;
                                            else
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[13]) > cb)
                                                        if(*(cache_0 + pixel[15]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if(*(cache_2+4) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[9]) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            if(*(cache_0 + pixel[13]) > cb)
                                                                if(*(cache_0 + pixel[15]) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[0]) < c_b)
                            if(*(cache_0 + pixel[7]) > cb)
                                if(*(cache_0 + pixel[10]) > cb)
                                    goto success;
                                else
                                    continue;
                            else
                                continue;
                        else
                            if(*(cache_0 + pixel[7]) > cb)
                                if(*(cache_0 + pixel[10]) > cb)
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_0 + pixel[6]) > cb)
                                            if(*(cache_0 + pixel[8]) > cb)
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[9]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else if(*(cache_2+4) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_1+-6) > cb)
                                                        if(*(cache_0 + pixel[9]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[6]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[15]) > cb)
                                                if(*(cache_0+-3) > cb)
                                                    if(*(cache_0 + pixel[9]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else if(*(cache_0 + pixel[3]) < c_b)
                                        continue;
                                    else
                                        if(*(cache_0+-3) > cb)
                                            if(*(cache_0 + pixel[8]) > cb)
                                                if(*(cache_1+-6) > cb)
                                                    if(*(cache_0 + pixel[6]) > cb)
                                                        if(*(cache_0 + pixel[9]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else if(*(cache_0 + pixel[6]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[15]) > cb)
                                                            if(*(cache_0 + pixel[13]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(*(cache_0 + pixel[10]) < c_b)
                                    continue;
                                else
                                    if(*(cache_0 + pixel[1]) > cb)
                                        if(*(cache_0 + pixel[9]) > cb)
                                            if(*(cache_0 + pixel[6]) > cb)
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[3]) > cb)
                                                        if(*(cache_0 + pixel[8]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else if(*(cache_0+3) < c_b)
                        if(*(cache_0+-3) > cb)
                            if(*(cache_0 + pixel[9]) > cb)
                                if(*(cache_1+-6) > cb)
                                    if(*(cache_0 + pixel[10]) > cb)
                                        if(*(cache_0 + pixel[6]) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        if(*(cache_0+-3) > cb)
                            if(*(cache_1+-6) > cb)
                                if(*(cache_0 + pixel[7]) > cb)
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[9]) > cb)
                                                if(*(cache_0 + pixel[6]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        goto success;
                                                    else if(*(cache_0 + pixel[8]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[0]) > cb)
                                                            if(*(cache_0 + pixel[1]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else if(*(cache_0 + pixel[6]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        if(*(cache_0 + pixel[8]) > cb)
                                                            goto success;
                                                        else if(*(cache_0 + pixel[8]) < c_b)
                                                            continue;
                                                        else
                                                            if(*(cache_0 + pixel[0]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                            else if(*(cache_0 + pixel[9]) < c_b)
                                                continue;
                                            else
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[3]) > cb)
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[7]) < c_b)
                                    if(*(cache_0 + pixel[10]) > cb)
                                        if(*(cache_2+4) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_0 + pixel[0]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[0]) > cb)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if(*(cache_2+4) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[9]) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            if(*(cache_0 + pixel[15]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else if(*(cache_0 + pixel[1]) < c_b)
                                                            continue;
                                                        else
                                                            if(*(cache_0 + pixel[8]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[3]) > cb)
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        if(*(cache_0 + pixel[13]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else
                                continue;
                        else
                            continue;
                else if(*cache_2 < c_b)
                    if(*(cache_0+3) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                            if(*(cache_0 + pixel[1]) > cb)
                                if(*(cache_0 + pixel[9]) > cb)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[6]) > cb)
                                            if(*(cache_0 + pixel[3]) > cb)
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_2+4) < c_b)
                                        continue;
                                    else
                                        if(*(cache_1+-6) > cb)
                                            if(*(cache_0 + pixel[3]) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(*(cache_0 + pixel[9]) < c_b)
                                    if(*(cache_0 + pixel[15]) > cb)
                                        if(*(cache_2+4) > cb)
                                            if(*(cache_0 + pixel[3]) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[0]) > cb)
                                        if(*(cache_0 + pixel[8]) > cb)
                                            if(*(cache_2+4) > cb)
                                                if(*(cache_0 + pixel[3]) > cb)
                                                    if(*(cache_0 + pixel[6]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[8]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[15]) > cb)
                                                if(*(cache_2+4) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else if(*(cache_0 + pixel[1]) < c_b)
                                if(*(cache_1+-6) > cb)
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[6]) > cb)
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_0 + pixel[3]) < c_b)
                                        continue;
                                    else
                                        if(*(cache_0+-3) > cb)
                                            if(*(cache_0 + pixel[10]) > cb)
                                                if(*(cache_0 + pixel[6]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(*(cache_1+-6) < c_b)
                                    if(*(cache_0 + pixel[9]) > cb)
                                        if(*(cache_0 + pixel[3]) > cb)
                                            if(*(cache_2+4) > cb)
                                                if(*(cache_0 + pixel[10]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else if(*(cache_2+4) < c_b)
                                                if(*(cache_0 + pixel[10]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[3]) < c_b)
                                            if(*(cache_0 + pixel[15]) < c_b)
                                                if(*(cache_0+-3) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_2+4) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0 + pixel[0]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        if(*(cache_2+4) < c_b)
                                            if(*(cache_0 + pixel[10]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[15]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                if(*(cache_0 + pixel[3]) < c_b)
                                                    if(*(cache_0 + pixel[15]) < c_b)
                                                        if(*(cache_0 + pixel[0]) < c_b)
                                                            if(*(cache_0+-3) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                else
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[8]) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                if(*(cache_0 + pixel[10]) > cb)
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_2+4) > cb)
                                            if(*(cache_0 + pixel[6]) > cb)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if(*(cache_2+4) < c_b)
                                            continue;
                                        else
                                            if(*(cache_1+-6) > cb)
                                                if(*(cache_0 + pixel[6]) > cb)
                                                    if(*(cache_0 + pixel[9]) > cb)
                                                        if(*(cache_0 + pixel[8]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else if(*(cache_0 + pixel[3]) < c_b)
                                        continue;
                                    else
                                        if(*(cache_0+-3) > cb)
                                            if(*(cache_1+-6) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[7]) < c_b)
                            if(*(cache_1+-6) < c_b)
                                if(*(cache_0 + pixel[15]) > cb)
                                    continue;
                                else if(*(cache_0 + pixel[15]) < c_b)
                                    if(*(cache_0+-3) < c_b)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0 + pixel[13]) < c_b)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        continue;
                                                    else if(*(cache_0 + pixel[8]) < c_b)
                                                        goto success;
                                                    else
                                                        if(*(cache_0 + pixel[1]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                else
                                                    if(*(cache_2+4) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0 + pixel[3]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[6]) < c_b)
                                        if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0+-3) < c_b)
                                                if(*(cache_0 + pixel[8]) < c_b)
                                                    if(*(cache_0 + pixel[13]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                        else
                            if(*(cache_0 + pixel[0]) < c_b)
                                if(*(cache_0 + pixel[10]) > cb)
                                    continue;
                                else if(*(cache_0 + pixel[10]) < c_b)
                                    if(*(cache_0 + pixel[9]) > cb)
                                        continue;
                                    else if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0+-3) < c_b)
                                            if(*(cache_0 + pixel[1]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[1]) < c_b)
                                                if(*(cache_1+-6) < c_b)
                                                    if(*(cache_0 + pixel[13]) < c_b)
                                                        if(*(cache_0 + pixel[15]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                if(*(cache_0 + pixel[8]) < c_b)
                                                    if(*(cache_0 + pixel[13]) < c_b)
                                                        if(*(cache_1+-6) < c_b)
                                                            if(*(cache_0 + pixel[15]) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        if(*(cache_2+4) < c_b)
                                            if(*(cache_0+-3) < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    if(*(cache_1+-6) < c_b)
                                                        if(*(cache_0 + pixel[13]) < c_b)
                                                            if(*(cache_0 + pixel[15]) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(*(cache_0 + pixel[3]) < c_b)
                                        if(*(cache_1+-6) < c_b)
                                            if(*(cache_0+-3) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else if(*(cache_0+3) < c_b)
                        if(*(cache_0+-3) > cb)
                            if(*(cache_0 + pixel[13]) > cb)
                                goto success;
                            else
                                continue;
                        else if(*(cache_0+-3) < c_b)
                            if(*(cache_0 + pixel[9]) > cb)
                                if(*(cache_0 + pixel[13]) < c_b)
                                    goto success;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[9]) < c_b)
                                goto success;
                            else
                                if(*(cache_0 + pixel[6]) > cb || *(cache_0 + pixel[6]) < c_b)
                                    continue;
                                else
                                    if(*(cache_2+4) < c_b)
                                        goto success;
                                    else
                                        continue;
                        else
                            continue;
                    else
                        if(*(cache_1+-6) > cb)
                            if(*(cache_0 + pixel[13]) > cb)
                                if(*(cache_0 + pixel[9]) > cb)
                                    if(*(cache_0 + pixel[7]) > cb)
                                        if(*(cache_0+-3) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(*(cache_1+-6) < c_b)
                            if(*(cache_0 + pixel[3]) > cb)
                                if(*(cache_0 + pixel[8]) < c_b)
                                    if(*(cache_0 + pixel[15]) > cb)
                                        continue;
                                    else if(*(cache_0 + pixel[15]) < c_b)
                                        if(*(cache_0 + pixel[13]) < c_b)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[0]) < c_b)
                                                goto success;
                                            else
                                                if(*(cache_0 + pixel[7]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        if(*(cache_0 + pixel[6]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[3]) < c_b)
                                if(*(cache_2+4) > cb)
                                    continue;
                                else if(*(cache_2+4) < c_b)
                                    if(*(cache_0 + pixel[0]) < c_b)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[1]) < c_b)
                                            if(*(cache_0 + pixel[15]) < c_b)
                                                if(*(cache_0+-3) < c_b)
                                                    if(*(cache_0 + pixel[13]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0 + pixel[8]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[1]) < c_b)
                                            if(*(cache_0+-3) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0 + pixel[8]) < c_b)
                                                if(*(cache_0 + pixel[0]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else
                                if(*(cache_0 + pixel[1]) > cb)
                                    continue;
                                else if(*(cache_0 + pixel[1]) < c_b)
                                    if(*(cache_0 + pixel[10]) < c_b)
                                        if(*(cache_0+-3) < c_b)
                                            if(*(cache_0 + pixel[9]) > cb)
                                                if(*(cache_2+4) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else if(*(cache_0 + pixel[9]) < c_b)
                                                if(*(cache_0 + pixel[15]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[15]) < c_b)
                                                    if(*(cache_0 + pixel[13]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    if(*(cache_0 + pixel[6]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                            else
                                                if(*(cache_2+4) < c_b)
                                                    if(*(cache_0 + pixel[15]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[7]) > cb)
                                        continue;
                                    else if(*(cache_0 + pixel[7]) < c_b)
                                        if(*(cache_0 + pixel[15]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[15]) < c_b)
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0+-3) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        if(*(cache_0 + pixel[13]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0 + pixel[6]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                    else
                                        if(*(cache_0 + pixel[0]) < c_b)
                                            if(*(cache_0 + pixel[8]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                        else
                            continue;
                else
                    if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[3]) > cb)
                            if(*(cache_0 + pixel[10]) > cb)
                                if(*(cache_0+3) > cb)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[6]) > cb)
                                            if(*(cache_0 + pixel[8]) > cb)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    goto success;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                            else if(*(cache_0 + pixel[8]) < c_b)
                                                continue;
                                            else
                                                if(*(cache_0 + pixel[15]) > cb)
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else if(*(cache_2+4) < c_b)
                                        if(*(cache_1+-6) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        if(*(cache_1+-6) > cb)
                                            if(*(cache_0 + pixel[6]) > cb)
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    if(*(cache_0 + pixel[9]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(*(cache_0+3) < c_b)
                                    continue;
                                else
                                    if(*(cache_0+-3) > cb)
                                        if(*(cache_0 + pixel[13]) > cb)
                                            if(*(cache_1+-6) > cb)
                                                if(*(cache_0 + pixel[6]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        if(*(cache_0 + pixel[9]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else if(*(cache_0 + pixel[10]) < c_b)
                                if(*(cache_0 + pixel[15]) > cb)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[6]) > cb)
                                            if(*(cache_0+3) > cb)
                                                if(*(cache_0 + pixel[0]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[15]) < c_b)
                                    continue;
                                else
                                    if(*(cache_0 + pixel[8]) > cb)
                                        if(*(cache_0 + pixel[0]) > cb)
                                            goto success;
                                        else if(*(cache_0 + pixel[0]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[9]) > cb)
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    if(*(cache_0 + pixel[6]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else
                                if(*(cache_0 + pixel[1]) > cb)
                                    if(*(cache_0 + pixel[9]) > cb)
                                        if(*(cache_0 + pixel[6]) > cb)
                                            if(*(cache_0+3) > cb)
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        goto success;
                                                    else if(*(cache_0 + pixel[8]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[15]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0 + pixel[0]) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        if(*(cache_0 + pixel[0]) > cb)
                                            if(*(cache_0+3) > cb)
                                                if(*(cache_0 + pixel[6]) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        if(*(cache_2+4) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else if(*(cache_0 + pixel[15]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[8]) > cb)
                                                            if(*(cache_2+4) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[3]) < c_b)
                            if(*(cache_0 + pixel[13]) > cb)
                                if(*(cache_1+-6) > cb)
                                    if(*(cache_0 + pixel[9]) > cb)
                                        if(*(cache_0+-3) > cb)
                                            if(*(cache_0 + pixel[6]) > cb)
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[13]) < c_b)
                                continue;
                            else
                                if(*(cache_0+3) > cb)
                                    if(*(cache_0+-3) > cb)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else
                            if(*(cache_0+-3) > cb)
                                if(*(cache_0 + pixel[13]) > cb)
                                    if(*(cache_1+-6) > cb)
                                        if(*(cache_0 + pixel[9]) > cb)
                                            if(*(cache_0 + pixel[6]) > cb)
                                                if(*(cache_0 + pixel[10]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[13]) < c_b)
                                    if(*(cache_0 + pixel[0]) > cb)
                                        goto success;
                                    else
                                        continue;
                                else
                                    if(*(cache_0+3) > cb)
                                        if(*(cache_0 + pixel[9]) > cb)
                                            if(*(cache_1+-6) > cb)
                                                if(*(cache_0 + pixel[6]) > cb)
                                                    if(*(cache_0 + pixel[10]) > cb)
                                                        if(*(cache_0 + pixel[8]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else
                        continue;
            else if(*cache_1 < c_b)
                if(*(cache_0 + pixel[15]) > cb)
                    if(*(cache_1+-6) > cb)
                        if(*(cache_2+4) > cb)
                            if(*(cache_0+-3) > cb)
                                if(*(cache_0 + pixel[10]) > cb)
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*cache_2 > cb)
                                                goto success;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[1]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[7]) > cb)
                                                goto success;
                                            else
                                                continue;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[10]) < c_b)
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_0 + pixel[13]) > cb)
                                            if(*cache_2 > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*cache_2 > cb)
                                                if(*(cache_0 + pixel[0]) > cb)
                                                    if(*(cache_0 + pixel[13]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                        else if(*(cache_2+4) < c_b)
                            if(*(cache_0 + pixel[7]) > cb)
                                if(*(cache_0+-3) > cb)
                                    if(*cache_2 > cb)
                                        if(*(cache_0 + pixel[13]) > cb)
                                            if(*(cache_0 + pixel[9]) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[7]) < c_b)
                                if(*(cache_0 + pixel[9]) > cb)
                                    if(*(cache_0 + pixel[1]) > cb)
                                        if(*(cache_0+-3) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[9]) < c_b)
                                    if(*(cache_0 + pixel[10]) > cb)
                                        continue;
                                    else if(*(cache_0 + pixel[10]) < c_b)
                                        if(*(cache_0 + pixel[3]) < c_b)
                                            if(*(cache_0+3) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        if(*(cache_0 + pixel[1]) < c_b)
                                            if(*(cache_0 + pixel[3]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(*(cache_0 + pixel[0]) < c_b)
                                        goto success;
                                    else
                                        continue;
                            else
                                if(*(cache_0 + pixel[0]) > cb)
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[9]) > cb)
                                            if(*cache_2 > cb)
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    if(*(cache_0 + pixel[10]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else if(*(cache_0 + pixel[1]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        if(*(cache_0+-3) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else
                            if(*(cache_0 + pixel[9]) > cb)
                                if(*(cache_0+-3) > cb)
                                    if(*(cache_0 + pixel[1]) > cb)
                                        if(*cache_2 > cb)
                                            if(*(cache_0 + pixel[10]) > cb)
                                                if(*(cache_0 + pixel[13]) > cb)
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_0 + pixel[1]) < c_b)
                                        continue;
                                    else
                                        if(*(cache_0 + pixel[7]) > cb)
                                            if(*(cache_0 + pixel[10]) > cb)
                                                if(*(cache_0 + pixel[13]) > cb)
                                                    if(*cache_2 > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[7]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[0]) > cb)
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    if(*(cache_0 + pixel[6]) < c_b)
                                                        if(*(cache_0 + pixel[10]) > cb)
                                                            if(*(cache_0 + pixel[13]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                else
                                    continue;
                            else
                                continue;
                    else if(*(cache_1+-6) < c_b)
                        if(*(cache_0 + pixel[3]) > cb)
                            if(*(cache_0 + pixel[13]) > cb)
                                if(*(cache_0+-3) > cb)
                                    if(*(cache_0+3) > cb)
                                        goto success;
                                    else
                                        continue;
                                else if(*(cache_0+-3) < c_b)
                                    if(*(cache_0+3) < c_b)
                                        if(*(cache_0 + pixel[6]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[13]) < c_b)
                                if(*(cache_0 + pixel[7]) < c_b)
                                    if(*(cache_0 + pixel[6]) < c_b)
                                        if(*(cache_0 + pixel[8]) < c_b)
                                            if(*(cache_0+-3) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                if(*(cache_0+3) < c_b)
                                    if(*(cache_0+-3) < c_b)
                                        if(*(cache_0 + pixel[7]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[3]) < c_b)
                            if(*(cache_0 + pixel[8]) < c_b)
                                if(*(cache_0 + pixel[9]) < c_b)
                                    if(*(cache_0 + pixel[7]) < c_b)
                                        if(*(cache_0+3) > cb)
                                            continue;
                                        else if(*(cache_0+3) < c_b)
                                            if(*(cache_0 + pixel[10]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[6]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                        else
                                            if(*(cache_0 + pixel[13]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            if(*(cache_0+-3) < c_b)
                                if(*(cache_0+3) > cb)
                                    continue;
                                else if(*(cache_0+3) < c_b)
                                    if(*(cache_0 + pixel[6]) < c_b)
                                        if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0 + pixel[9]) < c_b)
                                                if(*(cache_0 + pixel[7]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[13]) < c_b)
                                        if(*(cache_0 + pixel[7]) < c_b)
                                            if(*(cache_0 + pixel[6]) < c_b)
                                                if(*(cache_0 + pixel[10]) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        if(*(cache_0 + pixel[9]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else
                        if(*(cache_2+4) > cb)
                            if(*(cache_0+3) > cb)
                                if(*(cache_0+-3) > cb)
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*(cache_0 + pixel[3]) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(*(cache_2+4) < c_b)
                            if(*(cache_0 + pixel[10]) > cb)
                                continue;
                            else if(*(cache_0 + pixel[10]) < c_b)
                                if(*(cache_0+3) < c_b)
                                    if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0 + pixel[3]) < c_b)
                                            if(*(cache_0 + pixel[7]) < c_b)
                                                if(*(cache_0 + pixel[6]) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                if(*(cache_0 + pixel[1]) < c_b)
                                    if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0 + pixel[3]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else
                            continue;
                else if(*(cache_0 + pixel[15]) < c_b)
                    if(*(cache_0+3) > cb)
                        if(*(cache_0+-3) < c_b)
                            if(*(cache_1+-6) < c_b)
                                if(*(cache_0 + pixel[13]) < c_b)
                                    if(*(cache_0 + pixel[7]) > cb)
                                        continue;
                                    else if(*(cache_0 + pixel[7]) < c_b)
                                        goto success;
                                    else
                                        if(*(cache_0 + pixel[8]) < c_b)
                                            if(*(cache_0 + pixel[0]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if(*(cache_0+3) < c_b)
                        if(*(cache_0 + pixel[6]) > cb)
                            if(*(cache_0 + pixel[13]) > cb)
                                if(*cache_2 > cb)
                                    if(*(cache_0 + pixel[10]) > cb)
                                        goto success;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[13]) < c_b)
                                if(*(cache_0 + pixel[0]) < c_b)
                                    if(*(cache_2+4) < c_b)
                                        if(*cache_2 < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(*(cache_0 + pixel[6]) < c_b)
                            if(*(cache_0 + pixel[3]) > cb)
                                if(*(cache_0+-3) < c_b)
                                    if(*(cache_0 + pixel[1]) < c_b)
                                        continue;
                                    else
                                        goto success;
                                else
                                    continue;
                            else if(*(cache_0 + pixel[3]) < c_b)
                                if(*(cache_0 + pixel[7]) > cb)
                                    if(*cache_2 < c_b)
                                        goto success;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[7]) < c_b)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[10]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else if(*(cache_2+4) < c_b)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[1]) < c_b)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[0]) < c_b)
                                                goto success;
                                            else
                                                if(*(cache_0 + pixel[9]) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[8]) < c_b)
                                                    if(*(cache_0 + pixel[9]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(*(cache_1+-6) < c_b)
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[8]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(*cache_2 < c_b)
                                        if(*(cache_2+4) < c_b)
                                            if(*(cache_0 + pixel[0]) < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                if(*(cache_0+-3) < c_b)
                                    if(*(cache_1+-6) < c_b)
                                        if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0 + pixel[8]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[8]) < c_b)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    if(*(cache_0 + pixel[7]) > cb)
                                                        continue;
                                                    else if(*(cache_0 + pixel[7]) < c_b)
                                                        goto success;
                                                    else
                                                        if(*(cache_0 + pixel[13]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                else
                                                    if(*(cache_2+4) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                            else
                                                if(*(cache_0 + pixel[13]) < c_b)
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        if(*(cache_0 + pixel[7]) > cb || *(cache_0 + pixel[7]) < c_b)
                                                            continue;
                                                        else
                                                            goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else
                            if(*(cache_0 + pixel[13]) < c_b)
                                if(*(cache_2+4) > cb)
                                    continue;
                                else if(*(cache_2+4) < c_b)
                                    if(*cache_2 < c_b)
                                        if(*(cache_0 + pixel[3]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[3]) < c_b)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[0]) < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                if(*(cache_0 + pixel[7]) < c_b)
                                                    if(*(cache_1+-6) < c_b)
                                                        if(*(cache_0 + pixel[8]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(*(cache_0+-3) < c_b)
                                                if(*(cache_0 + pixel[10]) < c_b)
                                                    if(*(cache_0 + pixel[1]) > cb)
                                                        continue;
                                                    else if(*(cache_0 + pixel[1]) < c_b)
                                                        goto success;
                                                    else
                                                        if(*(cache_0 + pixel[7]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_1+-6) < c_b)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[0]) < c_b)
                                                if(*cache_2 < c_b)
                                                    if(*(cache_0 + pixel[10]) < c_b)
                                                        if(*(cache_0+-3) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                if(*(cache_0 + pixel[7]) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        if(*(cache_0 + pixel[1]) > cb || *(cache_0 + pixel[1]) < c_b)
                                                            continue;
                                                        else
                                                            goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else
                        if(*(cache_0+-3) < c_b)
                            if(*(cache_0 + pixel[13]) < c_b)
                                if(*(cache_1+-6) < c_b)
                                    if(*(cache_0 + pixel[9]) > cb)
                                        if(*(cache_0 + pixel[3]) < c_b)
                                            if(*(cache_2+4) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_0 + pixel[9]) < c_b)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0 + pixel[7]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[7]) < c_b)
                                                if(*cache_2 > cb || *cache_2 < c_b)
                                                    goto success;
                                                else
                                                    if(*(cache_0 + pixel[6]) < c_b)
                                                        if(*(cache_0 + pixel[8]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[1]) < c_b)
                                                    if(*cache_2 < c_b)
                                                        if(*(cache_0 + pixel[0]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        if(*(cache_0 + pixel[8]) < c_b)
                                                            if(*cache_2 < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                        else
                                            if(*(cache_0 + pixel[3]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                    else
                                        if(*(cache_2+4) < c_b)
                                            if(*(cache_0 + pixel[1]) < c_b)
                                                if(*(cache_0 + pixel[10]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[10]) < c_b)
                                                    if(*cache_2 < c_b)
                                                        if(*(cache_0 + pixel[0]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    if(*(cache_0 + pixel[3]) < c_b)
                                                        if(*(cache_0 + pixel[0]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                else
                    if(*(cache_0 + pixel[8]) > cb)
                        if(*(cache_0 + pixel[6]) > cb)
                            if(*cache_2 > cb)
                                if(*(cache_1+-6) > cb)
                                    if(*(cache_0 + pixel[10]) > cb)
                                        goto success;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if(*(cache_0 + pixel[8]) < c_b)
                        if(*(cache_0 + pixel[3]) > cb)
                            if(*(cache_0 + pixel[13]) > cb)
                                continue;
                            else if(*(cache_0 + pixel[13]) < c_b)
                                if(*(cache_0+-3) < c_b)
                                    if(*(cache_0 + pixel[7]) < c_b)
                                        if(*(cache_1+-6) < c_b)
                                            if(*(cache_0 + pixel[6]) < c_b)
                                                if(*(cache_0 + pixel[10]) < c_b)
                                                    if(*(cache_0 + pixel[9]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                if(*(cache_0+3) < c_b)
                                    if(*(cache_0+-3) < c_b)
                                        if(*(cache_0 + pixel[10]) < c_b)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[3]) < c_b)
                            if(*(cache_2+4) > cb)
                                if(*(cache_1+-6) < c_b)
                                    if(*(cache_0 + pixel[7]) < c_b)
                                        goto success;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(*(cache_2+4) < c_b)
                                if(*(cache_0 + pixel[6]) < c_b)
                                    if(*(cache_0+3) > cb)
                                        continue;
                                    else if(*(cache_0+3) < c_b)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                continue;
                                            else if(*(cache_0 + pixel[0]) < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                if(*(cache_0 + pixel[9]) < c_b)
                                                    if(*(cache_0 + pixel[1]) < c_b)
                                                        if(*(cache_0 + pixel[7]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0 + pixel[7]) < c_b)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    goto success;
                                                else
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0 + pixel[1]) < c_b)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    if(*(cache_0 + pixel[7]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        if(*(cache_0 + pixel[7]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                    else
                                        if(*(cache_0+-3) < c_b)
                                            if(*(cache_0 + pixel[13]) < c_b)
                                                if(*(cache_1+-6) < c_b)
                                                    if(*(cache_0 + pixel[7]) < c_b)
                                                        if(*(cache_0 + pixel[10]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                if(*(cache_1+-6) < c_b)
                                    if(*(cache_0+3) > cb)
                                        continue;
                                    else if(*(cache_0+3) < c_b)
                                        if(*(cache_0 + pixel[6]) < c_b)
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[7]) < c_b)
                                                    if(*(cache_0 + pixel[9]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        if(*(cache_0+-3) < c_b)
                                            if(*(cache_0 + pixel[13]) < c_b)
                                                if(*(cache_0 + pixel[6]) < c_b)
                                                    if(*(cache_0 + pixel[7]) < c_b)
                                                        if(*(cache_0 + pixel[10]) < c_b)
                                                            if(*(cache_0 + pixel[9]) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                        else
                            if(*(cache_0+-3) < c_b)
                                if(*(cache_0 + pixel[13]) > cb)
                                    if(*(cache_0+3) < c_b)
                                        goto success;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[13]) < c_b)
                                    if(*(cache_1+-6) < c_b)
                                        if(*(cache_0 + pixel[7]) < c_b)
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[6]) < c_b)
                                                    if(*(cache_0 + pixel[9]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0+3) < c_b)
                                        if(*(cache_0 + pixel[10]) < c_b)
                                            if(*(cache_0 + pixel[6]) < c_b)
                                                if(*(cache_1+-6) < c_b)
                                                    if(*(cache_0 + pixel[7]) < c_b)
                                                        if(*(cache_0 + pixel[9]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else
                        continue;
            else
                if(*(cache_0+-3) > cb)
                    if(*cache_2 > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                            if(*(cache_1+-6) > cb)
                                if(*(cache_0 + pixel[6]) > cb)
                                    if(*(cache_0 + pixel[13]) > cb)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[9]) > cb)
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    goto success;
                                                else if(*(cache_0 + pixel[8]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                            else if(*(cache_0 + pixel[9]) < c_b)
                                                continue;
                                            else
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[3]) > cb)
                                                if(*(cache_0 + pixel[0]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[6]) < c_b)
                                    continue;
                                else
                                    if(*(cache_0 + pixel[15]) > cb)
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_0 + pixel[9]) > cb)
                                                    if(*(cache_0 + pixel[8]) > cb)
                                                        goto success;
                                                    else if(*(cache_0 + pixel[8]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                else if(*(cache_0 + pixel[9]) < c_b)
                                                    continue;
                                                else
                                                    if(*(cache_2+4) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            if(*(cache_0 + pixel[0]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else if(*(cache_0 + pixel[10]) < c_b)
                                            continue;
                                        else
                                            if(*(cache_0 + pixel[3]) > cb)
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    if(*(cache_2+4) > cb)
                                                        if(*(cache_0 + pixel[13]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else if(*(cache_1+-6) < c_b)
                                continue;
                            else
                                if(*(cache_0+3) > cb)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                if(*(cache_0 + pixel[3]) > cb)
                                                    if(*(cache_0 + pixel[13]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[7]) < c_b)
                            if(*(cache_2+4) > cb)
                                if(*(cache_1+-6) > cb)
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_0 + pixel[15]) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_0 + pixel[1]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_0 + pixel[3]) < c_b)
                                        continue;
                                    else
                                        if(*(cache_0 + pixel[10]) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_0 + pixel[0]) > cb)
                                                    if(*(cache_0 + pixel[1]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(*(cache_1+-6) < c_b)
                                    if(*(cache_0+3) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*(cache_0 + pixel[0]) > cb)
                                                if(*(cache_0 + pixel[3]) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0+3) > cb)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_0 + pixel[3]) > cb)
                                                    if(*(cache_0 + pixel[0]) > cb)
                                                        if(*(cache_0 + pixel[15]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else if(*(cache_2+4) < c_b)
                                continue;
                            else
                                if(*(cache_0 + pixel[9]) > cb)
                                    if(*(cache_0 + pixel[0]) > cb)
                                        if(*(cache_1+-6) > cb)
                                            goto success;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else
                            if(*(cache_0 + pixel[0]) > cb)
                                if(*(cache_0 + pixel[10]) > cb)
                                    if(*(cache_2+4) > cb)
                                        if(*(cache_0 + pixel[13]) > cb)
                                            if(*(cache_1+-6) > cb)
                                                if(*(cache_0 + pixel[15]) > cb)
                                                    if(*(cache_0 + pixel[1]) > cb)
                                                        goto success;
                                                    else if(*(cache_0 + pixel[1]) < c_b)
                                                        continue;
                                                    else
                                                        if(*(cache_0 + pixel[8]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else if(*(cache_1+-6) < c_b)
                                                continue;
                                            else
                                                if(*(cache_0+3) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else if(*(cache_2+4) < c_b)
                                        if(*(cache_0 + pixel[1]) > cb)
                                            if(*(cache_0 + pixel[3]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        if(*(cache_0 + pixel[9]) > cb)
                                            if(*(cache_0 + pixel[1]) > cb)
                                                if(*(cache_0 + pixel[13]) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        if(*(cache_1+-6) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if(*(cache_0 + pixel[1]) < c_b)
                                                continue;
                                            else
                                                if(*(cache_0 + pixel[8]) > cb)
                                                    if(*(cache_1+-6) > cb)
                                                        if(*(cache_0 + pixel[13]) > cb)
                                                            if(*(cache_0 + pixel[15]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                else if(*(cache_0 + pixel[10]) < c_b)
                                    if(*(cache_0+3) > cb)
                                        if(*(cache_0 + pixel[13]) > cb)
                                            if(*(cache_2+4) > cb)
                                                if(*(cache_0 + pixel[3]) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(*(cache_0+3) < c_b)
                                        continue;
                                    else
                                        if(*(cache_1+-6) > cb)
                                            if(*(cache_0 + pixel[3]) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(*(cache_0 + pixel[3]) > cb)
                                        if(*(cache_1+-6) > cb)
                                            if(*(cache_0 + pixel[13]) > cb)
                                                if(*(cache_2+4) > cb)
                                                    if(*(cache_0 + pixel[15]) > cb)
                                                        if(*(cache_0 + pixel[1]) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if(*(cache_1+-6) < c_b)
                                            if(*(cache_0+3) > cb)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0+3) > cb)
                                                if(*(cache_0 + pixel[13]) > cb)
                                                    if(*(cache_0 + pixel[1]) > cb)
                                                        if(*(cache_2+4) > cb)
                                                            if(*(cache_0 + pixel[15]) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else
                                continue;
                    else
                        continue;
                else if(*(cache_0+-3) < c_b)
                    if(*(cache_0 + pixel[15]) > cb)
                        if(*cache_2 < c_b)
                            if(*(cache_0 + pixel[6]) < c_b)
                                if(*(cache_0 + pixel[10]) < c_b)
                                    if(*(cache_0 + pixel[7]) < c_b)
                                        goto success;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if(*(cache_0 + pixel[15]) < c_b)
                        if(*(cache_0 + pixel[10]) > cb)
                            if(*(cache_0+3) > cb)
                                continue;
                            else if(*(cache_0+3) < c_b)
                                if(*(cache_0 + pixel[3]) < c_b)
                                    if(*(cache_0 + pixel[13]) < c_b)
                                        goto success;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                if(*(cache_1+-6) < c_b)
                                    if(*(cache_0 + pixel[3]) < c_b)
                                        goto success;
                                    else
                                        continue;
                                else
                                    continue;
                        else if(*(cache_0 + pixel[10]) < c_b)
                            if(*cache_2 < c_b)
                                if(*(cache_0 + pixel[9]) > cb)
                                    if(*(cache_2+4) < c_b)
                                        goto success;
                                    else
                                        continue;
                                else if(*(cache_0 + pixel[9]) < c_b)
                                    if(*(cache_1+-6) > cb)
                                        continue;
                                    else if(*(cache_1+-6) < c_b)
                                        if(*(cache_0 + pixel[13]) < c_b)
                                            if(*(cache_0 + pixel[1]) > cb)
                                                if(*(cache_0 + pixel[7]) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else if(*(cache_0 + pixel[1]) < c_b)
                                                if(*(cache_0 + pixel[0]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[0]) < c_b)
                                                    goto success;
                                                else
                                                    if(*(cache_0 + pixel[7]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                            else
                                                if(*(cache_0 + pixel[7]) > cb)
                                                    continue;
                                                else if(*(cache_0 + pixel[7]) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        if(*(cache_0 + pixel[8]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                        else
                                            continue;
                                    else
                                        if(*(cache_0+3) < c_b)
                                            if(*(cache_0 + pixel[3]) < c_b)
                                                goto success;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(*(cache_2+4) < c_b)
                                        if(*(cache_1+-6) > cb)
                                            continue;
                                        else if(*(cache_1+-6) < c_b)
                                            if(*(cache_0 + pixel[13]) < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            if(*(cache_0+3) < c_b)
                                                if(*(cache_0 + pixel[3]) < c_b)
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                            else
                                continue;
                        else
                            if(*(cache_0 + pixel[3]) < c_b)
                                if(*(cache_1+-6) > cb)
                                    continue;
                                else if(*(cache_1+-6) < c_b)
                                    if(*(cache_2+4) < c_b)
                                        if(*(cache_0 + pixel[13]) < c_b)
                                            if(*cache_2 < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    if(*(cache_0 + pixel[0]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(*(cache_0+3) < c_b)
                                        if(*(cache_2+4) < c_b)
                                            if(*cache_2 < c_b)
                                                if(*(cache_0 + pixel[1]) < c_b)
                                                    if(*(cache_0 + pixel[13]) < c_b)
                                                        if(*(cache_0 + pixel[0]) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                    else
                        if(*(cache_0 + pixel[6]) < c_b)
                            if(*cache_2 < c_b)
                                if(*(cache_0 + pixel[7]) < c_b)
                                    if(*(cache_1+-6) < c_b)
                                        if(*(cache_0 + pixel[13]) < c_b)
                                            if(*(cache_0 + pixel[10]) < c_b)
                                                if(*(cache_0 + pixel[9]) < c_b)
                                                    if(*(cache_0 + pixel[8]) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                else
                    continue;
			success:	
#ifdef MV_ADAS_USE_FAST_STATIC
				if(total >= MAX_XY_CORNER)
				{
					goto mv_fast_end;
				}
#else
				if(total >= rsize)																
				{	
					nOldSize = rsize;
					rsize *=2;																	
					ret=(xy*)my_realloc(ret, rsize*sizeof(xy));
				}	
#endif
				ret[total].x = cache_0-line_min;												
				ret[total++].y = y;
		}																						
	}		
#ifdef MV_ADAS_USE_FAST_STATIC
mv_fast_end:
#endif

	*num = total;																				
	return ret;																					
}	

/*
Function process:
	+ fast corner detect
	Fan-in : 
	        + mvCornerDetct()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void oast9_16(const uint8_t* im,const uint8_t* pMask, s32 xsize, s32 ysize, s32 b, \
			  s32* num_corners,xy *pXyCorner,s32 nId,s32 nStart_y,s32 nEnd_y,s32 nStart_x,s32 nEnd_x)
{
	s32 total = 0;
	register s32 x, y;
	//register s32 xsizeB=xsize - 10;
	//register s32 ysizeB=ysize - 10;
	register s32 offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, \
		offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
	register s32 width;
	register s32 njump_x;
	 s32 cb ;
	 s32 c_b ;
	 s32 start_y,End_y;

    njump_x  = 16;
	start_y = nStart_y;
	End_y = nEnd_y;

	offset0=(-3)+(0)*xsize;
	offset1=(-3)+(-1)*xsize;
	offset2=(-2)+(-2)*xsize;
	offset3=(-1)+(-3)*xsize;
	offset4=(0)+(-3)*xsize;
	offset5=(1)+(-3)*xsize;
	offset6=(2)+(-2)*xsize;
	offset7=(3)+(-1)*xsize;
	offset8=(3)+(0)*xsize;
	offset9=(3)+(1)*xsize;
	offset10=(2)+(2)*xsize;
	offset11=(1)+(3)*xsize;
	offset12=(0)+(3)*xsize;
	offset13=(-1)+(3)*xsize;
	offset14=(-2)+(2)*xsize;
	offset15=(-3)+(1)*xsize;

	width = xsize;

	for(y = start_y; y < End_y; y++)			
	{										
		x = nStart_x;								
		while(1)							
		{									
			x++;			
			if(x > nEnd_x)	
				break;
			else
			{
				register const uint8_t* const p = im + y * width + x;
				register const uint8_t* const pmask = pMask + y * width + x;
				if (!*pmask )
				{
					x += njump_x;
					continue;
				}
				
				 cb = *p + b;
				 c_b = *p - b;
				if(p[offset0] > cb)
					if(p[offset2] > cb)
						if(p[offset4] > cb)
							if(p[offset5] > cb)
								if(p[offset7] > cb)
									if(p[offset3] > cb)
										if(p[offset1] > cb)
											if(p[offset6] > cb)
												if(p[offset8] > cb)
												{}
												else
													if(p[offset15] > cb)
													{}
													else
														continue;
											else
												if(p[offset13] > cb)
													if(p[offset14] > cb)
														if(p[offset15] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset8] > cb)
												if(p[offset9] > cb)
													if(p[offset10] > cb)
														if(p[offset6] > cb)
														{}
														else
															if(p[offset11] > cb)
																if(p[offset12] > cb)
																	if(p[offset13] > cb)
																		if(p[offset14] > cb)
																			if(p[offset15] > cb)
																			{}
																			else
																				continue;
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset10] > cb)
											if(p[offset11] > cb)
												if(p[offset12] > cb)
													if(p[offset8] > cb)
														if(p[offset9] > cb)
															if(p[offset6] > cb)
															{}
															else
																if(p[offset13] > cb)
																	if(p[offset14] > cb)
																		if(p[offset15] > cb)
																		{}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
														else
															if(p[offset1] > cb)
																if(p[offset13] > cb)
																	if(p[offset14] > cb)
																		if(p[offset15] > cb)
																		{}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
													else
														if(p[offset1] > cb)
															if(p[offset13] > cb)
																if(p[offset14] > cb)
																	if(p[offset15] > cb)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else if(p[offset7] < c_b)
									if(p[offset14] > cb)
										if(p[offset15] > cb)
											if(p[offset1] > cb)
												if(p[offset3] > cb)
													if(p[offset6] > cb)
													{}
													else
														if(p[offset13] > cb)
														{}
														else
															continue;
												else
													if(p[offset10] > cb)
														if(p[offset11] > cb)
															if(p[offset12] > cb)
																if(p[offset13] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset8] > cb)
													if(p[offset9] > cb)
														if(p[offset10] > cb)
															if(p[offset11] > cb)
																if(p[offset12] > cb)
																	if(p[offset13] > cb)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else if(p[offset14] < c_b)
										if(p[offset8] < c_b)
											if(p[offset9] < c_b)
												if(p[offset10] < c_b)
													if(p[offset11] < c_b)
														if(p[offset12] < c_b)
															if(p[offset13] < c_b)
																if(p[offset6] < c_b)
																{}
																else
																	if(p[offset15] < c_b)
																	{}
																	else
																		continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									if(p[offset14] > cb)
										if(p[offset15] > cb)
											if(p[offset1] > cb)
												if(p[offset3] > cb)
													if(p[offset6] > cb)
													{}
													else
														if(p[offset13] > cb)
														{}
														else
															continue;
												else
													if(p[offset10] > cb)
														if(p[offset11] > cb)
															if(p[offset12] > cb)
																if(p[offset13] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset8] > cb)
													if(p[offset9] > cb)
														if(p[offset10] > cb)
															if(p[offset11] > cb)
																if(p[offset12] > cb)
																	if(p[offset13] > cb)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
							else if(p[offset5] < c_b)
								if(p[offset12] > cb)
									if(p[offset13] > cb)
										if(p[offset14] > cb)
											if(p[offset15] > cb)
												if(p[offset1] > cb)
													if(p[offset3] > cb)
													{}
													else
														if(p[offset10] > cb)
															if(p[offset11] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset8] > cb)
														if(p[offset9] > cb)
															if(p[offset10] > cb)
																if(p[offset11] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset6] > cb)
													if(p[offset7] > cb)
														if(p[offset8] > cb)
															if(p[offset9] > cb)
																if(p[offset10] > cb)
																	if(p[offset11] > cb)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else if(p[offset12] < c_b)
									if(p[offset7] < c_b)
										if(p[offset8] < c_b)
											if(p[offset9] < c_b)
												if(p[offset10] < c_b)
													if(p[offset11] < c_b)
														if(p[offset13] < c_b)
															if(p[offset6] < c_b)
															{}
															else
																if(p[offset14] < c_b)
																	if(p[offset15] < c_b)
																	{}
																	else
																		continue;
																else
																	continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								if(p[offset12] > cb)
									if(p[offset13] > cb)
										if(p[offset14] > cb)
											if(p[offset15] > cb)
												if(p[offset1] > cb)
													if(p[offset3] > cb)
													{}
													else
														if(p[offset10] > cb)
															if(p[offset11] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset8] > cb)
														if(p[offset9] > cb)
															if(p[offset10] > cb)
																if(p[offset11] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset6] > cb)
													if(p[offset7] > cb)
														if(p[offset8] > cb)
															if(p[offset9] > cb)
																if(p[offset10] > cb)
																	if(p[offset11] > cb)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else if(p[offset12] < c_b)
									if(p[offset7] < c_b)
										if(p[offset8] < c_b)
											if(p[offset9] < c_b)
												if(p[offset10] < c_b)
													if(p[offset11] < c_b)
														if(p[offset13] < c_b)
															if(p[offset14] < c_b)
																if(p[offset6] < c_b)
																{}
																else
																	if(p[offset15] < c_b)
																	{}
																	else
																		continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
						else if(p[offset4] < c_b)
							if(p[offset11] > cb)
								if(p[offset12] > cb)
									if(p[offset13] > cb)
										if(p[offset10] > cb)
											if(p[offset14] > cb)
												if(p[offset15] > cb)
													if(p[offset1] > cb)
													{}
													else
														if(p[offset8] > cb)
															if(p[offset9] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset6] > cb)
														if(p[offset7] > cb)
															if(p[offset8] > cb)
																if(p[offset9] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset5] > cb)
													if(p[offset6] > cb)
														if(p[offset7] > cb)
															if(p[offset8] > cb)
																if(p[offset9] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset1] > cb)
												if(p[offset3] > cb)
													if(p[offset14] > cb)
														if(p[offset15] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else if(p[offset11] < c_b)
								if(p[offset7] < c_b)
									if(p[offset8] < c_b)
										if(p[offset9] < c_b)
											if(p[offset10] < c_b)
												if(p[offset6] < c_b)
													if(p[offset5] < c_b)
														if(p[offset3] < c_b)
														{}
														else
															if(p[offset12] < c_b)
															{}
															else
																continue;
													else
														if(p[offset12] < c_b)
															if(p[offset13] < c_b)
																if(p[offset14] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													if(p[offset12] < c_b)
														if(p[offset13] < c_b)
															if(p[offset14] < c_b)
																if(p[offset15] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else
							if(p[offset11] > cb)
								if(p[offset12] > cb)
									if(p[offset13] > cb)
										if(p[offset10] > cb)
											if(p[offset14] > cb)
												if(p[offset15] > cb)
													if(p[offset1] > cb)
													{}
													else
														if(p[offset8] > cb)
															if(p[offset9] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset6] > cb)
														if(p[offset7] > cb)
															if(p[offset8] > cb)
																if(p[offset9] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset5] > cb)
													if(p[offset6] > cb)
														if(p[offset7] > cb)
															if(p[offset8] > cb)
																if(p[offset9] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset1] > cb)
												if(p[offset3] > cb)
													if(p[offset14] > cb)
														if(p[offset15] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else if(p[offset11] < c_b)
								if(p[offset7] < c_b)
									if(p[offset8] < c_b)
										if(p[offset9] < c_b)
											if(p[offset10] < c_b)
												if(p[offset12] < c_b)
													if(p[offset13] < c_b)
														if(p[offset6] < c_b)
															if(p[offset5] < c_b)
															{}
															else
																if(p[offset14] < c_b)
																{}
																else
																	continue;
														else
															if(p[offset14] < c_b)
																if(p[offset15] < c_b)
																{}
																else
																	continue;
															else
																continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
					else if(p[offset2] < c_b)
						if(p[offset9] > cb)
							if(p[offset10] > cb)
								if(p[offset11] > cb)
									if(p[offset8] > cb)
										if(p[offset12] > cb)
											if(p[offset13] > cb)
												if(p[offset14] > cb)
													if(p[offset15] > cb)
													{}
													else
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset5] > cb)
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset4] > cb)
													if(p[offset5] > cb)
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset3] > cb)
												if(p[offset4] > cb)
													if(p[offset5] > cb)
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset1] > cb)
											if(p[offset12] > cb)
												if(p[offset13] > cb)
													if(p[offset14] > cb)
														if(p[offset15] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else if(p[offset9] < c_b)
							if(p[offset7] < c_b)
								if(p[offset8] < c_b)
									if(p[offset6] < c_b)
										if(p[offset5] < c_b)
											if(p[offset4] < c_b)
												if(p[offset3] < c_b)
													if(p[offset1] < c_b)
													{}
													else
														if(p[offset10] < c_b)
														{}
														else
															continue;
												else
													if(p[offset10] < c_b)
														if(p[offset11] < c_b)
															if(p[offset12] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset10] < c_b)
													if(p[offset11] < c_b)
														if(p[offset12] < c_b)
															if(p[offset13] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset10] < c_b)
												if(p[offset11] < c_b)
													if(p[offset12] < c_b)
														if(p[offset13] < c_b)
															if(p[offset14] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset10] < c_b)
											if(p[offset11] < c_b)
												if(p[offset12] < c_b)
													if(p[offset13] < c_b)
														if(p[offset14] < c_b)
															if(p[offset15] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
					else
						if(p[offset9] > cb)
							if(p[offset10] > cb)
								if(p[offset11] > cb)
									if(p[offset8] > cb)
										if(p[offset12] > cb)
											if(p[offset13] > cb)
												if(p[offset14] > cb)
													if(p[offset15] > cb)
													{}
													else
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset5] > cb)
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset4] > cb)
													if(p[offset5] > cb)
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset3] > cb)
												if(p[offset4] > cb)
													if(p[offset5] > cb)
														if(p[offset6] > cb)
															if(p[offset7] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset1] > cb)
											if(p[offset12] > cb)
												if(p[offset13] > cb)
													if(p[offset14] > cb)
														if(p[offset15] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else if(p[offset9] < c_b)
							if(p[offset7] < c_b)
								if(p[offset8] < c_b)
									if(p[offset10] < c_b)
										if(p[offset11] < c_b)
											if(p[offset6] < c_b)
												if(p[offset5] < c_b)
													if(p[offset4] < c_b)
														if(p[offset3] < c_b)
														{}
														else
															if(p[offset12] < c_b)
															{}
															else
																continue;
													else
														if(p[offset12] < c_b)
															if(p[offset13] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset12] < c_b)
														if(p[offset13] < c_b)
															if(p[offset14] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset12] < c_b)
													if(p[offset13] < c_b)
														if(p[offset14] < c_b)
															if(p[offset15] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else
							continue;
				else if(p[offset0] < c_b)
					if(p[offset2] > cb)
						if(p[offset9] > cb)
							if(p[offset7] > cb)
								if(p[offset8] > cb)
									if(p[offset6] > cb)
										if(p[offset5] > cb)
											if(p[offset4] > cb)
												if(p[offset3] > cb)
													if(p[offset1] > cb)
													{}
													else
														if(p[offset10] > cb)
														{}
														else
															continue;
												else
													if(p[offset10] > cb)
														if(p[offset11] > cb)
															if(p[offset12] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset10] > cb)
													if(p[offset11] > cb)
														if(p[offset12] > cb)
															if(p[offset13] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset10] > cb)
												if(p[offset11] > cb)
													if(p[offset12] > cb)
														if(p[offset13] > cb)
															if(p[offset14] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset10] > cb)
											if(p[offset11] > cb)
												if(p[offset12] > cb)
													if(p[offset13] > cb)
														if(p[offset14] > cb)
															if(p[offset15] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else if(p[offset9] < c_b)
							if(p[offset10] < c_b)
								if(p[offset11] < c_b)
									if(p[offset8] < c_b)
										if(p[offset12] < c_b)
											if(p[offset13] < c_b)
												if(p[offset14] < c_b)
													if(p[offset15] < c_b)
													{}
													else
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset5] < c_b)
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset4] < c_b)
													if(p[offset5] < c_b)
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset3] < c_b)
												if(p[offset4] < c_b)
													if(p[offset5] < c_b)
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset1] < c_b)
											if(p[offset12] < c_b)
												if(p[offset13] < c_b)
													if(p[offset14] < c_b)
														if(p[offset15] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
					else if(p[offset2] < c_b)
						if(p[offset4] > cb)
							if(p[offset11] > cb)
								if(p[offset7] > cb)
									if(p[offset8] > cb)
										if(p[offset9] > cb)
											if(p[offset10] > cb)
												if(p[offset6] > cb)
													if(p[offset5] > cb)
														if(p[offset3] > cb)
														{}
														else
															if(p[offset12] > cb)
															{}
															else
																continue;
													else
														if(p[offset12] > cb)
															if(p[offset13] > cb)
																if(p[offset14] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													if(p[offset12] > cb)
														if(p[offset13] > cb)
															if(p[offset14] > cb)
																if(p[offset15] > cb)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else if(p[offset11] < c_b)
								if(p[offset12] < c_b)
									if(p[offset13] < c_b)
										if(p[offset10] < c_b)
											if(p[offset14] < c_b)
												if(p[offset15] < c_b)
													if(p[offset1] < c_b)
													{}
													else
														if(p[offset8] < c_b)
															if(p[offset9] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset6] < c_b)
														if(p[offset7] < c_b)
															if(p[offset8] < c_b)
																if(p[offset9] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset5] < c_b)
													if(p[offset6] < c_b)
														if(p[offset7] < c_b)
															if(p[offset8] < c_b)
																if(p[offset9] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset1] < c_b)
												if(p[offset3] < c_b)
													if(p[offset14] < c_b)
														if(p[offset15] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else if(p[offset4] < c_b)
							if(p[offset5] > cb)
								if(p[offset12] > cb)
									if(p[offset7] > cb)
										if(p[offset8] > cb)
											if(p[offset9] > cb)
												if(p[offset10] > cb)
													if(p[offset11] > cb)
														if(p[offset13] > cb)
															if(p[offset6] > cb)
															{}
															else
																if(p[offset14] > cb)
																	if(p[offset15] > cb)
																	{}
																	else
																		continue;
																else
																	continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else if(p[offset12] < c_b)
									if(p[offset13] < c_b)
										if(p[offset14] < c_b)
											if(p[offset15] < c_b)
												if(p[offset1] < c_b)
													if(p[offset3] < c_b)
													{}
													else
														if(p[offset10] < c_b)
															if(p[offset11] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset8] < c_b)
														if(p[offset9] < c_b)
															if(p[offset10] < c_b)
																if(p[offset11] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset6] < c_b)
													if(p[offset7] < c_b)
														if(p[offset8] < c_b)
															if(p[offset9] < c_b)
																if(p[offset10] < c_b)
																	if(p[offset11] < c_b)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else if(p[offset5] < c_b)
								if(p[offset7] > cb)
									if(p[offset14] > cb)
										if(p[offset8] > cb)
											if(p[offset9] > cb)
												if(p[offset10] > cb)
													if(p[offset11] > cb)
														if(p[offset12] > cb)
															if(p[offset13] > cb)
																if(p[offset6] > cb)
																{}
																else
																	if(p[offset15] > cb)
																	{}
																	else
																		continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else if(p[offset14] < c_b)
										if(p[offset15] < c_b)
											if(p[offset1] < c_b)
												if(p[offset3] < c_b)
													if(p[offset6] < c_b)
													{}
													else
														if(p[offset13] < c_b)
														{}
														else
															continue;
												else
													if(p[offset10] < c_b)
														if(p[offset11] < c_b)
															if(p[offset12] < c_b)
																if(p[offset13] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset8] < c_b)
													if(p[offset9] < c_b)
														if(p[offset10] < c_b)
															if(p[offset11] < c_b)
																if(p[offset12] < c_b)
																	if(p[offset13] < c_b)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else if(p[offset7] < c_b)
									if(p[offset3] < c_b)
										if(p[offset1] < c_b)
											if(p[offset6] < c_b)
												if(p[offset8] < c_b)
												{}
												else
													if(p[offset15] < c_b)
													{}
													else
														continue;
											else
												if(p[offset13] < c_b)
													if(p[offset14] < c_b)
														if(p[offset15] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset8] < c_b)
												if(p[offset9] < c_b)
													if(p[offset10] < c_b)
														if(p[offset6] < c_b)
														{}
														else
															if(p[offset11] < c_b)
																if(p[offset12] < c_b)
																	if(p[offset13] < c_b)
																		if(p[offset14] < c_b)
																			if(p[offset15] < c_b)
																			{}
																			else
																				continue;
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset10] < c_b)
											if(p[offset11] < c_b)
												if(p[offset12] < c_b)
													if(p[offset8] < c_b)
														if(p[offset9] < c_b)
															if(p[offset6] < c_b)
															{}
															else
																if(p[offset13] < c_b)
																	if(p[offset14] < c_b)
																		if(p[offset15] < c_b)
																		{}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
														else
															if(p[offset1] < c_b)
																if(p[offset13] < c_b)
																	if(p[offset14] < c_b)
																		if(p[offset15] < c_b)
																		{}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
													else
														if(p[offset1] < c_b)
															if(p[offset13] < c_b)
																if(p[offset14] < c_b)
																	if(p[offset15] < c_b)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if(p[offset14] < c_b)
										if(p[offset15] < c_b)
											if(p[offset1] < c_b)
												if(p[offset3] < c_b)
													if(p[offset6] < c_b)
													{}
													else
														if(p[offset13] < c_b)
														{}
														else
															continue;
												else
													if(p[offset10] < c_b)
														if(p[offset11] < c_b)
															if(p[offset12] < c_b)
																if(p[offset13] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset8] < c_b)
													if(p[offset9] < c_b)
														if(p[offset10] < c_b)
															if(p[offset11] < c_b)
																if(p[offset12] < c_b)
																	if(p[offset13] < c_b)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
							else
								if(p[offset12] > cb)
									if(p[offset7] > cb)
										if(p[offset8] > cb)
											if(p[offset9] > cb)
												if(p[offset10] > cb)
													if(p[offset11] > cb)
														if(p[offset13] > cb)
															if(p[offset14] > cb)
																if(p[offset6] > cb)
																{}
																else
																	if(p[offset15] > cb)
																	{}
																	else
																		continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else if(p[offset12] < c_b)
									if(p[offset13] < c_b)
										if(p[offset14] < c_b)
											if(p[offset15] < c_b)
												if(p[offset1] < c_b)
													if(p[offset3] < c_b)
													{}
													else
														if(p[offset10] < c_b)
															if(p[offset11] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset8] < c_b)
														if(p[offset9] < c_b)
															if(p[offset10] < c_b)
																if(p[offset11] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset6] < c_b)
													if(p[offset7] < c_b)
														if(p[offset8] < c_b)
															if(p[offset9] < c_b)
																if(p[offset10] < c_b)
																	if(p[offset11] < c_b)
																	{}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
						else
							if(p[offset11] > cb)
								if(p[offset7] > cb)
									if(p[offset8] > cb)
										if(p[offset9] > cb)
											if(p[offset10] > cb)
												if(p[offset12] > cb)
													if(p[offset13] > cb)
														if(p[offset6] > cb)
															if(p[offset5] > cb)
															{}
															else
																if(p[offset14] > cb)
																{}
																else
																	continue;
														else
															if(p[offset14] > cb)
																if(p[offset15] > cb)
																{}
																else
																	continue;
															else
																continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else if(p[offset11] < c_b)
								if(p[offset12] < c_b)
									if(p[offset13] < c_b)
										if(p[offset10] < c_b)
											if(p[offset14] < c_b)
												if(p[offset15] < c_b)
													if(p[offset1] < c_b)
													{}
													else
														if(p[offset8] < c_b)
															if(p[offset9] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset6] < c_b)
														if(p[offset7] < c_b)
															if(p[offset8] < c_b)
																if(p[offset9] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset5] < c_b)
													if(p[offset6] < c_b)
														if(p[offset7] < c_b)
															if(p[offset8] < c_b)
																if(p[offset9] < c_b)
																{}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset1] < c_b)
												if(p[offset3] < c_b)
													if(p[offset14] < c_b)
														if(p[offset15] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else
								continue;
					else
						if(p[offset9] > cb)
							if(p[offset7] > cb)
								if(p[offset8] > cb)
									if(p[offset10] > cb)
										if(p[offset11] > cb)
											if(p[offset6] > cb)
												if(p[offset5] > cb)
													if(p[offset4] > cb)
														if(p[offset3] > cb)
														{}
														else
															if(p[offset12] > cb)
															{}
															else
																continue;
													else
														if(p[offset12] > cb)
															if(p[offset13] > cb)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset12] > cb)
														if(p[offset13] > cb)
															if(p[offset14] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset12] > cb)
													if(p[offset13] > cb)
														if(p[offset14] > cb)
															if(p[offset15] > cb)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else if(p[offset9] < c_b)
							if(p[offset10] < c_b)
								if(p[offset11] < c_b)
									if(p[offset8] < c_b)
										if(p[offset12] < c_b)
											if(p[offset13] < c_b)
												if(p[offset14] < c_b)
													if(p[offset15] < c_b)
													{}
													else
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
												else
													if(p[offset5] < c_b)
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if(p[offset4] < c_b)
													if(p[offset5] < c_b)
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset3] < c_b)
												if(p[offset4] < c_b)
													if(p[offset5] < c_b)
														if(p[offset6] < c_b)
															if(p[offset7] < c_b)
															{}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset1] < c_b)
											if(p[offset12] < c_b)
												if(p[offset13] < c_b)
													if(p[offset14] < c_b)
														if(p[offset15] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
				else
					if(p[offset7] > cb)
						if(p[offset8] > cb)
							if(p[offset9] > cb)
								if(p[offset6] > cb)
									if(p[offset5] > cb)
										if(p[offset4] > cb)
											if(p[offset3] > cb)
												if(p[offset2] > cb)
													if(p[offset1] > cb)
													{}
													else
														if(p[offset10] > cb)
														{}
														else
															continue;
												else
													if(p[offset10] > cb)
														if(p[offset11] > cb)
														{}
														else
															continue;
													else
														continue;
											else
												if(p[offset10] > cb)
													if(p[offset11] > cb)
														if(p[offset12] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset10] > cb)
												if(p[offset11] > cb)
													if(p[offset12] > cb)
														if(p[offset13] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset10] > cb)
											if(p[offset11] > cb)
												if(p[offset12] > cb)
													if(p[offset13] > cb)
														if(p[offset14] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if(p[offset10] > cb)
										if(p[offset11] > cb)
											if(p[offset12] > cb)
												if(p[offset13] > cb)
													if(p[offset14] > cb)
														if(p[offset15] > cb)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else if(p[offset7] < c_b)
						if(p[offset8] < c_b)
							if(p[offset9] < c_b)
								if(p[offset6] < c_b)
									if(p[offset5] < c_b)
										if(p[offset4] < c_b)
											if(p[offset3] < c_b)
												if(p[offset2] < c_b)
													if(p[offset1] < c_b)
													{}
													else
														if(p[offset10] < c_b)
														{}
														else
															continue;
												else
													if(p[offset10] < c_b)
														if(p[offset11] < c_b)
														{}
														else
															continue;
													else
														continue;
											else
												if(p[offset10] < c_b)
													if(p[offset11] < c_b)
														if(p[offset12] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if(p[offset10] < c_b)
												if(p[offset11] < c_b)
													if(p[offset12] < c_b)
														if(p[offset13] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if(p[offset10] < c_b)
											if(p[offset11] < c_b)
												if(p[offset12] < c_b)
													if(p[offset13] < c_b)
														if(p[offset14] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if(p[offset10] < c_b)
										if(p[offset11] < c_b)
											if(p[offset12] < c_b)
												if(p[offset13] < c_b)
													if(p[offset14] < c_b)
														if(p[offset15] < c_b)
														{}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else
						continue;
			}


			if(total >= MAX_XY_CORNER)
			{
				goto mv_fast_end;
			}
			
				
			pXyCorner[total].x = x;				
			pXyCorner[total].y = y;				
			total++;
			x += 5;
		}									
	}		

mv_fast_end:
	*num_corners = total;					
								
}
