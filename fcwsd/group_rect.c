/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.h
Version: 1.0		Date: 2017-02-28		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are defined to cluster and merge the rects of detector.
	The following function types are included:
	+ getGroupedRectanglesCar(): Clustered and merge the deteced list of rects
	+ predicateCar(): Decide two rects are adjacent
	+ partitionCar: Clustered the list of rects.
Deviation:

History:
	+ Version: 1.0		Date: 2017-02-28		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "vehicle_type.h"
#include "vehicle_det.h"
#include "group_rect.h"

/*
I/O:	   Name		        Type	                 Size			  	    Content

[in]       pVehicleDetor    FCWSDetectorGlobalPara*                         Global parameter for detector
[in/out]   rectList		    lbpRectCar*	             sizeof(lbpRectCar*)    Input/Output rects to be clustered
[in/out]   prectNum		    s32	                     4-Byte	                Number of input/output rects
[in]       groupThreshold	s32	                     4-Byte	                Threshold for a rect group being a cluster
[in]       eps		        float32_t	             sizeof(float32_t)	    Threshold for rects cluster.
					     			  					  
[out]	   returned value   s32                      4-Byte	                Clustering number.

Realized function:
	+ Clustered the deteced list of rects.
*/
int getGroupedRectanglesCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpRectCar *rectList,s32 *prectNum , const s32 groupThreshold, const float32_t eps);

/*
I/O:	   Name		     Type	       Size			  	      Content
					     			  					  
[in]       eps		     float32_t	   sizeof(float32_t)	  threshold for rects cluster.
[in]	   r1		     lbpRectCar	   sizeof(lbpRectCar)	  Input rect1.
[in]	   r2		     lbpRectCar	   sizeof(lbpRectCar)	  Input rect2.
					     			  					  
[out]	returned value   Bool          1-Byte	              If 1, rect1 and rect2 are in the same cluster.

Realized function:
	+ If rect1 is in the neighbourhood of rect2, return 1.
*/
static Bool predicateCar(const float32_t eps, const lbpRectCar r1, const lbpRectCar r2);

/*
I/O:	   Name		     Type	                  Size			  	      Content

[in]   pVehicleDetor     FCWSDetectorGlobalPara*                          Global parameter for detector
[in]       vec		     lbpRectCar*	          sizeof(lbpRectCar*)	  Input rects to be clustered
[in]	   nVecNum		 s32	                  4-Byte	              Number of input rects
[in]       eps		     float32_t	              sizeof(float32_t)	      Threshold for rects cluster.
					     			  					  
[out]	returned value   s32                      4-Byte	              Clustering number.

Realized function:
	+ Clustered the list of rects.
*/
static s32  partitionCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpRectCar *vecIn, const s32 nVecNum, const float32_t eps);

/*
Function process:
	+ If rect1 is in the neighbourhood of rect2, return 1
	Fan-in : 
	        + partitionCar()
	Fan-out: N/A

	ATTENTION: 
*/
static Bool  predicateCar(const float32_t eps, const lbpRectCar r1, const lbpRectCar r2)
{
	Bool ret;

	float delta = (float) (eps * (MIN(r1.width, r2.width) + MIN(r1.height, r2.height)) * 0.5);

	ret =  ABS(r1.x - r2.x) <= delta &&
		   ABS(r1.y - r2.y) <= delta &&
		   ABS(r1.x + r1.width - r2.x - r2.width) <= delta &&
		   ABS(r1.y + r1.height - r2.y - r2.height) <= delta;

    return ret;
}

/*
Function process:
	+ Clustered the list of rects.
	Fan-in : 
	        + getGroupedRectanglesCar()
	Fan-out: 
	        + predicateCar()

	ATTENTION: 
*/
static s32 partitionCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpRectCar *vecIn, const s32 nVecNum, const float32_t eps)
{
    s32 i, j, N = nVecNum;
    lbpRectCar* vec = vecIn;
    const s32 RANK = 1;

	s32 *nodes  = pVehicleDetor->pTreeNode;
    s32 *labels = pVehicleDetor->pLabNode;
	s32 k, root, root2, parent;
	s32 nclasses = 0;

	/* Build the queue element as the root node of the forest */
    for (i = 0; i < (N << 1); i++) 
	{
        nodes[i++] = -1;
        nodes[i]   = 0;
    }

    for (i = 0; i < N; i++)
	{
        root = i;

        while (nodes[(root<<1)] >= 0)
		{
			root = nodes[(root<<1)];
		}

        for (j = 0; j < N; j++ ) 
		{

            if( i == j || !predicateCar(eps, vec[i], vec[j]))
			{
                continue;
			}

            root2 = j;

            while (nodes[(root2<<1)] >= 0)
			{
                root2 = nodes[(root2<<1)];
			}

            if (root2 != root) 
			{
                int rank = nodes[(root<<1) + RANK], rank2 = nodes[(root2<<1) +RANK];
				 
                if (rank > rank2)
				{
                    nodes[(root2<<1)] = root;
				}
                else 
				{
                    nodes[(root<<1)] = root2;
                    nodes[(root2<<1) + RANK] += rank == rank2;
                    root = root2;
                }

                k = j;
                /* compress the path from node2 to root */
                while ((parent = nodes[(k<<1)]) >= 0) 
				{
                    nodes[(k<<1)] = root;
                    k = parent;
                }

                /* compress the path from node to root */
                k = i;
                while ((parent = nodes[(k<<1)]) >= 0)
				{
                    nodes[(k<<1)] = root;
                    k = parent;
                }
            }
        }
    }

    /* Final O(N) pass: enumerate classes */
    for (i = 0; i < N; i++) 
	{
        root = i;
        while (nodes[(root<<1)] >= 0)
		{
            root = nodes[(root<<1)];
		}

        if (nodes[(root<<1)+RANK] >= 0)
		{
            nodes[(root<<1)+RANK] = ~nclasses++;
		}
        labels[i] = ~nodes[(root<<1)+RANK];
    }

    return nclasses;
}

/*
Function process:
	+ Clustered and merge the deteced list of rects.
	Fan-in : 
	        + lbpGroupWindowsCar()()
	Fan-out: 
	        + partitionCar()

	ATTENTION: 
*/
int getGroupedRectanglesCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpRectCar *rectList, s32 *prectNum , const s32 groupThreshold, const float32_t eps)
{
	uint8_t *pTr = NULL;
	
	s32 *rWeights = NULL;
	s32 *labels   = NULL;

	lbpRectCar *rRects = NULL;

	s32 classesNum, cls, n1, n2, dx, dy, i, j, nLabels;
	
	float32_t s; 
	lbpRectCar r, r1,r2;
	
	classesNum = partitionCar(pVehicleDetor, rectList, *prectNum, eps);

	rRects = (lbpRectCar*)pVehicleDetor->pbulicBuff; 
	pTr = ALIGN_16BYTE(rRects + classesNum );
	memset( rRects, 0, classesNum * sizeof(lbpRectCar) );

	rWeights = (s32 *)pTr;
	pTr = ALIGN_16BYTE(rWeights  +  classesNum ); 
	memset(rWeights,0,classesNum * sizeof(s32) );

    nLabels = *prectNum;
	labels = pVehicleDetor->pLabNode;

    for (i = 0; i < nLabels; i++)
	{
        cls = labels[i];
        rRects[cls].x		+= rectList[i].x;
        rRects[cls].y	    += rectList[i].y;
        rRects[cls].width	+= rectList[i].width;
		rRects[cls].height	+= rectList[i].height;
        rWeights[cls]++;
    }
    for (i = 0; i < classesNum; i++) 
	{
		r = rRects[i];
		s = 1.f / rWeights[i];
		rRects[i].x = myRound(r.x*s);
		rRects[i].y = myRound(r.y*s);
		rRects[i].width = myRound(r.width*s);
		rRects[i].height = myRound(r.height*s);
		rRects[i].confidence = rWeights[i];
    }

    *prectNum = 0;

	for (i = 0; i < classesNum; i++) 
	{
		r1 = rRects[i];
		n1 = rWeights[i];
		if( n1 < groupThreshold )
		{
			continue;
		}

		/* filter out small car rectangles inside large rectangles */
		for (j = 0; j < classesNum; j++) 
		{
			n2 = rWeights[j];
			/* if it is the same rectangle, or the number of rectangles in class j is < group threshold, 
			 do nothing */
			if (j == i || n2 <= groupThreshold)
			{
				continue;
			}
			r2 = rRects[j];

			/*dx = myRound( r2.width * eps );
			dy = myRound( r2.height * eps );

			if (i != j &&
				r1.x >= r2.x - dx &&
				r1.y >= r2.y - dy &&
				r1.x + r1.width <= r2.x + r2.width + dx &&
				r1.y + r1.height <= r2.y + r2.height + dy &&
			(n2 > MAX(3, n1) || n1 < 3))
			{
				break;
			}*/


			dx = myRound( r2.width * 0.3 );
			dy = myRound( r2.height * 0.3 );

			if (i != j &&
				r1.x >= (r2.x) &&
				r1.x <= r2.x + dx &&
				r1.y >= (r2.y) &&
				r1.y <= r2.y + dy &&
				r1.x + r1.width <= r2.x + r2.width  &&
				r1.x + r1.width >= r2.x + r2.width - dx &&
				r1.y + r1.height <= r2.y + r2.height &&
				r1.y + r1.height >= r2.y + r2.height - dy && 
			(n2 > MAX(3, n1) || n1 < 3))
			{
				break;
			}


			dx = myRound( r1.width * 0.3 );
			dy = myRound( r1.height * 0.3 );

			if (i != j &&
				r2.x >= (r1.x) &&
				r2.x <= r1.x+dx &&
				r2.y >= (r1.y) &&
				r2.y<=r1.y +dy &&
				r2.x + r2.width <= r1.x + r1.width  &&
				r2.x + r2.width >= r1.x + r1.width-dx &&
				r2.y + r2.height <= r1.y + r1.height &&
				r2.y + r2.height >= r1.y + r1.height-dy && 
			(n2 >= MAX(3, n1) || n1 < 3))
			{
				break;
			}
		}

		if (j == classesNum) 
		{
			rectList[(*prectNum)++] = r1;
		}
	}


    return 0;
}
