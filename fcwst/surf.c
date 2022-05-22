/**************************************************************************************************************
Copyright Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: surf.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate of surf features.
	The following function types are included:
	+ mvComputeSurfDescriptor(): caculate the surf feature of input point.
	+ mvComputeSingleSurfDescriptor(): caculate the surf feature of one input point.
    + mvSurfDist(): caculate the distance between surf features.
			
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "surf.h"
#include "declare.h"

extern uint8_t uArrGaussRegion[36];
extern uint8_t uArrGaussSeed[16];
extern uint8_t sqrtData[255*255+1];
extern uint8_t stdDataSurf[511*160];

/*
Function process:
	+ caculate the surf feature of input point
	Fan-in : 
	        + mvFeatrueDescribe()
	Fan-out:
	        + mvComputeSingleSurfDescriptor()
	ATTENTION: __________
*/
void mvComputeSurfDescriptor(uint8_t *puImgData, AdaSize szImg, const AdasCorner *pPoint, \
							 s32 nPtCnt, uint8_t *pSurfFeature)
{
	s32 i;
	
	for (i = 0; i < nPtCnt; ++i)
	{
	     mvComputeSingleSurfDescriptor(puImgData, szImg, pPoint + i, pSurfFeature +   (i <<6) );		
	}
}

/*
Function process:
	+ caculate the surf feature of input point
	Fan-in : 
	        + mvComputeSurfDescriptor()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvComputeSingleSurfDescriptor(uint8_t *puImgGray, AdaSize szImg, const AdasCorner *pPoint, \
								   uint8_t *pSurfFeature)
{
	s32 i;
	s32 j;
	s32 nRow;
	s32 nCol;
	s32 nGaussIndex;
	s32 nSurFeaIndex = 0;
	s32 nSeedIndex = 0;
	s32 nArrDx[15][15];
	s32 nArrDy[15][15];
	s32 nPixelDx;
	s32 nPixelDy;
	s32 nSeedDx;
	s32 nSeedDy;
	s32 nSeedAbsDx;
	s32 nSeedAbsDy;
	s32 nArrSurfFea[SURF_DESC_DIMENTION];
	s32 nSquareSum = 0;
	s32 nSqrtSum;
	s32 nShift = 0;
	s32 nFeaThresh;
	s32 nMathSymbol;
	uint8_t *pRowIndex1 = 0;
	uint8_t *pRowIndex2 = 0;
	s32 pty0 = pPoint->y - 7;
	s32 pty1 = pPoint->y + 7;
	s32 ptx0 = pPoint->x - 7;
	s32 ptx1 = pPoint->x + 7;

	memset(nArrDx, 0, sizeof(s32) * 15 * 15);
	memset(nArrDy, 0, sizeof(s32) * 15 * 15);
	for (nRow = pty0, j = 0; nRow <= pty1; ++nRow, ++j)
	{
		if (nRow < 0 || nRow > szImg.height - 2)
		{
			continue;
		}

		pRowIndex1 = puImgGray + szImg.width * nRow;
		pRowIndex2 = puImgGray + szImg.width * (nRow + 1);

		for (nCol = ptx0, i = 0; nCol <= ptx1; ++nCol, ++i)
		{
			if (nCol < 0 || nCol > szImg.width - 2)
			{
				continue;
			}

			nArrDx[j][i] = *(pRowIndex1+nCol+1) - *(pRowIndex1+nCol) + *(pRowIndex2+nCol+1) - *(pRowIndex2+nCol);
			nArrDy[j][i] = *(pRowIndex2+nCol) - *(pRowIndex1+nCol) + *(pRowIndex2+nCol+1) - *(pRowIndex1+nCol+1);
		}
	}

	for (j = 0; j < 4; ++j)
	{
		for (i = 0; i < 4; ++i)
		{
			int j0 = j * 3;
			int j1 = j0 + 6;
			int i0 = i * 3;
			int i1 = i0 + 6;

			nSeedDx = nSeedDy = nSeedAbsDx = nSeedAbsDy = 0;
			nGaussIndex = 0;

			for (nRow = j0; nRow < j1; ++nRow)
			{
				for (nCol = i0; nCol < i1; ++nCol)
				{
					nPixelDx = uArrGaussRegion[nGaussIndex] * nArrDx[nRow][nCol];
					nPixelDy = uArrGaussRegion[nGaussIndex] * nArrDy[nRow][nCol];

					nSeedDx += nPixelDx;
					nSeedDy += nPixelDy;

					nPixelDx = (nPixelDx > 0) ? nPixelDx : -nPixelDx;
					nPixelDy = (nPixelDy > 0) ? nPixelDy : -nPixelDy;

					nSeedAbsDx += nPixelDx;
					nSeedAbsDy += nPixelDy;

					nGaussIndex++;
				}
			}

			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedDx;
			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedDy;
			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedAbsDx;
			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedAbsDy;

			nSeedIndex++;
		}
	}

	for (i = 0; i < SURF_DESC_DIMENTION; ++i)
	{
		nArrSurfFea[i] = nArrSurfFea[i]>>18;

		if (nArrSurfFea[i] > 510)
		{
			nArrSurfFea[i] = 510;
		}
		else if (nArrSurfFea[i] < -510)
		{
			nArrSurfFea[i] = -510;
		}

		nSquareSum += nArrSurfFea[i] * nArrSurfFea[i];
	}

	for (i = 0; i <= 8; ++i)
	{
		if ((nSquareSum>>i) <= 65025)
		{
			nShift = i;
			break;
		}
	}

	nSquareSum = nSquareSum>>nShift;
	nSqrtSum = sqrtData[nSquareSum];

	if (1 == nShift)
	{
		nSqrtSum = (nSqrtSum*362)>>8;
	}
	else if (2 == nShift)
	{
		nSqrtSum = nSqrtSum<<1;
	}
	else if (3 == nShift)
	{
		nSqrtSum = (nSqrtSum*362)>>7;
	}
	else if (4 == nShift)
	{
		nSqrtSum = nSqrtSum<<2;
	}
	else if (5 == nShift)
	{
		nSqrtSum = (nSqrtSum*362)>>6;
	}
	else if (6 == nShift)
	{
		nSqrtSum = nSqrtSum<<3;
	}
	else if (7 == nShift)
	{
		nSqrtSum = (nSqrtSum*362)>>5;
	}
	else if (8 == nShift)
	{
		nSqrtSum = nSqrtSum<<4;
	}
	
	nFeaThresh = (nSqrtSum*76)>>8;	

	for (i = 0; i <= 3; ++i)
	{
		if ((nSqrtSum>>i) <= 510)
		{
			nShift = i;
			break;
		}
	}

	nSqrtSum = nSqrtSum>>nShift;
	for (i = 0; i < SURF_DESC_DIMENTION; ++i)
	{
		nMathSymbol = 1;
		if (nArrSurfFea[i] < 0)
		{
			nArrSurfFea[i] = -nArrSurfFea[i];
			nMathSymbol = -1;
		}

		if (nArrSurfFea[i] > nFeaThresh)
		{
			nArrSurfFea[i] = nFeaThresh;
		}

		nArrSurfFea[i] = nArrSurfFea[i]>>nShift;

		pSurfFeature[i] = nMathSymbol * stdDataSurf[160*nSqrtSum+nArrSurfFea[i]] + 64;
	}
}

/*
Function process:
	+ caculate the distance between surf features
	Fan-in : 
	        + ()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
s32 mvSurfDist( const uint8_t *p1,  const uint8_t *p2)
{
	s32 i;
	s32 dif;
	s32 distsq;

	distsq = 0;
	for (i = 0; i < SURF_DESC_DIMENTION; ++i)
	{
		dif = *p1++ - *p2++;
		distsq += dif * dif;
	}

	return distsq;
}
