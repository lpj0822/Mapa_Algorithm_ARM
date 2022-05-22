#include "vehicle_proposals.h"
#include "LDWS_Interface.h"

#include <stdio.h>
#include <stdlib.h>

//day
#define OFS (1024) // 256*4
#define SOBEL_TABLE (2304) // OFS * 2 + 256
#define ftzero (80)
#define BLOCK_SIZE_X (6)
#define BLOCK_SIZE_Y (2)

//dark
#define MIN_HIST (170)
#define MAX_HIST (220)
#define HIST_COUNT (256 - MIN_HIST)
static unsigned char imageHist[HIST_COUNT] = { 0 };

static ProposalsGlobalPara proposalsPara = { 0 };

static void computeLightImage(const FCWSDImage *pOriGrayImg, const FCWSDRect *proposalsRoi, const s32 relativeRoiY, uint8_t *imageHist, uint8_t *darkImage);

static void initSobelTable(uint8_t *sobleTable);

static void prefilterYSobel(const FCWSDImage *pOriGrayImg, const FCWSDRect *proposalsRoi, const s32 relativeRoiY, const uint8_t *sobleTable, uint8_t *dyImage);

static void computeDyLine(const FCWSDRect *proposalsRoi, const s32 relativeRoiY, const uint8_t *dyImage, uint8_t *dyLineImage);

static void computeIntegrate(const FCWSDRect *proposalsRoi, const s32 relativeRoiY, const uint8_t *image, u32 *integralImage);

int initProposals(const s32 srcWidth, const s32 srcHeight, const float srcROIYFactor)
{
	float proposalsRoiYFactor = 0.0f;
	int allMallocSize = 0;
	int count = 0;
	int srcRoiY = 0;
	int srcRoiWidth = 0;
	int srcRoiHeight = 0;
	int stopY = 0;
	LDWS_InitGuid *pLDWSInit = NULL;
	uint8_t* copyMallPtr = NULL;
	if (srcROIYFactor > 0.45f)
	{
		proposalsRoiYFactor = srcROIYFactor;
	}
	else
	{
		proposalsRoiYFactor = 0.45f;
	}

	srcRoiY = (int)(srcHeight * srcROIYFactor);
	srcRoiWidth = srcWidth;
	srcRoiHeight = srcHeight - srcRoiY;
	count = srcRoiWidth * srcRoiHeight;

	LDWS_Getinit(&pLDWSInit);
	stopY = MAX(srcRoiY, pLDWSInit->pBoundPoint[2].y) + 1;
	stopY = MIN(stopY, srcHeight);

	proposalsPara.proposalsRoi.point.x = 0;
	proposalsPara.proposalsRoi.point.y = (int)(srcHeight * proposalsRoiYFactor);
	proposalsPara.proposalsRoi.size.width = srcWidth;
	proposalsPara.proposalsRoi.size.height = stopY - proposalsPara.proposalsRoi.point.y;

	proposalsPara.integrateRoi.point.x = 0;
	proposalsPara.integrateRoi.point.y = (int)(srcHeight * proposalsRoiYFactor);
	proposalsPara.integrateRoi.size.width = srcWidth;
	proposalsPara.integrateRoi.size.height = srcHeight - proposalsPara.integrateRoi.point.y;

	proposalsPara.relativeRoiY = proposalsPara.proposalsRoi.point.y - srcRoiY;

	allMallocSize = SOBEL_TABLE * sizeof(uint8_t)+count * 2 * sizeof(uint8_t)+count * sizeof(u32)+8 * sizeof(uint8_t);

	proposalsPara.allMemory = (uint8_t *)malloc(allMallocSize);

	if (proposalsPara.allMemory == NULL)
		return 0;

	memset(proposalsPara.allMemory, 0, allMallocSize * sizeof(uint8_t));

	copyMallPtr = proposalsPara.allMemory;
	proposalsPara.sobleTable = (uint8_t*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(proposalsPara.sobleTable + SOBEL_TABLE);
	proposalsPara.dyImage = (uint8_t*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(proposalsPara.dyImage + count);
	proposalsPara.dyLineImage = (uint8_t*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(proposalsPara.dyLineImage + count);
	proposalsPara.dyIntegralImage = (u32*)copyMallPtr;

	initSobelTable(proposalsPara.sobleTable);
	
	if (pLDWSInit != NULL)
	{
		LDWS_Freeinit(&pLDWSInit);
		pLDWSInit = NULL;
	}
	return 1;
}

void computeProposals(const unsigned char flag, const FCWSDImage *pOriGrayImg)
{
	if (flag == 0)//day
	{
		prefilterYSobel(pOriGrayImg, &proposalsPara.proposalsRoi, proposalsPara.relativeRoiY, proposalsPara.sobleTable, proposalsPara.dyImage);
		computeDyLine(&proposalsPara.proposalsRoi, proposalsPara.relativeRoiY, proposalsPara.dyImage, proposalsPara.dyLineImage);
		computeIntegrate(&proposalsPara.integrateRoi, proposalsPara.relativeRoiY, proposalsPara.dyLineImage, proposalsPara.dyIntegralImage);
	}
	else if(flag == 1)// night
	{
		computeLightImage(pOriGrayImg, &proposalsPara.proposalsRoi, proposalsPara.relativeRoiY, imageHist, proposalsPara.dyImage);
		computeIntegrate(&proposalsPara.integrateRoi, proposalsPara.relativeRoiY, proposalsPara.dyImage, proposalsPara.dyIntegralImage);
	}

}

int filterCarTask(const unsigned char flag, const lbpTaskCar* task, const FCWSDetectorGlobalPara *pVehicleDetor, const float factor)
{
	const int height = (int)(pVehicleDetor->pdetorModel->l->data.featureHeight * factor);
	int minX = (int)(task->x * factor);
	int minY = (int)(task->y * factor);
	int maxX = minX + height;
	int maxY = minY + height;
	unsigned int a, b, c, d;
	float taskFactor = 0;

	if (proposalsPara.dyIntegralImage == NULL)
		return 1;

	a = *(proposalsPara.dyIntegralImage + minY * proposalsPara.proposalsRoi.size.width + minX);
	b = *(proposalsPara.dyIntegralImage + minY * proposalsPara.proposalsRoi.size.width + maxX);
	c = *(proposalsPara.dyIntegralImage + maxY * proposalsPara.proposalsRoi.size.width + minX);
	d = *(proposalsPara.dyIntegralImage + maxY * proposalsPara.proposalsRoi.size.width + maxX);
	taskFactor = (float)(a + d - b - c) / (height * height);

	if (flag == 0)//day
	{
		if (height < 100)
		{
			if (taskFactor >= 0.04f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if (height < 300)
		{
			if (taskFactor >= 0.02f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if (taskFactor >= 0.001f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}

	}
	else if (flag == 1)// night
	{
		if (taskFactor > 0.9f)
		{
			return 0;
		}
		else if (height < 100)
		{
			if (taskFactor >= 0.02f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if (height < 300)
		{
			if (taskFactor >= 0.005f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if (taskFactor >= 0.001f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
	}
	else
	{
		return 0;
	}
}

int freeProposals()
{
	if (proposalsPara.allMemory != NULL)
	{
		free(proposalsPara.allMemory);
		proposalsPara.allMemory = NULL;
	}
	proposalsPara.sobleTable = NULL;
	proposalsPara.dyImage = NULL;
	proposalsPara.dyLineImage = NULL;
	proposalsPara.dyIntegralImage = NULL;

	return 1;
}

static void computeLightImage(const FCWSDImage *pOriGrayImg, const FCWSDRect *proposalsRoi, const s32 relativeRoiY, uint8_t *imageHist, uint8_t *darkImage)
{
	s32 row = 0;
	s32 col = 0;
	s32 x = 0;
	s32 stopRow = proposalsRoi->size.height + proposalsRoi->point.y;
	s32 stopCol = proposalsRoi->size.width + proposalsRoi->point.x;
	s32 index = 0;
	s32 maxValue = 0;
	s32 threshold = 0;
	uint8_t *srcData = pOriGrayImg->ptr + proposalsRoi->point.y * pOriGrayImg->nWid;
	uint8_t *dstData = darkImage + relativeRoiY * proposalsRoi->size.width;

	memset(imageHist, 0, HIST_COUNT * sizeof(uint8_t));

	for (row = proposalsRoi->point.y; row < stopRow; row++)
	{
		for (col = proposalsRoi->point.x; col < stopCol; col++)
		{
			if (srcData[col] >= MIN_HIST)
			{
				index = srcData[col] - MIN_HIST;
				imageHist[index]++;
			}
		}
		srcData += pOriGrayImg->nWid;
	}

	index = 0;
	for (x = 0; x < HIST_COUNT; x++)
	{
		if (imageHist[x] > maxValue)
		{
			index = x;
		}
	}
	index += MIN_HIST;
	if (index >= MIN_HIST && index <= MAX_HIST)
	{
		threshold = MIN_HIST;
	}
	else
	{
		threshold = (s32)(index * 0.8f);
	}

	srcData = pOriGrayImg->ptr + proposalsRoi->point.y * pOriGrayImg->nWid;
	for (row = proposalsRoi->point.y; row < stopRow; row++)
	{
		for (x = 0, col = proposalsRoi->point.x; col < stopCol; col++, x++)
		{
			if (srcData[col] > threshold)
			{
				dstData[x] = 1;
			}
			else
			{
				dstData[x] = 0;
			}
		}
		srcData += pOriGrayImg->nWid;
		dstData += proposalsRoi->size.width;
	}
}

static void initSobelTable(uint8_t *sobleTable)
{
	int x = 0;
	for (x = 0; x < SOBEL_TABLE; x++)
		sobleTable[x] = (uint8_t)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero * 2 : x - OFS + ftzero);
}

static void  prefilterYSobel(const FCWSDImage *pOriGrayImg, const FCWSDRect *proposalsRoi,
	const s32 relativeRoiY, const uint8_t *sobleTable, uint8_t *dyImage)
{
	s32 col = 0;
	s32 row = 0;
	s32 startX = proposalsRoi->point.x + 1;
	s32 startY = proposalsRoi->point.y + 1;
	s32 stopY = proposalsRoi->size.height + proposalsRoi->point.y - 1;
	s32 stopX = proposalsRoi->size.width + proposalsRoi->point.x - 2;
	uint8_t *srcData0 = pOriGrayImg->ptr + proposalsRoi->point.y * pOriGrayImg->nWid;
	uint8_t *srcData1 = srcData0 + pOriGrayImg->nWid;
	uint8_t *srcData2 = srcData1 + pOriGrayImg->nWid;
	uint8_t *dstData = dyImage + relativeRoiY * proposalsRoi->size.width;
	int16_t d0, d1, d2, d3, v0, v1;
	s32 index1, index2, index3;

	for (row = startY; row < stopY; row++)
	{
		for (col = startX; col < stopX; col += 2)
		{
			index1 = col - 1;
			index2 = col + 1;
			index3 = col + 2;
			d0 = srcData0[index1] - srcData2[index1];
			d1 = srcData0[col] - srcData2[col];
			d2 = srcData0[index2] - srcData2[index2];
			d3 = srcData0[index3] - srcData2[index3];

			v0 = sobleTable[d0 + (d1 << 1) + d2 + OFS];
			v1 = sobleTable[d1 + (d2 << 1) + d3 + OFS];

			if (v0 < 50)
			{
				dstData[col] = 255;
			}
			else
			{
				dstData[col] = 0;
			}

			if (v1 < 50)
			{
				dstData[index2] = 255;
			}
			else
			{
				dstData[index2] = 0;
			}
		}
		srcData0 += pOriGrayImg->nWid;
		srcData1 += pOriGrayImg->nWid;
		srcData2 += pOriGrayImg->nWid;
		dstData += proposalsRoi->size.width;
	}
}

static void computeDyLine(const FCWSDRect *proposalsRoi, const s32 relativeRoiY, const uint8_t *dyImage, uint8_t *dyLineImage)
{
	int row = 0;
	int col = 0;
	int y = 0;
	int x = 0;
	float factor = 0.0f;
	int stopCol = proposalsRoi->size.width;
	int stopRow = proposalsRoi->size.height - BLOCK_SIZE_Y;
	int stepX = 1;
	int stepY = BLOCK_SIZE_Y;
	int rowWidth = BLOCK_SIZE_Y * proposalsRoi->size.width;
	int stopX = 0;
	int stopY = 0;
	int hCount = 0;
	int count = 0;
	const uint8_t *dyData = NULL;
	const uint8_t *dyRow = dyImage + relativeRoiY * proposalsRoi->size.width;
	uint8_t * dyLineImageData = dyLineImage + relativeRoiY * proposalsRoi->size.width;
	int carWidth = 0;

	memset(dyLineImageData, 0, proposalsRoi->size.width * proposalsRoi->size.height * sizeof(uint8_t));

	for (row = 0; row < stopRow; row += stepY)
	{
		carWidth = (int)LDWS_GetXLengthofImage(1.3, row + proposalsRoi->point.y);
		if (carWidth < BLOCK_SIZE_X)
		{
			carWidth = BLOCK_SIZE_X;
		}
		for (col = 0; col < stopCol; col += stepX)
		{
			stepX = 1;
			if (dyRow[col] != 0)
			{
				hCount = 0;
				count = 0;
				stopX = MIN(col + carWidth, proposalsRoi->size.width);
				stopY = MIN(row + BLOCK_SIZE_Y, proposalsRoi->size.height);

				dyData = dyRow;

				for (y = row; y < stopY; y++)
				{
					for (x = col; x < stopX; x++)
					{
						if (dyData[x] != 0)
						{
							hCount++;
						}
						count++;
					}
					dyData += proposalsRoi->size.width;
				}
				factor = (float)hCount / count;
				if (factor >= 0.4f)
				{
					for (x = col; x < stopX; x++)
					{
						dyLineImageData[x] = 1;
					}
				}
				stepX = carWidth >> 1;
			}
		}
		dyRow += rowWidth;
		dyLineImageData += rowWidth;
	}
}


// Helper function that integrates an image
static void computeIntegrate(const FCWSDRect *proposalsRoi, const s32 relativeRoiY, const uint8_t *image, u32 *integralImage)
{
	int row = 0;
	int col = 0;
	int sum = 0;
	int width = proposalsRoi->size.width;
	int height = proposalsRoi->size.height;
	u32  *integralChannelPreviousRow = NULL;
	u32  *integralChannelRow = NULL;
	const uint8_t *srcData = image + relativeRoiY * proposalsRoi->size.width;

	integralImage += relativeRoiY * proposalsRoi->size.width;

	for (col = 0; col < width; col++)
	{
		sum += srcData[col];
		integralImage[col] = sum;
	}

	integralChannelPreviousRow = integralImage;
	integralChannelRow = integralImage + proposalsRoi->size.width;
	srcData += proposalsRoi->size.width;

	for (row = 1; row < height; row += 1)
	{
		sum = 0;
		for (col = 0; col < width; col += 1)
		{
			sum += srcData[col];
			integralChannelRow[col] = integralChannelPreviousRow[col] + sum;
		}
		integralChannelPreviousRow += proposalsRoi->size.width;
		integralChannelRow += proposalsRoi->size.width;
		srcData += proposalsRoi->size.width;
	}
}
