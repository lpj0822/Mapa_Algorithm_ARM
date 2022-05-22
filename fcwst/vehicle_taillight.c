#include "vehicle_taillight.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "clustering_rect.h"
#include "LDWS_Interface.h"
#include "utils.h"

#define MIN_HIST (180)
#define MAX_HIST (230)
#define HIST_COUNT (256 - MIN_HIST)

#define RED_THRESHOLD (155)

#define BLOCK_SIZE_X (5)
#define BLOCK_SIZE_Y (5)
#define CAR_MIN_WIDTH (5)
#define CLUSTERING_DELTA (1.0f)

#define MAX_DISTANCE (60)
#define CONFIRM_LIGHT_COUNT (6)

#define MAX_LIGHT_COUNT (5120)
#define TREE_NODE_COUNT (MAX_LIGHT_COUNT)

static float computeRectDistance(TailLightRect rect1, TailLightRect rect2);

static void computeLightImage(const imgage *pBuff, const AdasRect *lightRoi, uint8_t *imageHist, uint8_t *lightImage);

static void computeTailLightImage(const imgage *pBuff, const AdasRect *lightRoi, const uint8_t *lightImage,
                                  uint8_t* tailLightImage);

static void getTailLightRect(const uint8_t *lightImage, const uint8_t* tailLightImage, const AdasRect *lightRoi, const int lightWidth,
	const AdaSize *dataSize, TailLightResult *resultRect);

static void getLeftAndRightLight(const TailLightResult *srcRect, const int axisX, const int lightWidth, TailLightResult *leftLight, TailLightResult *rightLight);

static void filterTailLight(int *treeNode, int *labelsNode, TailLightResult *preLeftLight, TailLightResult *preRightLight,
	TailLightResult *leftLight, TailLightResult *rightLight, TailLightResult *finalRect);

static void mathchLeftAndRight(const TailLightRect leftResult, const TailLightRect rightResult, const int carWidth, const int lightWidth, TailLightResult *finalRect);

static void tailLightMatch(const TailLightResult *leftLight, const TailLightResult *rightLight, const int carWidth, const int lightWidth, TailLightResult *finalRect);

static void updatePreTailLight(const TailLightRect leftResult, const TailLightRect rightResult, int *confirmTailLightIndex, TailLightResult *preLeftLight, TailLightResult *preRightLight);

int initTailLight(const int srcWidth, const int srcHeight, DarkTailLightGlobalPara *tailLightPara)
{
    int allMallocSize = 0;
    int count = 0;
    uint8_t* copyMallPtr = NULL;

    count = srcWidth * srcHeight;

    tailLightPara->dataSize.width = srcWidth;
    tailLightPara->dataSize.height = srcHeight;

    allMallocSize = HIST_COUNT * sizeof(uint8_t) + 2 * count * sizeof(uint8_t) + \
			3 * TREE_NODE_COUNT * sizeof(int) + MAX_LIGHT_COUNT * 3 * sizeof(TailLightRect) + 2 * CONFIRM_LIGHT_COUNT * sizeof(TailLightRect) + \
			22 * sizeof(uint8_t);

    tailLightPara->allMemory = (uint8_t *)malloc(allMallocSize);

	if (tailLightPara->allMemory == NULL)
	{
		my_printf("the tail light detection malloc fail!\n");
		return 0;
	}

    memset(tailLightPara->allMemory, 0, allMallocSize * sizeof(uint8_t));

    copyMallPtr = tailLightPara->allMemory;
    tailLightPara->imageHist = (uint8_t*)copyMallPtr;

    copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->imageHist + HIST_COUNT);
    tailLightPara->lightImage = (uint8_t *)copyMallPtr;

    copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->lightImage + count);
    tailLightPara->tailLightImage = (uint8_t *)copyMallPtr;

    copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->tailLightImage + count);
    tailLightPara->treeNode = (int *)copyMallPtr;

    copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->treeNode + (TREE_NODE_COUNT << 1));
    tailLightPara->labelsNode = (int *)copyMallPtr;

    copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->labelsNode + TREE_NODE_COUNT);
	tailLightPara->leftLightResult.lightRect = (TailLightRect *)copyMallPtr;
	tailLightPara->leftLightResult.count = 0;

	copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->leftLightResult.lightRect + MAX_LIGHT_COUNT);
	tailLightPara->rightLightResult.lightRect = (TailLightRect *)copyMallPtr;
	tailLightPara->rightLightResult.count = 0;

	copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->rightLightResult.lightRect + MAX_LIGHT_COUNT);
	tailLightPara->finalResult.lightRect = (TailLightRect *)copyMallPtr;
	tailLightPara->finalResult.count = 0;

	copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->finalResult.lightRect + MAX_LIGHT_COUNT);
	tailLightPara->preLeftLight.lightRect = (TailLightRect *)copyMallPtr;
	tailLightPara->preLeftLight.count = 0;

	copyMallPtr = ADAS_ALIGN_16BYTE(tailLightPara->preLeftLight.lightRect + CONFIRM_LIGHT_COUNT);
	tailLightPara->preRightLight.lightRect = (TailLightRect *)copyMallPtr;
	tailLightPara->preRightLight.count = 0;

	tailLightPara->preObjectID = -1;
	tailLightPara->confirmTailLightIndex = 0;

    return 1;
}

void computeTailLight(const imgage *pBuff, const AdasRect object, const int objectID, DarkTailLightGlobalPara *tailLightPara)
{
	int carWidth = (int)LDWS_GetXLengthofImage(1.0, object.y + (object.height >> 1));
	int axisX = object.x + (object.width >> 1);
	int lightWidth = 0;
	if (carWidth < CAR_MIN_WIDTH)
	{
		carWidth = CAR_MIN_WIDTH;
	}
	lightWidth = WS_MAX(carWidth / 10, CAR_MIN_WIDTH);
    computeLightImage(pBuff, &object, tailLightPara->imageHist, tailLightPara->lightImage);
    computeTailLightImage(pBuff, &object, tailLightPara->lightImage, tailLightPara->tailLightImage);
    getTailLightRect(tailLightPara->lightImage, tailLightPara->tailLightImage, &object, lightWidth,
		&tailLightPara->dataSize, &tailLightPara->leftLightResult);
	clusteringRect(&tailLightPara->leftLightResult, CLUSTERING_DELTA, tailLightPara->treeNode, tailLightPara->labelsNode,
			&tailLightPara->finalResult);
	getLeftAndRightLight(&tailLightPara->finalResult, axisX, lightWidth, &tailLightPara->leftLightResult, &tailLightPara->rightLightResult);
	if (tailLightPara->preObjectID != objectID)
	{
		tailLightPara->confirmTailLightIndex = 0;
		tailLightPara->preLeftLight.count = 0;
		tailLightPara->preRightLight.count = 0;
	}
	if (tailLightPara->preLeftLight.count < (CONFIRM_LIGHT_COUNT >> 1))
	{
		tailLightMatch(&tailLightPara->leftLightResult, &tailLightPara->rightLightResult, carWidth, lightWidth, &tailLightPara->finalResult);
		if (tailLightPara->finalResult.count > 0)
		{
			updatePreTailLight(tailLightPara->finalResult.lightRect[0], tailLightPara->finalResult.lightRect[1], &tailLightPara->confirmTailLightIndex,
				&tailLightPara->preLeftLight, &tailLightPara->preRightLight);
		}
	}
	else
	{
		filterTailLight(tailLightPara->treeNode, tailLightPara->labelsNode, &tailLightPara->preLeftLight, &tailLightPara->preRightLight,
			&tailLightPara->leftLightResult, &tailLightPara->rightLightResult, &tailLightPara->finalResult);
		if (tailLightPara->finalResult.count > 0)
		{
			mathchLeftAndRight(tailLightPara->finalResult.lightRect[0], tailLightPara->finalResult.lightRect[1], carWidth, lightWidth, &tailLightPara->finalResult);
		}
		updatePreTailLight(tailLightPara->leftLightResult.lightRect[0], tailLightPara->rightLightResult.lightRect[0], &tailLightPara->confirmTailLightIndex,
			&tailLightPara->preLeftLight, &tailLightPara->preRightLight);
	}
	tailLightPara->preObjectID = objectID;
}

void freeTailLight(DarkTailLightGlobalPara *tailLightPara)
{
    if (tailLightPara->allMemory != NULL)
    {
        free(tailLightPara->allMemory);
        tailLightPara->allMemory = NULL;
    }
    tailLightPara->imageHist = NULL;
    tailLightPara->lightImage = NULL;
    tailLightPara->tailLightImage = NULL;
    tailLightPara->treeNode = NULL;
    tailLightPara->labelsNode = NULL;
    tailLightPara->leftLightResult.lightRect = NULL;
    tailLightPara->rightLightResult.lightRect = NULL;
    tailLightPara->finalResult.lightRect = NULL;
	tailLightPara->preLeftLight.lightRect = NULL;
	tailLightPara->preRightLight.lightRect = NULL;
}

static float computeRectDistance(TailLightRect rect1, TailLightRect rect2)
{
	int centerX1 = rect1.x + (rect1.width >> 1);
	int centerY1 = rect1.y + (rect1.height >> 1);
	int centerX2 = rect2.x + (rect2.width >> 1);
	int centerY2 = rect2.y + (rect2.height >> 1);
	int valueX = centerX1 - centerX2;
	int valueY = centerY1 - centerY2;
	float distance = 0;
	int value = valueX * valueX + valueY * valueY;
	distance = sqrtf(value);
	return distance;
}

static void computeLightImage(const imgage *pBuff, const AdasRect *lightRoi, uint8_t *imageHist, uint8_t *lightImage)
{
    int row = 0;
    int col = 0;
    int x = 0;
    int stopRow = lightRoi->y + lightRoi->height;
    int stopCol = lightRoi->x + lightRoi->width;
    int index = 0;
    int maxValue = 0;
    int threshold = 0;
    const uint8_t *copySrcData = pBuff->ptr + lightRoi->y * pBuff->nWid;
    const uint8_t *srcData = copySrcData;
    uint8_t *dstData = lightImage + lightRoi->y * pBuff->nWid;

	memset(lightImage, 0, pBuff->nHig * pBuff->nWid * sizeof(uint8_t));
    memset(imageHist, 0, HIST_COUNT * sizeof(uint8_t));

    for (row = lightRoi->y; row < stopRow; row++)
    {
        for (col = lightRoi->x; col < stopCol; col++)
        {
            if (srcData[col] >= MIN_HIST)
            {
                index = srcData[col] - MIN_HIST;
                imageHist[index]++;
            }
        }
        srcData += pBuff->nWid;
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
        threshold = (int)(index * 0.8f);
    }

    srcData = copySrcData;
    for (row = lightRoi->y; row < stopRow; row++)
    {
        for (x = lightRoi->x, col = lightRoi->x; col < stopCol; col++, x++)
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
        srcData += pBuff->nWid;
        dstData += pBuff->nWid;
    }
}

static void computeTailLightImage(const imgage *pBuff, const AdasRect *lightRoi, const uint8_t *lightImage, uint8_t* tailLightImage)
{
    int row = 0;
    int col = 0;
    int y = 0;
    int x = 0;
    float factor = 0.0f;
    int stopRow = lightRoi->y + lightRoi->height;
    int stopCol = lightRoi->x + lightRoi->width;
    int stepX = BLOCK_SIZE_X >> 1;
    int stepY = BLOCK_SIZE_Y;
    int rowWdith = BLOCK_SIZE_Y * pBuff->nWid;
    int stopX = 0;
    int stopY = 0;
    int lightCount = 0;
    int vCount = 0;
    int count = 0;
    const uint8_t *srcRow = lightImage + lightRoi->y * pBuff->nWid;
    const uint8_t *vRow = pBuff->pvtr + lightRoi->y * pBuff->nWid;
    const uint8_t *srcData = NULL;
    const uint8_t *vData = NULL;
    uint8_t *dstRow = tailLightImage + lightRoi->y * pBuff->nWid;
    uint8_t *dstData = NULL;

	memset(tailLightImage, 0, pBuff->nHig * pBuff->nWid * sizeof(uint8_t));

    for (row = lightRoi->y; row < stopRow; row += stepY)
    {
        for (col = lightRoi->x; col < stopCol; col += stepX)
        {
            lightCount = 0;
            vCount = 0;
            count = 0;
            stopX = WS_MIN(col + BLOCK_SIZE_X, stopCol);
            stopY = WS_MIN(row + BLOCK_SIZE_Y, stopRow);

            srcData = srcRow;
            vData = vRow;

            for (y = row; y < stopY; y++)
            {
                for (x = col; x < stopX; x++)
                {
                    lightCount += srcData[x];
                    vCount += vData[x];
                    count++;
                }
                srcData += pBuff->nWid;
                vData += pBuff->nWid;
            }
            factor = (float)lightCount / count;
            if (factor >= 0.01f && factor <= 0.9f)
            {
                factor = (float)vCount / count;
				if (factor > RED_THRESHOLD)
                {
                    srcData = srcRow;
                    dstData = dstRow;
                    for (y = row; y < stopY; y++)
                    {
                        for (x = col; x < stopX; x++)
                        {
                            dstData[x] = 1;
                        }
                        dstData += pBuff->nWid;
                    }
                }
            }
            stepX = BLOCK_SIZE_X >> 1;
        }
        srcRow += rowWdith;
        vRow += rowWdith;
        dstRow += rowWdith;
    }
}

static void getTailLightRect(const uint8_t *lightImage, const uint8_t* tailLightImage, const AdasRect *lightRoi, const int lightWidth,
	const AdaSize *dataSize, TailLightResult *resultRect)
{
    int resultCount = 0;
    int row = 0;
    int col = 0;
    int y = 0;
    int x = 0;
    float factor = 0.0f;
    int stopRow = lightRoi->y + lightRoi->height;
    int stopCol = lightRoi->x + lightRoi->width;
    int stepX = 1;
    int stepY = 1;
    int stopX = 0;
    int stopY = 0;
    int lightCount = 0;
    int count = 0;
	TailLightRect tailLightRect;
    const uint8_t *lightRow = lightImage + lightRoi->y * dataSize->width;
    const uint8_t *tailRow = tailLightImage + lightRoi->y * dataSize->width;
    const uint8_t *lightData = NULL;
    const uint8_t *tailData = NULL;
	int extendWidth = 0;
	stepY = lightWidth;

    for (row = lightRoi->y; row < stopRow; row += stepY)
    {
        for (col = lightRoi->x; col < stopCol; col += stepX)
        {
            stepX = 1;
            if(tailRow[col] != 0)
            {
                lightCount = 0;
                count = 0;
				stopX = WS_MIN(col + lightWidth, stopCol);
				stopY = WS_MIN(row + lightWidth, stopRow);

                lightData = lightRow;
                tailData = tailRow;

                for (y = row; y < stopY; y++)
                {
                    for (x = col; x < stopX; x++)
                    {
                        lightCount = lightCount + (tailData[x] | lightData[x]);
                        count++;
                    }
                    lightData += dataSize->width;
                    tailData += dataSize->width;
                }
                factor = (float)lightCount / count;
                if (factor >= 0.2f)
                {
					extendWidth = lightWidth >> 1;
                    x = WS_MAX(col - extendWidth, 0);
                    y = WS_MAX(row - extendWidth, 0);
                    stopX = WS_MIN(stopX + extendWidth, stopCol);
                    stopY = WS_MIN(stopY + extendWidth, stopRow);
                    tailLightRect.x = x;
                    tailLightRect.y = y;
                    tailLightRect.width = stopX - x;
                    tailLightRect.height = stopY - y;
                    resultRect->lightRect[resultCount] = tailLightRect;
                    resultCount++;
					if (resultCount >= MAX_LIGHT_COUNT)
					{
						col = stopX;
						row = stopY;
					}
                }
				stepX = lightWidth >> 1;
            }
        }
        lightRow += stepY * dataSize->width;
        tailRow += stepY * dataSize->width;
    }
    resultRect->count = resultCount;
}

static void getLeftAndRightLight(const TailLightResult *srcRect, const int axisX, const int lightWidth, TailLightResult *leftLight, TailLightResult *rightLight)
{
	const int srcCount = srcRect->count;
	const int axisX1 = axisX - (lightWidth << 1);
	const int axisX2 = axisX + (lightWidth << 1);
	int index = 0;
	int leftCount = 0;
	int rightCount = 0;
	int maxX = 0;
	for (index = 0; index < srcCount; index++)
	{
		maxX = srcRect->lightRect[index].x + srcRect->lightRect[index].width;
		if (maxX < axisX1)
		{
			leftLight->lightRect[leftCount] = srcRect->lightRect[index];
			leftCount++;
		}
		else if (srcRect->lightRect[index].x > axisX2)
		{
			rightLight->lightRect[rightCount] = srcRect->lightRect[index];
			rightCount++;
		}
	}
	leftLight->count = leftCount;
	rightLight->count = rightCount;
}

static void filterTailLight(int *treeNode, int *labelsNode, TailLightResult *preLeftLight, TailLightResult *preRightLight, 
	TailLightResult *leftLight, TailLightResult *rightLight, TailLightResult *finalRect)
{
	const int lightConfidence = (CONFIRM_LIGHT_COUNT >> 1);
	const int leftCount = leftLight->count;
	const int rightCount = rightLight->count;
	int leftIndex = 0;
	int rightIndex = 0;
	float distance = 0;
	float minDistance = 10000;
	int finalCount = 0;
	TailLightRect leftResult1;
	TailLightRect leftResult2;
	TailLightRect rightResult1;
	TailLightRect rightResult2;
	leftResult1.confidence = 0;
	leftResult2.confidence = 0;
	rightResult1.confidence = 0;
	rightResult2.confidence = 0;
	clusteringRect(preLeftLight, CLUSTERING_DELTA, treeNode, labelsNode, finalRect);
	for (leftIndex = 0; leftIndex < finalRect->count; leftIndex++)
	{
		if (finalRect->lightRect[leftIndex].confidence > leftResult1.confidence)
		{
			leftResult1 = finalRect->lightRect[leftIndex];
		}
	}
	minDistance = 10000;
	for (leftIndex = 0; leftIndex < leftCount; leftIndex++)
	{
		distance = computeRectDistance(leftLight->lightRect[leftIndex], leftResult1);
		if (distance < minDistance)
		{
			minDistance = distance;
			leftResult2 = leftLight->lightRect[leftIndex];
		}
	}

	clusteringRect(preRightLight, CLUSTERING_DELTA, treeNode, labelsNode, finalRect);
	for (rightIndex = 0; rightIndex < finalRect->count; rightIndex++)
	{
		if (finalRect->lightRect[rightIndex].confidence > rightResult1.confidence)
		{
			rightResult1 = finalRect->lightRect[rightIndex];
		}
	}
	minDistance = 10000;
	for (rightIndex = 0; rightIndex < rightCount; rightIndex++)
	{
		distance = computeRectDistance(rightLight->lightRect[rightIndex], rightResult1);
		if (distance < minDistance)
		{
			minDistance = distance;
			rightResult2 = rightLight->lightRect[rightIndex];
		}
	}

	if (leftResult1.confidence >= lightConfidence && rightResult1.confidence >= lightConfidence && minDistance < MAX_DISTANCE)
	{
		finalRect->lightRect[finalCount] = leftResult2;
		finalCount++;
		finalRect->lightRect[finalCount] = rightResult2;
		finalCount++;
	}
	finalRect->count = finalCount;
	leftLight->lightRect[0] = leftResult2;
	rightLight->lightRect[0] = rightResult2;
	leftLight->count = 1;
	rightLight->count = 1;
}

static void mathchLeftAndRight(const TailLightRect leftResult, const TailLightRect rightResult, const int carWidth, const int lightWidth, TailLightResult *finalRect)
{
	int finalCount = 0;
	int minWdith = (int)(carWidth * 0.8f);
	int maxWidth = carWidth << 4;
	int maxHeight = (lightWidth << 1) + lightWidth;
	int maxY1 = 0;
	int maxY2 = 0;
	int minX = 0;
	int maxX = 0;
	int detlaX = 0;
	int detlaY = 0;
	maxY1 = leftResult.y + leftResult.height;
	maxY2 = rightResult.y + rightResult.height;
	minX = leftResult.x;
	maxX = rightResult.x + rightResult.width;
	detlaX = maxX - minX;
	detlaY = WS_ABS(maxY1 - maxY2);
	if (detlaY <= maxHeight)
	{
		if (detlaX >= minWdith && detlaX <= maxWidth)
		{
			finalRect->lightRect[finalCount] = leftResult;
			finalCount++;
			finalRect->lightRect[finalCount] = rightResult;
			finalCount++;
		}
	}
	finalRect->count = finalCount;
}

static void tailLightMatch(const TailLightResult *leftLight, const TailLightResult *rightLight, const int carWidth, const int lightWidth, TailLightResult *finalRect)
{
	const int leftCount = leftLight->count;
	const int rightCount = rightLight->count;
	int leftIndex = 0;
	int rightIndex = 0;
	int finalCount = 0;
	int minWdith = (int)(carWidth * 0.8f);
	int maxWidth = carWidth << 4;
	int maxHeight = (lightWidth << 1) + lightWidth;
	int maxY1 = 0;
	int maxY2 = 0;
	int minX = 0;
	int maxX = 0;
	int detlaX = 0;
	int detlaY = 0;
	TailLightRect leftResult;
	TailLightRect rightResult;
	TailLightRect tailLightRect;
  	leftResult.confidence = 0;
	for (leftIndex = 0; leftIndex < leftCount; leftIndex++)
	{
        tailLightRect = leftLight->lightRect[leftIndex];
		maxY1 = tailLightRect.y + tailLightRect.height;
		minX = tailLightRect.x;
		rightResult.confidence = 0;
		for (rightIndex = 0; rightIndex < rightCount; rightIndex++)
		{
			maxY2 = rightLight->lightRect[rightIndex].y + rightLight->lightRect[rightIndex].height;
			maxX = rightLight->lightRect[rightIndex].x + rightLight->lightRect[rightIndex].width;
			detlaX = WS_ABS(maxX - minX);
			detlaY = WS_ABS(maxY1 - maxY2);
			if (detlaY <= maxHeight)
			{
				if (detlaX >= minWdith && detlaX <= maxWidth)
				{
					if (tailLightRect.confidence > leftResult.confidence)
					{
						leftResult = tailLightRect;
						if (rightLight->lightRect[rightIndex].confidence > rightResult.confidence)
						{
							rightResult = rightLight->lightRect[rightIndex];
						}
					}
				}
			}
		}
	}
	if (leftResult.confidence > 0)
	{
		finalRect->lightRect[finalCount] = leftResult;
		finalCount++;
		finalRect->lightRect[finalCount] = rightResult;
		finalCount++;
	}
	finalRect->count = finalCount;
}

static void updatePreTailLight(const TailLightRect leftResult, const TailLightRect rightResult, int *confirmTailLightIndex, TailLightResult *preLeftLight, TailLightResult *preRightLight)
{
	int index = (*confirmTailLightIndex) % CONFIRM_LIGHT_COUNT;
	preLeftLight->lightRect[index] = leftResult;
	preRightLight->lightRect[index] = rightResult;
	preLeftLight->count += 1;
	preRightLight->count += 1;
	*confirmTailLightIndex = index + 1;
	if (preLeftLight->count >= CONFIRM_LIGHT_COUNT)
	{
		preLeftLight->count = CONFIRM_LIGHT_COUNT;
		preRightLight->count = CONFIRM_LIGHT_COUNT;
	}
}