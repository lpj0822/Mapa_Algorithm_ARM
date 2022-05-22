#include "vehicle_shadow.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OFS (1024) // 256*4
#define SOBEL_TABLE (2304) // OFS * 2 + 256
#define ftzero (80)

#define BLOCK_SIZE_X (5)
#define BLOCK_SIZE_Y (5)

typedef struct ShadowRect
{
	int x;
	int y;
	int width;
	int height;
}ShadowRect;

typedef struct DayShadowGlobalPara
{
	//all memory
	unsigned char *allMemory;
	unsigned char *sobleTable;
	unsigned char *dyImage;
	ShadowRect shadowRoi;
	int relativeRoiY;
}DayShadowGlobalPara;

DayShadowGlobalPara shadowPara;

static int groundFlag = 0;

static void initSobelTable(unsigned char* sobleTable);

static void  prefilterYSobel(const imgage *pOriGrayImg, const ShadowRect *shadowRoi,
	const int relativeRoiY, const unsigned char* sobleTable, unsigned char* dyImage);

int findGroundValue(const imgage *pOriGrayImg, const ShadowRect *shadowRoi, const int relativeRoiY, const unsigned char* dxImage);

int initShadowDetection(const int srcWidth, const int srcHeight, const float srcROIYFactor)
{
    int allMallocSize = 0;
    int count = 0;
    int srcRoiY = 0;
    int srcRoiWidth = 0;
    int srcRoiHeight = 0;
    int stopY = 0;
    unsigned char* copyMallPtr = NULL;
    //LDWS_InitGuid *pLDWSInit = NULL;

    srcRoiY = (int)(srcHeight * srcROIYFactor);
    srcRoiWidth = srcWidth;
    srcRoiHeight = srcHeight - srcRoiY;
    count = srcRoiWidth * srcRoiHeight;

    //LDWS_Getinit(&pLDWSInit);
    //stopY = WS_MAX(srcRoiY, pLDWSInit->pBoundPoint[2].y) + 1;
    stopY = WS_MAX(stopY, srcHeight);

    shadowPara.shadowRoi.x = 0;
    shadowPara.shadowRoi.y = srcRoiY;
    shadowPara.shadowRoi.width = srcWidth;
    shadowPara.shadowRoi.height = stopY - shadowPara.shadowRoi.y;

    shadowPara.relativeRoiY = shadowPara.shadowRoi.y - srcRoiY;

    allMallocSize = SOBEL_TABLE * sizeof(unsigned char) + count * sizeof(unsigned char) + 4 * sizeof(unsigned char);

    shadowPara.allMemory = (unsigned char *)malloc(allMallocSize);

    if(shadowPara.allMemory == NULL)
        return 0;

    memset(shadowPara.allMemory, 0, allMallocSize * sizeof(unsigned char));

    copyMallPtr = shadowPara.allMemory;
    shadowPara.sobleTable = (unsigned char*)copyMallPtr;

    copyMallPtr = ADAS_ALIGN_16BYTE(shadowPara.sobleTable + SOBEL_TABLE);
    shadowPara.dyImage = (uint8_t*)copyMallPtr;

    initSobelTable(shadowPara.sobleTable);

    groundFlag = 0;
    return 1;
}

int computeShadow(const imgage *pOriGrayImg)
{
    int shadowThreshold = 0;
    prefilterYSobel(pOriGrayImg, &shadowPara.shadowRoi, shadowPara.relativeRoiY, shadowPara.sobleTable, shadowPara.dyImage);
    shadowThreshold = findGroundValue(pOriGrayImg, &shadowPara.shadowRoi, shadowPara.relativeRoiY, shadowPara.dyImage);
    //printf("shadowThreshold=%d\n", shadowThreshold);
	return shadowThreshold;
}

int freeShadowDetection()
{
    if (shadowPara.allMemory != NULL)
    {
        free(shadowPara.allMemory);
        shadowPara.allMemory = NULL;
    }
    shadowPara.sobleTable = NULL;
    shadowPara.dyImage = NULL;
    return 1;
}

static void initSobelTable(unsigned char* sobleTable)
{
    int x = 0;
    for (x = 0; x < SOBEL_TABLE; x++)
        sobleTable[x] = (unsigned char)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero * 2 : x - OFS + ftzero);
}

static void  prefilterYSobel(const imgage *pOriGrayImg, const ShadowRect *shadowRoi,
	const int relativeRoiY, const unsigned char* sobleTable, unsigned char* dyImage)
{
    int col = 0;
    int row = 0;
    int startX = shadowRoi->x + 1;
    int startY = shadowRoi->y + 1;
    int stopY = shadowRoi->height + shadowRoi->y - 1;
    int stopX = shadowRoi->width + shadowRoi->x - 2;
    uint8_t *srcData0 = pOriGrayImg->ptr + shadowRoi->y * pOriGrayImg->nWid;
    uint8_t *srcData1 = srcData0 + pOriGrayImg->nWid;
    uint8_t *srcData2 = srcData1 + pOriGrayImg->nWid;
    uint8_t *dstData = dyImage + (relativeRoiY + 1) * shadowRoi->width;
    int16_t d0, d1, d2, d3, v0, v1;
    int index1, index2, index3;

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

//            dstData[col] = (unsigned char)v0;
//            dstData[index2] = (unsigned char)v1;

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
        dstData += pOriGrayImg->nWid;
    }
}

//kalman
static float groundX, groundP, groundA, groundH, groundQ, groundR;

int findGroundValue(const imgage *pOriGrayImg, const ShadowRect *shadowRoi, const int relativeRoiY, const unsigned char* dxImage)
{
    int row = 0;
    int col = 0;
    int y = 0;
    int x = 0;
    int startRow = shadowRoi->y;
    int startCol = shadowRoi->x;
    int stopRow = startRow + shadowRoi->height;
    int stopCol = startCol + shadowRoi->width;
    int stepX = BLOCK_SIZE_X;
    int stepY = BLOCK_SIZE_Y;
    int rowWdith = BLOCK_SIZE_Y * pOriGrayImg->nWid;
    int stopX = 0;
    int stopY = 0;
    int gradientCount = 0;
    int count = 0;
    int value = 0;
    int groundHist[256] = { 0 };
    int numberCount = 0;
    float factor = 0.0f;
    float gain = 0;
    float groundValue = 0, groundVar = 0, groundMean = 0;
	const unsigned char* srcRow = pOriGrayImg->ptr + shadowRoi->y * pOriGrayImg->nWid;
	const unsigned char* gradientRow = dxImage + relativeRoiY * shadowRoi->width;
	const unsigned char* srcData = NULL;
	const unsigned char* gradientData = NULL;

    for (row = startRow; row < stopRow; row += stepY)
    {
        for (col = startCol; col < stopCol; col += stepX)
        {
            gradientCount = 0;
            count = 0;
            stopX = WS_MIN(col + BLOCK_SIZE_X, stopCol);
            stopY = WS_MIN(row + BLOCK_SIZE_Y, stopRow);

            gradientData = gradientRow;

            for (y = row; y < stopY; y++)
            {
                for (x = col; x < stopX; x++)
                {
                   if(gradientData[x])
                   {
                       gradientCount++;
                   }
                   count++;
                }
                gradientData += pOriGrayImg->nWid;
            }
            factor = (float)gradientCount / count;
            if (factor <= 0.02f)
            {
                srcData = srcRow;
                for (y = row; y < stopY; y++)
                {
                    for (x = col; x < stopX; x++)
                    {
                        if(srcData[x] < 200)
                        {
                            value = srcRow[x];
                            groundMean += value;
                            groundHist[value]++;
                            numberCount++;
                        }
                    }
                    srcData += pOriGrayImg->nWid;
                }
            }
        }
        srcRow += rowWdith;
        gradientRow += rowWdith;
    }

    groundMean /= numberCount;

    for (x = 0; x < 256; x++)
    {
        groundVar += (float)(fabs(x - groundMean) * groundHist[x]);
    }
    groundVar /= numberCount;

    groundValue = groundMean - 1.5f * groundVar;
    if (groundValue < 1)
    {
        groundValue = 1;
    }

    /* kalman */
    if (groundFlag == 0)
    {
        groundX = groundValue;
        groundP = 1;
        groundA = 1;
        groundH = 1;
        groundQ = 4e2f;
        groundR = 10e2f;
        groundFlag = 1;
    }
    else
    {
        groundX = groundA * groundX;
        groundP = groundA * groundA * groundP + groundQ;

        gain = groundP * groundH / (groundP * groundH * groundH + groundR);
        groundX = groundX + gain * (groundValue - groundH * groundX);
        groundP = (1 - gain * groundH) * groundP;
    }

    return (int)groundX;
}
