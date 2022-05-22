#ifndef VEHICLE_TAILLIGHT
#define VEHICLE_TAILLIGHT

#include "common_type_def.h"

typedef struct TailLightRect
{
	int x;
	int y;
	int width;
	int height;
	int confidence;
}TailLightRect;

typedef struct TailLightResult
{
	TailLightRect *lightRect;
    int count;
}TailLightResult;

typedef struct DarkTailLightGlobalPara
{
    //all memory
    uint8_t *allMemory;
    uint8_t *imageHist;
    uint8_t *lightImage;
    uint8_t *tailLightImage;
    int *treeNode;
    int *labelsNode;
	TailLightResult leftLightResult;
	TailLightResult rightLightResult;
	TailLightResult finalResult;
	TailLightResult preLeftLight;
	TailLightResult preRightLight;
    AdaSize dataSize;
	int confirmTailLightIndex;
	int preObjectID;
}DarkTailLightGlobalPara;

int initTailLight(const int srcWidth, const int srcHeight, DarkTailLightGlobalPara *tailLightPara);

void computeTailLight(const imgage *pBuff, const AdasRect object, const int objectID, DarkTailLightGlobalPara *tailLightPara);

void freeTailLight(DarkTailLightGlobalPara *tailLightPara);

#endif // VEHICLE_TAILLIGHT

