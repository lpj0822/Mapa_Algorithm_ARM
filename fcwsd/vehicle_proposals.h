#ifndef VEHICLE_PROPOSALS
#define VEHICLE_PROPOSALS

#include "vehicle_type.h"

typedef struct ProposalsGlobalPara
{
	//all memory
	uint8_t *allMemory;
	uint8_t *sobleTable;
	uint8_t *dyImage;
	uint8_t *dyLineImage;
	u32 *dyIntegralImage;
	FCWSDRect proposalsRoi;
	FCWSDRect integrateRoi;
	s32 relativeRoiY;
}ProposalsGlobalPara;

int initProposals(const s32 srcWidth, const s32 srcHeight, const float srcROIYFactor);

//flag 0-day 1-night
void computeProposals(const unsigned char flag, const FCWSDImage *pOriGrayImg);

//flag 0-day 1-night
int filterCarTask(const unsigned char flag, const lbpTaskCar* task, const FCWSDetectorGlobalPara *pVehicleDetor, const float factor);

int freeProposals();

#endif // VEHICLE_PROPOSALS

