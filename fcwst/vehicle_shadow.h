#ifndef VEHICLE_SHADOW_H
#define VEHICLE_SHADOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "type_def.h"
#include "LDWS_Interface.h"

extern int initShadowDetection(const int srcWidth, const int srcHeight, const float srcROIYFactor);

extern int computeShadow(const imgage *pOriGrayImg);

extern int freeShadowDetection();

#ifdef __cplusplus
}
#endif

#endif // VEHICLE_SHADOW_H
