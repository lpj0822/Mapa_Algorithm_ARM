#ifndef _DISP_INTERFACE_H_
#define _DISP_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common_type_def.h"
#include "cfg_disparity.h"

//set parameters
extern int setDispParams();

//stereo match
extern void setereoMatch(unsigned char* left, unsigned char* right);

//get disparity
extern void getDispResult(RES_DISP* params);

//release buffer
extern void releaseDispBuff();

#ifdef __cplusplus
}
#endif

#endif 
