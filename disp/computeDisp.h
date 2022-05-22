#ifndef _WISSEN_BM_H_
#define _WISSEN_BM_H_

#include "common_type_def.h"
#include "cfg_disparity.h"

extern unsigned char tab0[TABSZ];
extern unsigned char tab1[TABSK];

/*
*Function name:  computeDisp
*Parameter list: Name      I/O      Type	                     Description
*                left       I 	     const unsigned char*        left image buffer poniter
*                right      I       const unsigned char         right image buffer pointer
*                disp       O       short                       disparity image buffer pointer  
*Return value:   Type    Description
*                void*   The func tion returns the aligned pointer of the same type as the input pointer
*Function description: compute disparity image.
*/
extern void computeDisp(const unsigned char* left, const unsigned char* right, short* disp, cfgBM params);

#endif

