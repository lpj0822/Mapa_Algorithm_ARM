#ifndef _TARGETVERIFY_H
#define _TARGETVERIFY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "type_def.h"
#include "FCWSD_Interface.h"

#define  MAX_TRACK_MUM  128

extern MuliVerfobjRelt pVerfobjOutput;

//flag 0-->ObjVerf 1-->CNN
extern void MuliObjVerfInit(const int resizeWidth, const int resizeHeight, const int flag);

extern void verfObjByDetcor(MuliVerfobj *pVerfobjInput);

extern void updataMuliObjVerf( MuliVerfobj *pVerfobjInput);

extern void getMuliObjVerf();

//flag 0-->ObjVerf 1-->CNN
extern void MuliObjVerfRelease(const int flag);

extern void resizeVeryObject(const int resizeWidth, const int resizeHeight, MuliVerfobj *pVerfobjInput);

extern void resizeFCWSDObject(const int resizeWidth, const int resizeHeight, const FCWSDImage *pOriGrayImg, const objectCar rect, float* objectData);

#ifdef __cplusplus
}
#endif

#endif
