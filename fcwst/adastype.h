#ifndef ADAS_TYPE_H
#define ADAS_TYPE_H
#include "type_def.h"
#include "ObjGroup.h"


typedef struct ADASFCWINNERGLOBALPARA
{
	uint8_t *m_pAlloc;


	xy* m_pXYCorners;
	xy* m_pXYNoMax;
	s32 * m_pRowStart;
	s32 * m_pScore;
	AdasCorner *m_pCornerPass;
	s32 m_nCornerPassNum;
	s32 m_nCornerThresh;
	s32 m_nInitSuccess;
	AdasCorner *m_pFastCorner;
	s32 m_nFastCornerNum;
	uint8_t *m_pfeature;

	AdasCorner *m_Init_pFastCorner;
	s32 m_Init_nFastCornerNum;
	uint8_t *m__Init_pfeature;


	AdasRect *m_PNewRec;
	s32 m_NewRecNum;

	uint8_t *m_preGrayData;
	uint8_t *m_pGrayData;
	s32  m_ImgWidth;
	s32  m_ImgHeight;
	AdaSize m_ImgSize;

	uint8_t *m_pMask;

	obj_group *m_pGroupSets;

	s32 m_GroupId;
	PortInput *pOriInPutParam;
	s32 * m_ndx;
	s32 * m_ndy;
	float32_t *m_fsclare; 
	s32 *m_PublacSape;
	s32 *m_extern_Space;
	uint8_t *m_pGroupIndex;
	uint8_t m_GroupIndexNum;
	PortInput scaleInput;

	s32 nscope_start_y;
	s32 nscope_end_y;
	s32 nscope_start_x;
	s32 nscope_end_x;

}adas_fcw_inner_global_param;

#endif
