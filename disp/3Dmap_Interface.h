#ifndef _3DMAP_INTERFACE_H_
#define _3DMAP_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common_type_def.h"
#include "cfg_disparity.h"

#define SUBBITS_4 0
#define SUBBITS_3 1
#define SUBBITS_2 2
#define SUBBITS_1 3
#define SUBBITS_0 4

typedef struct
{
	int16_t up;
	int16_t down;
	int16_t left;
	int16_t right;
	int16_t biggest_disp;
	int16_t smallest_disp;

	//int64_t nFramSeq;

} TDMAP_FRAME_3D;

typedef struct 
{
	int16_t up;
	int16_t down;
	int16_t left;
	int16_t right;
	//int64_t nFramSeq;//图片帧号
	float32_t fttc;

} TDMAP_FRAME_2D;

typedef struct
{
	float X;//translation
	float Y;//height	
	float Z;//distance
} TDMAP_WORLD_POINT;

typedef struct
{
	TDMAP_WORLD_POINT P1;//left & up
	TDMAP_WORLD_POINT P3;//right & buttom	
} TDMAP_WORLD_RECT;//3D points in world coordinate(m)

typedef struct 
{

	TDMAP_FRAME_2D frame2D;   // 锟斤拷锟斤拷目锟斤拷锟轿伙拷锟�
	//	s32 DisToBottom;   //锟斤拷锟給bjRec锟侥底诧拷锟侥撅拷锟诫，锟斤拷没锟叫硷拷锟斤拷撞锟斤拷呖锟斤拷锟斤拷洗锟斤拷锟较⒃わ拷锟�
	//	int64_t nFramSeq;  // 锟斤拷锟斤拷目锟斤拷锟街★拷锟�
	//	TrackPoint *pCenTrajecy; //锟斤拷锟斤拷目锟斤拷锟斤拷锟斤拷锟斤拷硕锟斤拷旒�
	//	s32 nTrackLen;
	int64_t nId;
	uint8_t bTrue;
	uint8_t bBackground;

	TDMAP_FRAME_3D frame3D;

	TDMAP_WORLD_RECT rectworld; //3D框的世界坐标真实距离

	float speed;
	float dist;
	objtype nType;  //增加了目标类型这块{Car =0,People,Unknow}
	int64_t nTrkCnt;
	//	int16_t confidence;
	int64_t nColor;
}TDMAP_TRACK_OBJ;

typedef struct 
{
	TDMAP_TRACK_OBJ Trackerset[256]; // 鐠虹喕閲滄径姘毌娑擃亞娲伴弽锟�
	s32  nTrackeNum;

}TDMAP_TRACK_DISP;

typedef struct
{
	TDMAP_FRAME_2D frame2D;
	s32 nColor;
}FENCE_RECT;

typedef struct
{
	FENCE_RECT frences[512];
	s32 nFenceNum;
	s32 nWarningNum;
}FENCE_LIST;


void Stereo_detect_init();
void Stereo_detect_show(RES_DISP resDISP, int bitsSUBPIXEL);
void Stereo_detect_getresult(TDMAP_TRACK_DISP *res3DMap, FENCE_LIST *drawFrences, float32_t fSpeed);
void Stereo_detect_release();

#ifdef __cplusplus
}
#endif

#endif
