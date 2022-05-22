#include "3Dmap.h"
//#pragma(lib, "CMuliTrack.lib")
//#pragma(lib, "CMuliTrackd.lib")

#define OVERLAP_RATIO 0.7
#define DIST_RATIO 15.0
#define X_RATIO 100.0

#define LIMIT_WIDTH 800

#if Is_PC
//extern int img_seq;
//用来模拟FCW的识别结果
MuliTracker   g_MuliTracker_3Dmap;
#else

MuliTracker g_MuliTracker_3Dmap;

#endif

//extern Mat_uchar Saved_Img[10];
//extern int Saved_Img_index;
//extern s32 FCW_TRACK_SimpleTrack(const AdasRect SrcRec, imgage *pSrcImg, const imgage *pDstImg, AdasPoint MotionVec, 
//	AdasRect *pDstRec);

extern int skyline_x;
extern int skyline_y;
extern float disp_groundline;
extern int get_RINV_XY(int in_ROI_X,int in_ROI_Y,int *out_x,int *out_y);


int64_t ID_CNT;
//imgage ImgforTrack[2];


void getBoundADASRect(AdasRect src_rect, TDMAP_FRAME_3D *dst_frame)
{
	int ref_x[4], ref_y[4];

#if Is_PC
	get_RINV_XY(src_rect.x, src_rect.y, &ref_x[0], &ref_y[0]);
	get_RINV_XY(src_rect.x, src_rect.y + src_rect.height, &ref_x[1], &ref_y[1]);
	get_RINV_XY(src_rect.x + src_rect.width, src_rect.y, &ref_x[2], &ref_y[2]);
	get_RINV_XY(src_rect.x + src_rect.width, src_rect.y + src_rect.height, &ref_x[3], &ref_y[3]);
#else
	ref_x[0] = src_rect.x;		ref_y[0] = src_rect.y;
	ref_x[1] = src_rect.x;		ref_y[1] = src_rect.y + src_rect.height;
	ref_x[2] = src_rect.x + src_rect.width;		ref_y[2] = src_rect.y;
	ref_x[3] = src_rect.x + src_rect.width;		ref_y[3] = src_rect.y + src_rect.height;
#endif

	dst_frame->left = minof(minof(ref_x[0], ref_x[1]), minof(ref_x[2], ref_x[3]));
	dst_frame->right = maxof(maxof(ref_x[0], ref_x[1]), maxof(ref_x[2], ref_x[3]));
	dst_frame->up = minof(minof(ref_y[0], ref_y[1]), minof(ref_y[2], ref_y[3]));
	dst_frame->down = maxof(maxof(ref_y[0], ref_y[1]), maxof(ref_y[2], ref_y[3]));
}

uchar checkOverlap(TDMAP_FRAME_3D frame, Object obj)
{
	if (frame.left > obj.size_right) { return 0; }
	if (frame.up > obj.size_down) { return 0; }
	if (frame.right < obj.size_left) { return 0; }
	if (frame.down < obj.size_up) { return 0; }
	float colInt = minof(frame.right, obj.size_right) - maxof(frame.left, obj.size_left);
	float rowInt = minof(frame.down, obj.size_down) - maxof(frame.up, obj.size_up);
	float intersection = colInt * rowInt;
	float area = (obj.size_down - obj.size_up)*(obj.size_right - obj.size_left);
	

	if (area < 0.1)
		return 0;
	//	float area2 = box2.width*box2.height;
	if (intersection / area > OVERLAP_RATIO)
		return 1;
	else return 0;
}

TDMAP_TRACK_DISP pre_Tracker; //全局变量 上一帧的跟踪列表
TDMAP_TRACK_DISP cur_Tracker; //全局变量 本帧的跟踪列表

//将OBJ里的物体赋给跟踪列表
void doCopySet(Object objs, TDMAP_TRACK_OBJ *pDispSet, objtype Type)
{
	pDispSet->frame2D.up = objs.size_up;
	pDispSet->frame2D.down = objs.size_down;
	pDispSet->frame2D.left = objs.size_left;
	pDispSet->frame2D.right = objs.size_right;

	pDispSet->frame3D.up = objs.proj_up;
	pDispSet->frame3D.down = objs.proj_down;
	pDispSet->frame3D.left = objs.proj_left;
	pDispSet->frame3D.right = objs.proj_right;
	pDispSet->frame3D.biggest_disp = objs.biggest_disp;
	pDispSet->frame3D.smallest_disp = objs.smallest_disp;

	pDispSet->nType = Type;
	pDispSet->bTrue = 1;

	pDispSet->dist = objs.dist;
	pDispSet->bBackground = objs.isBackground;
}

void doMergeSet(Object objs, TDMAP_TRACK_OBJ *pDispSet)
{
	if (objs.size_up < pDispSet->frame2D.up)
		pDispSet->frame2D.up = objs.size_up;

	if (objs.size_down > pDispSet->frame2D.down)
		pDispSet->frame2D.down = objs.size_down;

	if (objs.size_left < pDispSet->frame2D.left)
		pDispSet->frame2D.left = objs.size_left;

	if (objs.size_right > pDispSet->frame2D.right)
		pDispSet->frame2D.right = objs.size_right;


	if (objs.proj_up < pDispSet->frame3D.up)
		pDispSet->frame3D.up = objs.proj_up;

	if (objs.proj_down > pDispSet->frame3D.down)
		pDispSet->frame3D.down = objs.proj_down;

	if (objs.proj_left < pDispSet->frame3D.left)
		pDispSet->frame3D.left = objs.proj_left;

	if (objs.proj_right > pDispSet->frame3D.right)
		pDispSet->frame3D.right = objs.proj_right;

	if (objs.biggest_disp > pDispSet->frame3D.biggest_disp)
	{
		pDispSet->frame3D.biggest_disp = objs.biggest_disp;
		pDispSet->dist = objs.dist;
	}


	if (objs.smallest_disp < pDispSet->frame3D.smallest_disp)
		pDispSet->frame3D.smallest_disp = objs.smallest_disp;

}

uchar checkRange(TDMAP_TRACK_OBJ trac_obj, Object obj)
{
	//	float dist = (trac_obj.dist + trac_obj.speed) - obj.dist;

	float diff_disp = trac_obj.frame3D.biggest_disp - obj.biggest_disp; //由于误差较大反而带来影响，暂不使用speed数据
	if (diff_disp > 20 || diff_disp < -20)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void doSelectVehicle(Object cur_objs[], MuliTracker FCW_Tracker)
{
	int i, j, k;

	for (i = 0; i < FCW_Tracker.nTrackeNum; i++)
	{
		//如果为可信度为0则不予计算
		if (FCW_Tracker.pTrackerset[i].bTrue == 0)
			continue;

		//判断前帧跟踪列表里是否已存在该物体
		int pre_index = -1;
		for (k = 0; k < pre_Tracker.nTrackeNum; k++)
		{
			if (pre_Tracker.Trackerset[k].nType == Car && pre_Tracker.Trackerset[k].nId == FCW_Tracker.pTrackerset[i].nId)
			{
				pre_index = k;
				break;
			}
		}

		//反LUT换算得到最小包围矩形
		TDMAP_FRAME_3D Bound_Frame;
		getBoundADASRect(FCW_Tracker.pTrackerset[i].objRec, &Bound_Frame);

		int index = cur_Tracker.nTrackeNum;
		for (j = 0; j < 256; j++)
		{
			if (cur_objs[j].cnt == 0 || cur_objs[j].isSelected == 1)
				continue;

			//如果存在，判断一下前列表里的距离
			if (pre_index >= 0)
			{
				if (!checkRange(pre_Tracker.Trackerset[pre_index], cur_objs[j]))
					continue;
			}

			if (checkOverlap(Bound_Frame, cur_objs[j]))
			{
				cur_objs[j].isSelected = 1; //标识此物体已被匹配


				if (cur_Tracker.Trackerset[index].nTrkCnt == 0)
				{
					doCopySet(cur_objs[j], cur_Tracker.Trackerset + index, Car);
					cur_Tracker.nTrackeNum++;
					cur_Tracker.Trackerset[index].nId = FCW_Tracker.pTrackerset[i].nId;

					if (pre_index >= 0) //如果前一帧里已经存在该车辆，更新速度并使计数器加1
					{
						cur_Tracker.Trackerset[index].speed = cur_Tracker.Trackerset[index].dist - pre_Tracker.Trackerset[pre_index].dist;
						cur_Tracker.Trackerset[index].nTrkCnt = pre_Tracker.Trackerset[pre_index].nTrkCnt + 1;

						pre_Tracker.Trackerset[pre_index].bTrue = 0; //已经跟踪到的车辆则将其剔除，用于FCW无结果时的跟踪
					}
					else
					{
						cur_Tracker.Trackerset[index].nTrkCnt = 1;
					}

				}
				else
				{
				//	doMergeSet(cur_objs[j], cur_Tracker.Trackerset + index);
				}

			}
		}

		//使用FCW的结果检测框来修正空间位置
		if (cur_Tracker.nTrackeNum == index) //如果没有匹配成功
		{
			if(pre_index >= 0)
			{
				cur_Tracker.nTrackeNum++;

				cur_Tracker.Trackerset[index].frame3D.biggest_disp = pre_Tracker.Trackerset[pre_index].frame3D.biggest_disp;
				cur_Tracker.Trackerset[index].frame3D.smallest_disp = pre_Tracker.Trackerset[pre_index].frame3D.smallest_disp;
				cur_Tracker.Trackerset[index].nTrkCnt = pre_Tracker.Trackerset[pre_index].nTrkCnt + 1;

				pre_Tracker.Trackerset[pre_index].bTrue = 0; //已经跟踪到的车辆则将其剔除，用于FCW无结果时的跟踪

				cur_Tracker.Trackerset[index].nId = FCW_Tracker.pTrackerset[i].nId;
				cur_Tracker.Trackerset[index].nType = Car;
				cur_Tracker.Trackerset[index].bTrue = 1;
				cur_Tracker.Trackerset[index].dist = DISP2DIST / (float)cur_Tracker.Trackerset[index].frame3D.biggest_disp;

			}
			
		}
		//修正3D位置
		if (cur_Tracker.nTrackeNum > index)
		{
			float proj_ratio = disp_groundline / (float)(cur_Tracker.Trackerset[index].frame3D.biggest_disp);
			cur_Tracker.Trackerset[index].frame3D.up = Bound_Frame.up * proj_ratio + skyline_y * (1 - proj_ratio);
			cur_Tracker.Trackerset[index].frame3D.down = Bound_Frame.down * proj_ratio + skyline_y * (1 - proj_ratio);

			cur_Tracker.Trackerset[index].frame3D.left = Bound_Frame.left * proj_ratio + skyline_x * (1 - proj_ratio);
			cur_Tracker.Trackerset[index].frame3D.right = Bound_Frame.right * proj_ratio + skyline_x * (1 - proj_ratio);

			cur_Tracker.Trackerset[index].frame2D.up = Bound_Frame.up;
			cur_Tracker.Trackerset[index].frame2D.down = Bound_Frame.down;
			cur_Tracker.Trackerset[index].frame2D.left = Bound_Frame.left;
			cur_Tracker.Trackerset[index].frame2D.right = Bound_Frame.right;
		}
	}
}



void doTrackUnvehicle(Object cur_objs[])
{
//	memcpy(ImgforTrack[0].ptr, Saved_Img[(Saved_Img_index - 2) % 10].data, ImgforTrack[0].nHig * ImgforTrack[0].nWid * sizeof(uchar));
//	memcpy(ImgforTrack[1].ptr, Saved_Img[(Saved_Img_index - 1) % 10].data, ImgforTrack[1].nHig * ImgforTrack[1].nWid * sizeof(uchar));
//	ImgforTrack[0].nChannle = 1;
//	ImgforTrack[1].nChannle = 1;

	AdasRect preRec;
	AdasRect curRec;
	AdasPoint motionvec;
	motionvec.x = 0;
	motionvec.y = 0;

#if Is_DEBUG
	//for debug
//	Mat img_forshow1(IMG_H, IMG_W, CV_8UC1);
//	Mat img_forshow2(IMG_H, IMG_W, CV_8UC1);

//	memcpy(img_forshow1.data, Saved_Img[(Saved_Img_index - 2) % 10].data, IMG_H*IMG_W*sizeof(uchar));
//	memcpy(img_forshow2.data, Saved_Img[(Saved_Img_index - 1) % 10].data, IMG_H*IMG_W*sizeof(uchar));

#endif

	int i, j;
	for (i = 0; i < pre_Tracker.nTrackeNum; i++)
	{
		//如果不是非车，PASS
		if (pre_Tracker.Trackerset[i].nType == Unknow || pre_Tracker.Trackerset[i].bTrue == 0)
			continue;



		preRec.x = pre_Tracker.Trackerset[i].frame2D.left;
		preRec.y = pre_Tracker.Trackerset[i].frame2D.up;
		preRec.width = pre_Tracker.Trackerset[i].frame2D.right - preRec.x;
		preRec.height = pre_Tracker.Trackerset[i].frame2D.down - preRec.y;

#if Is_PC

		curRec = preRec;
		if (0)//preRec.width < 200 && preRec.height < 200)
		{
		//	FCW_TRACK_SimpleTrack(preRec, &ImgforTrack[0], &ImgforTrack[1], motionvec, &curRec);
		}
		else
		{
		//	curRec = preRec;
		}
#else
		curRec = preRec;
#endif


#if Is_DEBUG
		//for debug
//		rectangle(img_forshow1, Point2i(preRec.x, preRec.y), Point2i(preRec.x + preRec.width, preRec.y + preRec.height), Scalar(255, 255, 255), 1, 8, 0);
//		rectangle(img_forshow2, Point2i(curRec.x, curRec.y), Point2i(curRec.x + curRec.width, curRec.y + curRec.height), Scalar(255, 255, 255), 1, 8, 0);
#endif

		TDMAP_FRAME_3D curFrm;
		curFrm.up = curRec.y;
		curFrm.down = curRec.y + curRec.height;
		curFrm.left = curRec.x;
		curFrm.right = curRec.x + curRec.width;

		int index = cur_Tracker.nTrackeNum;
		for (j = 0; j < 256; j++)
		{
			if (cur_objs[j].cnt == 0 || cur_objs[j].isSelected == 1)
				continue;

			if (checkOverlap(curFrm, cur_objs[j]) && checkRange(pre_Tracker.Trackerset[i], cur_objs[j])) //如果两个矩形框重合度高 *预定加入距离判断的内容
			{
				cur_objs[j].isSelected = 1;


				if (cur_Tracker.Trackerset[index].nTrkCnt == 0)
				{
					doCopySet(cur_objs[j], cur_Tracker.Trackerset + index, pre_Tracker.Trackerset[i].nType);
					cur_Tracker.Trackerset[index].nId = pre_Tracker.Trackerset[i].nId;
					cur_Tracker.Trackerset[index].speed = cur_Tracker.Trackerset[index].dist - pre_Tracker.Trackerset[i].dist;
					cur_Tracker.nTrackeNum++;

					//跟踪计数加1
					cur_Tracker.Trackerset[index].nTrkCnt = pre_Tracker.Trackerset[i].nTrkCnt + 1;
				}
				else
				{
				//	doMergeSet(cur_objs[j], cur_Tracker.Trackerset + index);
				//	cur_Tracker.Trackerset[index].speed = cur_Tracker.Trackerset[index].dist - pre_Tracker.Trackerset[i].dist;
				}



			}


		}
#if Is_DEBUG
		/*rectangle(img_forshow2, Point2i(cur_Tracker.Trackerset[index].frame2D.left,
			cur_Tracker.Trackerset[index].frame2D.up), Point2i(cur_Tracker.Trackerset[index].frame2D.right,
			cur_Tracker.Trackerset[index].frame2D.down), Scalar(0, 0, 0), 1, 8, 0);

		char message[50];
		if (cur_Tracker.Trackerset[index].nType == Car)
		{
			sprintf(message, "C%06d", cur_Tracker.Trackerset[index].nId);
		}
		else if (cur_Tracker.Trackerset[index].nType == People)
		{
			sprintf(message, "P%06d", cur_Tracker.Trackerset[index].nId);
		}

		putText(img_forshow2, message, Point2i(cur_Tracker.Trackerset[index].frame2D.left,
			cur_Tracker.Trackerset[index].frame2D.up), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1.5);*/
#endif

	}

#if Is_DEBUG
	//画出车
	//for (i = 0; i < cur_Tracker.nTrackeNum; i++)
	//{
	//	if (cur_Tracker.Trackerset[i].nType == Car)
	//	{
	//		rectangle(img_forshow2, Point2i(cur_Tracker.Trackerset[i].frame2D.left,
	//			cur_Tracker.Trackerset[i].frame2D.up), Point2i(cur_Tracker.Trackerset[i].frame2D.right,
	//			cur_Tracker.Trackerset[i].frame2D.down), Scalar(0, 0, 0), 1, 8, 0);

	//		char message[50];
	//		sprintf(message, "C%06d", cur_Tracker.Trackerset[i].nId);
	//		putText(img_forshow2, message, Point2i(cur_Tracker.Trackerset[i].frame2D.left,
	//			cur_Tracker.Trackerset[i].frame2D.up), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1.5);
	//	}
	//}

	//	imshow("tracker1", img_forshow1);
	//	imshow("tracker2", img_forshow2);
#endif
}

void doRemoveOcclused()
{
	int i, j;
	for (i = 0; i < cur_Tracker.nTrackeNum; i++)
	{
		for (j = i + 1; j < cur_Tracker.nTrackeNum; j++)
		{
			if (cur_Tracker.Trackerset[i].bTrue == 0 || cur_Tracker.Trackerset[j].bTrue == 0)
			{
				continue;
			}

			if (cur_Tracker.Trackerset[i].frame2D.left > cur_Tracker.Trackerset[j].frame2D.right) { continue; }
			if (cur_Tracker.Trackerset[i].frame2D.up > cur_Tracker.Trackerset[j].frame2D.down) { continue; }
			if (cur_Tracker.Trackerset[i].frame2D.right < cur_Tracker.Trackerset[j].frame2D.left) { continue; }
			if (cur_Tracker.Trackerset[i].frame2D.down < cur_Tracker.Trackerset[j].frame2D.up) { continue; }
			float colInt = minof(cur_Tracker.Trackerset[i].frame2D.right, cur_Tracker.Trackerset[j].frame2D.right)
				- maxof(cur_Tracker.Trackerset[i].frame2D.left, cur_Tracker.Trackerset[j].frame2D.left);
			float rowInt = minof(cur_Tracker.Trackerset[i].frame2D.down, cur_Tracker.Trackerset[j].frame2D.down)
				- maxof(cur_Tracker.Trackerset[i].frame2D.up, cur_Tracker.Trackerset[j].frame2D.up);
			float intersection = colInt * rowInt;
			float area = (cur_Tracker.Trackerset[j].frame2D.down - cur_Tracker.Trackerset[j].frame2D.up)*(cur_Tracker.Trackerset[j].frame2D.right - cur_Tracker.Trackerset[j].frame2D.left);

			if ((intersection / area > 0.4) && (cur_Tracker.Trackerset[j].frame3D.biggest_disp < cur_Tracker.Trackerset[i].frame3D.biggest_disp))
			{
				cur_Tracker.Trackerset[j].bTrue = 0;
			}
		}
	}
}

void doIdentifyRest(Object cur_objs[])
{
	int i;
	for (i = 0; i < 256; i++)
	{
		if (cur_objs[i].cnt == 0 || cur_objs[i].isSelected == 1)
			continue;

		int index = cur_Tracker.nTrackeNum;
		if (cur_objs[i].isBackground == 0)
		{
			doCopySet(cur_objs[i], cur_Tracker.Trackerset + index, People);
			cur_Tracker.nTrackeNum++;
			cur_Tracker.Trackerset[index].nId = ID_CNT;
			cur_Tracker.Trackerset[index].nTrkCnt = 1;
			cur_Tracker.Trackerset[index].bTrue = 1;
			ID_CNT++;
		}
		else
		{
			doCopySet(cur_objs[i], cur_Tracker.Trackerset + index, Unknow);
			cur_Tracker.nTrackeNum++;
		}
	}
}

//使用车道线的结果对较大物体进行分割，暂不使用
//void doLineInfoProcessing()
//{
//	LDWS_Output *pTest = NULL;
//	LDWS_GetResult(&pTest);
//
//	LDWS_Point pt[2];
//	pt[0] = pTest->pPoint[42];
//	pt[1] = pTest->pPoint[85];
//
//#if Is_DEBUG
//
//	pt[0].x = 190;
//	pt[0].y = 290;
//
//	pt[1].x = 680;
//	pt[1].y = 290;
//
//#endif
//
//	float proj_ratio = (IMG_H - SKYLINE_Y) / (float)(pt[0].y - SKYLINE_Y);
//	int proj_u_left = pt[0].x * proj_ratio + SKYLINE_X * (1 - proj_ratio);
//	int proj_u_right = pt[1].x * proj_ratio + SKYLINE_X * (1 - proj_ratio);
//
//	int proj_u[3];
//	proj_u[1] = (proj_u_left + proj_u_right) >> 1;
//	proj_u[0] = proj_u_left + proj_u_left - proj_u[1];
//	proj_u[2] = proj_u_right + proj_u_right - proj_u[1];
//
//	int i, j;
//	//融合本车道内的物体
//	for (i = 0; i < cur_Tracker.nTrackeNum; i++)
//	{
//		if (cur_Tracker.Trackerset[i].bTrue == 0 || cur_Tracker.Trackerset[i].nType == Car)
//		{
//			continue;
//		}
//
//		if (cur_Tracker.Trackerset[i].frame3D.left < proj_u_left || cur_Tracker.Trackerset[i].frame3D.right > proj_u_right)
//		{
//			continue;
//		}
//
//
//		for (j = i + 1; j < cur_Tracker.nTrackeNum; j++)
//		{
//			if (cur_Tracker.Trackerset[j].bTrue == 0 || cur_Tracker.Trackerset[j].nType == Car)
//			{
//				continue;
//			}
//
//			if (cur_Tracker.Trackerset[j].frame3D.left < proj_u_left || cur_Tracker.Trackerset[j].frame3D.right > proj_u_right)
//			{
//				continue;
//			}
//
//			int disp_diff = cur_Tracker.Trackerset[i].frame3D.biggest_disp - cur_Tracker.Trackerset[j].frame3D.biggest_disp;
//			if (disp_diff > -10 && disp_diff < 10)
//			{
//				//融合两个物体
//				cur_Tracker.Trackerset[i].frame2D.up = minof(cur_Tracker.Trackerset[i].frame2D.up, cur_Tracker.Trackerset[j].frame2D.up);
//				cur_Tracker.Trackerset[i].frame2D.left = minof(cur_Tracker.Trackerset[i].frame2D.left, cur_Tracker.Trackerset[j].frame2D.left);
//				cur_Tracker.Trackerset[i].frame2D.down = maxof(cur_Tracker.Trackerset[i].frame2D.down, cur_Tracker.Trackerset[j].frame2D.down);
//				cur_Tracker.Trackerset[i].frame2D.right = maxof(cur_Tracker.Trackerset[i].frame2D.right, cur_Tracker.Trackerset[j].frame2D.right);
//
//				cur_Tracker.Trackerset[i].frame3D.up = minof(cur_Tracker.Trackerset[i].frame3D.up, cur_Tracker.Trackerset[j].frame3D.up);
//				cur_Tracker.Trackerset[i].frame3D.left = minof(cur_Tracker.Trackerset[i].frame3D.left, cur_Tracker.Trackerset[j].frame3D.left);
//				cur_Tracker.Trackerset[i].frame3D.down = maxof(cur_Tracker.Trackerset[i].frame3D.down, cur_Tracker.Trackerset[j].frame3D.down);
//				cur_Tracker.Trackerset[i].frame3D.right = maxof(cur_Tracker.Trackerset[i].frame3D.right, cur_Tracker.Trackerset[j].frame3D.right);
//
//				cur_Tracker.Trackerset[i].frame3D.smallest_disp = minof(cur_Tracker.Trackerset[i].frame3D.smallest_disp, cur_Tracker.Trackerset[j].frame3D.smallest_disp);
//				cur_Tracker.Trackerset[i].frame3D.biggest_disp = maxof(cur_Tracker.Trackerset[i].frame3D.biggest_disp, cur_Tracker.Trackerset[j].frame3D.biggest_disp);
//
//				cur_Tracker.Trackerset[i].dist = minof(cur_Tracker.Trackerset[i].dist, cur_Tracker.Trackerset[j].dist);
//
//				cur_Tracker.Trackerset[j].bTrue = 0;
//			}
//
//		}
//	}
//
//
//
//	//切割较大物体
//	for (i = 0; i < cur_Tracker.nTrackeNum; i++)
//	{
//		if (cur_Tracker.Trackerset[i].bTrue == 0)
//			continue;
//
//		//如果物体的投影宽度大于预设值
//		if ((cur_Tracker.Trackerset[i].frame3D.right - cur_Tracker.Trackerset[i].frame3D.left) > LIMIT_WIDTH)
//		{
//			float ratio = cur_Tracker.Trackerset[i].frame3D.biggest_disp / PROJ_DISP;
//
//			if ((cur_Tracker.Trackerset[i].frame3D.left < proj_u_left) && (cur_Tracker.Trackerset[i].frame3D.right > proj_u[1]))
//			{
//				//沿左线切割，保留接近中位线的部分为原物体，然后添加新物体
//
//				int u_left = proj_u_left * ratio + SKYLINE_X * (1 - ratio);
//
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum] = cur_Tracker.Trackerset[i];
//
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].frame3D.right = proj_u_left;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].frame2D.right = u_left;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].nType = People;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].nTrkCnt = 1;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].nId = ID_CNT;
//				ID_CNT++;
//
//
//				cur_Tracker.Trackerset[i].frame3D.left = proj_u_left;
//				cur_Tracker.Trackerset[i].frame2D.left = u_left;
//
//				cur_Tracker.nTrackeNum++;
//			}
//
//			if ((cur_Tracker.Trackerset[i].frame3D.left < proj_u[1]) && (cur_Tracker.Trackerset[i].frame3D.right > proj_u_right))
//			{
//				//沿右线切割，保留接近中位线的部分为原物体，然后添加新物体
//
//				int u_right = proj_u_right * ratio + SKYLINE_X * (1 - ratio);
//
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum] = cur_Tracker.Trackerset[i];
//
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].frame3D.left = proj_u_right;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].frame2D.left = u_right;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].nType = People;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].nTrkCnt = 1;
//				cur_Tracker.Trackerset[cur_Tracker.nTrackeNum].nId = ID_CNT;
//				ID_CNT++;
//
//				cur_Tracker.Trackerset[i].frame3D.right = proj_u_right;
//				cur_Tracker.Trackerset[i].frame2D.right = u_right;
//
//				cur_Tracker.nTrackeNum++;
//			}
//		}
//
//
//	}
//}
void Init_TrackObstacle()
{
//	ImgforTrack[0].ptr = (uchar *)malloc(Saved_Img[0].rows*Saved_Img[0].cols*sizeof(uchar));
//	ImgforTrack[0].nHig = Saved_Img[0].rows;
//	ImgforTrack[0].nWid = Saved_Img[0].cols;

//	ImgforTrack[1].ptr = (uchar *)malloc(Saved_Img[0].rows*Saved_Img[0].cols*sizeof(uchar));
//	ImgforTrack[1].nHig = Saved_Img[0].rows;
//	ImgforTrack[1].nWid = Saved_Img[0].cols;

	ID_CNT = 1;
}
void doTrackObstacle(Object cur_objs[])
{

	memset(&cur_Tracker, 0, sizeof(TDMAP_TRACK_DISP));

#if Is_PC
	//	if (img_seq == 0)
	//	{
	//		memset(&pre_Tracker, 0, sizeof(TDMAP_TRACK_DISP));
	//	}
#endif


	doSelectVehicle(cur_objs, g_MuliTracker_3Dmap);

//	if (Saved_Img_index > 1)
//	{
//		doTrackUnvehicle(cur_objs);
//	}

	doIdentifyRest(cur_objs);

	//移除被其他物体所遮挡的物体
	//doRemoveOcclused();


	//借助车道线信息进行以下处理，达成以下效果
	//前方车道内的物体，如果比较接近则进行融合
	//如果存在横向较大物体，则强行进行切割
	//	doLineInfoProcessing();

	//最后把cur_trakcer的值赋给pre_trakcer
	memcpy(&pre_Tracker, &cur_Tracker, sizeof(TDMAP_TRACK_DISP));
}

//if (Saved_Img_index > 1)
//{
//	memcpy(ImgforTrack[0].ptr, Saved_Img[(Saved_Img_index - 2) % 10].data, ImgforTrack[0].nHig * ImgforTrack[0].nWid * sizeof(uchar));
//	memcpy(ImgforTrack[1].ptr, Saved_Img[(Saved_Img_index - 1) % 10].data, ImgforTrack[1].nHig * ImgforTrack[1].nWid * sizeof(uchar));
//
//	while (1)
//	{
//		cin >> srcRec.x >> srcRec.y >> srcRec.width >> srcRec.height;
//		FCW_TRACK_SimpleTrack(srcRec, &ImgforTrack[0], &ImgforTrack[1], motionvec, &dstRec);
//
//		Saved_Img[(Saved_Img_index - 2) % 10].copyTo(img_forshow1);
//		Saved_Img[(Saved_Img_index - 1) % 10].copyTo(img_forshow2);
//
//
//		rectangle(img_forshow1, Point2i(srcRec.x, srcRec.y), Point2i(srcRec.x + srcRec.width, srcRec.y + srcRec.height), Scalar(255, 255, 255), 1, 8, 0);
//		rectangle(img_forshow2, Point2i(dstRec.x, dstRec.y), Point2i(dstRec.x + dstRec.width, dstRec.y + dstRec.height), Scalar(255, 255, 255), 1, 8, 0);
//
//		cout << dstRec.x << " " << dstRec.y << " " << dstRec.width << " " << dstRec.height << endl;
//
//		imshow("tracker1", img_forshow1);
//		imshow("tracker2", img_forshow2);
//
//		waitKey(0);
//	}
//
//}
