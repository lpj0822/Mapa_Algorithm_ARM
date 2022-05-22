#include "3Dmap.h"
#include "LDWS_Interface.h"

int skyline_x;
int skyline_y;
float disp_groundline;
float disp_limit_up;
float camera_height, limit_height, limit_dist;
int disp_limit_lower;

extern TDMAP_TRACK_DISP cur_Tracker;
extern cfgRectify inPutParams;
extern LDWS_Point ptVanish, ptOrg, ptTel;
extern float32_t slpOrg, slpTel;
static float32_t avrDis[MAX_MOTION_NUM] = { 0 };

int get_RINV_XY(int in_ROI_X,int in_ROI_Y,int *out_x,int *out_y)
{
	const int img_W = inPutParams.orgWidth;
	const int img_H = inPutParams.orgHeight;
	float* in_p_INVx = inPutParams.INV_Rx;
	float* in_p_INVy = inPutParams.INV_Ry;

	int index_XY;
	if(in_ROI_X < 0 ||\
		in_ROI_X >= img_W ||\
		in_ROI_Y < 0 ||\
		in_ROI_Y >= img_H)
	{
		*out_x = 0;
		*out_y = 0;
		return 0 ;
	}
	index_XY = in_ROI_Y * img_W  + in_ROI_X;
	*out_x = *(in_p_INVx + index_XY);
	*out_y = *(in_p_INVy + index_XY);

	return 1;
}

int get_LINV_XY(int in_ROI_X,int in_ROI_Y,int *out_x,int *out_y)
{
	const int img_W = inPutParams.orgWidth;
	const int img_H = inPutParams.orgHeight;
	float* in_p_INVx = inPutParams.INV_Lx;
	float* in_p_INVy = inPutParams.INV_Ly;

	int index_XY;
	if(in_ROI_X < 0 ||\
		in_ROI_X >= img_W ||\
		in_ROI_Y < 0 ||\
		in_ROI_Y >= img_H)
	{
		*out_x = 0;
		*out_y = 0;
		return 0 ;
	}
	index_XY = in_ROI_Y * img_W  + in_ROI_X;
	*out_x = *(in_p_INVx + index_XY);
	*out_y = *(in_p_INVy + index_XY);

	return 1;
}


int get_XY(int in_x, int in_y, int *out_Lx, int *out_Ly, int *out_Rx, int *out_Ry)
{
	int orgWidth  = inPutParams.orgWidth;
	int orgHeight = inPutParams.orgHeight;
	int cutWidth  = inPutParams.cutWidth;
	int cutHeight = inPutParams.cutHeight;
	float* RTF_Lx = inPutParams.RTF_Lx;
	float* RTF_Ly = inPutParams.RTF_Ly;
	float* RTF_Rx = inPutParams.RTF_Rx;
	float* RTF_Ry = inPutParams.RTF_Ry;

	int Sx = inPutParams.cutSx;
	//int Ex = Sx + cutWidth;
	int Sy = inPutParams.cutSy;
	//int Ey = Sy + cutHeight;

	//if(in_x >= cutWidth || in_x < 0 || in_y >= cutHeight || in_y < 0)
	//{
	//	return 0;
	//}

	int i = WS_MIN(WS_MAX(in_x + Sx, 0), orgWidth - 1);
	int j = WS_MIN(WS_MAX(in_y + Sy, 0), orgHeight - 1);
	int L_x = RTF_Lx[j * orgWidth + i];
	int L_y = RTF_Ly[j * orgWidth + i];
	int R_x = RTF_Rx[j * orgWidth + i];
	int R_y = RTF_Ry[j * orgWidth + i];

	if (L_x >= 0 && L_y >= 0 && L_x < orgWidth && L_y < orgHeight)
	{
		*out_Lx = L_x;
		*out_Ly = L_y;
	}
	//else if (L_y >= orgHeight)
	//{
	//	do {
	//		j--;
	//		L_x = RTF_Lx[j * orgWidth + i];
	//		L_y = RTF_Ly[j * orgWidth + i];
	//	} while (L_y >= orgHeight);
	//}
	else
	{
		*out_Lx = 0;
		*out_Ly = 0;
	}
	if (R_x >= 0 && R_y >= 0 && R_x < orgWidth && R_y < orgHeight)
	{
		*out_Rx = R_x;
		*out_Ry = R_y;
	}
	else if (R_y >= orgHeight)
	{
		do {
			j--;
			R_x = RTF_Rx[j * orgWidth + i];
			R_y = RTF_Ry[j * orgWidth + i];
		} while (R_y >= orgHeight);
		*out_Rx = R_x;
		*out_Ry = R_y;
	}
	else
	{
		*out_Rx = 0;
		*out_Ry = 0;
	}
	return 1;
}

void doCalcVMaskforVoting(Mat_uchar* VProjMask_Disp)
{
	int v = 0, u = 0;

	int Y_ENDPOINT = skyline_y;
	int X_BOTTOM = disp_groundline - 5;
	int X_UP = disp_limit_up;

	int x_min = 1;

	//权重为10区域
	int x_up = X_UP;
	int y_limit_0 = (Y_ENDPOINT - 25)>0?(Y_ENDPOINT - 25):0;
	int y_limit_1 = Y_ENDPOINT;
	int x_down = X_BOTTOM;
	
	int value = 10;
	
	for (v = 0; v < y_limit_0; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			if (u >(x_up - v*(x_up - x_min) / y_limit_0))
			{
				VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			}
		}
	}
	for (v = y_limit_0; v < y_limit_1; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
		}
	}
	for (v = y_limit_1; v < VProjMask_Disp->rows; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			if (u >((v - y_limit_1)*(x_down - x_min) / (VProjMask_Disp->rows - y_limit_1) + x_min))
			{
				VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			}
		}
	}
	
	

	//权重为20区域
	x_up = X_UP;
	y_limit_0 = (Y_ENDPOINT - 25>0)?(Y_ENDPOINT - 25):0;
	y_limit_1 = Y_ENDPOINT;
	x_down = X_BOTTOM + 30;

	value = 20;

	for (v = 0; v < y_limit_0; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			if (u >(x_up - v*(x_up - x_min) / y_limit_0))
			{
				VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			}
		}
	}
	for (v = y_limit_0; v < y_limit_1; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
		}
	}
	for (v = y_limit_1; v < VProjMask_Disp->rows; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			if (u >((v - y_limit_1)*(x_down - x_min) / (VProjMask_Disp->rows - y_limit_1) + x_min))
			{
				VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			}
		}
	}
	
	
	//权重为50区域
	x_up = X_UP;
	y_limit_0 = (Y_ENDPOINT - 25)>0?(Y_ENDPOINT - 25):0;
	y_limit_1 = Y_ENDPOINT;
	x_down = X_BOTTOM + 80;

	value = 50;//高价值较远区域
	for (v = 0; v < y_limit_0; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			if (u >(x_up - v*(x_up - x_min) / y_limit_0))
			{
				VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			}
		}
	}
	for (v = y_limit_0; v < y_limit_1; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
		}
	}
	for (v = y_limit_1; v < VProjMask_Disp->rows; v++)
	{
		for (u = x_min; u < VProjMask_Disp->cols; u++)
		{
			if (u >((v - y_limit_1)*(x_down - x_min) / (VProjMask_Disp->rows - y_limit_1) + x_min))
			{
				VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			}
		}
	}
	
	//最终给远处的值增加权重
	
	for (v = 0; v < VProjMask_Disp->rows; v++)
	{
		for (u = x_min; u < 50; u++)
		{
			
			value = VProjMask_Disp->data[v*VProjMask_Disp->cols + u] * 50/u;
			if (value > 200) 
				value = 200;
			VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = value;
			
		}

	}

	//切掉超过距离上限的部分
	for (v = 0; v < VProjMask_Disp->rows; v++)
	{
		for (u = 0; u < disp_limit_lower; u++)
		{
			VProjMask_Disp->data[v*VProjMask_Disp->cols + u] = 0;
		}
	}

}

static Mat_short DispImg = { NULL, 0, 0 };
static Mat_uchar VMask = { NULL, 0, 0 };
static Mat_short DXImg = { NULL, 0, 0 };
static Mat_short lableImg = { NULL, 0, 0 };
static Mat_color ColorImg = { NULL, 0, 0 };
static Mat_uchar GrayImg = { NULL, 0, 0 };

void Stereo_detect_init()
{
	doInitialMatShort(&DispImg, IMG_H, IMG_W);
	doInitialMatUchar(&VMask, IMG_H, 256);
	doInitialMatShort(&DXImg, COM_H, UX_WIDTH);
	doInitialMatShort(&lableImg, DXImg.rows, DXImg.cols);
	doInitialMatColor(&ColorImg, IMG_H, IMG_W);
	doInitialMatUchar(&GrayImg, IMG_H, IMG_W);


	float elev_angle = atan((skyline_y - inPutParams.cutHeight / 2) / (96 / 0.08)) * 180 / 3.1415926;
	float radi_angle = elev_angle*3.1415926 / 180.0;

	//计算消失线的y坐标和接地点的视差, 通过单目测距得到最近的路面视差值
	//float radi_angle = elev_angle*3.1415926 / 180.0;
	//skyline_y = IMG_H / 2.0 + (96 / 0.08)*tan(radi_angle);
	//skyline_y = 25;
	disp_groundline = cos(1.308997 + radi_angle) * DISP2DIST / (cos(0.261799)*camera_height);
	disp_limit_up = cos(1.308997 - radi_angle) * DISP2DIST / (cos(0.261799)*(limit_height - camera_height));
	disp_limit_lower = DISP2DIST / limit_dist;
	printf("skyline_y %d, disp_groundline %f, disp_limit_lower %d, disp_limit_up %f\n",skyline_y,disp_groundline,disp_limit_lower,disp_limit_up);

	doCalcVMaskforVoting(&VMask);

	//初始化DX转换用Mat
 	doInitMatUX(DispImg, DXImg);

	/*int x, y;
	get_INV_XY(502, 262, &x, &y);
	cout << endl << x << "         " << y << endl;
	int lx, ly, rx, ry;
	get_XY(288, 243, &lx, &ly, &rx, &ry);
	cvWaitKey(0);*/

	Init_CalcObstacle();
}

void Stereo_detect_show(RES_DISP resDISP, int bitsSUBPIXEL)
{
 
	memset(DXImg.data, 0, DXImg.rows*DXImg.cols*sizeof(short));
	memset(lableImg.data, 0, lableImg.rows*lableImg.cols*sizeof(short));
	memset(DispImg.data, 0, DispImg.rows*DispImg.cols*sizeof(short));
	memset(ColorImg.data, 0, ColorImg.rows*ColorImg.cols*sizeof(COLOR));
	memset(GrayImg.data, 0, GrayImg.rows*GrayImg.cols*sizeof(uchar));

#if Is_PC
	Mat OriImg(IMG_H, IMG_W, CV_8UC3);
#endif
	//复制到DISP
	//将原景观图的值赋给ColorImg结构体
	int u,v, value ;
	for (v = 0; v < IMG_H; v++)
	{
		//==========只做下部分的视差=====================================================
//		if(v<131 || v>=293) {
//			for (u = 0; u < IMG_W; u++) {
//				DispImg.data[v * IMG_W + u] = 0;
//				ColorImg.data[v * ColorImg.cols + u].blue = 0;
//				ColorImg.data[v * ColorImg.cols + u].green = 0;
//				ColorImg.data[v * ColorImg.cols + u].red = 0;
//				GrayImg.data[v * ColorImg.cols + u] = 0;
//#if Is_PC
//			OriImg.at<Vec3b>(v, u) = Vec3b(value, value, value);
//#endif
//			}
//		}
//		else {
//			for (u = 0; u < IMG_W; u++) {
//				value = resDISP.disp[(v-131) * IMG_W + u];//[4 * v * IMG_W + 2 * u];
//				DispImg.data[v * IMG_W + u] = value >> bitsSUBPIXEL;
//
//				value = resDISP.imgL[(v-131) * IMG_W + u];//[4*v*IMG_W + 2*u];
//				ColorImg.data[v * ColorImg.cols + u].blue = value;
//				ColorImg.data[v * ColorImg.cols + u].green = value;
//				ColorImg.data[v * ColorImg.cols + u].red = value;
//				GrayImg.data[v * ColorImg.cols + u] = value;
//
//#if Is_PC
//			OriImg.at<Vec3b>(v, u) = Vec3b(value, value, value);
//#endif
//			}
//		}
		//===============================================================

		for (u = 0; u < IMG_W; u++)
			{
				int value = resDISP.disp[v*IMG_W + u];
				if(value<0)
					continue;
				DispImg.data[v*IMG_W + u] = value >> bitsSUBPIXEL;

				value = resDISP.imgL[v*IMG_W + u];

				ColorImg.data[v*ColorImg.cols + u].blue = value;
				ColorImg.data[v*ColorImg.cols + u].green = value;
				ColorImg.data[v*ColorImg.cols + u].red = value;

				GrayImg.data[v*ColorImg.cols + u] = value;

#if Is_PC
			OriImg.at<Vec3b>(v, u) = Vec3b(value, value, value);
#endif

		}
	}

	getDistImg(&DispImg); //保存视差图,用来供其他接口取得视差图

	doCalcDXimg(&DispImg, &DXImg, VMask); //计算DX图

	doCalcObstacle(DXImg, &lableImg);//提取障碍物


#if Is_PC
	doFilterObstacle(lableImg, DispImg, ColorImg, OriImg);
#else
	doFilterObstacle(lableImg, DispImg, ColorImg);
#endif

#if Is_PC

	Mat DXImg_forshow(DXImg.rows, DXImg.cols, CV_8UC1);

	for (int v = 0; v < DXImg.rows; v++)
	{
		for (int u = 0; u < DXImg.cols; u++)
		{
			int value = DXImg.data[v*lableImg.cols + u] / 16;
			if (value <= 0) value = 0;
			else if (value > 255) value = 255;
			DXImg_forshow.at<uchar>(v, u) = value;
		}
	}

	//imshow("DXImg", DXImg_forshow);

	Mat Img_forshow(DXImg.rows, DXImg.cols, CV_8UC1);

	for (int v = 0; v < lableImg.rows; v++)
	{
		for (int u = 0; u < lableImg.cols; u++)
		{
			int value = lableImg.data[v*lableImg.cols + u];
			if (value <= 0) value = 255;
			else if (value > 255) value = 255;
			Img_forshow.at<uchar>(v, u) = 255 - value;
		}
	}

	//imshow("forshow01", Img_forshow);

	Mat VProjMask_DispMat(VMask.rows, VMask.cols, CV_8UC1, Scalar(0));


	for (int v = 0; v < VMask.rows; v++) //显示用
	{
		for (int u = 0; u < VMask.cols; u++)
		{
			VProjMask_DispMat.at<uchar>(v, u) = VMask.data[v*VMask.cols + u];
		}
	}

	//imshow("VMask1", VProjMask_DispMat);

#endif

}

void Stereo_detect_getresult(TDMAP_TRACK_DISP *res3DMap, FENCE_LIST *drawFrences, float32_t fSpeed)
{


	memcpy(res3DMap, &cur_Tracker, sizeof(TDMAP_TRACK_DISP));
	int i = 0;
//	int up, down, left, right;
	int ref_x[4], ref_y[4];
	int dummy_x, dummy_y;

	drawFrences->nFenceNum = 0;
	drawFrences->nWarningNum = 0;

	for (i = 0; i < res3DMap->nTrackeNum; i++)
	{
		/*up = res3DMap->Trackerset[i].frame2D.up;
		down = res3DMap->Trackerset[i].frame2D.down;
		left = res3DMap->Trackerset[i].frame2D.left;
		right = res3DMap->Trackerset[i].frame2D.right;*/

		if (res3DMap->Trackerset[i].bTrue == 0)
		{
			continue;
		}

		/*if (res3DMap->Trackerset[i].frame3D.left > 376 || res3DMap->Trackerset[i].frame3D.right<100)
		{
			res3DMap->Trackerset[i].bTrue = 0;
			continue;
		}

		if (res3DMap->Trackerset[i].frame3D.right - res3DMap->Trackerset[i].frame3D.left >850)
		{
			res3DMap->Trackerset[i].bTrue = 0;
			continue;
		}*/
		//所有的物体都归为背景
		res3DMap->Trackerset[i].bTrue = 0;
		continue;
	
		int up = res3DMap->Trackerset[i].frame2D.up;
		int down = res3DMap->Trackerset[i].frame2D.down;
		int left = res3DMap->Trackerset[i].frame2D.left;
		int right = res3DMap->Trackerset[i].frame2D.right;

		if (up < 30) up = 30;
		else if (up > (IMG_H - 30)) up = IMG_H - 30;

		if (down < 30) down = 30;
		else if (down > (IMG_H - 30)) down = IMG_H - 30;

		if (left < 30) left = 30;
		else if (left > (IMG_W - 30)) left = IMG_W - 30;

		if (right < 30) right = 30;
		else if (right > (IMG_W - 30)) right = IMG_W - 30;

		get_XY(left, up,&dummy_x,&dummy_y, &ref_x[0], &ref_y[0]);
		get_XY(left, down, &dummy_x, &dummy_y, &ref_x[1], &ref_y[1]);
		get_XY(right, up, &dummy_x, &dummy_y, &ref_x[2], &ref_y[2]);
		get_XY(right, down, &dummy_x, &dummy_y, &ref_x[3], &ref_y[3]);

		left = 2 * minof(minof(ref_x[0], ref_x[1]), minof(ref_x[2], ref_x[3]));
		right = 2 * maxof(maxof(ref_x[0], ref_x[1]), maxof(ref_x[2], ref_x[3]));
		up = 2 * minof(minof(ref_y[0], ref_y[1]), minof(ref_y[2], ref_y[3]));
		down = 2 * maxof(maxof(ref_y[0], ref_y[1]), maxof(ref_y[2], ref_y[3]));
			
		res3DMap->Trackerset[i].frame2D.left = left;
		res3DMap->Trackerset[i].frame2D.right = right;
		res3DMap->Trackerset[i].frame2D.up = up;
		res3DMap->Trackerset[i].frame2D.down = down;

		res3DMap->Trackerset[i].nColor = COLOR_ARRAY[res3DMap->Trackerset[i].frame3D.biggest_disp];

		res3DMap->Trackerset[i].rectworld.P1.X = (4 * 0.08 * (res3DMap->Trackerset[i].frame3D.left - skyline_x)) / disp_groundline;
		res3DMap->Trackerset[i].rectworld.P1.Y = (4 * 0.08 * (res3DMap->Trackerset[i].frame3D.up - skyline_y)) / disp_groundline;
		res3DMap->Trackerset[i].rectworld.P1.Z = res3DMap->Trackerset[i].dist;

		res3DMap->Trackerset[i].rectworld.P3.X = (4 * 0.08 * (res3DMap->Trackerset[i].frame3D.right - skyline_x)) / disp_groundline;
		res3DMap->Trackerset[i].rectworld.P3.Y = (4 * 0.08 * (res3DMap->Trackerset[i].frame3D.down - skyline_y)) / disp_groundline;
		res3DMap->Trackerset[i].rectworld.P3.Z = res3DMap->Trackerset[i].dist;
	}


	//画短册
	int x_ori, y_ori;
	int x, y;
	int u, v;
	int flag;
	int disp_ref = 0;
	int disp;
	short *p_disp;
	int up, down;
	int strx, stry;
	for (x_ori = 0; x_ori < 640; x_ori += 8)
	{
		get_XY(0, skyline_y, &strx, &stry, &dummy_x, &dummy_y);
		if (stry < 0 || stry > 360 - 1)
		{
			continue;
		}
		y_ori = stry;

		//get_RINV_XY(x_ori, y_ori, &x, &y);// 原始图像映射到视差图
		get_LINV_XY(x_ori, y_ori, &x, &y);// 原始图像映射到视差图
 		if (x <= 0 || y <= 0)
		{
			continue;
		}

		u = x;// 视差图的一列
		flag = 0;
		int lable_index[10] = { 0 };
		int lable_disp[10] = { 0 };
		int lable_num = 0;
		int lable_up[10] = { 0 };
		int front_up = IMG_H;

		//原doDrawBackground内容	
		for (v = 0; v < DispImg.rows; v++)
		{
			p_disp = &DispImg.data[v*DispImg.cols + u];
			if (*p_disp == 0)
				continue;

			int X = getX(*p_disp, u);
			if (X == -1)
			{
				*p_disp = 0;
				continue;
			}

			int value = COM_ARRAY[*p_disp];
			int ObtLable = lableImg.data[value*lableImg.cols + X];
			if (ObtLable == 0)
			{
				*p_disp = 0;
				continue;
			}
			else
			{
				if (lable_num == 0)
				{
					lable_index[0] = ObtLable;
					lable_disp[0] = *p_disp;
					lable_up[0] = v;
					lable_num++;
				}
				else if (ObtLable != lable_index[lable_num - 1]&&lable_num < 10)
				{
					lable_index[lable_num] = ObtLable;
					lable_disp[lable_num] = *p_disp;
					lable_up[lable_num] = v;
					lable_num++;
				}
				flag = 1;
			}
		}

		if (flag == 0)// 这一列的所有像素点没有标签
		{
			continue;
		}

		int down_max = 0;
		int max_disp = 0;
		for (i = 0; i < lable_num; i++)
		{
			if (lable_up[i] == 0)
			{
				continue;
			}
			disp = lable_disp[i];
			max_disp = max_disp>disp? max_disp:disp;

			float ratio = (float)(disp) / disp_groundline;

			int up = lable_up[i];
			int down = IMG_H * ratio + skyline_y * (1 - ratio);//计算每一个视差的接地点

			down = minof(down, IMG_H);

			for (v = up; v < down; v++)
			{
				DispImg.data[v*DispImg.cols + u] = disp;
			}

			down_max = maxof(down_max, down);

			front_up = minof(up, front_up);
		}

		for (v = down_max; v < IMG_H; v++)
		{
			disp = DispImg.data[v*DispImg.cols + u];

			if (disp != 0 && disp != 255)
			{
				DispImg.data[v*DispImg.cols + u] = 0;
			}
		}
	
		/////////////////////////////////////////
		flag = 0;
		disp_ref = 0;
		for (v = 0; v < DispImg.rows; v++)
		{
			disp = DispImg.data[v*DispImg.cols + u];
			/*if(disp!=max_disp)
				continue;*/
			
			if (flag == 0)
			{
				if (disp>0 && disp != 255 && disp == max_disp)
				{
					flag = 1;
					disp_ref = disp;
					up = v;
				}
				else
				{
					continue;
				}
			}
			else if (flag == 1)
			{
				if (disp != disp_ref)
				{
					flag = 0;
					down = v;

					//get_XY(u, up, &dummy_x, &dummy_y, &ref_x[0], &ref_y[0]);
					//get_XY(u, down, &dummy_x, &dummy_y, &ref_x[1], &ref_y[1]);
					get_XY(u, up, &ref_x[0], &ref_y[0], &dummy_x, &dummy_y);
					get_XY(u, down, &ref_x[1], &ref_y[1], &dummy_x, &dummy_y);

					if (ref_y[1] - ref_y[0] > 10)
					{
						drawFrences->frences[drawFrences->nFenceNum].frame2D.left = 2 * x_ori;
						drawFrences->frences[drawFrences->nFenceNum].frame2D.right = 2 * x_ori + 8;
						drawFrences->frences[drawFrences->nFenceNum].frame2D.up = 2 * ref_y[0];
						drawFrences->frences[drawFrences->nFenceNum].frame2D.down = 2 * ref_y[1];
						drawFrences->frences[drawFrences->nFenceNum].frame2D.fttc = disp_ref;
						drawFrences->frences[drawFrences->nFenceNum].nColor = COLOR_ARRAY[disp_ref];
						drawFrences->nFenceNum++;
					}
					

					if (disp > 0&&disp!=255)
					{
						flag = 1;
						disp_ref = disp;
						up = v;
					}
				}
				
			}

			if (v == DispImg.rows - 1&&flag == 1)
			{
				flag = 0;
				down = v;

				//get_XY(u, up, &dummy_x, &dummy_y, &ref_x[0], &ref_y[0]);
                //get_XY(u, down, &dummy_x, &dummy_y, &ref_x[1], &ref_y[1]);
				get_XY(u, up, &ref_x[0], &ref_y[0], &dummy_x, &dummy_y);
				get_XY(u, down, &ref_x[1], &ref_y[1], &dummy_x, &dummy_y);


				if (ref_y[1] - ref_y[0] > 10)
				{
					drawFrences->frences[drawFrences->nFenceNum].frame2D.left = 2 * x_ori;
					drawFrences->frences[drawFrences->nFenceNum].frame2D.right = 2 * x_ori + 8;
					drawFrences->frences[drawFrences->nFenceNum].frame2D.up = 2 * ref_y[0];
					drawFrences->frences[drawFrences->nFenceNum].frame2D.down = 2 * ref_y[1]; 
					drawFrences->frences[drawFrences->nFenceNum].frame2D.fttc = disp_ref;
					drawFrences->frences[drawFrences->nFenceNum].nColor = COLOR_ARRAY[disp_ref];
					drawFrences->nFenceNum++;
				}
			}
		}
	}

//    printf("fence num: %d, %f\n", drawFrences->nFenceNum, fSpeed);
	float32_t tempDis = 0;
	for (int k = 0; k < drawFrences->nFenceNum; k++)
	{
		if (drawFrences->frences[k].frame2D.down > ptVanish.y )
		{
            float32_t org = ptOrg.x + slpOrg * (drawFrences->frences[k].frame2D.down - ptOrg.y);
            float32_t tel = ptTel.x + slpTel * (drawFrences->frences[k].frame2D.down - ptTel.y);
            if(fSpeed > 0.0f)
            {
                float32_t distence = DISP2DIST/drawFrences->frences[k].frame2D.fttc;
                float32_t dttc = distence / fSpeed; 

 //                   my_printf("3DMAP org: %f, tel: %f,right: %d,distence: %d, dttc: %f\n", org, tel,drawFrences->frences[k].frame2D.right,distence,dttc);
                if (drawFrences->frences[k].frame2D.right > org && drawFrences->frences[k].frame2D.right < tel && ((distence < 6)||(dttc < 0.8f))) 
                {
                    my_printf("3DMAP dis: %f, dttc: %f \n", distence, dttc);

					tempDis += distence;
                    drawFrences->nWarningNum++;
                }

            }
        }
    }

	if (drawFrences->nWarningNum > 0)
	{
		tempDis /= drawFrences->nWarningNum;
		for (int x = 1; x < MAX_MOTION_NUM; x++)
			 avrDis[x - 1] = avrDis[x];
		avrDis[MAX_MOTION_NUM - 1] = tempDis;

		if (avrDis[MAX_MOTION_NUM - 1] >= avrDis[0] || avrDis[MAX_MOTION_NUM - 1] > 4.0f)
		{
			drawFrences->nWarningNum = 0;
		}
	}
	
}

void Stereo_detect_release()
{
	doReleaseMatShort(&DispImg);
	doReleaseMatShort(&DXImg);
	doReleaseMatShort(&lableImg);
	doReleaseMatUchar(&VMask);
	doReleaseMatUchar(&GrayImg);
	doReleaseMatColor(&ColorImg);
}


//void doCalcDistImg(const Mat_short src, Mat_uchar *dst, const Mat_uchar vmask, const Mat_uchar umask)
//{
//	int v, u;
//	short *src_p = src.data;
//	uchar *dst_p = dst->data;
//	for (v = 0; v < src.rows; v++)
//	{
//		for (u = 0; u < src.cols; u++)
//		{
//			int value = *src_p;
//			int dist = FOCUS_DIST / (value * 4);
//			if (vmask.data[v*vmask.cols + value] == 3)//&&umask.data[(value>>2)*umask.cols + u]>0)
//				*dst_p = dist;
//
//			src_p++;
//			dst_p++;
//		}
//	}
//}

