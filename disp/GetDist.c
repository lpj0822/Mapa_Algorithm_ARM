#include "3Dmap.h"

extern int get_RINV_XY(int in_ROI_X,int in_ROI_Y,int *out_x,int *out_y);
extern int get_XY(int in_x, int in_y, int *out_Lx, int *out_Ly, int *out_Rx, int *out_Ry);

//void getLUT(int x, int y, int *ref_x, int *ref_y)
//{
//	*ref_x = x;
//	*ref_y = y;
//}

void getBoundRect(int x, int y, int w, int h, int *up, int *down, int *left, int *right)
{
	int ref_x[4], ref_y[4];
	get_RINV_XY(x, y, &ref_x[0], &ref_y[0]);
	get_RINV_XY(x, y + h, &ref_x[1], &ref_y[1]);
	get_RINV_XY(x + w, y, &ref_x[2], &ref_y[2]);
	get_RINV_XY(x + w, y + h, &ref_x[3], &ref_y[3]);

	*up = minof(minof(ref_y[0], ref_y[1]), minof(ref_y[2], ref_y[3]));
	*down = maxof(maxof(ref_y[0], ref_y[1]), maxof(ref_y[2], ref_y[3]));
	*left = minof(minof(ref_x[0], ref_x[1]), minof(ref_x[2], ref_x[3]));
	*right = maxof(maxof(ref_x[0], ref_x[1]), maxof(ref_x[2], ref_x[3]));
}

void getDistImg(Mat_short *DispImg)
{
	static Mat_short Saved_Img = { NULL, 0, 0 };

	if (DispImg->data == NULL)
	{
		doInitialMatShort(DispImg, Saved_Img.rows, Saved_Img.cols);//初始化视差图结构体
		memcpy(DispImg->data, Saved_Img.data, Saved_Img.rows*Saved_Img.cols*sizeof(short));//复制Mat的内容。
	}
	else
	{
		doInitialMatShort(&Saved_Img, DispImg->rows, DispImg->cols);//初始化视差图结构体
		memcpy(Saved_Img.data, DispImg->data, DispImg->rows*DispImg->cols*sizeof(short));//复制Mat的内容。
	}
}

float getdist(int x, int y, int w, int h)
{
	int up, down, left, right;

	getBoundRect(x, y, w, h, &up, &down, &left, &right);

	Mat_short DispImg = { NULL, 0, 0 };
	getDistImg(&DispImg);

	//如果映射的区域越出原图，则进行调整
	if (up < 0) up = 0;
	if (down > DispImg.rows - 1) down = DispImg.rows - 1;
	if (left < 0) left = 0;
	if (right > DispImg.cols - 1) right = DispImg.cols - 1;

	int v, u;
	short disp_histo[256];
	int cnt = 0;

	memset(disp_histo, 0, sizeof(short)* 256);

	for (v = up; v <= down; v++)
	{
		for (u = left; u <= right; u++)
		{
			int value = DispImg.data[v*DispImg.cols + u];

			if (value == 0)
				continue;

			if (value > 255)
				value = 255;

			disp_histo[value]++;
			cnt++;
		}
	}

	cnt = cnt / 5;
	int i;
	int sum = 0;
	for (i = 255; i >= 0; i--)
	{
		sum += disp_histo[i];

		if (sum > cnt)
			return i;
	}

	return 0;
}
