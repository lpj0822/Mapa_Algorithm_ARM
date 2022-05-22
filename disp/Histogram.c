#include "3Dmap.h"

extern int skyline_x;
extern int skyline_y;
extern float disp_groundline;

void getHistoBorder(short histo[], int histosize, short index[], const short cnt)
{
	short sum = 0;

	for (index[0] = 0; index[0] < histosize; index[0]++)
	{
		sum += histo[index[0]];
		if (sum >= cnt)
			break;
	}

	sum = 0;
	for (index[1] = histosize - 1; index[1] >= 0; index[1]--)
	{
		sum += histo[index[1]];
		if (sum >= cnt)
			break;
	}
}

void doHistoResize(Object cur_objs[], int Img_Height)
{
	int i, u, v;

	//	float size_list[256] = { 0 };
	for (i = 1; i < 256; i++)//计算各个物体的真实大小
	{
		if (cur_objs[i].cnt == 0)
		{
			cur_objs[i].proj_size = 0;
		}
		else
		{
			cur_objs[i].proj_size = (cur_objs[i].proj_down - cur_objs[i].proj_up) * (cur_objs[i].proj_right - cur_objs[i].proj_left) * 0.001;
		}

		//		size_list[i] = cur_objs[i].proj_size;
	}

	//{
	//	int j;
	//	//对物体大小进行排序
	//	for (i = 1; i < 256; i++)
	//	{
	//		for (j = 1; j < 256 - i; j++)
	//		{
	//			if (size_list[j] < size_list[j + 1])
	//			{
	//				float tempvalue = size_list[j];
	//				size_list[j] = size_list[j + 1];
	//				size_list[j + 1] = tempvalue;
	//			}
	//		}
	//	}
	//}

	for (i = 0; i < 256; i++)
	{
		if (cur_objs[i].cnt == 0)
			continue;


		if ((cur_objs[i].biggest_disp < 20 && cur_objs[i].cnt < 50) || (cur_objs[i].biggest_disp >= 20 && cur_objs[i].cnt < 100))
		{
			cur_objs[i].cnt = 0;
			continue;
		}

		if (cur_objs[i].biggest_disp < 20)
		{
			if (cur_objs[i].cnt < 50)
			{
				cur_objs[i].cnt = 0;
				continue;
			}
		}
		else if (cur_objs[i].biggest_disp < 40)
		{
			if (cur_objs[i].cnt < 100)
			{
				cur_objs[i].cnt = 0;
				continue;
			}
		}
		else
		{
			if (cur_objs[i].cnt < 150)
			{
				cur_objs[i].cnt = 0;
				continue;
			}
		}


		//histogram操作

		short index[2];
		short dis_cnt = cur_objs[i].cnt / 20 + 1;

		getHistoBorder(cur_objs[i].histo_X, HISTOSIZE_X, index, dis_cnt);
		cur_objs[i].proj_left = MIN_X + index[0] * STEP_X;
		cur_objs[i].proj_right = MIN_X + index[1] * STEP_X;

		getHistoBorder(cur_objs[i].histo_Y, HISTOSIZE_Y, index, dis_cnt);
		cur_objs[i].proj_up = MIN_Y + index[0] * STEP_Y;
		cur_objs[i].proj_down = MIN_Y + index[1] * STEP_Y;

		getHistoBorder(cur_objs[i].histo_Z, HISTOSIZE_Z, index, dis_cnt);
		cur_objs[i].smallest_disp = index[0];
		cur_objs[i].biggest_disp = index[1];

		cur_objs[i].dist = DISP2DIST / (float)cur_objs[i].biggest_disp;

		//如果物体实际高度低（扁平的物体）有可能是地面的误检予以删除
		if ((cur_objs[i].proj_down - cur_objs[i].proj_up) < HEIGHT_THD && (cur_objs[i].biggest_disp < 100))
		{
			cur_objs[i].cnt = 0;
			continue;
		}
		else if ((cur_objs[i].proj_down - cur_objs[i].proj_up) < HEIGHT_THD* 0.5)
		{
			cur_objs[i].cnt = 0;
			continue;
		}

		//将上下左右再外扩百分之8保证包含整个物体。
		float center = (cur_objs[i].proj_left + cur_objs[i].proj_right) / 2.0;
		cur_objs[i].proj_left += (cur_objs[i].proj_left - center) * 0.1;
		cur_objs[i].proj_right += (cur_objs[i].proj_right - center) * 0.1;

		center = (cur_objs[i].proj_up + cur_objs[i].proj_down) / 2.0;
		cur_objs[i].proj_up += (cur_objs[i].proj_up - center) * 0.2;
		//cur_objs[i].proj_down += (cur_objs[i].proj_down - center) * 0.08;

		if (/*(cur_objs[i].proj_size > OBJ_SIZE_FILTER) && (/*(cur_objs[i].proj_size >= size_list[OBJ_AMUNT]) && */(cur_objs[i].size_down - cur_objs[i].size_up) > OBJ_HEIGHT_FILTER)
		{

			//2016.4.22 计算每个物体在框中的占空比以及空间长度
			float occupy_ratio = (float)cur_objs[i].cnt / ((cur_objs[i].size_down - cur_objs[i].size_up)*(cur_objs[i].size_right - cur_objs[i].size_left));
			float length = DISP2DIST / (float)cur_objs[i].smallest_disp - DISP2DIST / (float)cur_objs[i].biggest_disp;
			float width = cur_objs[i].proj_right - cur_objs[i].proj_left;
			float height = Img_Height - cur_objs[i].proj_up;

			
			if (occupy_ratio < 0.1)
			{
				cur_objs[i].cnt = 0;
				continue;
			}
		}
		else
		{
			cur_objs[i].cnt = 0;
			continue;
		}

		float close_ratio1 = (float)cur_objs[i].smallest_disp / disp_groundline;
		int up1 = cur_objs[i].proj_up * close_ratio1 + skyline_y * (1 - close_ratio1);
		int down1 = maxof_float(IMG_H, cur_objs[i].proj_down) * close_ratio1 + skyline_y * (1 - close_ratio1);
		int left1 = cur_objs[i].proj_left *close_ratio1 + skyline_x * (1 - close_ratio1);
		int right1 = cur_objs[i].proj_right * close_ratio1 + skyline_x * (1 - close_ratio1);

		float close_ratio2 = (float)(cur_objs[i].biggest_disp) / disp_groundline;
		int up2 = cur_objs[i].proj_up * close_ratio2 + skyline_y * (1 - close_ratio2);
		int down2 = maxof_float(IMG_H, cur_objs[i].proj_down) * close_ratio2 + skyline_y * (1 - close_ratio2);
		int left2 = cur_objs[i].proj_left * close_ratio2 + skyline_x * (1 - close_ratio2);
		int right2 = cur_objs[i].proj_right * close_ratio2 + skyline_x * (1 - close_ratio2);

		cur_objs[i].size_up = minof(up1, up2);
		cur_objs[i].size_down = maxof(down1, down2);
		cur_objs[i].size_left = minof(left1, left2);
		cur_objs[i].size_right = maxof(right1, right2);
	}

}
