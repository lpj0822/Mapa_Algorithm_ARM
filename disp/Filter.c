#include "3Dmap.h"

#define DIFF_DISP_LIMIT (8)
#define DIFF_SPACE_LIMIT (50.0)

extern int skyline_x;
extern int skyline_y;
extern float disp_groundline;

void doMergeCloseObjs(Object objs[])
{
	int i, j;
	for (i = 0; i < 256; i++)
	{
		if (objs[i].cnt == 0)
		{
			continue;
		}

		for (j = 0; j < 256; j++)
		{
			if (objs[j].cnt == 0 || i == j)
			{
				continue;
			}

			int diff_disp = objs[i].biggest_disp - objs[j].biggest_disp;

			if (diff_disp > DIFF_DISP_LIMIT || diff_disp < -DIFF_DISP_LIMIT)
			{
				continue;
			}

			float diff_space = objs[j].proj_left - objs[i].proj_right;
			
			float middle = (objs[j].proj_left + objs[i].proj_right)*0.5;

			if (middle>150 && middle<IMG_W-150)
			{
				if (diff_space > DIFF_SPACE_LIMIT * 2 || diff_space < -DIFF_SPACE_LIMIT * 2)
				{
					continue;
				}
			}
			else if (middle>0 && middle<IMG_W)
			{
				if (diff_space > DIFF_SPACE_LIMIT * 1.5 || diff_space < -DIFF_SPACE_LIMIT * 1.5)
				{
					continue;
				}
			}
			else
			{
				if (diff_space > DIFF_SPACE_LIMIT || diff_space < -DIFF_SPACE_LIMIT)
				{
					continue;
				}
			}

			//通过筛选，进行融合
			objs[i].size_up = minof(objs[i].size_up, objs[j].size_up);
			objs[i].size_left = minof(objs[i].size_left, objs[j].size_left);
			objs[i].size_down = maxof(objs[i].size_down, objs[j].size_down);
			objs[i].size_right = maxof(objs[i].size_right, objs[j].size_right);

			objs[i].proj_up = minof(objs[i].proj_up, objs[j].proj_up);
			objs[i].proj_left = minof(objs[i].proj_left, objs[j].proj_left);
			objs[i].proj_down = maxof(objs[i].proj_down, objs[j].proj_down);
			objs[i].proj_right = maxof(objs[i].proj_right, objs[j].proj_right);

			objs[i].proj_size = objs[i].proj_size + objs[j].proj_size;

			objs[i].smallest_disp = minof(objs[i].smallest_disp, objs[j].smallest_disp);
			objs[i].biggest_disp = maxof(objs[i].biggest_disp, objs[j].biggest_disp);
			
			objs[i].dist = minof(objs[i].dist, objs[j].dist);
			
			objs[i].cnt = objs[i].cnt + objs[j].cnt;
			objs[j].cnt = 0;
					
		}
	}
}

void doRemoveOcclusedObjs(Object objs[])
{
	int i, j;
	for(i = 0; i < 256; i++)
	{
		if(objs[i].cnt == 0)
			continue;

		for(j = 0; j < 256; j++)
		{
			if(objs[i].cnt == 0 || i == j)
			{
				continue;
			}

			if (objs[i].size_left > objs[j].size_right) { continue; }
			if (objs[i].size_up > objs[j].size_down) { continue; }
			if (objs[i].size_right < objs[j].size_left) { continue; }
			if (objs[i].size_down < objs[j].size_up) { continue; }
			float colInt = minof(objs[i].size_right, objs[j].size_right)
				- maxof(objs[i].size_left, objs[j].size_left);
			float rowInt = minof(objs[i].size_down, objs[j].size_down)
				- maxof(objs[i].size_up, objs[j].size_up);
			float intersection = colInt * rowInt;
			float area = (objs[j].size_down - objs[j].size_up)*(objs[j].size_right - objs[j].size_left);

			if ((intersection / area > 0.7) && (objs[j].biggest_disp < objs[i].biggest_disp))
			{
				objs[j].cnt = 0;
			}
		}
	}

}

void doSelectBackground(Object objs[])
{
	int i;
	for (i = 0; i < 256; i++)
	{
		if (objs[i].cnt == 0)
		{
			continue;
		}

		/*if (objs[i].proj_left > 376 || objs[i].proj_right<100)
		{
			objs[i].isBackground = 1;
		}

		if (objs[i].proj_right - objs[i].proj_left > 850)
		{
			objs[i].isBackground = 1;
		}*/
		//所有的物体都归为背景
		objs[i].isBackground = 1;
	}
}

void doDrawBackground(Object objs[], Mat_short LableImg, Mat_short DispImg)
{
	int u, v;
	for (v = 0; v < LableImg.rows; v++)
	{
		for (u = 0; u < LableImg.cols; u++)
		{
			int lable = LableImg.data[v*LableImg.cols + u];

			if (lable == 0)
			{
				continue;
			}

			if (objs[lable].isBackground == 0)
			{
				LableImg.data[v*LableImg.cols + u] = 0;
			}
		}
	}

	//short *p_disp = DispImg.data;
	//for (v = 0; v < DispImg.rows; v++)
	//{
	//	for (u = 0; u < DispImg.cols; u++, p_disp++)
	//	{
	//		if (*p_disp == 0)
	//			continue;

	//		if (*p_disp == 255)
	//		{
	//			*p_disp = 0;
	//			continue;
	//		}

	//		int X = getX(*p_disp, u);
	//		if (X == -1)
	//		{
	//			*p_disp = 0;
	//			continue;
	//		}

	//		int value = COM_ARRAY[*p_disp];
	//		int ObtLable = LableImg.data[value*LableImg.cols + X];
	//		if (ObtLable == 0)
	//		{
	//			*p_disp = 0;
	//			continue;
	//		}
	//	}
	//}

	int i;
	for (i = 0; i < 256; i++)
	{
		if (objs[i].cnt != 0 && objs[i].isBackground == 0)
		{
			for (v = objs[i].size_up; v < objs[i].size_down; v++)
			{
				for (u = objs[i].size_left; u < objs[i].size_right; u++)
				{
					DispImg.data[v*DispImg.cols + u] = 255;
				}
			}
			
		}
	}

}

void doDrawFrences(Object objs[], Mat_short LableImg, Mat_short DispImg)
{
	/*int v, u,disp;
	int i;
	for (u = 0; u < DispImg.cols; u++)
	{
		int lable_index[10] = { 0 };
		int lable_disp[10] = { 0 };
		int lable_num = 0;
		int lable_up[10] = { 0 };
		int front_up = IMG_H;
		
		for (disp = 255; disp>0; disp--)
		{
			int X = getX(disp, u);
			if (X == -1)
			{
				continue;
			}

			int value = COM_ARRAY[disp];
			int ObtLable = LableImg.data[value*LableImg.cols + X];

			if (ObtLable > 0)
			{
				if (lable_num == 0)
				{
					lable_index[0] = ObtLable;
					lable_disp[0] = disp;
					lable_num++;
				}
				else if (ObtLable != lable_index[lable_num - 1]&&lable_num < 10)
				{
					lable_index[lable_num] = ObtLable;
					lable_disp[lable_num] = disp;
					lable_num++;
				}
			}
		}

		if (lable_num == 0)
		{
			continue;
		}

		for (v = 0; v < DispImg.rows; v++)
		{
			disp = DispImg.data[v*DispImg.cols + u];

			if (front_up == IMG_H && disp == 255)
			{
				front_up = v;
			}
			
			int X = getX(disp, u);
			if (X == -1)
			{
				continue;
			}

			int value = COM_ARRAY[disp];
			int ObtLable = LableImg.data[value*LableImg.cols + X];

			for (i = 0; i < lable_num; i++)
			{
				if (ObtLable == lable_index[i]&&lable_up[i]==0)
				{
					lable_up[i] = v;
				}
			}
		}

		int down_max = 0;
		for (i = 0; i < lable_num; i++)
		{
			if (lable_up[i] == 0)
			{
				continue;
			}
			disp = lable_disp[i];
			float ratio = (float)(disp) / PROJ_DISP;
			int up = lable_up[i];
			int down = IMG_H * ratio + SKYLINE_Y * (1 - ratio);
			down = minof(down, front_up);

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
	}*/
}

#if Is_PC
void doFilterObstacle(const Mat_short LableImg, const Mat_short DispImg, Mat_color OriImg, Mat& OriImg_forshow)
#else
void doFilterObstacle(const Mat_short LableImg, const Mat_short DispImg, Mat_color OriImg)
#endif
{
	int v, u, i, j, k;
	Object objs[256];
	//	uchar objs_redirect[256] = { 0 };
	//	uchar objs_index = 128;

	memset(objs, 0, sizeof(Object)* 256);


	//	double t = (double)getTickCount();//计时用函数
	short *p_disp = DispImg.data;
	for (v = 0; v < DispImg.rows; v++)
	{
		for (u = 0; u < DispImg.cols; u++, p_disp++)
		{
			if (*p_disp == 0)
				continue;


			//DX相关修改
			//	int X = UX_SCALE * (u - DispImg.cols / 2.0) / (float)(*p_disp) + ObtImg.cols / 2.0;
			int X = getX(*p_disp, u);
			if (X == -1)
				continue;

			int value = COM_ARRAY[*p_disp];
			int ObtLable = LableImg.data[value*LableImg.cols + X];
			if (ObtLable == 0)
				continue;

			//forshow		
			float proj_ratio = disp_groundline / (float)(*p_disp);
			float proj_u = u * proj_ratio + skyline_x * (1 - proj_ratio);
			float proj_v = v * proj_ratio + skyline_y * (1 - proj_ratio);


			//while (objs_redirect[ObtLable] > 0) //顺次找到最新的标签值
			//{
			//	ObtLable = objs_redirect[ObtLable];
			//}

			Object *p_obj = &objs[ObtLable];//获新新标签值指向的物体

			//如果此点与现有物体发生明显偏离，则另外定义一个新的标签值
			//if (v > (p_obj->size_down + V_COM_INDEX) && p_obj->cnt > 0)
			//{
			//	objs_redirect[ObtLable] = objs_index; //指向新的标签值
			//	ObtLable = objs_index;
			//	p_obj = &objs[ObtLable]; //指向新的物体
			//	objs_index++;//下次重指向时使用的index增加
			//}



			value = DispImg.data[v*DispImg.cols + u];
			//v_revised = v; //testcode

			/*if (v_revised > 187)
			v_revised = 187;
			else if (v_revised < 0)
			v_revised = 0;*/

			//在3D空间中计算物体的直方图分布
			int index = (proj_u - MIN_X) / STEP_X;
			if (index < 0)
				index = 0;
			else if (index > HISTOSIZE_X - 1)
				index = HISTOSIZE_X - 1;
			p_obj->histo_X[index]++;

			index = (proj_v - MIN_Y) / STEP_Y;
			if (index < 0)
				index = 0;
			else if (index > HISTOSIZE_Y - 1)
				index = HISTOSIZE_Y - 1;
			p_obj->histo_Y[index]++;

			index = *p_disp;
			if (index > HISTOSIZE_Z - 1)
				index = HISTOSIZE_Z - 1;
			p_obj->histo_Z[index] ++;

			//计算物体在图像上的空间范围以及距离
			if (p_obj->cnt == 0)
			{
				p_obj->size_right = u;
				p_obj->size_left = u;
				p_obj->size_down = v;
				p_obj->size_up = v;
				p_obj->biggest_disp = *p_disp;

				//forshow
				p_obj->smallest_disp = *p_disp; //forshow

				p_obj->proj_right = proj_u;
				p_obj->proj_left = proj_u;
				p_obj->proj_down = proj_v;
				p_obj->proj_up = proj_v;
			}
			else
			{
				if (u > p_obj->size_right)
				{
					p_obj->size_right = u;
				}

				if (u < p_obj->size_left)
				{
					p_obj->size_left = u;
				}

				if (v > p_obj->size_down)
				{
					p_obj->size_down = v;
				}

				/*if (v < p_obj->size_up)
				{
				p_obj->size_up = v;
				}*/

				if (*p_disp > p_obj->biggest_disp)
				{
					p_obj->biggest_disp = *p_disp;
				}

				//for show
				if (*p_disp < p_obj->smallest_disp)
				{
					p_obj->smallest_disp = *p_disp;
				}

				if (proj_u > p_obj->proj_right)
				{
					p_obj->proj_right = proj_u;
				}

				if (proj_u < p_obj->proj_left)
				{
					p_obj->proj_left = proj_u;
				}

				if (proj_v > p_obj->proj_down)
				{
					p_obj->proj_down = proj_v;
				}

				if (proj_v < p_obj->proj_up)
				{
					p_obj->proj_up = proj_v;
				}



			}

			p_obj->cnt++; //计数


			//	OriImg.at<Vec3b>(v * 2, u * 2) = colors2[ObtLable % 12];;
		}
	}


	//	cout << (((double)getTickCount() - t) / getTickFrequency()) * 1000 << endl;

	//for (int i = 0; i < 256; i++) //跟据各个物体的大小尺寸进行过滤
	//{
	//	if (objs[i].cnt != 0)
	//	{
	//		int size = objs[i].size_down - objs[i].size_up;
	//		if (objs[i].most_close < LEVEL_1)
	//		{
	//			if (size < 20 || objs[i].cnt<30)
	//				objs[i].cnt = 0;
	//		}
	//		else if (objs[i].Dist < LEVEL_2)
	//		{
	//			if (size < 20 || objs[i].cnt<30)
	//				objs[i].cnt = 0;
	//		}
	//		else if (objs[i].Dist < LEVEL_3)
	//		{
	//			if (size < 10 || objs[i].cnt<15)
	//				objs[i].cnt = 0;
	//		}
	//		else
	//		{
	//			if (size < 5 || objs[i].cnt<10)
	//				objs[i].cnt = 0;
	//		}
	//	}
	//}


	//对物体的大小，分离与否等条件进行过滤


	//	imshow("Monitor02", ObtImg);
	doHistoResize(objs, DispImg.rows);
	doMergeCloseObjs(objs);
	doRemoveOcclusedObjs(objs);


	doTrackObstacle(objs);

	doSelectBackground(objs);

	//doDrawBackground(objs, LableImg, DispImg);

	doDrawFrences(objs, LableImg, DispImg);

	//


#if Is_PC

	doDrawObjects(objs, LableImg, DispImg, OriImg_forshow);
	imshow("OriImg_forshow", OriImg_forshow);

	//	static VideoWriter dispwriter("Result_Show_0523.avi", -1, 3.0, Size(760, 340));

	//	dispwriter << OriImg_forshow;

	Mat DispImg_forshow(IMG_H, IMG_W, CV_8UC1);

	for ( v = 0; v < IMG_H; v++)
	{
		for ( u = 0; u < IMG_W; u++)
		{
			int value = DispImg.data[v*IMG_W + u];
			DispImg_forshow.at<uchar>(v, u) = value;
		}
	}

	//imshow("DispImg", DispImg_forshow);


#endif

}

