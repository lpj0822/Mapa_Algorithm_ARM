#include "3Dmap.h"

#define MAX_OBJECT 5000



void doFindConection(const Mat_uchar src, Mat_short* lableImg)
{
	int v, u, i, j, k;

	short lable_sheet[MAX_OBJECT] = { 0 };
	uchar lable_flag[MAX_OBJECT] = { 0 };

	int lableNo = 1;
	int cnt = 0;
	int lable_temp = 0;
	int lable1 = 0;
	int lable2 = 0;
	short *p;
	for (v = 1; v < src.rows - 1; v++) //第一次遍历
	{
		for (u = 1; u < src.cols - 1; u++)
		{
			if (src.data[v*src.cols + u] == 0)//如果此点不为0，则对其邻域已被标注的点进行分析
			{
				continue;
			}

			p = lableImg->data + v*lableImg->cols + u;

			lable1 = *(p - lableImg->cols); //第一个邻域
			lable2 = *(p - 1); //第二个邻域


			if (lable1 == 0 && lable2 == 0) //没有找到已被标注的邻域,则标注一个新的编号,退出循环
			{
				*p = lableNo;
				lableNo++;
				continue;
			}

			//如果仅有一个领域,保存为该邻域并退出
			if (lable1 == 0)
			{
				*p = lable2;
				continue;
			}
			if (lable2 == 0)
			{
				*p = lable1;
				continue;
			}

			//如果两个领域相同，保存为该邻域并退出
			if (lable1 == lable2)
			{
				*p = lable1;
				continue;
			}

			//如果存在两个不同邻域
			*p = minof(lable1, lable2);//先将窗口中心标注为领域编号中较小的那个


			//如果存在两种不同的领域编号，则对lablesheet进行处理
			//分别计算各个lable可以指向的最小lable
			while (lable_sheet[lable1] != 0)
			{
				lable1 = lable_sheet[lable1];
			}
			while (lable_sheet[lable2] != 0)
			{
				lable2 = lable_sheet[lable2];
			}

			if (lable1 > lable2)//如果最小lable相同则不作处理
			{
				lable_sheet[lable1] = lable2;
			}
			else if (lable1 < lable2)
			{
				lable_sheet[lable2] = lable1;
			}

		}
	}




	//	Mat lableImg_forshow(lableImg.rows, lableImg.cols, CV_8UC1);
	//
	//	for (int v = 0; v < lableImg.rows; v++)
	//	{
	//	for (int u = 0; u < lableImg.cols; u++)
	//	{
	//
	//	lableImg_forshow.at<uchar>(v, u) = lableImg.data[v*lableImg.cols + u];
	//
	//	}
	//	}
	//
	//	imshow("Win03", lableImg_forshow);
	//	waitKey(0);
	//double t = (double)getTickCount();//计时用函数

	p = lableImg->data + lableImg->cols;
	for (v = 1; v < src.rows - 1; v++) //第二次遍历
	{
		for (u = 0; u < src.cols; u++, p++)
		{
			if (*p == 0)
			{
				continue;
			}

			lable_temp = *p;
			int smallest = lable_temp;

			if (lable_sheet[smallest] == 0)
			{
				if (lable_flag[smallest] == 0)
				{
					lable_flag[smallest] = 1;
				}
				continue;
			}

			while (lable_sheet[smallest] != 0)
			{
				smallest = lable_sheet[smallest];
			}


			lable_sheet[lable_temp] = smallest;
			*p = smallest; //lableImg上的对应值被赋予最小lable值

			if (lable_flag[smallest] == 0)
			{
				lable_flag[smallest] = 1;
			}

		}
	}




	memset(lable_sheet, 0, sizeof(short)* MAX_OBJECT);
	for (i = 1, j = 1; i < MAX_OBJECT; i++)
	{
		if (lable_flag[i])
		{
			lable_sheet[i] = j;
			j++;
		}
	}

	for (v = 1; v < src.rows - 1; v++) //第三次遍历，压缩序号列表
	{
		for (u = 1; u < src.cols - 1; u++)
		{
			int value = lableImg->data[v*lableImg->cols + u];
			if (value == 0)
			{
				continue;
			}

			if (lable_sheet[value] != value)
			{
				lableImg->data[v*lableImg->cols + u] = lable_sheet[value];
			}
		}
	}
	//cout << (((double)getTickCount() - t) / getTickFrequency()) * 1000 << endl;

}

void doExtendConection(const Mat_uchar src, Mat_short *lableImg)
{
	int v, u, i, j, r;

	short lable_sheet[MAX_OBJECT] = { 0 };
	uchar lable_flag[MAX_OBJECT] = { 0 };


	//对每个点的右方作水平延伸，看是否可以连接上其他色块。
	short *p = lableImg->data + lableImg->cols;
	for (v = 1; v < lableImg->rows; v++)
	{
		for (u = 0; u < lableImg->cols; u++, p++)
		{
			if (u == lableImg->cols - 1)
				continue;

			int value = *p;
			int value_sft = *(p + 1);

			if (value == 0 || value_sft > 0)
				continue;


			int max_sft;
			/*if (v < LEVEL_1)
			max_sft = 5;
			else*/ if (v < LEVEL_2)
				max_sft = 5;
			else if (v < LEVEL_3)
				max_sft = 4;
			else
				max_sft = 3;

			int max_extend = 0; //最终的延伸量
			int smallest_value = value;
			for (i = 1; i <= max_sft && i < src.cols - u; i++)
			{

				if (src.data[v*src.cols + u + i] == 0) //如果路径上有在原图上为空的值，则不对其进行连接
					break;

				value_sft = *(p + i - src.cols);

				if (value_sft > 0 && value_sft != value)
				{
					max_extend = i;

					while (lable_sheet[smallest_value] != 0)
					{
						smallest_value = lable_sheet[smallest_value];
					}

					while (lable_sheet[value_sft] != 0)
					{
						value_sft = lable_sheet[value_sft];
					}

					if (smallest_value > value_sft)
					{
						lable_sheet[smallest_value] = value_sft;
					}
					else if (smallest_value > value_sft)
					{
						lable_sheet[value_sft] = smallest_value;
					}
				}

				value_sft = *(p + i + src.cols);
				if (value_sft > 0 && value_sft != value)
				{
					max_extend = i;

					while (lable_sheet[smallest_value] != 0)
					{
						smallest_value = lable_sheet[smallest_value];
					}

					while (lable_sheet[value_sft] != 0)
					{
						value_sft = lable_sheet[value_sft];
					}

					if (smallest_value > value_sft)
					{
						lable_sheet[smallest_value] = value_sft;
					}
					else if (smallest_value > value_sft)
					{
						lable_sheet[value_sft] = smallest_value;
					}
				}



				value_sft = *(p + i);
				if (value_sft > 0 && value_sft != value)
				{
					max_extend = i - 1;

					while (lable_sheet[smallest_value] != 0)
					{
						smallest_value = lable_sheet[smallest_value];
					}

					while (lable_sheet[value_sft] != 0)
					{
						value_sft = lable_sheet[value_sft];
					}

					if (smallest_value > value_sft)
					{
						lable_sheet[smallest_value] = value_sft;
					}
					else if (smallest_value > value_sft)
					{
						lable_sheet[value_sft] = smallest_value;
					}

					break;
				}
			}

			if (max_extend > 0)
			{
				for (i = 1; i <= max_extend; i++)
				{
					*(p + i) = value;
				}
			}


		}
	}

	for (v = 1; v < src.rows - 1; v++) //第二次遍历
	{
		for (u = 1; u < src.cols - 1; u++)
		{
			int value = lableImg->data[v*lableImg->cols + u];

			if (value == 0)
			{
				continue;
			}

			int smallestlable = value;
			while (lable_sheet[smallestlable] != 0)
			{
				smallestlable = lable_sheet[smallestlable];
			}

			if (smallestlable != value) //指向找到的最小lable
			{
				if (lable_sheet[value] != smallestlable)
				{
					lable_sheet[value] = smallestlable;
				}
				lableImg->data[v*lableImg->cols + u] = smallestlable; //lableImg上的对应值被赋予最小lable值
			}

			if (lable_flag[smallestlable] == 0)
			{
				lable_flag[smallestlable] = 1;
			}
		}
	}

	//去掉不存在的序号
	memset(lable_sheet, 0, sizeof(short)* MAX_OBJECT);
	for (i = 1, j = 1; i < MAX_OBJECT; i++)
	{
		if (lable_flag[i])
		{
			lable_sheet[i] = j;
			j++;
		}
	}

	for (v = 1; v < src.rows - 1; v++) //第三次遍历，压缩序号列表
	{
		for (u = 1; u < src.cols - 1; u++)
		{
			int value = lableImg->data[v*lableImg->cols + u];
			if (value == 0)
			{
				continue;
			}

			if (lable_sheet[value] != value)
			{
				lableImg->data[v*lableImg->cols + u] = lable_sheet[value];
			}
		}
	}

	//以下代码为显示用
	//	Mat lableImg_forshow(src.size(), CV_8UC1, Scalar(0));
	//	for (int v = 0; v < lableImg.rows; v++)
	//	{
	//	for (int u = 0; u < lableImg.cols; u++)
	//	{
	//	if (lableImg.at<short>(v, u)>255)
	//	{
	//	lableImg.at<short>(v, u) = 255;
	//	}
	//	if (lableImg.at<short>(v, u) == 0)
	//	continue;
	//	lableImg_forshow.at<uchar>(v, u) = 255 - lableImg.at<short>(v, u);
	//	}
	//	}
	//
	//	namedWindow("connection");
	//	imshow("connection", lableImg_forshow);
	//	waitKey(0);
}

