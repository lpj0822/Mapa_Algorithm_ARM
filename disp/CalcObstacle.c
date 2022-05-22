#include "3Dmap.h"

#define PROJ_THRESHOLD 1000

static Mat_uchar tempImg = { NULL, 0, 0 };
static Mat_short lableImg_temp = { NULL, 0, 0 };

void Init_CalcObstacle()
{
	doInitialMatUchar(&tempImg, COM_H, (UX_WIDTH >> U_COM_INDEX));
	doInitialMatShort(&lableImg_temp, tempImg.rows, tempImg.cols);
}


void doCalcObstacle(const Mat_short src, Mat_short* lableImg)
{
	memset(tempImg.data, 0, tempImg.rows*tempImg.cols*sizeof(uchar));
	memset(lableImg_temp.data, 0, lableImg_temp.rows*lableImg_temp.cols*sizeof(short));

	int i, v, u;
	//	src -= 5;

	//	Mat SumImg(src.size(), CV_32SC1);
	//	integral(src, SumImg, -1);
	int pt_sum;
	int pt[3];

	//	double t = now_ms();

	for (v = 2; v < src.rows - 2; v++)
	{
		for (u = 2; u < src.cols - 2; u++)
		{
			if (src.data[v*src.cols + u] == 0)
				continue;

			pt_sum = src.data[v*src.cols + u - 1] +
				src.data[v*src.cols + u] +
				src.data[v*src.cols + u + 1];

			if (pt_sum > PROJ_THRESHOLD)
			{
				tempImg.data[v*tempImg.cols + (u >> U_COM_INDEX)] = 255;

				if (tempImg.data[(v - 1)*tempImg.cols + (u >> U_COM_INDEX)] == 0)
				{
					if (src.data[(v - 1)*src.cols + u] > 300)
					{
						tempImg.data[(v - 1)*tempImg.cols + (u >> U_COM_INDEX)] = 255;
					}
				}
			}
		}
	}



	//	t = now_ms() - t;
	//	LOGI("Time %.3f   \n", t);

	//Mat lableImg(tempImg.clone()); //旧代码

	//

	//vector<vector<Point>> contours;
	//findContours(lableImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//lableImg = 0;

	//for (int i = 0; i < contours.size(); i++)
	//{
	//	drawContours(lableImg, contours,      //画出各个图像的色块
	//		i,
	//		Scalar(i + 1), // 依次色度加1
	//		CV_FILLED); // with a thickness of 2
	//}

	/*Mat img_forshow(tempImg.rows, tempImg.cols, CV_8UC1);
	for (int v = 0; v < tempImg.rows; v++)
	{
	for (int u = 0; u < tempImg.cols; u++)
	{
	img_forshow.at<uchar>(v, u) = tempImg.data[v*tempImg.cols + u];
	}
	}

	imshow("Win06", img_forshow);*/

	doFindConection(tempImg, &lableImg_temp);



	//恢复在U方向被压缩的lableImg
	for (v = 0; v < lableImg_temp.rows; v++)
	{
		for (u = 0; u < lableImg_temp.cols - 1; u++)
		{
			short value = lableImg_temp.data[v*lableImg_temp.cols + u];
			if (value>0)
			{
				short *sp = &lableImg->data[v*lableImg->cols + (u << U_COM_INDEX)];

				int comstep = (1 << U_COM_INDEX);
				for (i = 0; i < comstep; i++)
				{
					*sp = value; sp++;
				}
			}
		}
	}

	//Mat img_forshow(lableImg->rows, lableImg->cols, CV_8UC1);
	//for (int v = 0; v < lableImg->rows; v++)
	//{
	//	for (int u = 0; u < lableImg->cols; u++)
	//	{
	//		img_forshow.at<uchar>(v, u) = lableImg->data[v*lableImg->cols + u];
	//	}
	//}

	//imshow("Win06", img_forshow);


	//	doExtendConection(src, lableImg);






	//	waitKey(0);
}

void doCalcUProjImg(const Mat_short src, Mat_short *dst, const Mat_uchar vmask)
{
	int v, u;

	memset(dst->data, 0, dst->rows*dst->cols*sizeof(short));

	for (v = 0; v < src.rows; v++)
	{

		for (u = 0; u < src.cols; u++)
		{
			int value = src.data[v*src.cols + u];
			if (value == 0 || value >= 256)
				continue;


			int com_value = COM_ARRAY[value];
			uchar mask_value = vmask.data[v*vmask.cols + value];


			switch (mask_value)
			{
			case 0:
				break;
			case 1:
				dst->data[com_value*dst->cols + u] += 400;
				break;
			case 2:
				dst->data[com_value*dst->cols + u] += 50;
				break;
			case 3:
				dst->data[com_value*dst->cols + u] += 10;
				break;
			case 4:
				dst->data[com_value*dst->cols + u] += 2;
				break;
			default:
				break;
			}

		}
	}
}
