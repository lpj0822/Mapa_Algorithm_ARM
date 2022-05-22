#include "3Dmap.h"

static Mat_short MAT_UXtrans;
static float VAL_INV[256];

//初始化MAT_UXtrans和VAL_INV
void doInitMatUX(const Mat_short DispImg, const Mat_short DXImg)
{
	int i, v, u;

	doInitialMatShort(&MAT_UXtrans, 256, DispImg.cols);

	for (v = 0; v < MAT_UXtrans.rows; v++)
	{
		for (u = 0; u < MAT_UXtrans.cols; u++)
		{
			if (v == 0)
			{
				MAT_UXtrans.data[v*MAT_UXtrans.cols + u] = -1;
				continue;
			}

			int X = UX_SCALE*(u - MAT_UXtrans.cols / 2.0) / (float)v + DXImg.cols / 2.0;

			if (X < 0 || X > DXImg.cols)
			{
				MAT_UXtrans.data[v*MAT_UXtrans.cols + u] = -1;
				continue;
			}

			MAT_UXtrans.data[v*MAT_UXtrans.cols + u] = X;
		}
	}

	for (i = 0; i < 256; i++)
	{
		VAL_INV[i] = UX_SCALE / (i + 1e-10);
	}


}

short getX(int value, int u)
{
	return MAT_UXtrans.data[value*MAT_UXtrans.cols + u];
}



void doCalcDXimg(Mat_short *src, Mat_short *dst, const Mat_uchar vmask)
{
	int i, v, u;

	memset(dst->data, 0, dst->rows*dst->cols*sizeof(short));

	for (v = 0; v < src->rows; v++)
	{

		for (u = 0; u < src->cols; u++)
		{
			int value = src->data[v*src->cols + u];
			if (value <= 0 || value > 255)
			{
				src->data[v*src->cols + u] = 0;
				continue;
			}


			//int X = UX_SCALE*(u - src->cols / 2.0) / (float)value + dst->cols / 2.0;
			int X = MAT_UXtrans.data[value * MAT_UXtrans.cols + u];
			if (X == -1)
			{
				src->data[v*src->cols + u] = 0;
				continue;
			}


			int com_value = COM_ARRAY[value];

			if (dst->data[com_value*dst->cols + X] > 10000)//value > UX_SCALE && 
				continue;

			float mask_value = vmask.data[v*vmask.cols + value];

			if (mask_value < 1)
			{
				src->data[v*src->cols + u] = 0;
				continue;
			}

			if (value >= 200)
			{
				dst->data[com_value*dst->cols + X] += ((VAL_INV[value] * mask_value * 3)+1);
			}
			else
			{
				dst->data[com_value*dst->cols + X] += mask_value;
				for (i = 1; i < (VAL_INV[value]); i++)
				{
					dst->data[com_value*dst->cols + X + i] += mask_value;
					dst->data[com_value*dst->cols + X - i] += mask_value;
				}

				dst->data[com_value*dst->cols + X + i] += mask_value*(VAL_INV[value] - i + 1);
				dst->data[com_value*dst->cols + X - i] += mask_value*(VAL_INV[value] - i + 1);

			}


		
		}
	}

}
