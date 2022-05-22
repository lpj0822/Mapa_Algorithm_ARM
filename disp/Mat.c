#include "3Dmap.h"

void doInitialMatInt(Mat_int *matint, int rows, int cols)
{
	if (matint->data != NULL)
		free(matint->data);

	matint->rows = rows;
	matint->cols = cols;
	matint->data = (int *)malloc(rows*cols*sizeof(int));
	memset(matint->data, 0, rows*cols*sizeof(int));
}

void doReleaseMatInt(Mat_int* matint)
{
	free(matint->data);

	matint->data = NULL;
	matint->rows = 0;
	matint->cols = 0;
}

void doInitialMatShort(Mat_short* matshort, int rows, int cols)
{
	if (matshort->data != NULL)
		free(matshort->data);

	matshort->rows = rows;
	matshort->cols = cols;
	matshort->data = (short *)malloc(rows*cols*sizeof(short));
	memset(matshort->data, 0, rows*cols*sizeof(short));
}

void doReleaseMatShort(Mat_short* matshort)
{
	free(matshort->data);

	matshort->data = NULL;
	matshort->rows = 0;
	matshort->cols = 0;
}

void doInitialMatUchar(Mat_uchar* matuchar, int rows, int cols)
{
	if (matuchar->data == NULL)
		matuchar->data = (uchar *)malloc(rows*cols*sizeof(uchar));

	matuchar->rows = rows;
	matuchar->cols = cols;

	memset(matuchar->data, 0, rows*cols*sizeof(uchar));
}

void doReleaseMatUchar(Mat_uchar* matuchar)
{
	free(matuchar->data);

	matuchar->data = NULL;
	matuchar->rows = 0;
	matuchar->cols = 0;
}

void doInitialMatColor(Mat_color* matcolor, int rows, int cols)
{
	if (matcolor->data != NULL)
		free(matcolor->data);

	matcolor->rows = rows;
	matcolor->cols = cols;

	matcolor->data = (COLOR *)malloc(rows*cols*sizeof(COLOR));
	memset(matcolor->data, 0, rows*cols*sizeof(COLOR));
}

void doReleaseMatColor(Mat_color* matcolor)
{
	free(matcolor->data);

	matcolor->data = NULL;

	matcolor->rows = 0;
	matcolor->cols = 0;
}
