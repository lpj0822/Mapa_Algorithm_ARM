#include "day_or_night.h"
#include <stdlib.h>

#define  DAY_NUM_THR   60
#define  GRAY_NUM     256

static int ndaylab[DAY_NUM_THR] = { 0 };
static int nFramecount = 0;
static int daylabtotal = 0;

typedef struct DRECT
{
	int x;
	int y;
	int w;
	int h;
}Drect;

int mvDayOrNight(const unsigned char*  grayImg, const int width, const int height)
{
	int i, j;
	//int maxval = 0;
	//int maxgray = 0;
	//int imageVar = 0;
	int imageMean = 0;
	int numberCount = 0;
	int tsum = 0;
	int graybins[GRAY_NUM] = { 0 };
	int graythre = 100;
	const unsigned char *src = NULL;

	Drect rect = { 0, 0, width, 216};

	for (i = rect.y; i < rect.y + rect.h; i++)
	{
		src = grayImg + i * width;

		for (j = rect.x; j < rect.x + rect.w; j++)
		{
			graybins[src[j]]++;
			imageMean += src[j];
			numberCount++;
		}
	}

	imageMean /= numberCount;
	/*
	for (i = 0; i < GRAY_NUM; i++)
	{
		imageVar += abs(i - imageMean) * graybins[i];
	}
	imageVar /= numberCount;

	printf("imageVar : %d\n", imageVar);
	*/
	/*
	for (i = 0; i < GRAY_NUM; i++)
	{
		if (graybins[i] > maxval)
		{
			maxval = graybins[i];
			maxgray = i;
		}
	}
	*/

	if (imageMean > graythre)
	{
		ndaylab[nFramecount%DAY_NUM_THR] = 1;//day 

	}
	else
	{
		ndaylab[nFramecount%DAY_NUM_THR] = -1;//night
	}

	if (nFramecount%DAY_NUM_THR == 0 && nFramecount >= DAY_NUM_THR)
	{
		for (i = 0; i < DAY_NUM_THR; i++)
		{
			tsum += ndaylab[i];
		}

		if (tsum >= 0 && daylabtotal < 8)
		{
			daylabtotal++;

		}
		else if (tsum<0 && daylabtotal > -8)
		{
			daylabtotal--;
		}
	}
	nFramecount++;
	return daylabtotal;
}
