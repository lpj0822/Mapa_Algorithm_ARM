#include "computeDisp.h"
#include "utils.h"

//#define CV_NEON
#define OPENMP

#ifdef CV_NEON
#include <arm_neon.h>
#endif

/*
*Function name:  preFilterXSobel
*Parameter list:  Name         I/O           Type	                  Description
*                 src           I 	          const unsigned char*     input image pointer
*                 dst           O             unsigned char*           LUT parameter
*                 preFilterCap  I             int                      Alignment size that must be a power of two
*                 width         I             int                      image's width
*                 height        I             int                      image's height
*Return value:    Type          Description
*                 void          Null                 
*Function description: Image sobel filter in horizontal direction
*/
static void preFilterXSobel(const unsigned char* src, unsigned char* dst, int preFilterCap, int width, int height)
{
	int x, y, d0, d1, d2, d3, v0, v1;	
	unsigned char val0;
	const unsigned char* srow1;
	const unsigned char* srow0;
	const unsigned char* srow2;
	const unsigned char* srow3;
	unsigned char* dptr0;
	unsigned char* dptr1;
	unsigned char* dptr;

	val0 = tab1[0 + OFS];
	for( y = 0; y < height - 1; y += 2 )
	{
		srow1 = src + width * y;
		srow0 = y > 0 ? srow1 - width : srow1 + width;
		srow2 = y < height-1 ? srow1 + width: srow1 - width;
		srow3 = y < height-2 ? srow1 + width * 2: srow1;
		dptr0 = dst + width * y;
		dptr1 = dptr0 + width;

		dptr0[0] = dptr0[width - 1] = dptr1[0] = dptr1[width - 1] = val0;
		for( x = 1; x < width - 1; x++ )
		{
			d0 = srow0[x + 1] - srow0[x - 1];
			d1 = srow1[x + 1] - srow1[x - 1];
			d2 = srow2[x + 1] - srow2[x - 1];
			d3 = srow3[x + 1] - srow3[x - 1];
			v0 = tab1[d0 + (d1 << 1) + d2 + OFS];
			v1 = tab1[d1 + (d2 << 1) + d3 + OFS];

			dptr0[x] = (unsigned char)v0;
			dptr1[x] = (unsigned char)v1;
		}
	}

	for( ; y < height; y++ )
	{
		dptr = dst + width * y;
		for( x = 0; x < width; x++ )
			dptr[x] = val0;
	}
}

/*
*Function name: filterSpecklesImpl
*Parameter list:Name                I/O            Type	            Description
*               src                 I/O 	        short*              input disparity map pointer
*               newVal              I              int                 bad disparity value
*               maxSpeckleSize      I              int                 max Speckle Size
*               maxDiff             I              unsigned char*      max Difference
*               _buf                I              unsigned char*      input buffer
*               width               I              int                 input image width
*               height              I              int                 input image height 
*Return value:  Type           Description 
*               void           Null                 
*Function description: filter Speckles
*/
static void filterSpecklesImpl(short* src, int newVal, int maxSpeckleSize, int maxDiff, unsigned char* _buf, int width, int height)
{
	int npixels = width * height;
	unsigned int bufSize = npixels * (int)(sizeof(wsPoint) + sizeof(int) + sizeof(unsigned char));
	wsPoint* wbuf;
	unsigned char* rtype;
	int curlabel ;
	unsigned char* buf = _buf;
	int i, j, dstep = width;
	int* labels = (int* )buf;
	buf += npixels * sizeof(labels[0]);
	wbuf = (wsPoint* )buf;
	buf += npixels * sizeof(wbuf[0]);
	rtype = (unsigned char* )buf;
	curlabel = 0;

	// clear out label assignments
	memset(labels, 0, npixels * sizeof(labels[0]));

	for( i = 0; i < height; i++ )
	{
		short* ds = src + width * i;
		int* ls = labels + width * i;

		for( j = 0; j < width; j++ ) 
		{
			if( ds[j] != newVal )   // not a bad disparity
			{
				if( ls[j] )     // has a label, check for bad label
				{
					if( rtype[ls[j]] ) // small region, zero out disparity
						ds[j] = (short)newVal;
				}
				// no label, assign and propagate
				else
				{
					wsPoint* ws = wbuf; // initialize wavefront
					wsPoint p;
					int count ;

					p.x= (short)j;
					p.y =(short)i;  // current pixel

					curlabel++; // next label
					count = 0;  // current region size
					ls[j] = curlabel;

					// wavefront propagation
					while( ws >= wbuf ) // wavefront not empty
					{
						short* dpp ;
						short dp;
						int* lpp ;
						count++;
						// put neighbors onto wavefront

						dpp = src + p.y * width + p.x;
						dp = *dpp;
						lpp = labels + width * p.y + p.x;

						if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != newVal && abs(dp - dpp[+dstep]) <= maxDiff )
						{
							lpp[+width] = curlabel;
							ws->x = p.x;
							ws->y = p.y+1;
							ws++;
						}

						if( p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && abs(dp - dpp[-dstep]) <= maxDiff )
						{
							lpp[-width] = curlabel;
							ws->x = p.x;
							ws->y = p.y-1;
							ws++;
						}

						if( p.x < width-1 && !lpp[+1] && dpp[+1] != newVal && abs(dp - dpp[+1]) <= maxDiff )
						{
							lpp[+1] = curlabel;
							ws->x = p.x+1;
							ws->y = p.y;
							ws++;
						}

						if( p.x > 0 && !lpp[-1] && dpp[-1] != newVal && abs(dp - dpp[-1]) <= maxDiff )
						{
							lpp[-1] = curlabel;
							ws->x = p.x-1;
							ws->y = p.y;
							ws++;
						}

						// pop most recent and propagate
						// NB: could try least recent, maybe better convergence
						--ws;
						p = *ws;
					}

					// assign label type
					if( count <= maxSpeckleSize )   // speckle region
					{
						rtype[ls[j]] = 1;   // small region label
						ds[j] = (short)newVal;
					}
					else
						rtype[ls[j]] = 0;   // large region label
				}
			}
		}
	}
}

/*
*Function name: medianBlur
*Parameter list:Name           I/O            Type	                   Description
*               src            I/O 	       short*                  input & output image pointer
*               width          I              int                     image width
*               height         I              int                     image height
*Return value:  Type           Description
*               void           Null                 
*Function description: median Blur
*/
static void medianBlur( short* src, const int width, const int height )
{
	short temp[9];
	int y, x, k, y0, x0;
	short *dst;

	dst = (short* )my_malloc(width * height * sizeof(short));
	memcpy(dst, src, width * height * sizeof(short));
	for(y = 1; y < height - 1; y++) 
	{
		for(x = 1; x < width - 1; x++)
		{
			k = 0;
			for(y0 = -1; y0 <= 1; y0++) 
				for(x0 = -1; x0 <= 1; x0++) 
					temp[k++] = dst[(y + y0) * width + x + x0];

			for(y0 = 0; y0 < 5; y0++) 
				for(x0 = 0; x0 < 8 - y0; x0++) 
					if(temp[x0] > temp[x0 + 1]) 
					{
						short t = temp[x0];
						temp[x0] = temp[x0 + 1];
						temp[x0 + 1] = t;
					}
					src[y * width + x] = temp[4];
		}
	}
	my_free(dst);
}

/*
*Function name: findStereoCorrespondenceBM
*Parameter list:Name           I/O            Type	                     Description
*               left           I 	           const unsigned char*      input left image pointer
*               right          I              const unsigned char*      input right image pointer
*               disp           O              short*                    output disparity map
*               buf            I              unsigned char*            input buffer
*               params         I              cfgBM                     input parameters
*Return value:  Type           Description
*               void           Null                 
*Function description: find stereo correspondence using fast SAD algorithm
*/
static void findStereoCorrespondenceBM(const unsigned char *left, const unsigned char *right, short *disp, unsigned char *buf, int width, int height, 
									   int SADWindowSize, int numDisparities, int minDisparity, int preFilterCap, int uniquenessRatio, int textureThreshold)
{
	int x, y, d;
	int wsz = SADWindowSize;
	int wsz2 = wsz / 2;
	int dy0 = wsz2;
	int dy1 = wsz2 + 1;
	int ndisp = numDisparities;
	int mindisp = minDisparity;
	int lofs = WS_MAX(ndisp - 1 + mindisp, 0);
	int rofs = -WS_MIN(ndisp - 1 + mindisp, 0);
	int width1 = width - rofs - ndisp + 1;
	short FILTERED = (short)((mindisp - 1) << 4);
	int *sad, *hsad0, *hsad, *hsad_sub, *htext;
	unsigned char *cbuf0, *cbuf;
	const unsigned char* lptr0 = left + lofs;
	const unsigned char* rptr0 = right + rofs;
	const unsigned char *lptr, *lptr_sub, *rptr;
	short* dptr = (short *)disp;
	int sstep = width;
	int dstep = width;
	int cstep = (height + dy0 + dy1) * ndisp;
	int costbuf = 0;
	int coststep = (int)(width * sizeof(short)/sizeof(costbuf));
	const unsigned char* cbuf_sub;
	int x0, x1, diff, minsad, mind, currsad, thresh, p, n, lval, tsum;

	sad = (int*)ADAS_ALIGN_16BYTE(buf + sizeof(sad[0]));
	hsad0 = (int*)ADAS_ALIGN_16BYTE(sad + ndisp + 1 + dy0*ndisp);
	htext = (int*)ADAS_ALIGN_16BYTE((int*)(hsad0 + (height+dy1)*ndisp) + wsz2 + 2);
	cbuf0 = (unsigned char*)ADAS_ALIGN_16BYTE((unsigned char*)((int*)(hsad0 + (height + dy1) * ndisp) + wsz + height + 4) + dy0*ndisp);

	memset( hsad0 - dy0 * ndisp, 0, (height + dy0 + dy1) * ndisp * sizeof(hsad0[0]) );
	memset( htext - wsz2 - 1, 0, (height + wsz + 1)*sizeof(htext[0]) );


	for( x = -wsz2 - 1; x < wsz2; x++ )
	{
		hsad = hsad0 - dy0 * ndisp; 
		cbuf = cbuf0 + (x + wsz2 + 1) * cstep - dy0 * ndisp;
		lptr = lptr0 + WS_MIN(WS_MAX(x, -lofs), width - lofs - 1) - dy0 * sstep;
		rptr = rptr0 + WS_MIN(WS_MIN(x, -rofs), width - rofs - ndisp) - dy0 * sstep;

		for( y = -dy0; y < height + dy1; y++ )
		{
			lval = lptr[0];

#ifdef CV_NEON
			int16x8_t lv = vdupq_n_s16 ((int16_t)lval);

			for( d = 0; d < ndisp; d += 8 )
			{
				int16x8_t rv = vreinterpretq_s16_u16 (vmovl_u8 (vld1_u8 (rptr + d)));
				int32x4_t hsad_l = vld1q_s32 (hsad + d);
				int32x4_t hsad_h = vld1q_s32 (hsad + d + 4);
				int16x8_t diff = vabdq_s16 (lv, rv);
				vst1_u8 (cbuf + d, vmovn_u16(vreinterpretq_u16_s16(diff)));
				hsad_l = vaddq_s32 (hsad_l, vmovl_s16(vget_low_s16 (diff)));
				hsad_h = vaddq_s32 (hsad_h, vmovl_s16(vget_high_s16 (diff)));
				vst1q_s32 ((hsad + d), hsad_l);
				vst1q_s32 ((hsad + d + 4), hsad_h);
			}
#else
			for( d = 0; d < ndisp; d++ )
			{
				// BTSAD 
				//int rval0 = ((rptr[d] + rptr[d - 1]) >> 1);
				//int rval1 = ((rptr[d] + rptr[d + 1]) >> 1);	
				//int Ir0 = WS_MAX(WS_MIN(rptr[d], rval0), rval1);
				//int Ir1 = WS_MAX(WS_MAX(rptr[d], rval0), rval1);
				//diff = WS_MAX(WS_MAX(lval - Ir1, 0), Ir0 - lval);
				diff = abs(lval - rptr[d]);

				cbuf[d] = (unsigned char)diff;
				hsad[d] = (int)(hsad[d] + diff);
			}
#endif
			htext[y] += tab0[lval];
			hsad += ndisp;
			cbuf += ndisp;
			lptr += sstep;
			rptr += sstep;
		}
	}

	for( y = 0; y < height; y++ )
	{
		for( x = 0; x < lofs; x++ )
			dptr[y*dstep + x] = FILTERED;

		for( x = lofs + width1; x < width; x++ )
			dptr[y*dstep + x] = FILTERED;
	}
	dptr += lofs;

	for( x = 0; x < width1; x++, dptr++ )
	{
		x0 = x - wsz2 - 1;
		x1 = x + wsz2;
		cbuf_sub = cbuf0 + ((x0 + wsz2 + 1) % (wsz + 1)) * cstep - dy0 * ndisp;
		cbuf = cbuf0 + ((x1 + wsz2 + 1) % (wsz + 1)) * cstep - dy0 * ndisp;
		hsad = hsad0 - dy0 * ndisp;
		lptr_sub = lptr0 + WS_MIN(WS_MAX(x0, -lofs), width - 1 - lofs) - dy0 * sstep;
		lptr = lptr0 + WS_MIN(WS_MAX(x1, -lofs), width - 1 - lofs) - dy0 * sstep;
		rptr = rptr0 + WS_MIN(WS_MAX(x1, -rofs), width - ndisp - rofs) - dy0 * sstep;
		for( y = -dy0; y < height + dy1; y++ )
		{
			lval = lptr[0];
#ifdef CV_NEON
			int16x8_t lv = vdupq_n_s16 ((int16_t)lval);
			for( d = 0; d < ndisp; d += 8 )
			{
				int16x8_t rv = vreinterpretq_s16_u16 (vmovl_u8 (vld1_u8 (rptr + d)));
				int32x4_t hsad_l = vld1q_s32 (hsad + d);
				int32x4_t hsad_h = vld1q_s32 (hsad + d + 4);
				int16x8_t cbs = vreinterpretq_s16_u16 (vmovl_u8 (vld1_u8 (cbuf_sub + d)));
				int16x8_t diff = vabdq_s16 (lv, rv);
				int32x4_t diff_h = vsubl_s16 (vget_high_s16 (diff), vget_high_s16 (cbs));
				int32x4_t diff_l = vsubl_s16 (vget_low_s16 (diff), vget_low_s16 (cbs));
				vst1_u8 (cbuf + d, vmovn_u16(vreinterpretq_u16_s16(diff)));
				hsad_h = vaddq_s32 (hsad_h, diff_h);
				hsad_l = vaddq_s32 (hsad_l, diff_l);
				vst1q_s32 ((hsad + d), hsad_l);
				vst1q_s32 ((hsad + d + 4), hsad_h);
			}
#else
			for( d = 0; d < ndisp; d++ )
			{
				// BTSAD
				//int rval0 = ((rptr[d] + rptr[d - 1]) >> 1);
				//int rval1 = ((rptr[d] + rptr[d + 1]) >> 1);
				//int Ir0 = WS_MAX(WS_MIN(rptr[d], rval0), rval1);
				//int Ir1 = WS_MAX(WS_MAX(rptr[d], rval0), rval1);
				//diff = WS_MAX(WS_MAX(lval - Ir1, 0), Ir0 - lval);
				diff = abs(lval - rptr[d]);

				cbuf[d] = (unsigned char)diff;
				hsad[d] = hsad[d] + diff - cbuf_sub[d];
			}
#endif

			cbuf += ndisp;
			cbuf_sub += ndisp;
			hsad += ndisp;
			lptr += sstep;
			lptr_sub += sstep;
			rptr += sstep;
			htext[y] += tab0[lval] - tab0[lptr_sub[0]];
		}

		// fill borders
		for( y = dy1; y <= wsz2; y++ )
			htext[height+y] = htext[height+dy1-1];
		for( y = -wsz2-1; y < -dy0; y++ )
			htext[y] = htext[-dy0];

		// initialize sums
		for( d = 0; d < ndisp; d++ )
			sad[d] = (int)(hsad0[d - ndisp * dy0]*(wsz2 + 2 - dy0));

		hsad = hsad0 + (1 - dy0)*ndisp;
		for( y = 1 - dy0; y < wsz2; y++ )
		{
			for( d = 0; d < ndisp; d++ )
				sad[d] = (int)(sad[d] + hsad[d]);

			hsad += ndisp;
		}

		tsum = 0;
		for( y = -wsz2-1; y < wsz2; y++ )
			tsum += htext[y];

		// finally, start the real processing
		for( y = 0; y < height; y ++ )
		{
			minsad = WS_INT_MAX;
			mind = -1;
			hsad = hsad0 + WS_MIN(y + wsz2, height + dy1 - 1) * ndisp;
			hsad_sub = hsad0 + WS_MAX(y - wsz2 - 1, -dy0) * ndisp;

			// WTA
			for( d = 0; d < ndisp; d++ )
			{
				currsad = sad[d] + hsad[d] - hsad_sub[d];
				sad[d] = currsad;
				if( currsad < minsad )
				{
					minsad = currsad;
					mind = d;
				}
			}

			tsum += htext[y + wsz2] - htext[y - wsz2 - 1];
			if( tsum < textureThreshold )
			{
				dptr[y*dstep] = FILTERED;
				continue;
			}

			// RANSAC
			if( uniquenessRatio > 0 )
			{
				thresh = minsad + (minsad * uniquenessRatio >> 7);
				for( d = 0; d < ndisp; d++ )
				{
					if( (d < mind - 1 || d > mind + 1) && sad[d] <= thresh)
						break;
				}
				if( d < ndisp )
				{
					dptr[y * dstep] = FILTERED;
					continue;
				}
			}

			// Subpixel
			sad[-1] = sad[1];
			sad[ndisp] = sad[ndisp - 2];
			p = sad[mind + 1]; 
			n = sad[mind - 1];
			d = p + n - (sad[mind] << 1) + abs(p - n);
			dptr[y * dstep] = (short)((((ndisp - mind - 1 + mindisp) << 8) + (d != 0 ? ((p - n) << 8) / d : 0) + 15) >> 4);
		}
	}
}


/*
*Function name: computeDisp
*Parameter list:Name           I/O            Type	                     Description
*               left           I 	           const unsigned char*      input left image pointer
*               right          I              const unsigned char*      input right image pointer
*               disp           O              short*                    output disparity map
*               params         I              cfgBM                     input parameters
*Return value:  Type           Description
*               void           Null                 
*Function description: compute the disparity map of entire image
*/
extern void computeDisp(const unsigned char* left, const unsigned char* right, short* disp, cfgBM params)
{
	int i, wsz2, omprow, FILTERED;
	int row0, row1, rows, cols, bgrow, bufSize;
	unsigned char *ompleft, *ompright, *ptr;
	short* local;

	wsz2 = params.SADWindowSize / 2;
	omprow = params.height / params.ompNum;
	FILTERED = (params.minDisparity - 1) << 4; 
#ifdef OPENMP
#pragma omp parallel for private(ompleft, ompright, ptr, local, row0, row1, rows, cols, bgrow, bufSize) num_threads(2)
#endif // OPENMP                
	for (i = 0; i < params.ompNum; i++)
	{	
		row0 = (i == 0) ?  0 : i * omprow - wsz2; 
		row1 = (i == params.ompNum - 1) ? params.height : (i + 1) * omprow + wsz2;
		bgrow = (i == 0) ?  0 : wsz2; 
		rows = row1 - row0;
		cols = params.width;
		bufSize = rows * cols * (sizeof(wsPoint) + sizeof(int) + sizeof(unsigned char));	 
		ptr = (unsigned char *)my_malloc(bufSize);
		local = (short* )my_malloc(rows * cols * sizeof(short));
		ompleft = (unsigned char *)my_malloc((omprow + params.SADWindowSize) * cols);
		ompright = (unsigned char *)my_malloc((omprow + params.SADWindowSize) * cols);

		//filter X-direction Sobel
		preFilterXSobel(left + row0 * cols, ompleft, params.preFilterCap, cols, rows);
		preFilterXSobel(right + row0 * cols, ompright, params.preFilterCap, cols, rows);

		//find Stereo Correspondence
		findStereoCorrespondenceBM(ompleft + cols * wsz2, ompright + cols * wsz2, local + cols * wsz2, ptr, cols, rows - params.SADWindowSize + 1, 
			params.SADWindowSize, params.numDisparities, params.minDisparity, params.preFilterCap, params.uniquenessRatio, params.textureThreshold);

		//filter Speckles	
		filterSpecklesImpl(local, FILTERED, params.speckleWindowSize, params.speckleRange, ptr, cols, rows);

		//median filter
		//medianBlur(local, cols, rows);

		//merge all thread
#ifdef OPENMP
#pragma omp critical
#endif
		{
			memcpy(disp + i * omprow * params.width, local + bgrow * params.width, omprow * params.width * sizeof(short));
		}

		my_free(ptr);
		my_free(local);
		my_free(ompleft);
		my_free(ompright);
	}
}





