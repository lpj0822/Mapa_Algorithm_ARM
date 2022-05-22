/**************************************************************************************************************
Copyright  Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: kalmanfilter.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of kalman filter.
	The following function types are included:
	+ kalman1_init(): init the values for kalman1_state for one Dimension.
	+ kalman1_filter(): Do the Kalman filter for x based on the measurement z.
    + kalman_rect_init(): init the values for kalman1_state for four Dimension.
	+ kalman_rect_filter(): Do the Kalman filter for x based on the measurement z.
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H

#include "type_def.h"
/* 
 * NOTES: n Dimension means the state is n dimension, 
 * measurement always 1 dimension 
 */

/* 1 Dimension */
typedef struct {
    float32_t x;  /* state */
    float32_t A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float32_t H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float32_t q;  /* process(predict) noise convariance */
    float32_t r;  /* measure noise convariance */
    float32_t p;  /* estimated error convariance */
    float32_t gain;
} kalman1_state;
       

/* Rect Dimension{x,y,w,h} */
typedef struct KALMAL_RECT_STATE
{
	float32_t x[4];  /* state */
	float32_t A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
	float32_t H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
	float32_t q[4];  /* process(predict) noise convariance */
	float32_t r[4];  /* measure noise convariance */
	float32_t p[4];  /* estimated error convariance */
	float32_t gain[4];
} kalman_rect_state;


/* system state */
typedef struct {
	double x[7];
	double p[7 * 7];
	double q[7 * 7];
	double r[5 * 5];

	double A[7 * 7];
	double At[7 * 7];
	double M[7 * 7];
	double Mt[7 * 7];
	double H[5 * 7];
	double Ht[7 * 5];

	double fx;
	double fy;
	double u0;
	double v0;
	double camH;
} sysKalmanState;


/*
I/O:	    Name		          Type	     		          Content
					    						  
[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    init_x		          float		                  init value for state->x.
[in]	    init_p		          float		                  init value for state->p.

Realized function:
    + init the values for kalman1_state for one Dimension.
*/
 void kalman1_init(kalman1_state *state, float init_x, float init_p);

 /*
I/O:	    Name		          Type	     		          Content
					    						  
[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    z_measure		      float		                  measurement for x.

Realized function:
    + Do the Kalman filter for x based on the measurement z.
*/
float kalman1_filter(kalman1_state *state, float z_measure);

/*
I/O:	    Name		          Type	     		          Content
					    						  
[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    init_x		          AdasRect		              init value for state->x.
[in]	    init_p		          float32_t		              init value for state->p.

Realized function:
    + init the values for kalman1_state for four Dimension.
*/
void kalman_rect_init(kalman_rect_state *state,  AdasRect rec, float32_t init_p[4]);

 /*
I/O:	    Name		          Type	     		          Content
					    						  
[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    z_measure		      float		                  measurement for x.

Realized function:
    + Do the Kalman filter for x based on the measurement z.
*/
AdasRect kalman_rect_filter(kalman_rect_state *state, AdasRect Measure_Rec);


/******************************************************/
/* system kalman filter function*/
/******************************************************/
void sysKalmanInit(sysKalmanState *state, AdasRect init_rec);

void sysKalmanFilter(sysKalmanState *state, double tpf, AdasRect Measure_Rec, double vasy);

#endif  /*_KALMAN_FILTER_H*/

