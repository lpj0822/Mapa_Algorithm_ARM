#ifndef  GPU_LBP_DETECT_H
#define  GPU_LBP_DETECT_H

extern void Gpu_init(FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l);
extern void Gpu_object_detector_lbp_detect_window( lbpCar *l, u32 *img,float32_t factor);
extern void Gpu_Uninit(void);
#endif


 