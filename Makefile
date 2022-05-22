# 1、准备工作，编译方式、目标文件名、依赖库路径的定义。
CC = /usr/local/linaro-aarch64-2017.08-gcc7.1/bin/aarch64-linux-gnu-gcc

#LOCAL_PATH	:= $(call my-dir)
LOCAL_PATH	:= .
CFLAGS  := -I$(LOCAL_PATH)/inc -Wformat \
		-Werror=format-security -Wall -fPIC \
		-D_REENTRENT -D_GNU_SOURCE -O3  \
		-fdiagnostics-color=auto -march=armv8-a \
		-mcpu=cortex-a53+crypto -mlittle-endian \
		-MMD -fopenmp -g

#LDFLAGS := -fopenmp

OBJS := $(LOCAL_PATH)/ldws/AMF.o \
	   $(LOCAL_PATH)/ldws/Bspline.o \
	   $(LOCAL_PATH)/ldws/Caractere.o \
	   $(LOCAL_PATH)/ldws/Classement_Zone.o \
	   $(LOCAL_PATH)/ldws/Detection_Zone.o \
	   $(LOCAL_PATH)/ldws/fileUtil.o \
	   $(LOCAL_PATH)/ldws/Init_Struct.o \
	   $(LOCAL_PATH)/ldws/Initialisation.o \
	   $(LOCAL_PATH)/ldws/LDWS_AlarmDecision.o \
	   $(LOCAL_PATH)/ldws/LDWS_Interface.o \
	   $(LOCAL_PATH)/ldws/Matrice.o \
	   $(LOCAL_PATH)/ldws/Median.o \
	   $(LOCAL_PATH)/ldws/Mise_A_Jour.o \
	   $(LOCAL_PATH)/ldws/Points.o \
	   $(LOCAL_PATH)/ldws/Recherche.o \
	   $(LOCAL_PATH)/ldws/Road_Tracker.o \
	   $(LOCAL_PATH)/ldws/Sauvegarde.o \
	   $(LOCAL_PATH)/fcwsd/FCWSD_Interface.o \
	   $(LOCAL_PATH)/fcwsd/GPULbpDetect.o \
	   $(LOCAL_PATH)/fcwsd/group_rect.o \
	   $(LOCAL_PATH)/fcwsd/lbp_detect.o \
	   $(LOCAL_PATH)/fcwsd/vehicle_det.o \
	   $(LOCAL_PATH)/fcwsd/vehicle_proposals.o \
	   $(LOCAL_PATH)/fcwst/CMulitTrack.o \
	   $(LOCAL_PATH)/fcwst/fast.o \
	   $(LOCAL_PATH)/fcwst/Geometry.o \
	   $(LOCAL_PATH)/fcwst/harriscroner.o \
	   $(LOCAL_PATH)/fcwst/kalmanfilter.o \
	   $(LOCAL_PATH)/fcwst/ObjGroup.o \
	   $(LOCAL_PATH)/fcwst/preprocess.o \
	   $(LOCAL_PATH)/fcwst/surf.o \
	   $(LOCAL_PATH)/fcwst/table.o \
	   $(LOCAL_PATH)/fcwst/trajectory.o \
	   $(LOCAL_PATH)/fcwst/vehicle_shadow.o \
	   $(LOCAL_PATH)/fcwst/clustering_rect.o \
	   $(LOCAL_PATH)/fcwst/vehicle_taillight.o \
	   $(LOCAL_PATH)/fcwst/MyMath.o \
	   $(LOCAL_PATH)/objverf/OBJVERF_Interface.o \
	   $(LOCAL_PATH)/disp/CalcObstacle.o \
	   $(LOCAL_PATH)/disp/computeDisp.o \
	   $(LOCAL_PATH)/disp/Conection.o \
	   $(LOCAL_PATH)/disp/DISP_Interface.o \
	   $(LOCAL_PATH)/disp/DXmethod.o \
	   $(LOCAL_PATH)/disp/Filter.o \
	   $(LOCAL_PATH)/disp/function.o \
	   $(LOCAL_PATH)/disp/GetDist.o \
	   $(LOCAL_PATH)/disp/Histogram.o \
	   $(LOCAL_PATH)/disp/Mat.o \
	   $(LOCAL_PATH)/disp/rectifyImg.o \
	   $(LOCAL_PATH)/disp/Tracker.o \
	   $(LOCAL_PATH)/disp/remap.o \
	   $(LOCAL_PATH)/dayOrNight/day_or_night.o

# 目标文件名 
LIB = libSmart.so

all : $(LIB)

# 2. 生成.o文件 
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 3. 生成静态库文件
$(LIB) : $(OBJS)
	rm -f $@
	#/usr/local/linaro-aarch64-2017.08-gcc7.1/bin/aarch64-linux-gnu-ar crsD $@ $(OBJS)
	$(CC) -shared -o $@ $(OBJS)
	rm -f $(OBJS)

tags :
	ctags -R *

# 4. 删除中间过程生成的文件 
clean:
	rm -f $(OBJS) $(TARGET) $(LIB)

