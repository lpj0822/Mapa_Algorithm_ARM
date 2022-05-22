#include "utils.h"
#include "vehicle_type.h"
#include "vehicle_det.h"
#include "GPULbpDetect.h"
#include <stdlib.h>
#include <string.h>

#ifdef _USE_GPU_
#include "CL/cl.h"

void Gpu_init(FCWSDetectorGlobalPara *pVehicleDetor, lbp_car *l);
void Gpu_object_detector_lbp_detect_window( lbp_car *l, u32 *img, float32_t factor);
void Gpu_Uninit(void);

static void	checkErr(cl_int err, const char * name);
static Bool	writeBinaryToFile(const char* fileName, const char* birary, size_t numBytes);
static char	*readBinaryFromFile(const char* fileName, size_t* szFinalLength);
static char	*LoadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength);
static int	Ceil_car( float32_t value );


s32 g_gpustate  = 0;

static s32				g_nMAX_Det_Num;

static cl_platform_id	g_platform;
static cl_device_id		g_device;

static cl_context		g_context;
static cl_program		g_program;
static cl_command_queue	g_queue;
static cl_mem			g_c_Detbuf;
static cl_mem			c_stagebuf;
static cl_mem			c_weak_classifierbuf;
static cl_mem			c_lbp_rectbuf;
static cl_kernel		lbp_detect_kernel;

static reduce_lbp		rlbp;
static size_t localws_score[2] ;
static size_t globalws_score[2];

static void checkErr(cl_int err, const char * name)
{
	if (err != CL_SUCCESS)
	{
		my_printf("error:%s,%d",name,err);
		exit(EXIT_FAILURE);
	}
}

static Bool writeBinaryToFile(const char* fileName, const char* birary, size_t numBytes)
{
	void *output = NULL;

	output = my_fopen(fileName, "wb");
	if(output == NULL)
	{
		return 0;
	}

	my_fwrite(birary, sizeof(char), numBytes, output);

	my_fclose(output);

	return 1;
}

static char *readBinaryFromFile(const char* fileName, size_t* szFinalLength)
{
	void *input  = NULL;
	size_t	size    = 0;
	char	*binary = NULL;

	input = my_fopen(fileName, "rb");
	if(input == NULL)
	{
		return NULL;
	}

	fseek(input, 0L, SEEK_END);
	size = ftell(input);

	rewind(input);
	binary = (char*)my_malloc(size);
    memset(binary, 0, size);

	if(binary == NULL)
	{
		return NULL;
	}

	my_fread(binary, sizeof(char), size, input);
	my_fclose(input);

	*szFinalLength = size;

	return binary;
}

//read kerenl file
static char* LoadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength)
{
	void	*pFileStream = NULL;
	size_t	szSourceLength;
	char	*cSourceString = NULL;
	size_t	szPreambleLength;

	// open the OpenCL source code file
	pFileStream = my_fopen(cFilename, "rb");
	if(pFileStream == NULL)
	{
		return NULL;
	}

	szPreambleLength = strlen(cPreamble);

	// get the length of the source code
	fseek(pFileStream, 0, SEEK_END);
	szSourceLength = ftell(pFileStream);
	fseek(pFileStream, 0, SEEK_SET);

	// allocate a buffer for the source code string and read it in
	cSourceString = (char *)my_malloc(szSourceLength + szPreambleLength + 1);
    memset(cSourceString, 0, szSourceLength + szPreambleLength + 1);
	memcpy(cSourceString, cPreamble, szPreambleLength);

	if (my_fread((cSourceString) + szPreambleLength, szSourceLength, 1, pFileStream) != 1)
	{
		my_fclose(pFileStream);
		my_free(cSourceString);
		return NULL;
	}

	// close the file and return the total length of the combined (preamble + source) string
	my_fclose(pFileStream);

	if(szFinalLength != NULL)
	{
		*szFinalLength = szSourceLength + szPreambleLength;
	}
	cSourceString[szSourceLength + szPreambleLength] = '\0';

	return cSourceString;
}

static int Ceil_car( float32_t value )
{
	s32 i = (s32)(value + (value >= 0 ? 0.5 : -0.5));

	float32_t diff = (float32_t)(i - value);

	return i + (diff < 0);
}

void  Gpu_Uninit(void)
{
	//delete OpenCL obj

	clReleaseKernel(lbp_detect_kernel);

	clReleaseMemObject(c_lbp_rectbuf);
	clReleaseMemObject(c_weak_classifierbuf);
	clReleaseMemObject(c_stagebuf);
	clReleaseMemObject(g_c_Detbuf);

	clReleaseProgram(g_program);
	clReleaseCommandQueue(g_queue);
	clReleaseContext(g_context);
}

void  Gpu_init(FCWSDetectorGlobalPara *pVehicleDetor, lbp_car *l) 
{
	
	s32 i;
	uint8_t *pTr;

	cl_int	nErroNum;
	cl_uint numPlatforms, status;

	size_t	szKernelLength = 0;
	size_t	*binarySizes = NULL;

	char	*binary		= NULL;
	char	**binaries	= NULL;
    char	*kernelSourceCode = NULL;

	AMBA_FS_FILE	*pFileStream = NULL;

	reduce_stage		*stage			= NULL;
    weak_classifier_car *classifiers	= NULL;
	weak_classifier_car *ptr			= NULL;
	lbp_rect_car		*pRec			= NULL;

	const char options[] = "-cl-mad-enable -cl-no-signed-zeros";

	status = clGetPlatformIDs(0,NULL,&numPlatforms);
	checkErr((status != CL_SUCCESS) ? status :(numPlatforms <= 0 ? -1 : CL_SUCCESS), "clGetPlatformIDs");

	if (numPlatforms > 0)
	{
		cl_platform_id	*platforms = NULL;
		char pbuff[128];
		u32 i;
		platforms = (cl_platform_id*)my_malloc(numPlatforms * sizeof(cl_platform_id));
        memset(platforms, 0, numPlatforms * sizeof(cl_platform_id));

		status = clGetPlatformIDs(numPlatforms,platforms,NULL);
		for ( i = 0 ; i < numPlatforms; ++i)
		{
			status = clGetPlatformInfo(platforms[i], CL_PLATFORM_NAME, sizeof(pbuff), pbuff,NULL);
			g_platform = platforms[i];
			my_printf("%s\n",pbuff);
		}
		my_free(platforms);
	}
	status = clGetDeviceIDs(g_platform,CL_DEVICE_TYPE_GPU,1,&g_device,NULL);
	checkErr(status, "clGetDeviceIDs");

	{
		char        *value;
		size_t      valueSize;
		size_t      maxWorkItemPerGroup;
		cl_uint     maxComputeUnits=0;
		cl_uint     maxWorkItemDim=0;
		cl_ulong    maxGlobalMemSize=0;
		cl_ulong    maxConstantBufferSize=0;
		cl_ulong    maxLocalMemSize=0;
		size_t*		MaxItemSize;

		//print the device name
		clGetDeviceInfo(g_device, CL_DEVICE_NAME, 0, NULL, &valueSize);
		value = (char*) my_malloc(valueSize);
        memset(value, 0, valueSize);

		clGetDeviceInfo(g_device, CL_DEVICE_NAME, valueSize, value, NULL);
		my_printf("Device Name: %s\n", value);
		my_free(value);
		// print parallel compute units(CU)
		clGetDeviceInfo(g_device, CL_DEVICE_MAX_COMPUTE_UNITS,sizeof(maxComputeUnits), &maxComputeUnits, NULL);
		my_printf("Parallel compute units: %u\n", maxComputeUnits);

		// maxWorkItemDim
		clGetDeviceInfo(g_device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,sizeof(maxWorkItemDim), &maxWorkItemDim, NULL);
		my_printf("maxWorkItemDim: %u\n", maxWorkItemDim);

		MaxItemSize = (size_t *)my_malloc(maxWorkItemDim);
        memset(MaxItemSize, 0, maxWorkItemDim);

		clGetDeviceInfo(g_device,CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(size_t) * maxWorkItemDim,(void *)MaxItemSize,NULL);
		for (i = 0; i < maxWorkItemDim; ++i)
		{
			my_printf("dim: %d maxItemSize: %d\n", i, MaxItemSize[i]);
		}

		//maxWorkItemPerGroup
		clGetDeviceInfo(g_device, CL_DEVICE_MAX_WORK_GROUP_SIZE,sizeof(maxWorkItemPerGroup), &maxWorkItemPerGroup, NULL);
		my_printf("maxWorkItemPerGroup: %d\n", maxWorkItemPerGroup);

		// print maxGlobalMemSize
		clGetDeviceInfo(g_device, CL_DEVICE_GLOBAL_MEM_SIZE,sizeof(maxGlobalMemSize), &maxGlobalMemSize, NULL);
		my_printf("maxGlobalMemSize: %lu(MB)\n", maxGlobalMemSize/1024/1024);

		// print maxConstantBufferSize
		clGetDeviceInfo(g_device, CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,sizeof(maxConstantBufferSize), &maxConstantBufferSize, NULL);
		my_printf("maxConstantBufferSize: %lu(KB)\n", maxConstantBufferSize/1024);

		// print maxLocalMemSize
		clGetDeviceInfo(g_device, CL_DEVICE_LOCAL_MEM_SIZE,sizeof(maxLocalMemSize), &maxLocalMemSize, NULL);
		my_printf("maxLocalMemSize: %lu(KB)\n", maxLocalMemSize/1024);
	}

	g_context = clCreateContext( NULL, 1, &g_device, NULL, NULL, &nErroNum);
	checkErr(nErroNum , "clCreateContext");

	g_queue = clCreateCommandQueue( g_context, g_device, CL_QUEUE_PROFILING_ENABLE, &nErroNum );
	checkErr(nErroNum , "clCreateCommandQueue");

	pFileStream = my_fopen(FCWS_GPU_KERNEL_BFILE_PATH, "rb");
	if(pFileStream)
	{
		my_fclose(pFileStream);
		binary = readBinaryFromFile(FCWS_GPU_KERNEL_BFILE_PATH, &szKernelLength);
		if(!binary)
		{
			my_printf("Failed to load binary file \n");
			exit(0);
		}

		g_program = clCreateProgramWithBinary(g_context,1,&g_device, (const size_t *)&szKernelLength,
													(const unsigned char**)&binary,NULL,NULL);

		my_free(binary);

		status = clBuildProgram( g_program, 1, &g_device, options, NULL, NULL );
		if(status != 0)
		{
			char	tbuf[0x1000];
			my_printf("clBuild failed:%d\n", status);
			clGetProgramBuildInfo(g_program, g_device, CL_PROGRAM_BUILD_LOG, 0x10000, tbuf, NULL);
			my_printf("\n%s\n", tbuf);
			return ;
		}
	}
	else
	{
		kernelSourceCode = LoadProgSource(FCWS_GPU_KERNEL_CFILE_PATH, "", &szKernelLength);
		if(kernelSourceCode != NULL)
		{
			g_program  = clCreateProgramWithSource(g_context, 1, &kernelSourceCode, &szKernelLength,&nErroNum );
			checkErr( nErroNum , "clCreateProgramWithSource");

			my_free(kernelSourceCode);

			status = clBuildProgram(g_program,1,&g_device,options,NULL,NULL);

			if(status != 0)
			{
				char	tbuf[0x1000];
				my_printf("clBuild failed:%d\n", status);
				clGetProgramBuildInfo(g_program, g_device, CL_PROGRAM_BUILD_LOG, 0x10000, tbuf, NULL);
				my_printf("\n%s\n", tbuf);
				return ;
			}

			binaries = (char **)my_malloc( sizeof(char *) * 1 );
            memset(binaries, 0, sizeof(char *) * 1);
			binarySizes = (size_t*)my_malloc( sizeof(size_t) * 1 );
            memset(binarySizes, 0, sizeof(size_t) * 1);

			status = clGetProgramInfo(g_program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t) * 1, binarySizes, NULL);

			binaries[0] = (char *)my_malloc( sizeof(char) * binarySizes[0]);
            memset(binaries[0], 0, sizeof(char) * binarySizes[0]);

			status = clGetProgramInfo(g_program, CL_PROGRAM_BINARIES,sizeof(char *) * 1, binaries, NULL);

			writeBinaryToFile(FCWS_GPU_KERNEL_BFILE_PATH, binaries[0],binarySizes[0]);

			my_free(binaries);
			my_free(binarySizes);
		}
	}

	g_nMAX_Det_Num = MAX_DET_NUM;

	g_c_Detbuf = clCreateBuffer(g_context, CL_MEM_WRITE_ONLY, MAX_DET_NUM * sizeof(lbp_rect_car),NULL,&nErroNum);

	rlbp.data.feature_width		= pVehicleDetor->pdetorModel->l->data.feature_width;
	rlbp.data.feature_height	= pVehicleDetor->pdetorModel->l->data.feature_height;
	rlbp.data.num_stages		= pVehicleDetor->pdetorModel->l->data.num_stages;
	rlbp.data.num_rects			= pVehicleDetor->pdetorModel->l->data.num_rects;
	rlbp.para					= pVehicleDetor->pdetorModel->l->para;

	pTr = pVehicleDetor->pGpuBuff;
	stage = (reduce_stage *)pTr;
	pTr = ALIGN_16BYTE(stage + pVehicleDetor->pdetorModel->l->data.num_stages);
	for ( i = 0; i < pVehicleDetor->pdetorModel->l->data.num_stages; i++ )
	{
		stage[i].stage_threshold		= pVehicleDetor->pdetorModel->l->data.s[i].stage_threshold;
		stage[i].num_weak_classifiers	= pVehicleDetor->pdetorModel->l->data.s[i].num_weak_classifiers;
	}

	classifiers = (weak_classifier_car *)pTr;
	pTr = ALIGN_16BYTE(classifiers + pVehicleDetor->pdetorModel->l->data.num_stages * WEAK_CLASSIFER_NUM);
	for ( i = 0; i < pVehicleDetor->pdetorModel->l->data.num_stages; i++ )
	{
		ptr = classifiers + i * WEAK_CLASSIFER_NUM;
		memcpy(ptr,pVehicleDetor->pdetorModel->l->data.s[i].classifiers,pVehicleDetor->pdetorModel->l->data.s[i].num_weak_classifiers * sizeof(weak_classifier_car) );
	}

	if(pVehicleDetor->pdetorModel->l->data.s[pVehicleDetor->pdetorModel->l->data.num_stages - 1].num_weak_classifiers >= WEAK_CLASSIFER_NUM)
	{
		my_printf("error_____________\n");
		exit(0);
	}

	pRec = (lbp_rect_car*)pTr;
	pTr = ALIGN_16BYTE(pRec  +  pVehicleDetor->pdetorModel->l->data.num_rects );
	memcpy(pRec,pVehicleDetor->pdetorModel->l->data.r,sizeof(lbp_rect_car) * pVehicleDetor->pdetorModel->l->data.num_rects);

	 c_stagebuf = clCreateBuffer(g_context, CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,
		pVehicleDetor->pdetorModel->l->data.num_stages * sizeof( reduce_stage),stage,&nErroNum);
	checkErr(nErroNum, "c_stagebuf_clCreateBuffer");

	 c_weak_classifierbuf = clCreateBuffer(g_context, CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,
		pVehicleDetor->pdetorModel->l->data.num_stages * WEAK_CLASSIFER_NUM * sizeof( weak_classifier_car),classifiers,&nErroNum);
	checkErr(nErroNum, "c_classifierbuf_clCreateBuffer");

	 c_lbp_rectbuf = clCreateBuffer(g_context, CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,
		pVehicleDetor->pdetorModel->l->data.num_rects * sizeof(lbp_rect_car),pRec,&nErroNum);
	checkErr(nErroNum, "c_feature_clCreateBuffer");

	if( 0 == g_gpustate )
	{
		lbp_detect_kernel = clCreateKernel(g_program,"feature_detect_window_unLocal",NULL);
	}
	else
	{
		lbp_detect_kernel = clCreateKernel(g_program,"feature_detect_window",NULL);
	}

	nErroNum =clSetKernelArg(lbp_detect_kernel, 1, sizeof(cl_mem),  (void *)&c_stagebuf);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 2, sizeof(cl_mem),  (void *)&c_weak_classifierbuf);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 3, sizeof(cl_mem),  (void *)&c_lbp_rectbuf);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 9, sizeof(s32),  (void *)&g_nMAX_Det_Num);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 10, sizeof(cl_mem),  (void *)&g_c_Detbuf);
	checkErr(nErroNum, "clSetKernelArg");

	localws_score[0] = LOCKA_X_NUM;
	localws_score[1] = LOCKA_Y_NUM;
}

void Gpu_object_detector_lbp_detect_window( lbp_car *l, u32 *img, float32_t factor)
{
	s32 i, nInitDetNum = 0;
	lbp_task_car *pTasks = NULL;
	cl_int nErroNum;
	cl_mem c_lbpbuf, c_Tasksbuf, c_imgbuf, c_DetNumbuf;

	cl_event ev;
	s32 nFinalDetNum = 0 ;

	if ( !l->ntaskNum )
	{
		return;
	}

	pTasks		= l->ptasks;
	rlbp.width	= l->width;
	rlbp.height	= l->height;

	 c_lbpbuf = clCreateBuffer(g_context, CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,
		                  sizeof( reduce_lbp),&rlbp,&nErroNum);
	checkErr(nErroNum, "c_lbpbuf_clCreateBuffer");

	 c_Tasksbuf = clCreateBuffer(g_context, CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,
		l->ntaskNum * sizeof( lbp_task_car),pTasks,&nErroNum);
	checkErr(nErroNum, "c_Tasksbuf_clCreateBuffer");

	c_imgbuf = clCreateBuffer(g_context, CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,
		                 l->width * l->height * sizeof(u32),(img),&nErroNum);
	checkErr(nErroNum, "c_imgbuf_clCreateBuffer");

	c_DetNumbuf = clCreateBuffer(g_context, CL_MEM_WRITE_ONLY|CL_MEM_COPY_HOST_PTR,
		                sizeof(s32),&nInitDetNum,&nErroNum);
	checkErr(nErroNum, "c_DetNumbuf_clCreateBuffer");

	nErroNum = clSetKernelArg(lbp_detect_kernel, 0, sizeof(cl_mem),  (void *)&c_lbpbuf);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 4, sizeof(cl_mem),  (void *)&c_Tasksbuf);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 5, sizeof(cl_mem),  (void *)&c_imgbuf);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 6, sizeof(s32),  (void *)&l->width);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 7, sizeof(s32),  (void *)&l->height);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 8, sizeof(s32),  (void *)&l->ntaskNum);
	nErroNum |=clSetKernelArg(lbp_detect_kernel, 11, sizeof(cl_mem),  (void *)&c_DetNumbuf);
	checkErr(nErroNum, "clSetKernelArg");

	//Assume divisible by 16
	globalws_score[0] = (Ceil_car(sqrtf(l->ntaskNum)) + (LOCKA_X_NUM-1))/LOCKA_X_NUM * LOCKA_X_NUM;
	globalws_score[1] = (Ceil_car(sqrtf(l->ntaskNum)) + (LOCKA_Y_NUM-1))/LOCKA_Y_NUM * LOCKA_Y_NUM;

	nErroNum = clEnqueueNDRangeKernel( g_queue,lbp_detect_kernel, 2, 0,
	                                    globalws_score, localws_score, 0, NULL, &ev);
	checkErr(nErroNum, "clEnqueueNDRangeKernel");

	clEnqueueReadBuffer(g_queue,c_DetNumbuf,CL_TRUE,0, sizeof(int),&nFinalDetNum,0,NULL,NULL);
	if (nFinalDetNum)
	{
		lbp_rect_car *pDetRec = NULL;
		lbp_rect_car r;
		pDetRec = (lbp_rect_car*)my_malloc(nFinalDetNum * sizeof(lbp_rect_car));
        memset(pDetRec, 0, nFinalDetNum * sizeof(lbp_rect_car));

		clEnqueueReadBuffer(g_queue,g_c_Detbuf,CL_TRUE,0,nFinalDetNum * sizeof(lbp_rect_car),pDetRec,0,NULL,NULL);

		for (i = 0 ; i < nFinalDetNum; i++)
		{
			r.x = (s32)( pDetRec[i].x * factor );
			r.y = (s32)( pDetRec[i].y * factor );
			r.width  = (s32)( pDetRec[i].width * factor );
			r.height = (s32)( pDetRec[i].height * factor );
			l->pdetected_r[l->ndetdNum++] = r;
		}

		if (pDetRec != NULL)
		{
			my_free(pDetRec);
		}
	}
	clReleaseMemObject(c_lbpbuf);
	clReleaseMemObject(c_Tasksbuf);
	clReleaseMemObject(c_imgbuf);
	clReleaseMemObject(c_DetNumbuf);
}

#endif
