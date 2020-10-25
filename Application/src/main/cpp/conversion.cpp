//
// Created by Allan Suen on 12/2/2016.
//
#include <android/log.h>
#include <jni.h>

#include "kernel.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "conversion", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "conversion", __VA_ARGS__))

// Kernel names
char _kname_raw2yuv[] = "RAW2Y";

// Kernel filenames
char _kfname_raw2yuv[] = "raw2y.cl";

cl_kernel kernel1, kernel2;
cl_mem d_rawInput1, d_rawInput2;
cl_mem d_rgbOut1, d_rgbOut2;
size_t d_rawInput_sz;
size_t d_rgbOut_sz;
cl_int err;
size_t globalWorkItems;
size_t localWorkItems;

// Setup the kernel and associated arguements. Only the fixed arguements will be created here.
bool CreateRaw2YuvKernel(kernel_control* ctrl, int width, int height) {
    bool succeed = false;
    err = CL_SUCCESS;
    ctrl->name = _kname_raw2yuv;        // Kernel name
    ctrl->filename = _kfname_raw2yuv;   // Kernel file name
    succeed = CreateKernel(ctrl);
    if (succeed) {  // Setup arguements
      //        d_rawInput_sz = width*height*sizeof(jbyte)*2;
        d_rawInput_sz = width*height*sizeof(jbyte)*10/8;      
        d_rgbOut_sz = 4*width*height*sizeof(jbyte);

        kernel1 = ctrl->kernel;
        kernel2 = DuplicateKernel(ctrl);

        d_rawInput1=clCreateBuffer(GetContext(),CL_MEM_READ_ONLY, d_rawInput_sz,
                                  NULL, &err);
        if (err < 0) return err;

        d_rawInput2=clCreateBuffer(GetContext(),CL_MEM_READ_ONLY, d_rawInput_sz,
                                   NULL, &err);
        if (err < 0) return err;

        err=clSetKernelArg(kernel1,0,sizeof(cl_mem),((void *)(&d_rawInput1)));
        if (err < 0) return err;
        err=clSetKernelArg(kernel2,0,sizeof(cl_mem),((void *)(&d_rawInput2)));
        if (err < 0) return err;

        d_rgbOut1=clCreateBuffer(GetContext(),CL_MEM_WRITE_ONLY, d_rgbOut_sz,
                                NULL, &err);
        if (err < 0) return err;
        d_rgbOut2=clCreateBuffer(GetContext(),CL_MEM_WRITE_ONLY, d_rgbOut_sz,
                                NULL, &err);
        if (err < 0) return err;

        err=clSetKernelArg(kernel1,1,sizeof(cl_mem),((void *)(&d_rgbOut1)));
        if (err < 0) return err;
        err=clSetKernelArg(kernel2,1,sizeof(cl_mem),((void *)(&d_rgbOut2)));
        if (err < 0) return err;

	globalWorkItems = width*height/4;
	localWorkItems = 16;
	
	//        globalWorkItems = width*height;
	//        localWorkItems = 64;
    }
    return succeed;
}

// Create run time arguements and run the kernel
cl_int runRaw2YuvKernel(jbyte* rawImage1, jbyte* rawImage2, jbyte* rgbOut1, jbyte* rgbOut2) {

    err = clEnqueueNDRangeKernel(GetCommandQueue(), kernel2, 1, 0,
                                 &globalWorkItems, &localWorkItems,NULL,NULL,NULL);

    if (err < 0) return err;

    err = clEnqueueReadBuffer(GetReadWriteQueue(), d_rgbOut1, CL_FALSE, 0, d_rgbOut_sz, rgbOut1, 0, NULL, NULL);
    if(err < 0) return err;

    err = clEnqueueWriteBuffer(GetReadWriteQueue(), d_rawInput1, CL_FALSE, 0, d_rawInput_sz, rawImage1, NULL, NULL, NULL);
    if (err < 0) return err;

    clFinish((GetReadWriteQueue()));    // Make sure the read and write is already done
    clFinish(GetCommandQueue());    // Wait for kernel2 to finish
    err = clEnqueueNDRangeKernel(GetCommandQueue(), kernel1, 1, 0,
				 //                                 &globalWorkItems, &localWorkItems,NULL,NULL,NULL);
                                 &globalWorkItems, NULL,NULL,NULL,NULL);				 
    if (err < 0) return err;


    err = clEnqueueReadBuffer(GetReadWriteQueue(), d_rgbOut2, CL_FALSE, 0, d_rgbOut_sz, rgbOut2, 0, NULL, NULL);
    if(err < 0) return err;

    err = clEnqueueWriteBuffer(GetReadWriteQueue(), d_rawInput2, CL_FALSE, 0, d_rawInput_sz, rawImage2, NULL, NULL, NULL);
    if (err < 0) return err;

    clFinish((GetReadWriteQueue()));    // Make sure the read and write is already done
    clFinish(GetCommandQueue());    // Wait for kernel1 to finish
    return err;
}

// Release all the memory objects related to this kernel
void ReleaseRaw2YuvKernel(kernel_control* ctrl) {
    ReleaseKernel(ctrl);
    clReleaseKernel(kernel2);
    clReleaseMemObject(d_rawInput1);
    clReleaseMemObject(d_rawInput2);
    clReleaseMemObject(d_rgbOut1);
    clReleaseMemObject(d_rgbOut2);
}
