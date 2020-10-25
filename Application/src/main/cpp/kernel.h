//
// Created by Allan Suen on 11/7/2016.
//
#ifndef KERNEL_H
#define KERNEL_H

#include <android/asset_manager.h>

#include <CL/cl.h>

extern AAssetManager *_asset_manager;

typedef struct _kernel_control {
    // each kernel should either have a filename (which gets loaded to ksrc/klen)
    //  or the kernel source text file has already been included in a resident memory
    //   array (ksrc) of size klen
    char*           name;       // Name of the kernel
    char*           filename;   // File contains the program source
    char*           coption;    // Compiler Options
    cl_program      program;    // program that contains the kernel source
    cl_kernel       kernel;     // the kernel compiled
} kernel_control;

bool CreateKernelEnvironment();
bool ReleaseKernelEnvironment();
bool CreateKernel(kernel_control* ctrl);
bool ReleaseKernel(kernel_control* ctrl);
cl_context GetContext();
cl_device_id GetDeviceId();
cl_command_queue GetCommandQueue();
cl_command_queue GetReadWriteQueue();
cl_kernel DuplicateKernel(kernel_control* ctrl);

#endif //KERNEL_H
