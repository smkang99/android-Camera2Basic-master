//
// Created by Allan Suen on 11/7/2016.
//
#include <android/log.h>
#include <malloc.h>

#include "kernel.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "kernel", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "kernel", __VA_ARGS__))

static char const* const COMPILER_OPTIONS = "-w";

cl_context _context = NULL;
cl_device_id _device_id = NULL;
cl_command_queue _command_queue = NULL;
cl_command_queue _readwrite_queue = NULL;

cl_platform_id GetPlatformId();
cl_device_id GetDeviceId(cl_platform_id platform_id);
cl_context CreateContext(cl_platform_id platform_id);
cl_command_queue CreateCommandQueue(cl_context context, cl_device_id device_id);
cl_command_queue CreateReadWriteQueue(cl_context context, cl_device_id device_id);
char* ReadFileAsArray(const char* const filename, size_t* size);

bool CreateKernelEnvironment() {
    cl_platform_id platform_id = GetPlatformId();
    if (platform_id == NULL) {
        LOGE("Get platform ID failed\n");
        return false;
    }
    _context = CreateContext(platform_id);
    if (_context == NULL) {
        LOGE("Create context failed\n");
        return false;
    }
    _device_id = GetDeviceId(platform_id);
    if (_device_id == NULL) {
        LOGE("Get device ID failed\n");
        return false;
    }
    _command_queue = CreateCommandQueue(_context, _device_id);
    if (_command_queue == NULL) {
        LOGE("Create command queue failed\n");
        return false;
    }

    //    _readwrite_queue = CreateCommandQueue(_context, _device_id);
    _readwrite_queue = CreateReadWriteQueue(_context, _device_id);    
    if (_readwrite_queue == NULL) {
        LOGE("Create read write queue failed\n");
        return false;
    }

    return true;
}

bool ReleaseKernelEnvironment() {
    if (clReleaseCommandQueue(_readwrite_queue) != CL_SUCCESS) {
        LOGE("Release read write queue failed\n");
        return false;
    }

    if (clReleaseCommandQueue(_command_queue) != CL_SUCCESS) {
        LOGE("Release command queue failed\n");
        return false;
    }
    if (clReleaseDevice(_device_id) != CL_SUCCESS) {
        LOGE("Release device ID failed\n");
        return false;
    }
    if (clReleaseContext(_context) != CL_SUCCESS) {
        LOGE("Release context failed\n");
        return false;
    }
    return true;
}

bool CreateKernel(kernel_control* ctrl) {
    cl_int err = CL_SUCCESS;
    if (_context == NULL) {
        LOGE("Context is missing\n");
        return false;
    }
    if (ctrl->filename == NULL) {
        LOGE("Kernel filename is missing\n");
        return false;
    }
    if (ctrl->name == NULL) {
        LOGE("Kernel name is missing\n");
    }
    if (ctrl->coption == NULL) {
        ctrl->coption = (char *)COMPILER_OPTIONS;
    }
    

    // Read file into source
    size_t length = 0;
    char* source = ReadFileAsArray(ctrl->filename, &length);
    if (source == NULL) {
        LOGE("Read kernel file failed\n");
        return false;
    }

    // Build program from source
    ctrl->program = clCreateProgramWithSource(_context, 1, (const char**)&source, &length, &err);
    free(source);
    if (ctrl->program == NULL) {
        LOGE("Create program failed\n");
        return false;
    }
//    if (clBuildProgram(ctrl->program, 1, &_device_id, COMPILER_OPTIONS, NULL, NULL) != CL_SUCCESS) {
    if (clBuildProgram(ctrl->program, 1, &_device_id, ctrl->coption, NULL, NULL) != CL_SUCCESS) {    
        LOGE("Build program failed\n");
        return false;
    }

    // Build kernel
    ctrl->kernel = clCreateKernel(ctrl->program, ctrl->name, &err);
    if (ctrl->kernel == NULL) {
        LOGE("Build kernel failed\n");
        return false;
    }
    return true;
}

cl_kernel DuplicateKernel(kernel_control* ctrl) {
    cl_int err = CL_SUCCESS;
    cl_kernel temp;
    if (ctrl->kernel == NULL) {
        LOGE("Kernel is missing\n");
    }

    // Build kernel
    temp = clCreateKernel(ctrl->program, ctrl->name, &err);
    if (temp == NULL) {
        LOGE("Build kernel failed\n");
    }
    return temp;
}

bool ReleaseKernel(kernel_control* ctrl) {
    if (clReleaseKernel(ctrl->kernel) != CL_SUCCESS) {
        LOGE("Release kernel failed\n");
        return false;
    }
    if (clReleaseProgram(ctrl->program) != CL_SUCCESS) {
        LOGE("Release program failed\n");
        return false;
    }
    return true;
}

cl_context GetContext() {
    return _context;
}

cl_device_id GetDeviceId() {
    return _device_id;
}

cl_command_queue GetCommandQueue() {
    return _command_queue;
}

cl_command_queue GetReadWriteQueue() {
    return _readwrite_queue;
}

cl_platform_id GetPlatformId() {
    char buffer[256];

    // Use default OpenCL platform
    cl_platform_id platform_id;
    if (clGetPlatformIDs(1, &platform_id, NULL) != CL_SUCCESS) {
        LOGE("Get platform ID failed\n");
        return NULL;
    }
    if (clGetPlatformInfo(platform_id,
                          CL_PLATFORM_NAME,
                          sizeof(buffer),
                          buffer,
                          NULL) == CL_SUCCESS) {
        LOGI("OpenCL Platform Name: %s\n", buffer);
    }
    return platform_id;
}

cl_device_id GetDeviceId(cl_platform_id platform_id) {
    char buffer[256];

    cl_device_id device_id;
    if (clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_ALL, 1, &device_id, NULL) != CL_SUCCESS) {
        LOGE("Get device ID failed\n");
        return NULL;
    }
    if (clGetDeviceInfo(device_id, CL_DEVICE_VENDOR, sizeof(buffer), buffer, NULL) == CL_SUCCESS) {
        LOGI("OpenCL Device Vendor: %s\n", buffer);
    }
    if (clGetDeviceInfo(device_id, CL_DEVICE_NAME, sizeof(buffer), buffer, NULL) == CL_SUCCESS) {
        LOGI("OpenCL Device Name: %s\n", buffer);
    }
    return device_id;
}

cl_context CreateContext(cl_platform_id platform_id) {
    cl_context_properties cps[] = {
            CL_CONTEXT_PLATFORM,
            (cl_context_properties)platform_id,
            0
    };
    return clCreateContextFromType(cps, CL_DEVICE_TYPE_ALL, NULL, NULL, NULL);
}

cl_command_queue CreateCommandQueue(cl_context context, cl_device_id device_id) {
    cl_queue_properties props[] = {CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0};

  //    return _command_queue = clCreateCommandQueueWithProperties(context, device_id, 0, NULL);
    return clCreateCommandQueueWithProperties(context, device_id, props, NULL);  
}

cl_command_queue CreateReadWriteQueue(cl_context context, cl_device_id device_id) {
    cl_queue_properties props[] = {CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0};
  //    return _readwrite_queue = clCreateCommandQueueWithProperties(context, device_id, 0, NULL);
    return clCreateCommandQueueWithProperties(context, device_id, props, NULL);  
}

char* ReadFileAsArray(const char* const filename, size_t* size)
{
    AAsset* asset = AAssetManager_open(_asset_manager, filename, AASSET_MODE_UNKNOWN);
    if (asset == NULL) {
        LOGE("Open asset manager failed\n");
        return NULL;
    }

    size_t length = AAsset_getLength(asset);
    char* content = (char*)malloc(length);
    if (AAsset_read(asset, content, length) != length) {
        LOGE("Asset data missing from read\n");
        free(content);
        return NULL;
    }

    AAsset_close(asset);
    *size = length;
    return content;
}
