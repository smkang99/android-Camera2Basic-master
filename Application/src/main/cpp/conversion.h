//
// Created by Allan Suen on 12/2/2016.
//
#ifndef CONVERSION_H
#define CONVERSION_H

#include "kernel.h"

bool CreateRaw2YuvKernel(kernel_control* ctrl, int width, int height);
cl_int runRaw2YuvKernel(jbyte* rawImage1, jbyte* rawImage2, jbyte* rgbOut1, jbyte* rgbOut2);
void ReleaseRaw2YuvKernel(kernel_control* ctrl);

#endif //CONVERSION_H
