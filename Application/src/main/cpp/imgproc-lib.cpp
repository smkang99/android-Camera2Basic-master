//
// Created by Allan Suen on 10/26/2016.
//
#include <jni.h>
#include <android/log.h>
#include <android/asset_manager_jni.h>
#include <cstring>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "types.h"
#include "conversion.h"
#include "hassproc.h"

HassProc HassProc;

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "imgproc-lib", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "imgproc-lib", __VA_ARGS__))

#define RAW_WIDTH  1952
#define RAW_HEIGHT 2346

AAssetManager* _asset_manager;
kernel_control _kernel_controls;

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_CameraActivity_sendAssets(JNIEnv *env, jobject obj, jobject mgr) {
    _asset_manager = AAssetManager_fromJava(env, mgr);  // Transfer asset to global variable
    if (_asset_manager == NULL) {
        LOGE("Send assets failed\n");
        return JNI_FALSE;
    }
    return JNI_TRUE;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_CameraActivity_createImgprocKernels(JNIEnv* env, jobject obj) {
    CreateKernelEnvironment();

    //CreateRaw2YuvKernel(&_kernel_controls, RAW_WIDTH, RAW_HEIGHT);
    HassProc.CreateHassKernels();

    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_CameraActivity_releaseImgprocKernels(JNIEnv* env, jobject obj) {

    //ReleaseRaw2YuvKernel(&_kernel_controls);
    HassProc.ReleaseHassKernels();

    ReleaseKernelEnvironment();
    return 0;
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_Raw2Yuv(JNIEnv *env, jobject obj, jbyteArray raw, jint width, jint height){
    jint frameSize = width * height;
    jint arraySize = frameSize * sizeof(u_short);
    jbyteArray raw_result = env->NewByteArray(arraySize);
    jbyte* input = env->GetByteArrayElements(raw, NULL);
    jbyte* output = env->GetByteArrayElements(raw_result, NULL);

    cl_int err = CL_SUCCESS;
    cl_context context = GetContext();
    cl_command_queue command_queue = GetCommandQueue();

    LOGI("Create Buffers...\n") ;
    cl_mem rawD = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, arraySize, input, &err);
    cl_mem yD   = clCreateBuffer(context, CL_MEM_WRITE_ONLY, arraySize, NULL, &err);

    LOGI("Set Arguments...\n") ;
    err = clSetKernelArg(_kernel_controls.kernel, 0, sizeof(rawD), &rawD);
    err = clSetKernelArg(_kernel_controls.kernel, 1, sizeof(yD), &yD);

    LOGI("Run Kernel...\n") ;
    size_t const GLOBAL_WORK_ITEMS = frameSize;

    err = clEnqueueNDRangeKernel(command_queue, _kernel_controls.kernel, 1, NULL, &GLOBAL_WORK_ITEMS, NULL, 0, NULL, NULL);

    LOGI("Fill Chroma Region with 128...\n") ;
    unsigned char const chroma_value = 128;
    err = clEnqueueFillBuffer(command_queue, yD, &chroma_value, sizeof(unsigned char), frameSize, frameSize, 0, NULL, NULL);
    if(err != CL_SUCCESS) {
        LOGE("Couldn't fill a buffer object");
    }

    LOGI("Read Buffer...\n") ;
    err = clEnqueueReadBuffer(command_queue, yD, CL_TRUE, 0, arraySize, output, 0, NULL, NULL);

    LOGI("Release...\n") ;
    err = clReleaseMemObject(rawD);
    err = clReleaseMemObject(yD);

    env->ReleaseByteArrayElements(raw, input, JNI_ABORT);
    env->ReleaseByteArrayElements(raw_result, output, 0);

    return raw_result;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_openclYuv2Rgb(JNIEnv *env, jobject obj,
                                                           jbyteArray rawImage1,
                                                           jbyteArray rawImage2,
                                                           jbyteArray rgbOut1,
                                                           jbyteArray rgbOut2){
    jbyte* rawImageC1 = env->GetByteArrayElements(rawImage1, 0);
    jbyte* rawImageC2 = env->GetByteArrayElements(rawImage2, 0);
    jbyte* rgbOutC1 = env->GetByteArrayElements(rgbOut1, 0);
    jbyte* rgbOutC2 = env->GetByteArrayElements(rgbOut2, 0);

    runRaw2YuvKernel(rawImageC1, rawImageC2, rgbOutC1, rgbOutC2);

    env->ReleaseByteArrayElements(rgbOut1, rgbOutC1, JNI_COMMIT);
    env->ReleaseByteArrayElements(rgbOut2, rgbOutC2, JNI_COMMIT);
    env->ReleaseByteArrayElements(rawImage1, rawImageC1, JNI_ABORT);
    env->ReleaseByteArrayElements(rawImage2, rawImageC2, JNI_ABORT);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_openclHassProcDB(JNIEnv *env, jobject obj,
                                                              jobject rawImage1, jobject rawImage2,
                                                              jobject rgbOut1, jobject rgbOut2){
    void* rawImage1Ptr = env->GetDirectBufferAddress(rawImage1);
    void* rawImage2Ptr = env->GetDirectBufferAddress(rawImage2);
    void* rgbOut1Ptr = env->GetDirectBufferAddress(rgbOut1);
    void* rgbOut2Ptr = env->GetDirectBufferAddress(rgbOut2);

    HassProc.RunHassKernelsDB(rawImage1Ptr, rawImage2Ptr, rgbOut1Ptr, rgbOut2Ptr);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_openclHassProc(JNIEnv *env, jobject obj,
                                                            jobject rawImage,
                                                            jobject rgbOut){
    void* rawImagePtr = env->GetDirectBufferAddress(rawImage);
    void* rgbOutPtr = env->GetDirectBufferAddress(rgbOut);

    HassProc.RunHassKernelsSB(rawImagePtr, rgbOutPtr);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_openclHassWdrRawProc(JNIEnv *env, jobject obj,
                                                            jobject rawImage, jobject rgbOut, jobject dispOut, int fnum, int displayoff){
    void* rawImagePtr = env->GetDirectBufferAddress(rawImage);
    void* rgbOutPtr = env->GetDirectBufferAddress(rgbOut);
    void* dispOutPtr = env->GetDirectBufferAddress(dispOut);

    HassProc.RunHassWdrRawKernelsSB(rawImagePtr, rgbOutPtr, dispOutPtr, fnum, displayoff);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_HassModeSPC(JNIEnv* env, jobject obj, int s) {
    HassProc.SetHassModeSPC(s);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_HassModeRGB(JNIEnv* env, jobject obj) {
    HassProc.SetHassModeRGB();
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_HassModeMULT(JNIEnv* env, jobject obj) {
    HassProc.SetHassModeMULT();
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_HassModeNDVI(JNIEnv* env, jobject obj) {
    HassProc.SetHassModeNDVI();
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassNRTap(JNIEnv* env, jobject obj, int tap) {
    HassProc.SetHassNRTap(tap);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_GetHassNRTap(JNIEnv* env, jobject obj) {
    return HassProc.GetHassNRTap();
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassGamma(JNIEnv* env, jobject obj, float gamma) {
    HassProc.SetHassGamma(gamma);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassAutoExposureSpeedLong(JNIEnv* env, jobject obj, int AutoExposureSpeedLong) {
    HassProc.SetHassAutoExposureSpeedLong(AutoExposureSpeedLong);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassAutoExposureSpeedShort(JNIEnv* env, jobject obj, int AutoExposureSpeedShort) {
    HassProc.SetHassAutoExposureSpeedShort(AutoExposureSpeedShort);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassAutoExposureTarget(JNIEnv* env, jobject obj, int AutoExposureTarget) {
    HassProc.SetHassAutoExposureTarget(AutoExposureTarget);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassDisplayLsb(JNIEnv* env, jobject obj, int HassDisplayLsb) {
    HassProc.SetHassDisplayLsb(HassDisplayLsb);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassAverageGain(JNIEnv* env, jobject obj, float tmp_ave_gain) {
    HassProc.SetHassAverageGain(tmp_ave_gain);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassAutoExposureRange(JNIEnv* env, jobject obj, int AutoExposureRange) {
    HassProc.SetHassAutoExposureRange(AutoExposureRange);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_SetHassGammaTable(JNIEnv* env, jobject obj, jbyteArray tbl) {

    jbyte* gamma_tbl = env->GetByteArrayElements(tbl, 0);
    HassProc.SetHassGammaTable(gamma_tbl);
    env->ReleaseByteArrayElements(tbl, gamma_tbl, JNI_ABORT);
    return 0;
}

extern "C"
JNIEXPORT float JNICALL
Java_com_example_android_camera2basic_GetHassGamma(JNIEnv* env, jobject obj) {
    return HassProc.GetHassGamma();
}

extern "C"
JNIEXPORT float JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_GetHassAutoExposureSpeedLong(JNIEnv* env, jobject obj) {
    return HassProc.GetHassAutoExposureSpeedLong();
}

extern "C"
JNIEXPORT float JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_GetHassAutoExposureSpeedShort(JNIEnv* env, jobject obj) {
    return HassProc.GetHassAutoExposureSpeedShort();
}

extern "C"
JNIEXPORT jintArray JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_GetHassImageConfiguration(JNIEnv* env, jobject obj) {
    //int rep, tap, size, ch, sp;
    int configCount = 5;
    int config[configCount];
    HassProc.GetHassImageConfiguration(&config[0], &config[1], &config[2], &config[3], &config[4]);
    jintArray configuration = env->NewIntArray(configCount);
    env->SetIntArrayRegion(configuration, 0, configCount, config);
    return configuration;
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_SetHassImageConfiguration(JNIEnv* env, jobject obj,
                                                                       jintArray configuration) {
    //int rep, tap, size, ch, sp;
    int configCount = 5;
    int config[configCount];
    env->GetIntArrayRegion(configuration, 0, configCount, config);
    HassProc.SetHassImageConfiguration(config[0], config[1], config[2], config[3], config[4]);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_SetHassImageOffset(JNIEnv* env, jobject obj,
                                                                int dx, int dy) {
    HassProc.SetHassImageOffset(dx,dy);
    return 0;
}

extern "C"
JNIEXPORT jobjectArray JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_GetHassFilterStructure(JNIEnv* env, jobject obj) {
    int rows = 8;
    int columns = 8;
    jclass intArrayClass = env->FindClass("[I");
    jobjectArray filterArray = env->NewObjectArray(rows, intArrayClass, NULL);
    for (int i = 0; i < rows; i++) {
        jintArray row = env->NewIntArray(columns);
        env->SetIntArrayRegion(row, 0, columns, HassProc.hass_filter_no[i]);
        env->SetObjectArrayElement(filterArray, i, row);
    }
    return filterArray;
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_SetHassFilterStructure(JNIEnv* env, jobject obj,
                                                                    jobjectArray filterStruct) {
    int size = env->GetArrayLength(filterStruct);
    for (int i = 0; i < size; i++) {
        jintArray row = (jintArray)env->GetObjectArrayElement(filterStruct, i);
        env->GetIntArrayRegion(row, 0, size, HassProc.hass_filter_no[i]);
    }
    //HassProc.SetHassFilterStructure(filterStructPtr);
    return 0;
}

extern "C"
JNIEXPORT jobjectArray JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_GetHassFilterMatrix(JNIEnv* env, jobject obj) {
    int rows = 67;
    int columns = 16;
    jclass floatArrayClass = env->FindClass("[F");
    jobjectArray filterMatrix = env->NewObjectArray(rows, floatArrayClass, NULL);
    for (int i = 0; i < rows; i++) {
        jfloatArray row = env->NewFloatArray(columns);
        env->SetFloatArrayRegion(row, 0, columns, HassProc.LutHassLambda[i]);
        env->SetObjectArrayElement(filterMatrix, i, row);
    }
    return filterMatrix;
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_SetHassFilterMatrix(JNIEnv* env, jobject obj,
                                                                 jobjectArray filterMatrix) {
    int rows = 67;
    int columns = 16;
    for (int i = 0; i < rows; i++) {
        jfloatArray row = (jfloatArray)env->GetObjectArrayElement(filterMatrix, i);
        env->GetFloatArrayRegion(row, 0, columns, HassProc.LutHassLambda[i]);
    }
    //HassProc.SetHassFilterMatrix(filterMatrixPtr);
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_GetHassIntermediateData(JNIEnv* env, jobject obj, int sel, float *data) {
    return (jint)HassProc.GetHassIntermediateData(sel, data);
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_GpuImageProcessing_GetHassRawData(JNIEnv* env, jobject obj, jobject data) {
    void* rgbOutPtr = env->GetDirectBufferAddress(data);

    return (jint)HassProc.GetHassRawData(rgbOutPtr);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_camera2basic_Exposure_SetAEreq(JNIEnv* env, jobject obj, int req) {
    HassProc.SetAEreq(req);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_jniRaw102Raw(JNIEnv* env, jobject obj,
                                                       jobject raw10, jobject raw) {
    u8* raw10Ptr = (u8*)env->GetDirectBufferAddress(raw10);
    u16* rawPtr = (u16*)env->GetDirectBufferAddress(raw);
    // RAW10 format
    //          bit 7     bit 6     bit 5     bit 4     bit 3     bit 2     bit 1     bit 0
    // Byte 0:  P0[9]     P0[8]     P0[7]     P0[6]     P0[5]     P0[4]     P0[3]     P0[2]
    // Byte 1:  P1[9]     P1[8]     P1[7]     P1[6]     P1[5]     P1[4]     P1[3]     P1[2]
    // Byte 2:  P2[9]     P2[8]     P2[7]     P2[6]     P2[5]     P2[4]     P2[3]     P2[2]
    // Byte 3:  P3[9]     P3[8]     P3[7]     P3[6]     P3[5]     P3[4]     P3[3]     P3[2]
    // Byte 4:  P3[1]     P3[0]     P2[1]     P2[0]     P1[1]     P1[0]     P0[1]     P0[0]
    u8 byte0, byte1, byte2, byte3, byte4;
    jlong bytes = env->GetDirectBufferCapacity(raw10);
    jint iterations = bytes / 5;  // Raw10 is grouped 5 bytes for every 4 pixels
    for (jint i = 0; i < iterations; ++i) {
        byte0 = *raw10Ptr++;
        byte1 = *raw10Ptr++;
        byte2 = *raw10Ptr++;
        byte3 = *raw10Ptr++;
        byte4 = *raw10Ptr++;
        *rawPtr++ = byte0 << 2 | ((byte4 >> 0) & 0x3);
        *rawPtr++ = byte1 << 2 | ((byte4 >> 2) & 0x3);
        *rawPtr++ = byte2 << 2 | ((byte4 >> 4) & 0x3);
        *rawPtr++ = byte3 << 2 | ((byte4 >> 6) & 0x3);
    }
}
extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_jniRaw2Grayscale(JNIEnv* env, jobject obj,
                                                           jobject raw, jobject grayscale, int display_lsb) {
    jshort* rawPtr = (jshort*)env->GetDirectBufferAddress(raw);
    jint* grayscalePtr = (jint*)env->GetDirectBufferAddress(grayscale);
    jlong bytes = env->GetDirectBufferCapacity(raw);
    jint pixels = bytes / 2;
    jint shift_val = display_lsb;

    if(shift_val > 15)      shift_val = 15;
    else if(shift_val < 0)  shift_val = 0;

    for (jint i = 0; i < pixels; ++i) {
        // Data is 10 bits, take top 8 bits
        //jint pixel = ((*rawPtr++) >> 2) & 0xFF;
        //otsuki for WDR16
        jint pixel = (*rawPtr++);
        if(pixel > 60)  pixel = (pixel - 60)>>shift_val;
        else            pixel = 0;

        if(pixel > 0xFF)   pixel = 0xFF;

        *grayscalePtr++ = 0xFF << 24 | pixel << 16 | pixel << 8 | pixel;
    }
}
extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_Exposure_GetSHS1(JNIEnv* env, jobject obj) {
    return HassProc.GetSHS1();
}

extern "C"
JNIEXPORT jint JNICALL
        Java_com_example_android_camera2basic_Exposure_GetSHS2(JNIEnv* env, jobject obj) {
    return HassProc.GetSHS2();
}

extern "C"
JNIEXPORT bool JNICALL
Java_com_sony_isdc_musarc_Exposure_DoneAE(JNIEnv* env, jobject obj) {
    return HassProc.DoneAE();
}

extern "C"
JNIEXPORT jintArray JNICALL
        Java_com_example_android_camera2basic_ImageProcessing_yuv2Rgb(JNIEnv *env, jobject obj,
                                                  jbyteArray y,
                                                  jbyteArray u,
                                                  jbyteArray v,
                                                  jint width,
                                                  jint height){
    jint frameSize = width * height;
    jintArray rgb = env->NewIntArray(frameSize);
    jbyte* input_y = env->GetByteArrayElements(y, NULL);
    jbyte* input_u = env->GetByteArrayElements(u, NULL);
    jbyte* input_v = env->GetByteArrayElements(v, NULL);
    jint* output = env->GetIntArrayElements(rgb, NULL);

    memset(output, 0xFF, frameSize * 4);

    env->ReleaseByteArrayElements(y, input_y, JNI_ABORT);
    env->ReleaseByteArrayElements(u, input_u, JNI_ABORT);
    env->ReleaseByteArrayElements(v, input_v, JNI_ABORT);
    env->ReleaseIntArrayElements(rgb, output, 0);

    return rgb;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_camera2basic_ImageProcessing_jniRaw102Raw(JNIEnv* env, jobject obj,
                                                       jobject raw10, jobject raw) {
    u8* raw10Ptr = (u8*)env->GetDirectBufferAddress(raw10);
    u16* rawPtr = (u16*)env->GetDirectBufferAddress(raw);
    // RAW10 format
    //          bit 7     bit 6     bit 5     bit 4     bit 3     bit 2     bit 1     bit 0
    // Byte 0:  P0[9]     P0[8]     P0[7]     P0[6]     P0[5]     P0[4]     P0[3]     P0[2]
    // Byte 1:  P1[9]     P1[8]     P1[7]     P1[6]     P1[5]     P1[4]     P1[3]     P1[2]
    // Byte 2:  P2[9]     P2[8]     P2[7]     P2[6]     P2[5]     P2[4]     P2[3]     P2[2]
    // Byte 3:  P3[9]     P3[8]     P3[7]     P3[6]     P3[5]     P3[4]     P3[3]     P3[2]
    // Byte 4:  P3[1]     P3[0]     P2[1]     P2[0]     P1[1]     P1[0]     P0[1]     P0[0]
    u8 byte0, byte1, byte2, byte3, byte4;
    jlong bytes = env->GetDirectBufferCapacity(raw10);
    jint iterations = bytes / 5;  // Raw10 is grouped 5 bytes for every 4 pixels
    for (jint i = 0; i < iterations; ++i) {
        byte0 = *raw10Ptr++;
        byte1 = *raw10Ptr++;
        byte2 = *raw10Ptr++;
        byte3 = *raw10Ptr++;
        byte4 = *raw10Ptr++;
        *rawPtr++ = byte0 << 2 | ((byte4 >> 0) & 0x3);
        *rawPtr++ = byte1 << 2 | ((byte4 >> 2) & 0x3);
        *rawPtr++ = byte2 << 2 | ((byte4 >> 4) & 0x3);
        *rawPtr++ = byte3 << 2 | ((byte4 >> 6) & 0x3);
    }
}

extern "C"
JNIEXPORT int JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_jniRawEncrypt(JNIEnv* env, jobject obj, jobject raw) {

    u16* rawPtr = (u16*)env->GetDirectBufferAddress(raw);
    jlong bytes = env->GetDirectBufferCapacity(raw);

    // Encrypt RAW buffer
    srand((int)time(NULL)); // set time as seed
    int seed = (rand() % 32766) + 1;  // random range 1 to 32767
    int aa = seed;
    int rn;
    int rawpixs = (bytes / 2);  // bytes -> words

    for (int i = 0; i < rawpixs; i++) {
        aa = aa * 214013 + 2531011;
        rn = ( ( aa >> 16 ) & 0x7fff );
        rawPtr[i] = rawPtr[i] ^ rn;
    }

    return seed;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_camera2basic_ImageProcessing_jniRaw2Grayscale(JNIEnv* env, jobject obj,
                                                           jobject raw, jobject grayscale) {
    jshort* rawPtr = (jshort*)env->GetDirectBufferAddress(raw);
    jint* grayscalePtr = (jint*)env->GetDirectBufferAddress(grayscale);
    jlong bytes = env->GetDirectBufferCapacity(raw);
    jint pixels = bytes / 2;
    for (jint i = 0; i < pixels; ++i) {
        // Data is 10 bits, take top 8 bits
        jint pixel = ((*rawPtr++) >> 2) & 0xFF;
        *grayscalePtr++ = 0xFF << 24 | pixel << 16 | pixel << 8 | pixel;
    }
}

extern "C"
JNIEXPORT float JNICALL
Java_com_example_android_camera2basic_GpuImageProcessing_jniGetHpf(JNIEnv* env, jobject obj, jobject raw,
                                                  jint width, jint height) {

    jint* rawPtr = (jint*)env->GetDirectBufferAddress(raw);

    float diff_sum = 0;
    float inte_sum = 0;

    for (int i = (height*4/9); i < (height*5/9); i++) {
        for (int j = (width*4/9); j < (width*5/9); j++) {
            float leftside = rawPtr[i*width+j]&0xFF + rawPtr[i*width+j-1]&0xFF + rawPtr[i*width+j-2]&0xFF + rawPtr[i*width+j-3]&0xFF;
            float rightside= rawPtr[i*width+j+4]&0xFF + rawPtr[i*width+j+1]&0xFF + rawPtr[i*width+j+2]&0xFF + rawPtr[i*width+j+3]&0xFF;

            if(rightside > leftside) {
                if(leftside > 0) diff_sum += (rightside/ leftside);
            }
            else{
                if(rightside> 0) diff_sum += (leftside/ rightside);
            }

            inte_sum += rawPtr[i*width+j];
            rawPtr[i*width+j] = rawPtr[i*width+j]&0xFFFFFF00;
        }
    }
    return diff_sum;
}
