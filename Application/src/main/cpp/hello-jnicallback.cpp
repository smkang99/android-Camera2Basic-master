//
// Created by sam on 9/23/20.
//

/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <string>
#include <inttypes.h>
#include <pthread.h>
#include <jni.h>
#include <android/log.h>
#include <assert.h>
#include <vector>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

// Android log function wrappers
static const char* kTAG = "hello-jniCallback";
#define LOGI(...) \
  ((void)__android_log_print(ANDROID_LOG_INFO, kTAG, __VA_ARGS__))
#define LOGW(...) \
  ((void)__android_log_print(ANDROID_LOG_WARN, kTAG, __VA_ARGS__))
#define LOGE(...) \
  ((void)__android_log_print(ANDROID_LOG_ERROR, kTAG, __VA_ARGS__))


/* This is a trivial JNI example where we use a native method
 * to return a new VM String. See the corresponding Java source
 * file located at:
 *
 *   hello-jniCallback/app/src/main/java/com/example/hellojnicallback/MainActivity.java
 */

// processing callback to handler class
typedef struct tick_context {
    JavaVM  *javaVM;
    jclass   jniHelperClz;
    jobject  jniHelperObj;
    jclass   mainActivityClz;
    jobject  mainActivityObj;
    pthread_mutex_t  lock;
    int      done;
} TickContext;
TickContext g_ctx;

jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    JNIEnv* env;

    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR; // JNI version not supported.
    }

    return  JNI_VERSION_1_6;
}

enum RectificationMode { STANDARD, UNCALIBRATED, REPROJECTION, HOMOGRAPHY };

static bool rgbdCalibration(const string rgbFile, const string tofFile,
                            Size boardSize, float squareSize,
                            bool displayCorners, bool useCalibrated, RectificationMode mode);

static bool findCorners(Mat img, Size boardSize, vector<Point2f>& corners, bool displayCorners);

/* RGB-ToF Stereo Calibration
   Given a RGB image and a ToF image of the same chessboard calibration pattern,
   this function calculates the rectification transformations for each camera.
*/
static bool rgbdCalibration(const vector<string> rgbFile,
                            const vector<string> tofFile,
                            Size boardSize,
                            float squareSize,
                            bool displayCorners=true,
                            bool showRectified=true,
                            RectificationMode mode=STANDARD)
{
    // Make sure there is the same number of RGB and ToF images
    assert(rgbFile.size() == tofFile.size());

    // Allocate vectors
    int nimages = rgbFile.size();
    int nCorners = boardSize.height * boardSize.width;
    vector<vector<Point2f>> imagePoints[2];
    vector<Point2f> detectedCorners[2];
    vector<vector<Point3f>> objectPoints;
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    detectedCorners[0].resize(nimages*nCorners);
    detectedCorners[1].resize(nimages*nCorners);
    objectPoints.resize(nimages);

    // Coordinates of chessboard corners in real world
    for (int i = 0; i < nimages; i++)
        for (int j = 0; j < boardSize.height; j++)
            for (int k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));

    // Iterate over images
    // ToF = 0
    // RGB = 1
    Size imageSizeRGB, imageSizeToF;
    Mat img_RGB[nimages], img_ToF[nimages], img_tmp, cimg[2], rimg[2];
    for (int i = 0; i < nimages; i++)
    {
        // Read images
        img_RGB[i] = imread(rgbFile[i], 0);
        if (img_RGB[i].empty()) return false;
        img_ToF[i] = imread(tofFile[i], 0);
        if (img_ToF[i].empty()) return false;

        // Image sizes
        imageSizeRGB = img_RGB[i].size();
        imageSizeToF = img_ToF[i].size();

        // Detect corners
        bool success;
        success = findCorners(img_ToF[i], boardSize, imagePoints[0][i], displayCorners);
        if (!success) return false;
        success = findCorners(img_RGB[i], boardSize, imagePoints[1][i], displayCorners);
        if (!success) return false;

        // Copy corner coordinates
        for (int j = 0; j < nCorners; j++)
        {
            detectedCorners[0][i*nCorners + j] = imagePoints[0][i][j];
            detectedCorners[1][i*nCorners + j] = imagePoints[1][i][j];
        }
    }

    /*** Intrinsic calibration ***/
    Mat cameraMatrix[2], distCoeffs[2];
    vector<Mat> rvecs, tvecs;
    Size imageSize = imageSizeRGB;
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSizeToF, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSizeRGB, 0);

    double error_tof = calibrateCamera(objectPoints, imagePoints[0], imageSizeToF,
                                       cameraMatrix[0], distCoeffs[0], rvecs, tvecs,
                                       CV_CALIB_USE_INTRINSIC_GUESS +
                                       CV_CALIB_FIX_ASPECT_RATIO +
                                       CV_CALIB_ZERO_TANGENT_DIST +
                                       CV_CALIB_FIX_K2 +
                                       CV_CALIB_FIX_K3 +
                                       CV_CALIB_FIX_K4 +
                                       CV_CALIB_FIX_K5 +
                                       CV_CALIB_FIX_K6,
                                       TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 1000, 1e-8));

    double error_rgb = calibrateCamera(objectPoints, imagePoints[1], imageSizeRGB,
                                       cameraMatrix[1], distCoeffs[1], rvecs, tvecs,
                                       CV_CALIB_USE_INTRINSIC_GUESS +
                                       CV_CALIB_FIX_ASPECT_RATIO +
                                       CV_CALIB_ZERO_TANGENT_DIST +
                                       CV_CALIB_FIX_K2 +
                                       CV_CALIB_FIX_K3 +
                                       CV_CALIB_FIX_K4 +
                                       CV_CALIB_FIX_K5 +
                                       CV_CALIB_FIX_K6,
                                       TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 1000, 1e-8));

    std::cout << "RGB intrinsic calibration error: " << error_rgb << std::endl;
    std::cout << "ToF intrinsic calibration error: " << error_tof << std::endl;


    /*** Stereo calibration ***/
    Mat R, T, E, F;
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSizeRGB, R, T, E, F,
                                 CV_CALIB_FIX_INTRINSIC,
                                 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 1000, 1e-8) );


    // Assemble matrix
    // [ R   T ]
    // [ 0   1 ]
    Mat RT, RT0;
    hconcat(R, T, RT);
    vconcat(RT, Mat::zeros(1,4,CV_64F), RT0);
    RT0.at<double>(3,3) = 1;

    std::cout << "ToF camera matrix:" << std::endl << cameraMatrix[0] << std::endl;
    std::cout << "ToF distortion coefficients:" << std::endl << distCoeffs[0] << std::endl;
    std::cout << "RGB camera matrix:" << std::endl << cameraMatrix[1] << std::endl;
    std::cout << "RGB distortion coefficients:" << std::endl << distCoeffs[1] << std::endl;
    std::cout << "Rotation matrix:" << std::endl << R << std::endl;
    std::cout << "Translation vector:" << std::endl << T << std::endl;
    std::cout << "Essential matrix:" << std::endl << E << std::endl;
    std::cout << "Fundamental matrix:" << std::endl << F << std::endl;

    std::cout << "============================================================" << std::endl;
    std::cout << "RMS reprojection error: " << rms << std::endl;

    /*** CALIBRATION QUALITY CHECK ***/
    // because the output fundamental matrix implicitly includes all the output information,
    // we can check the quality of calibration using the epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (int i = 0; i < nimages; i++)
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for (int k = 0; k < 2; k++)
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for (int j = 0; j < npt; j++)
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "Average epipolar err = " <<  err/npoints << endl;
    std::cout << "============================================================" << std::endl;



    /*********************************        RECTIFICATION        ******************************/
    // Since rectification with one pair of images (each having different intrinsic parameters)
    // is very difficult, we will try several different approaches for rectification.
    // They may or may not work.
    Mat rmap[2][2], H, H1, H2, Fm, Pm;
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    Mat img_RGB_undistort, img_ToF_undistort;

    /// Standard Stereo Rectification
    {
        stereoRectify(cameraMatrix[0], distCoeffs[0],
                      cameraMatrix[1], distCoeffs[1],
                      imageSizeRGB, R, T, R1, R2, P1, P2, Q,
                      0, 1, imageSizeRGB, &validRoi[0], &validRoi[1]);

        // For some reason, maybe because of different camera resolutions and intrinsic parameters,
        // stereoRectify produces extremely wrong projection matrices P1 and P2.
        std::cout << "Rectification matrix 1:" << std::endl << R1 << std::endl;
        std::cout << "Rectification matrix 2:" << std::endl << R2 << std::endl;
        std::cout << "Projection matrix 1:" << std::endl << P1 << std::endl;
        std::cout << "Projection matrix 2:" << std::endl << P2 << std::endl;
    }

    /// Stereo Rectify Uncalibrated
    {
        // ToF -> RGB
        Fm = findFundamentalMat(detectedCorners[0], detectedCorners[1]);
        stereoRectifyUncalibrated(detectedCorners[0], detectedCorners[1], Fm, imageSize, H1, H2);
        std::cout << "Fm: " << std::endl << Fm << std::endl;
        std::cout << "H1: " << std::endl << H1 << std::endl;
        std::cout << "H2: " << std::endl << H2 << std::endl;
    }

    /// Manual composition of rotation, translation, and projection matrices.
    {
        // Compute overall projection transform from ToF to RGB
        double cx1 = cameraMatrix[0].at<double>(0,2);
        double cy1 = cameraMatrix[0].at<double>(1,2);
        double cx2 = cameraMatrix[1].at<double>(0,2);
        double cy2 = cameraMatrix[1].at<double>(1,2);
        double fx1 = cameraMatrix[0].at<double>(0,0);
        double fy1 = cameraMatrix[0].at<double>(1,1);
        double fx2 = cameraMatrix[1].at<double>(0,0);
        double fy2 = cameraMatrix[1].at<double>(1,1);

        Mat P[2], Pr0;
        P[0] = Mat::zeros(4,4,CV_64F);
        P[1] = Mat::zeros(4,4,CV_64F);
        P[0].at<double>(0,0) = fx1; // fx1
        P[0].at<double>(1,1) = fy1; // fy1
        P[0].at<double>(0,2) = cx1; // cx1
        P[0].at<double>(1,2) = cy1; // cy1
        P[0].at<double>(2,2) = 1.0;
        P[0].at<double>(3,3) = 1.0;
        P[1].at<double>(0,0) = fx2; // fx2
        P[1].at<double>(1,1) = fy2; // fy2
        P[1].at<double>(0,2) = cx2; // cx2
        P[1].at<double>(1,2) = cy2; // cy2
        P[1].at<double>(2,2) = 1.0;
        P[1].at<double>(3,3) = 1.0;
        Pr0 = Mat::eye(3,4,CV_64F) * P[1] * RT0 * P[0].inv();

        Pm = Mat::eye(3,3,CV_64F);
        Pm.at<double>(0,0) = Pr0.at<double>(0,0);
        Pm.at<double>(0,1) = Pr0.at<double>(0,1);
        Pm.at<double>(0,2) = Pr0.at<double>(0,2) + Pr0.at<double>(0,3);
        Pm.at<double>(1,0) = Pr0.at<double>(1,0);
        Pm.at<double>(1,1) = Pr0.at<double>(1,1);
        Pm.at<double>(1,2) = Pr0.at<double>(1,2) + Pr0.at<double>(1,3);
        Pm.at<double>(2,0) = Pr0.at<double>(2,0);
        Pm.at<double>(2,1) = Pr0.at<double>(2,1);
        Pm.at<double>(2,2) = Pr0.at<double>(2,2) + Pr0.at<double>(2,3);
        std::cout << "ToF-to-RGB Homography Matrix: " << std::endl << Pm << std::endl;
    }

    /// Direct Homography
    {
        H = findHomography(detectedCorners[0], detectedCorners[1], CV_RANSAC);
        std::cout << "Direct Homography: " << std::endl << H << std::endl;
    }


    /**** VISUAL QUALITY CHECK ****/
    // Prepare rectified images for display
    if (showRectified)
    {
        Mat canvas, canvasPart;
        double sf = 600./MAX(imageSize.width, imageSize.height);
        int w = cvRound(imageSize.width*sf);
        int h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
        for (int i = 0; i < nimages; i++)
        {
            switch (mode)
            {
                case STANDARD:
                    //Precompute maps for cv::remap()
                    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
                    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

                    // Remap
                    remap(img_ToF[i], rimg[0], rmap[0][0], rmap[0][1], INTER_LINEAR);
                    remap(img_RGB[i], rimg[1], rmap[1][0], rmap[1][1], INTER_LINEAR);
                    break;

                case UNCALIBRATED:
                    // Warp images
                    warpPerspective(img_ToF[i], rimg[0], H1, imageSizeRGB);
                    warpPerspective(img_RGB[i], rimg[1], H2, imageSizeRGB);
                    break;

                case REPROJECTION:
                    // Remove lens distortion
                    undistort(img_ToF[i], img_ToF_undistort, cameraMatrix[0], distCoeffs[0]);
                    undistort(img_RGB[i], img_RGB_undistort, cameraMatrix[1], distCoeffs[1]);

                    // Warp images
                    warpPerspective(img_ToF_undistort, rimg[0], Pm, imageSizeRGB);
                    rimg[1] = img_RGB_undistort;
                    break;

                case HOMOGRAPHY:
                    // Warp ToF to RGB
                    warpPerspective(img_ToF[i], rimg[0], H, imageSizeRGB);
                    rimg[1] = img_RGB[i];
                    break;
            }


            // Convert to 3-channel BGR
            cvtColor(rimg[0], cimg[0], COLOR_GRAY2BGR);
            cvtColor(rimg[1], cimg[1], COLOR_GRAY2BGR);

            // Draw RGB image
            canvasPart = canvas(Rect(0, 0, w, h));
            resize(cimg[0], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);

            // Draw Hass image
            canvasPart = canvas(Rect(w, 0, w, h));
            resize(cimg[1], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);

            // Draw epipolar lines
            for (int j = 0; j < canvas.rows; j += 16)
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

            // Render on screen
            if(false)
              imshow("rectified", canvas);
            imwrite("/sdcard/DCIM/Camera/imx454_rectified.jpeg", canvas);

            //waitKey();
            imwrite("Rectified_Image.bmp",canvas);
        }
    }

    /******* Save data to file *******/
    string filename = "calibration.yml";
    FileStorage fs(filename, FileStorage::WRITE);
    fs << "corners_tof" << detectedCorners[0];
    fs << "corners_rgb" << detectedCorners[1];
    fs << "cameraMatrix_tof" << cameraMatrix[0];
    fs << "cameraMatrix_rgb" << cameraMatrix[1];
    fs << "distortionCoeffs_tof" << distCoeffs[0];
    fs << "distortionCoeffs_rgb" << distCoeffs[1];
    fs << "R" << R;
    fs << "T" << T;
    fs << "Fm" << Fm;
    fs << "H_tof" << H1;
    fs << "H_rgb" << H2;
    fs << "H" << H;
    fs.release();

    return true;
}

static bool findCorners(Mat img,
                        Size boardSize,
                        vector<Point2f>& corners,
                        bool displayCorners=true)
{
    const int maxScale = 4;
    bool found = false;

    // Try detecting corners at different scales
    for (int scale = 1; scale <= maxScale; scale++)
    {
        Mat timg;

        // Resize original image
        if (scale == 1)
            timg = img;
        else
            resize(img, timg, Size(), scale, scale, 1/*INTER_LINEAR_EXACT*/);

        // findChessboardCorners
        found = findChessboardCorners(timg,
                                      boardSize,
                                      corners,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        // Scale coordinates back to original size if necessary
        if (found)
        {
            if (scale > 1)
            {
                Mat cornersMat(corners);
                cornersMat *= 1./scale;
            }
            break;
        }
    }
    if (!found) { return false; }

    // Refine corners
    cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01));

    // Display
    if (displayCorners)
    {
        Mat cimg, cimg1;
        cvtColor(img, cimg, COLOR_GRAY2BGR);
        drawChessboardCorners(cimg, boardSize, corners, found);
        double sf = 640./MAX(img.rows, img.cols);
        resize(cimg, cimg1, Size(), sf, sf, 1/*INTER_LINEAR_EXACT*/);
        if(false)
          imshow("corners", cimg1);
        imwrite("/sdcard/DCIM/Camera/imx586_1_corner.jpeg", cimg1);
        //waitKey();
        //imwrite("corners.bmp",cimg1);
    }

    return found;
}

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_android_camera2basic_CameraActivity_stringFromJNI(JNIEnv *env, jobject thiz, jobject img, jstring rgbImgP, jstring hassImgP, jint width, jint height) {
    // TODO: implement stringFromJNI()
#if defined(__arm__)
    #if defined(__ARM_ARCH_7A__)
    #if defined(__ARM_NEON__)
      #if defined(__ARM_PCS_VFP)
        #define ABI "armeabi-v7a/NEON (hard-float)"
      #else
        #define ABI "armeabi-v7a/NEON"
      #endif
    #else
      #if defined(__ARM_PCS_VFP)
        #define ABI "armeabi-v7a (hard-float)"
      #else
        #define ABI "armeabi-v7a"
      #endif
    #endif
  #else
   #define ABI "armeabi"
  #endif
#elif defined(__i386__)
#define ABI "x86"
#elif defined(__x86_64__)
    #define ABI "x86_64"
#elif defined(__mips64)  /* mips64el-* toolchain defines __mips__ too */
#define ABI "mips64"
#elif defined(__mips__)
#define ABI "mips"
#elif defined(__aarch64__)
#define ABI "arm64-v8a"
#else
#define ABI "unknown"
#endif

    //m.create(width, height, CV_8UC4);
    //void* buffer = malloc(width * height * sizeof(CV_8UC4));
    auto buffer =  (env->GetDirectBufferAddress(img));
    jlong len = env->GetDirectBufferCapacity(img);
    //static Mat m( height, width, CV_8UC4, buffer);
    static Mat m, rgbImg, hassImg;
    //memcpy(m.data, buffer, len);
    char * hassImgPath, *rgbImgPath;

    hassImgPath = const_cast<char *>(env->GetStringUTFChars(hassImgP, NULL));
    rgbImgPath = const_cast<char *>(env->GetStringUTFChars(rgbImgP, NULL));
    hassImg = imread(hassImgPath,0);
    rgbImg = imread(rgbImgPath, 0);
    Size imageSize, imageSizeRGB = hassImg.size();
    imageSize = imageSizeRGB;
    Mat canvas, canvasPart, rimg, rmap[2];
    vector<string> rgbFile;// ={rgbImgPath};
    rgbFile.push_back(rgbImgPath);
    vector<string> tofFile;  tofFile.push_back(hassImgPath);
    //double sf = 600./MAX(imageSize.width, imageSize.height);
    //int w = cvRound(imageSize.width*sf);
    //int h = cvRound(imageSize.height*sf);
    //canvas.create(h, w*2, CV_8UC3);

    //Mat H2 = Mat::zeros(3,3, CV_64FC1);

    //warpPerspective(rgbImg, rimg, H2, imageSizeRGB);

    Size boardSize = Size(10, 7); //Size(9, 8);
    float squareSize = 0.0423; // 42.3 millimeters

    //float focalLengthToF = 0.00188; // 0.00188 // 1.88 millimeters
    //float focalLengthRGB = 0.00474; // 0.00474 // 4.74 millimeters
    //float pixelSizeToF = 0.00001; // 0.00001 // 10 micrometers
    //float pixelSizeRGB = 0.0000008; // 0.8 micrometers

    bool displayCorners = true;
    bool displayRectified = true;
    RectificationMode mode = HOMOGRAPHY; // STANDARD, UNCALIBRATED, REPROJECTION, HOMOGRAPHY
    bool success = rgbdCalibration(rgbFile, tofFile, boardSize, squareSize,
                                   displayCorners, displayRectified, mode);


    return env->NewStringUTF("Hello from JNI !  Compiled with ABI " ABI ".");
}