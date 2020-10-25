package com.example.android.camera2basic;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;

//import com.sony.isdc.musarc.camera.CameraContext;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Observable;
import java.util.Observer;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Created by Allan Suen on 1/12/2017.
 */

public class GpuImageProcessing extends Observable implements Observer {
    public static final int HASS_MODE_RGB = 0;
    public static final int HASS_MODE_NDVI = 1;
    public static final int HASS_MODE_SPC = 2;
    public static final int HASS_MODE_RAW = 3;

    private static final int YUV2RGB = 0;
    private static final int RAW2RGB = 1;
    private static final int RAW102RGB = 2;
    private static final int RAW102RAW = 3;

    private static final int IMAGE_QUEUE_SIZE = Camera2BasicFragment.IMAGE_QUEUE_SIZE;

    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;

    private RefCountedAutoCloseable<HassImage>[] mHassImages;
    private RefCountedAutoCloseable<HassImage>[] mHassEncImages;
    private RefCountedAutoCloseable<HassImage>[] mHassDisponlyImages;
    private int mHassImageIndex;
    private int mHassEncImageIndex;
    private int mHassDisponlyImageIndex;
    private ArrayBlockingQueue<RefCountedAutoCloseable<HassImage>> mImageQueue;

    private AtomicBoolean mUseDoubleBuffer;
    public boolean translate_1ch_to_3ch;

    static {
        System.loadLibrary("imgproc-lib");
    }

    private static int mAverageNumber;
    private static int mTotalAverageNumber;

    private native int openclYuv2Rgb(byte[] imagePtr1, byte[] imagePtr2, byte[] rgbOut1, byte[] rgbOut2);
    private native int openclHassProc(ByteBuffer imagePtr1, ByteBuffer rgbOut1);
    private native int openclHassWdrRawProc(ByteBuffer imagePtr1, ByteBuffer rgbOut1, ByteBuffer rgbOut2, int fnum, int displayoff);
    private native int jniRaw102Raw(ByteBuffer imagePtr1, ByteBuffer rgbOut1);
    private native int openclHassProcDB(ByteBuffer imagePtr1, ByteBuffer imagePtr2, ByteBuffer rgbOut1, ByteBuffer rgbOut2);
    private native int HassModeSPC(int s);
    private native int HassModeRGB();
    private native int HassModeNDVI();
    private native int SetHassNRTap(int tap);
    private native int SetHassGamma(float gamma);
    private native int SetHassAutoExposureSpeedLong(int value);
    private native int SetHassAutoExposureSpeedShort(int value);
    private native int SetHassAutoExposureTarget(int value);
    private native int SetHassAutoExposureRange(int value);
    private native int SetHassDisplayLsb(int value);
    private native int SetHassAverageGain(float value);
    private native int GetHassNRTap();
    private native float GetHassGamma();
    private native float GetHassAutoExposureSpeedLong();
    private native float GetHassAutoExposureSpeedShort();
    private native float GetHassAutoExposureRange();
    private native float GetHassAutoExposureTarget();
    private native int GetHassIntermediateData(int sel, float[] data);
    private native int GetHassEncData(ByteBuffer data);
    private native int[] GetHassImageConfiguration();
    private native int SetHassImageConfiguration(int[] config);
    private native int SetHassImageOffset(int dx, int dy);
    private native int[][] GetHassFilterStructure();
    private native int SetHassFilterStructure(int[][] data);
    private native float[][] GetHassFilterMatrix();
    private native int SetHassFilterMatrix(float[][] data);
    private native int jniRawEncrypt(ByteBuffer raw);
    private native float jniGetHpf(ByteBuffer raw, int width, int height);

    private int mTempareture;
    private int mLensNumber;
    private int mCameraSerialNumber;
    private int mSensorIDNum;
    private int mSensorTypeNum;
    private int dbg_frame_counter = 0;
    private int mDisplayOff = 0;
    private boolean hpf_mode = false;
    private int hpf_val = 0;
    private int hpf_cnt = 0;
    private int memory_size = 0;

    /**
     * Observer receiving object from CameraContext
     * @param o CameraContext object
     * @param arg HassImage object
     */
    @Override
    public void update(Observable o, Object arg) {
        RefCountedAutoCloseable<HassImage> refHassImage = (RefCountedAutoCloseable<HassImage>)arg;
        if (enqueue(refHassImage)) {
            HassImage hassImage = refHassImage.getAndRetain();
            switch (hassImage.mType) {
                case HassImage.TYPE_YUV:
                    processYuv();
                    break;
                case HassImage.TYPE_RAW:
                    processRaw();
                    break;
                case HassImage.TYPE_RAW10:
                    processRaw10();
                    break;
            }
        }
    }

    /**
     * Create GpuImageProcessing object
     */
    public GpuImageProcessing() {
        mImageQueue = new ArrayBlockingQueue<>(Camera2BasicFragment.IMAGE_QUEUE_SIZE_ORG, true);
        mHassImages = new RefCountedAutoCloseable[IMAGE_QUEUE_SIZE];
        mHassEncImages = new RefCountedAutoCloseable[IMAGE_QUEUE_SIZE];
        mHassDisponlyImages = new RefCountedAutoCloseable[IMAGE_QUEUE_SIZE];
        for (int i = 0; i < IMAGE_QUEUE_SIZE; i++) {
            mHassImages[i] = new RefCountedAutoCloseable<>(new HassImage(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, HassImage.TYPE_RGB));
            mHassEncImages[i] = new RefCountedAutoCloseable<>(new HassImage(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, HassImage.TYPE_ENC));
            mHassDisponlyImages[i] = new RefCountedAutoCloseable<>(new HassImage(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, HassImage.TYPE_DISPONLY));
        }

        mUseDoubleBuffer = new AtomicBoolean(false);
        mAverageNumber = 0;
        mTotalAverageNumber = 1;
    }

    /**
     * Start background thread for GpuImageProcessing class
     */
    public void startBackgroundThread() {
        if (mBackgroundThread == null) {
            mBackgroundThread = new HandlerThread("GpuImageProcessingThread");
            mBackgroundThread.start();
            mBackgroundHandler = new Handler(mBackgroundThread.getLooper()) {
                @Override
                public void handleMessage(Message msg) {
                    switch (msg.what) {
                        case YUV2RGB:
                            yuv2Rgb();
                            break;
                        case RAW2RGB:
                            raw2Rgb();
                            break;
                        case RAW102RGB:
                            if (mUseDoubleBuffer.get()) {
                                raw102RgbDoubleBuffer();
                            } else {
                                raw102Rgb();
                            }
                            break;
                        case RAW102RAW:
                            raw102Raw();
                            break;
                    }
                    super.handleMessage(msg);
                }
            };
        }
    }

    /**
     * Stop background thread for GpuImageProcessing class
     */
    public void stopBackgroundThread() {
        if (mBackgroundThread != null) {
            mBackgroundThread.quitSafely();
            try {
                mBackgroundThread.join();
                mBackgroundThread = null;
                mBackgroundHandler = null;
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

//    public void enableDoubleBuffer(boolean enable) {
//        mUseDoubleBuffer.set(enable);
//    }

    /**
     * Sets the GPU processing mode for images
     * @param mode GPU processing mode
     * @param wavelength Wavelength under Single Wavelength mode
     */
    public void setHassMode(int mode, int wavelength) {
        synchronized (this) {
            switch (mode) {
                case HASS_MODE_RGB:
                    HassModeRGB();
                    break;
                case HASS_MODE_NDVI:
                    HassModeNDVI();
                    break;
                case HASS_MODE_SPC:
                    HassModeSPC(wavelength);
                    break;
            }
        }
    }

    /**
     * Sets the GPU processing mode for images
     * @param mode GPU processing mode
     * @param wavelength Wavelength under Single Wavelength mode
     */
    public void setDisplayOff(int mode) {
        mDisplayOff = mode;
    }

    /**
     * Set the gamma value used to create gamma table
     * @param value gamma value
     */
    public void setGamma(float value) {
        synchronized (this) {
            SetHassGamma(value);
        }
    }

    public void set_memory_size(long tmp_memory_size){
        memory_size = (int)(tmp_memory_size/1000000);
    }

    /**
     * Return the current gamma value
     * @return gamma value
     */
    public float getGamma() {
        return GetHassGamma();
    }

    /**
     * Set Auto Exposure Speed Long
     * @param Auto Exposure Speed Short
     */
    public void setAverageFrameNum(int value) {
        mTotalAverageNumber = value;
        synchronized (this) {
            if(value >= 1.0) SetHassAverageGain((float)1.0/value);
        }
    }

    /**
     * Set Auto Exposure Speed Long
     * @param Auto Exposure Speed Short
     */
    public void setAutoExposureSpeedLong(int value) {
        synchronized (this) {
            SetHassAutoExposureSpeedLong(value);
        }
    }
    /**
     * Set Auto Exposure Speed
     * @param Auto Exposure Speed
     */
    public void setAutoExposureSpeedShort(int value) {
        synchronized (this) {
            SetHassAutoExposureSpeedShort(value);
        }
    }
    /**
     * Set Auto Exposure target
     * @param Auto Exposure target
     */
    public void setAutoExposureTarget(int value) {
        synchronized (this) {
            SetHassAutoExposureTarget(value);
        }
    }
    /**
     * Set Auto Exposure Speed
     * @param Auto Exposure Speed
     */
    public void setAutoExposureRange(int value) {
        synchronized (this) {
            SetHassAutoExposureRange(value);
        }
    }
    /**
     * Set Auto Exposure Speed
     * @param Auto Exposure Speed
     */
    public void setHassDisplayLsb(int value) {
        synchronized (this) {
            SetHassDisplayLsb(value);
        }
    }
    /**
     * GetAuto Exposure Speed
     * @param Auto Exposure Speed
     */
    public float getAutoExposureSpeedLong() {
        return GetHassAutoExposureSpeedLong();
    }

    /**
     * GetAuto Exposure Speed
     * @param Auto Exposure Speed
     */
    public float getAutoExposureSpeedShort() {
        return GetHassAutoExposureSpeedShort();
    }

    /**
     * GetAuto Exposure Range
     * @param Auto Exposure Range
     */
    public float getAutoExposureRange() {
        return GetHassAutoExposureRange();
    }

    /**
     * GetAuto Exposure Target
     * @param Auto Exposure Target
     */
    public float getAutoExposureTarget() {
        return GetHassAutoExposureTarget();
    }

    /**
     * Set the noise reduction filter size
     * @param tap
     */
    public void setNrTap(int tap) {
        synchronized (this) {
            SetHassNRTap(tap);
        }
    }

    /**
     * Return the noise reduction filter size
     * @return
     */
    public int getNrTap() {
        return GetHassNRTap();
    }

    /**
     * Set the GPU processing configuration
     * @param configuration
     */
    public void setHassConfiguration(int[] configuration) {
        synchronized (this) {
            SetHassImageConfiguration(configuration);
        }
    }

    /**
     * Return the GPU processing configuration
     * @return
     */
    public int[] getHassConfiguration() {
        return GetHassImageConfiguration();
    }

    /**
     * Set the offset of image (0,0)
     * @param dx offset in the X direction
     * @param dy offset in the Y direction
     */
    public void setHassOffset(int dx, int dy) {
        synchronized (this) {
            SetHassImageOffset(dx, dy);
        }
    }

    public void setHassMosaic(int[][] mosaic) {
        synchronized (this) {
            SetHassFilterStructure(mosaic);
        }
    }

    public int[][] getHassMosaic() {
        return GetHassFilterStructure();
    }

    public void setHassCoefficients(float[][] coefficients) {
        synchronized (this) {
            SetHassFilterMatrix(coefficients);
        }
    }

    public float[][] getHassCoefficients() {
        return GetHassFilterMatrix();
    }

    public int getHpf() {
        int rethpf = 0;
        if(hpf_cnt > 0) rethpf = (int)(hpf_val / (float)hpf_cnt);

        hpf_cnt = 0;
        hpf_val = 0;

        return rethpf;
    }

    public void setHpfMode(boolean mode) {
        hpf_mode = mode;
    }

    private boolean enqueue(RefCountedAutoCloseable<HassImage> image) {
        return mImageQueue.offer(image);
    }

    private void processYuv() {
        mBackgroundHandler.obtainMessage(YUV2RGB).sendToTarget();
    }

    private void processRaw() {
        mBackgroundHandler.obtainMessage(RAW2RGB).sendToTarget();
    }

    private void processRaw10() {
        if(translate_1ch_to_3ch) mBackgroundHandler.obtainMessage(RAW102RGB).sendToTarget();
        else					 mBackgroundHandler.obtainMessage(RAW102RAW).sendToTarget();
    }

    private void yuv2Rgb() {
//        if (mImageQueue.size() >= 1) {
//            RefCountedAutoCloseable<HassImage> refYuv = mImageQueue.poll();
//            HassImage yuv = refYuv.get();
//            RefCountedAutoCloseable<HassImage> refRgb = getHassImage();
//            HassImage rgb = refRgb.get();
//            synchronized (this) {
//                openclYuv2Rgb(yuv.mPixels, rgb.mPixels);
//            }
//            rgb.mTimestamp = yuv.mTimestamp;
//            rgb.mMainAls = yuv.mMainAls;
//            rgb.mSlaveAls = yuv.mSlaveAls;
//            refYuv.close();
//            setChanged();
//            notifyObservers(refRgb);
//            refRgb.close();
//        }
    }

    private void raw2Rgb() {
//        if (mImageQueue.size() >= 1) {
//            RefCountedAutoCloseable<HassImage> refRaw = mImageQueue.poll();
//            HassImage raw = refRaw.get();
//            RefCountedAutoCloseable<HassImage> refRgb = getHassImage();
//            HassImage rgb = refRgb.get();
//            synchronized (this) {
//                openclHassProc(raw.mPixels, rgb.mPixels);
//            }
//            rgb.mTimestamp = raw.mTimestamp;
//            rgb.mMainAls = raw.mMainAls;
//            rgb.mSlaveAls = raw.mSlaveAls;
//            refRaw.close();
//            setChanged();
//            notifyObservers(refRgb);
//            refRgb.close();
//        }
    }

    private void raw102RgbDoubleBuffer() {
        if (mImageQueue.size() >= 2) {
            RefCountedAutoCloseable<HassImage> refRaw1 = mImageQueue.poll();
            HassImage raw1 = refRaw1.get();
            RefCountedAutoCloseable<HassImage> refRaw2 = mImageQueue.poll();
            HassImage raw2 = refRaw2.get();
            RefCountedAutoCloseable<HassImage> refRgb1 = getHassImage();
            HassImage rgb1 = refRgb1.get();
            RefCountedAutoCloseable<HassImage> refRgb2 = getHassImage();
            HassImage rgb2 = refRgb2.get();
            //raw1.mReadLock.lock();
            //raw2.mReadLock.lock();
            //rgb1.mWriteLock.lock();
            //rgb2.mWriteLock.lock();
            synchronized (this) {
                openclHassProcDB(raw1.mPixels, raw2.mPixels, rgb1.mPixels, rgb2.mPixels);
            }
            //raw1.mReadLock.unlock();
            //raw2.mReadLock.unlock();
            //rgb1.mWriteLock.unlock();
            //rgb2.mWriteLock.unlock();
            rgb1.mTimestamp = raw1.mTimestamp;
            rgb1.mLongExposureTime = raw1.mLongExposureTime;
            rgb1.mShortExposureTime = raw1.mShortExposureTime;
            rgb1.mMainAlsEnabled = raw1.mMainAlsEnabled;
            rgb1.mMainAls = raw1.mMainAls;
            rgb1.mSlaveAlsEnabled = raw1.mSlaveAlsEnabled;
            rgb1.mSlaveAls = raw1.mSlaveAls;
            rgb1.mGpsEnabled = raw1.mGpsEnabled;
            rgb1.mLocation = raw1.mLocation;
            refRaw1.close();
            setChanged();
            notifyObservers(refRgb1);
            refRgb1.close();
            rgb2.mTimestamp = raw2.mTimestamp;
            rgb2.mMainAlsEnabled = raw2.mMainAlsEnabled;
            rgb2.mMainAls = raw2.mMainAls;
            rgb2.mSlaveAlsEnabled = raw2.mSlaveAlsEnabled;
            rgb2.mSlaveAls = raw2.mSlaveAls;
            rgb2.mGpsEnabled = raw2.mGpsEnabled;
            rgb2.mLocation = raw2.mLocation;
            refRaw2.close();
            setChanged();
            notifyObservers(refRgb2);
            refRgb2.close();
        }
    }

    private void raw102Rgb() {
        if (mImageQueue.size() >= 1) {
            RefCountedAutoCloseable<HassImage> refRaw = mImageQueue.poll();
            HassImage raw = refRaw.get();
            RefCountedAutoCloseable<HassImage> refRgb = getHassImage();
            HassImage rgb = refRgb.get();
            //raw.mReadLock.lock();
            //rgb.mWriteLock.lock();
            synchronized (this) {
                openclHassProc(raw.mPixels, rgb.mPixels);
            }
            //raw.mReadLock.unlock();
            //rgb.mWriteLock.unlock();
            rgb.mTimestamp = raw.mTimestamp;
            rgb.mLongExposureTime = raw.mLongExposureTime ;
            rgb.mShortExposureTime = raw.mShortExposureTime;
            rgb.mMainAlsEnabled = raw.mMainAlsEnabled;
            rgb.mMainAls = raw.mMainAls;
            rgb.mSlaveAlsEnabled = raw.mSlaveAlsEnabled;
            rgb.mSlaveAls = raw.mSlaveAls;
            rgb.mGpsEnabled = raw.mGpsEnabled;
            rgb.mLocation = raw.mLocation;
            refRaw.close();
            setChanged();
            notifyObservers(refRgb);
            refRgb.close();
        }
    }

    private void raw102Raw() {
        if (mImageQueue.size() >= 1) {
            RefCountedAutoCloseable<HassImage> refRaw10 = mImageQueue.poll();
            HassImage raw10 = refRaw10.get();

            RefCountedAutoCloseable<HassImage> refEnc = getHassEncImage();
            RefCountedAutoCloseable<HassImage> refGrayscale = getHassDisponlyImage();

            HassImage enc = refEnc.getAndRetain();
            HassImage grayscale = refGrayscale.getAndRetain();

            //raw10.mReadLock.lock();
            //raw.mWriteLock.lock();

            int mode;

            if(mTotalAverageNumber == 1)                        mode = 0; // average mode off
            else if(mAverageNumber == 0)                        mode = 1; // average mode start
            else if(mAverageNumber != mTotalAverageNumber-1)    mode = 2; // average mode adding
            else                                                mode = 3; // average mode end

            synchronized (this) {
                openclHassWdrRawProc(raw10.mPixels, enc.mPixels, grayscale.mPixels, mode, mDisplayOff);
            }

            if(hpf_mode) {
                hpf_val += jniGetHpf(grayscale.mPixels, 1920, 1080);
                hpf_cnt++;
            }

            //raw10.mReadLock.unlock();
            //raw.mWriteLock.unlock();
            if(mAverageNumber == mTotalAverageNumber-1){

                int seed = 0;

                enc.mTimestamp = raw10.mTimestamp;
                enc.mLongExposureTime = raw10.mLongExposureTime ;
                enc.mShortExposureTime = raw10.mShortExposureTime;
                enc.mMainAlsEnabled = raw10.mMainAlsEnabled;
                enc.mMainAls = raw10.mMainAls;
                enc.mSlaveAlsEnabled = raw10.mSlaveAlsEnabled;
                enc.mSlaveAls = raw10.mSlaveAls;
                enc.mGpsEnabled = raw10.mGpsEnabled;
                enc.mLocation = raw10.mLocation;
                enc.mMainAls.clear = seed;    // store seed to ALS Clear channel

                writeHassInformation(enc);

                refRaw10.close();

                //to storage and USB
                setChanged();
                notifyObservers(refEnc);
                refEnc.close();

                //to display
                if(mDisplayOff == 0){
                    setChanged();
                    notifyObservers(refGrayscale);
                }
                refGrayscale.close();
            }
            else {
                refRaw10.close();
                refEnc.close();
                refGrayscale.close();
            }
            mAverageNumber++;
            if(mAverageNumber >= mTotalAverageNumber) mAverageNumber = 0;
        }
    }


    private RefCountedAutoCloseable<HassImage> getHassImage() {
        RefCountedAutoCloseable<HassImage> refHassImage = mHassImages[mHassImageIndex++];
        while (refHassImage.inUse()) {
            if (mHassImageIndex == IMAGE_QUEUE_SIZE) {
                mHassImageIndex = 0;
            }
            refHassImage = mHassImages[mHassImageIndex++];
        }
        if (mHassImageIndex == IMAGE_QUEUE_SIZE) {
            mHassImageIndex = 0;
        }
        return refHassImage;
    }

    private RefCountedAutoCloseable<HassImage> getHassEncImage() {
        RefCountedAutoCloseable<HassImage> refHassEncImage = mHassEncImages[mHassEncImageIndex++];
        while (refHassEncImage.inUse()) {
            if (mHassEncImageIndex == IMAGE_QUEUE_SIZE) {
                mHassEncImageIndex = 0;
            }
            refHassEncImage = mHassEncImages[mHassEncImageIndex++];
        }
        if (mHassEncImageIndex == IMAGE_QUEUE_SIZE) {
            mHassEncImageIndex = 0;
        }
        return refHassEncImage;
    }

    private RefCountedAutoCloseable<HassImage> getHassDisponlyImage() {
        RefCountedAutoCloseable<HassImage> refHassImage = mHassDisponlyImages[mHassDisponlyImageIndex++];
        while (refHassImage.inUse()) {
            if (mHassDisponlyImageIndex == IMAGE_QUEUE_SIZE) {
                mHassDisponlyImageIndex = 0;
            }
            refHassImage = mHassDisponlyImages[mHassDisponlyImageIndex++];
        }
        if (mHassDisponlyImageIndex == IMAGE_QUEUE_SIZE) {
            mHassDisponlyImageIndex = 0;
        }
        return refHassImage;
    }

    private void writeHassInformation(HassImage src) {
        // ****************ALS data / Encription key***********************
        final int ALS_DATA_SIZE = 10;
        final int ENQ_DATA_SIZE = 2;

        int pos = src.mWidth * src.mHeight * 2;

        //if (ALSSensor.isEnabled()) {
        //    src.mPixels.put(pos, (byte)(src.mMainAls.clear & 0xFF        )); pos++;
        //    src.mPixels.put(pos, (byte)((src.mMainAls.clear >> 8) & 0xFF )); pos++;
        //    src.mPixels.put(pos, (byte)(src.mMainAls.red & 0xFF          )); pos++;
        //    src.mPixels.put(pos, (byte)((src.mMainAls.red >> 8) & 0xFF   )); pos++;
        //    src.mPixels.put(pos, (byte)(src.mMainAls.green & 0xFF        )); pos++;
        //    src.mPixels.put(pos, (byte)((src.mMainAls.green >> 8) & 0xFF )); pos++;
        //    src.mPixels.put(pos, (byte)(src.mMainAls.blue & 0xFF         )); pos++;
        //    src.mPixels.put(pos, (byte)((src.mMainAls.blue >> 8) & 0xFF  )); pos++;
        //    src.mPixels.put(pos, (byte)(src.mMainAls.infrared & 0xFF     )); pos++;
        //    src.mPixels.put(pos, (byte)((src.mMainAls.infrared >> 8) & 0xFF )); pos++;
        //} else
        {
            for (int i = ENQ_DATA_SIZE; i < ALS_DATA_SIZE; i++) {
                src.mPixels.put(pos,(byte)0); pos++;
            }
            // write offset which is generated by random() to ALS Clear channel
            src.mPixels.put(pos, (byte)(src.mMainAls.clear & 0xFF       )); pos++;
            src.mPixels.put(pos, (byte)((src.mMainAls.clear >> 8) & 0xFF)); pos++;
        }

        // ****************GPS data **********************************


        final int GPS_DATA_SIZE = 40;
        if (src.mGpsEnabled && (src.mLocation != null)) {
            src.mPixels.putLong(pos, src.mLocation.getTime()).order(ByteOrder.LITTLE_ENDIAN); pos+=8;
            src.mPixels.putDouble(pos, src.mLocation.getLatitude()).order(ByteOrder.LITTLE_ENDIAN); pos+=8;
            src.mPixels.putDouble(pos, src.mLocation.getLongitude()).order(ByteOrder.LITTLE_ENDIAN); pos+=8;
            src.mPixels.putDouble(pos, src.mLocation.getAltitude()).order(ByteOrder.LITTLE_ENDIAN); pos+=8;
            src.mPixels.putFloat(pos, src.mLocation.getSpeed()).order(ByteOrder.LITTLE_ENDIAN); pos+=4;
            src.mPixels.putFloat(pos, src.mLocation.getBearing()).order(ByteOrder.LITTLE_ENDIAN); pos+=4;
        } else {
            for (int i = 0; i < GPS_DATA_SIZE; i++) {
                src.mPixels.put(pos, (byte)1); pos++;
            }
        }

        // ****************Other Meta data **********************************

        final int META_DATA_VERSION = 0x0001;
        src.mPixels.put(pos,(byte)(META_DATA_VERSION & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((META_DATA_VERSION >> 8) & 0xFF)); pos+=1;

        final int CAMERA_VERSION = 0x0010;
        src.mPixels.put(pos,(byte)(CAMERA_VERSION & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((CAMERA_VERSION >> 8) & 0xFF)); pos+=1;

        src.mPixels.put(pos,(byte)(mLensNumber & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((mLensNumber >> 8) & 0xFF)); pos+=1;

        src.mPixels.put(pos,(byte)(mCameraSerialNumber & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((mCameraSerialNumber >> 8) & 0xFF)); pos+=1;

        src.mPixels.put(pos,(byte)(mSensorIDNum & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((mSensorIDNum >> 8) & 0xFF)); pos+=1;

        src.mPixels.put(pos,(byte)(mSensorTypeNum & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((mSensorTypeNum >> 8) & 0xFF)); pos+=1;

        src.mPixels.putFloat(pos, src.mLongExposureTime).order(ByteOrder.LITTLE_ENDIAN); pos+=4;
        src.mPixels.putFloat(pos, src.mShortExposureTime).order(ByteOrder.LITTLE_ENDIAN); pos+=4;
        src.mPixels.putLong(pos, src.mTimestamp).order(ByteOrder.LITTLE_ENDIAN); pos+=8;

        src.mPixels.put(pos,(byte)(mTempareture & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((mTempareture >> 8) & 0xFF)); pos+=1;

        src.mPixels.put(pos,(byte)(dbg_frame_counter & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((dbg_frame_counter >> 8) & 0xFF)); pos+=1;

        src.mPixels.put(pos,(byte)1); pos+=1;
        src.mPixels.put(pos,(byte)60); pos+=1;

        src.mPixels.put(pos,(byte)(memory_size & 0xFF)); pos+=1;
        src.mPixels.put(pos,(byte)((memory_size>> 8) & 0xFF)); pos+=1;

        dbg_frame_counter++;
    }

    /**
     * Return set tempareture
     * @return none
     */
    public void setTempareture(int tmp_Tempareture) {
        mTempareture = tmp_Tempareture;
    }

    public void setLensInformation(int iLensNumber, int iCameraSerialNumber) {
        mLensNumber             = iLensNumber;
        mCameraSerialNumber     = iCameraSerialNumber;
    }

    public void setSensorInformation(int iSensorIDNum, int iSensorTypeNum) {
        mSensorIDNum            = iSensorIDNum;
        mSensorTypeNum          = iSensorTypeNum;
    }
}

