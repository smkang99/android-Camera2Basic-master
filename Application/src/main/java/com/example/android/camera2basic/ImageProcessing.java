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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Observable;
import java.util.Observer;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static android.R.attr.syncable;
import static android.R.attr.value;
import static android.R.attr.width;

/**
 * Created by Allan Suen on 1/12/2017.
 */

public class ImageProcessing extends Observable implements Observer {
    private static final int YUV2JPEG2RGB = 0;
    private static final int RAW2GRAYSCALE = 1;
    private static final int RAW102GRAYSCALE = 2;

    private static final int IMAGE_QUEUE_SIZE = Camera2BasicFragment.IMAGE_QUEUE_SIZE;

    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;

    private RefCountedAutoCloseable<HassImage>[] mRawImages;
    private int mRawImageIndex;
    private RefCountedAutoCloseable<HassImage>[] mGrayscaleImages;
    private int mGrayscaleImageIndex;
    private ArrayBlockingQueue<RefCountedAutoCloseable<HassImage>> mImageQueue;

    static {
        System.loadLibrary("imgproc-lib");
    }

    private native int[] yuv2Rgb(byte[] y, byte[] u, byte[] v, int width, int height);
    private native void jniRaw102Raw(ByteBuffer raw10, ByteBuffer raw);
    private native void jniRaw2Grayscale(ByteBuffer raw, ByteBuffer grayscale);

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
     * Create ImageProcessing object
     */
    public ImageProcessing() {
        mImageQueue = new ArrayBlockingQueue<>(Camera2BasicFragment.IMAGE_QUEUE_SIZE, true);
        mRawImages = new RefCountedAutoCloseable[IMAGE_QUEUE_SIZE];
        mGrayscaleImages = new RefCountedAutoCloseable[IMAGE_QUEUE_SIZE];
        for (int i = 0; i < IMAGE_QUEUE_SIZE; i++) {
            mRawImages[i] = new RefCountedAutoCloseable<>(new HassImage(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, HassImage.TYPE_RAW));
            mGrayscaleImages[i] = new RefCountedAutoCloseable<>(new HassImage(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, HassImage.TYPE_GRAYSCALE));
        }
    }

    /**
     * Start background thread for ImageProcessing class
     */
    public void startBackgroundThread() {
        if (mBackgroundThread == null) {
            mBackgroundThread = new HandlerThread("ImageProcessingThread");
            mBackgroundThread.start();
            mBackgroundHandler = new Handler(mBackgroundThread.getLooper()) {
                @Override
                public void handleMessage(Message msg) {
                    switch (msg.what) {
                        case YUV2JPEG2RGB:
                            yuv2Jpeg2Rgb();
                            break;
                        case RAW2GRAYSCALE:
                            raw2Grayscale();
                            break;
                        case RAW102GRAYSCALE:
                            raw102Grayscale();
                            break;
                    }
                    super.handleMessage(msg);
                }
            };
        }
    }

    /**
     * Stop background thread for ImageProcessing class
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

    private boolean enqueue(RefCountedAutoCloseable<HassImage> image) {
        return mImageQueue.offer(image);
    }

    private void processYuv() {
        mBackgroundHandler.obtainMessage(YUV2JPEG2RGB).sendToTarget();
    }

    private void processRaw() {
        mBackgroundHandler.obtainMessage(RAW2GRAYSCALE).sendToTarget();
    }

    private void processRaw10() {
        mBackgroundHandler.obtainMessage(RAW102GRAYSCALE).sendToTarget();
    }

    private void yuv2Jpeg2Rgb() {
        if (mImageQueue.size() >= 1) {
            RefCountedAutoCloseable<HassImage> refYuv = mImageQueue.poll();
            HassImage yuv = refYuv.get();
            //yuv.mReadLock.lock();
            YuvImage yuvImage = new YuvImage(yuv.mPixels.array(), ImageFormat.NV21, yuv.mWidth, yuv.mHeight, null);
            //yuv.mReadLock.unlock();
            ByteArrayOutputStream jpegStream = new ByteArrayOutputStream();
            yuvImage.compressToJpeg(new Rect(0, 0, yuv.mWidth, yuv.mHeight), 50, jpegStream);
            refYuv.close();
            byte[] jpegBytes = jpegStream.toByteArray();
            BitmapFactory.Options bitmapFatoryOptions = new BitmapFactory.Options();
            bitmapFatoryOptions.inPreferredConfig = Bitmap.Config.ARGB_8888;
            Bitmap bitmap = BitmapFactory.decodeByteArray(jpegBytes, 0, jpegBytes.length, bitmapFatoryOptions);
            RefCountedAutoCloseable<HassImage> refRgb = getGrayscaleImage();
            HassImage rgb = refRgb.get();
            //rgb.mWriteLock.lock();
            bitmap.copyPixelsToBuffer(rgb.mPixels);
            //rgb.mWriteLock.unlock();
            rgb.mTimestamp = yuv.mTimestamp;
            rgb.mMainAlsEnabled = yuv.mMainAlsEnabled;
            rgb.mMainAls = yuv.mMainAls;
            rgb.mSlaveAlsEnabled = yuv.mSlaveAlsEnabled;
            rgb.mSlaveAls = yuv.mSlaveAls;
            rgb.mGpsEnabled = yuv.mGpsEnabled;
            rgb.mLocation = yuv.mLocation;
            setChanged();
            notifyObservers(refRgb);
            refRgb.close();
        }
    }

    private void raw2Grayscale() {
        if (mImageQueue.size() >= 1) {
            RefCountedAutoCloseable<HassImage> refRaw = mImageQueue.poll();
            HassImage raw = refRaw.get();
            RefCountedAutoCloseable<HassImage> refGrayscale = getGrayscaleImage();
            HassImage grayscale = refGrayscale.getAndRetain();
            //raw.mReadLock.lock();
            //grayscale.mWriteLock.lock();
            synchronized (this) {
                jniRaw2Grayscale(raw.mPixels, grayscale.mPixels);
            }
            //raw.mReadLock.unlock();
            //grayscale.mWriteLock.unlock();
            grayscale.mTimestamp = raw.mTimestamp;
            grayscale.mMainAlsEnabled = raw.mMainAlsEnabled;
            grayscale.mMainAls = raw.mMainAls;
            grayscale.mSlaveAlsEnabled = raw.mSlaveAlsEnabled;
            grayscale.mSlaveAls = raw.mSlaveAls;
            grayscale.mGpsEnabled = raw.mGpsEnabled;
            grayscale.mLocation = raw.mLocation;
            refRaw.close();
            setChanged();
            notifyObservers(refGrayscale);
            refGrayscale.close();
        }
    }

    private void raw102Grayscale() {
        if (mImageQueue.size() >= 1) {
            RefCountedAutoCloseable<HassImage> refRaw10 = mImageQueue.poll();
            HassImage raw10 = refRaw10.get();
            RefCountedAutoCloseable<HassImage> refRaw = getRawImage();
            HassImage raw = refRaw.getAndRetain();
            //raw10.mReadLock.lock();
            //raw.mWriteLock.lock();
            synchronized (this) {
                jniRaw102Raw(raw10.mPixels, raw.mPixels);
            }
            //raw10.mReadLock.unlock();
            //raw.mWriteLock.unlock();
            raw.mTimestamp = raw10.mTimestamp;
            raw.mMainAlsEnabled = raw10.mMainAlsEnabled;
            raw.mMainAls = raw10.mMainAls;
            raw.mSlaveAlsEnabled = raw10.mSlaveAlsEnabled;
            raw.mSlaveAls = raw10.mSlaveAls;
            raw.mGpsEnabled = raw10.mGpsEnabled;
            raw.mLocation = raw10.mLocation;
            refRaw10.close();
            setChanged();
            notifyObservers(refRaw);

            RefCountedAutoCloseable<HassImage> refGrayscale = getGrayscaleImage();
            HassImage grayscale = refGrayscale.getAndRetain();
            //raw.mReadLock.lock();
            //grayscale.mWriteLock.lock();
            synchronized (this) {
                jniRaw2Grayscale(raw.mPixels, grayscale.mPixels);
            }
            //raw.mReadLock.unlock();
            //grayscale.mWriteLock.unlock();
            grayscale.mTimestamp = raw.mTimestamp;
            grayscale.mMainAlsEnabled = raw.mMainAlsEnabled;
            grayscale.mMainAls = raw.mMainAls;
            grayscale.mSlaveAlsEnabled = raw.mSlaveAlsEnabled;
            grayscale.mSlaveAls = raw.mSlaveAls;
            grayscale.mGpsEnabled = raw.mGpsEnabled;
            grayscale.mLocation = raw.mLocation;
            refRaw.close();
            setChanged();
            notifyObservers(refGrayscale);
            refGrayscale.close();
        }
    }

    private RefCountedAutoCloseable<HassImage> getRawImage() {
        RefCountedAutoCloseable<HassImage> refHassImage = mRawImages[mRawImageIndex++];
        while (refHassImage.inUse()) {
            if (mRawImageIndex == IMAGE_QUEUE_SIZE) {
                mRawImageIndex = 0;
            }
            refHassImage = mRawImages[mRawImageIndex++];
        }
        if (mRawImageIndex == IMAGE_QUEUE_SIZE) {
            mRawImageIndex = 0;
        }
        return refHassImage;
    }

    private RefCountedAutoCloseable<HassImage> getGrayscaleImage() {
        RefCountedAutoCloseable<HassImage> refHassImage = mGrayscaleImages[mGrayscaleImageIndex++];
        while (refHassImage.inUse()) {
            if (mGrayscaleImageIndex == IMAGE_QUEUE_SIZE) {
                mGrayscaleImageIndex = 0;
            }
            refHassImage = mGrayscaleImages[mGrayscaleImageIndex++];
        }
        if (mGrayscaleImageIndex == IMAGE_QUEUE_SIZE) {
            mGrayscaleImageIndex = 0;
        }
        return refHassImage;
    }
}

