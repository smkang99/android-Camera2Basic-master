package com.example.android.camera2basic;

import android.location.Location;

import java.nio.ByteBuffer;
import java.util.Observable;
import java.util.Observer;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Created by Allan Suen on 7/26/2017.
 */

public class USB implements Observer {
    public static final int MODE_ADB = 0;
    public static final int MODE_ACCESSORY = 1;

    private AtomicBoolean mTransferringImage;
    private int mFramesTransferred;
    private ArrayBlockingQueue<RefCountedAutoCloseable<HassImage>> mImageQueue;
    private RefCountedAutoCloseable<HassImage> mCurRefHassImage;

    static {
        System.loadLibrary("imgproc-lib");
    }

    private native boolean reigsterNativeMethods();
    private native void rgbFrame(ByteBuffer frame, long timestamp,
                                 boolean mainAlsEnabled, HassImage.ALS main,
                                 boolean slaveAlsEnabled, HassImage.ALS slave,
                                 boolean gpsEnabled, Location location);
    private native void rawFrame(ByteBuffer frame, long timestamp,
                                 boolean mainAlsEnabled, HassImage.ALS main,
                                 boolean slaveAlsEnabled, HassImage.ALS slave,
                                 boolean gpsEnabled, Location location);
    private native void setAccessoryMode();
    private native void setAdbMode();

    private USB() {
        mTransferringImage = new AtomicBoolean(false);
        mImageQueue = new ArrayBlockingQueue<RefCountedAutoCloseable<HassImage>>(2, true);
        reigsterNativeMethods();
    }

    static class SingletonHolder {
        static USB instance = new USB();
    }

    /**
     * Uses singleton pattern to create or return an USB object
     * @return USB object reference
     */
    public static USB getDevice() {
        return SingletonHolder.instance;
    }

    /**
     * Observer receiving object from ImageProcessing/GpuImageProcessing
     * @param o ImageProcessing/GpuImageProcessing object
     * @param arg HassImage object
     */
    @Override
    public void update(Observable o, Object arg) {
        if (mTransferringImage.compareAndSet(false, true)) {
            mCurRefHassImage = (RefCountedAutoCloseable<HassImage>)arg;
            HassImage hassImage = mCurRefHassImage.getAndRetain();

            switch (hassImage.mType) {
                case HassImage.TYPE_RGB:
                case HassImage.TYPE_GRAYSCALE:
                    //hassImage.mReadLock.lock();
                    rgbFrame(hassImage.mPixels, hassImage.mTimestamp,
                            hassImage.mMainAlsEnabled, hassImage.mMainAls,
                            hassImage.mSlaveAlsEnabled, hassImage.mSlaveAls,
                            hassImage.mGpsEnabled, hassImage.mLocation);
                    break;
                case HassImage.TYPE_RAW:
                case HassImage.TYPE_ENC:
                    //hassImage.mReadLock.lock();
                    rawFrame(hassImage.mPixels, hassImage.mTimestamp,
                            hassImage.mMainAlsEnabled, hassImage.mMainAls,
                            hassImage.mSlaveAlsEnabled, hassImage.mSlaveAls,
                            hassImage.mGpsEnabled, hassImage.mLocation);
                    break;
//                case HassImage.TYPE_RAW10:
//                    raw10Frame(hassImage.mPixels, (int)hassImage.mTimestamp, mainAls, slaveAls);
//                    break;
                default:
                    mCurRefHassImage.close();
                    mTransferringImage.set(false);
                    break;
            }
        }
    }

    /**
     * Counter keeps track of frames transferred per second and reset counter
     * @return counter
     */
    public int getFrameCountAndReset() {
        int framesTransferred = mFramesTransferred;
        mFramesTransferred = 0;
        return framesTransferred;
    }

    /**
     * Set USB connection mode
     * @param mode
     */
    public void setMode(int mode) {
        switch (mode) {
            case MODE_ADB:
                setAdbMode();
                break;
            case MODE_ACCESSORY:
                setAccessoryMode();
                break;
        }
    }

    /**
     * Pause USB transfer
     * @param pause boolean state
     */
    public void pauseTransfer(boolean pause) {
        mTransferringImage.set(pause);
    }

    private void transferComplete(boolean success) {
        if (success) {
            mFramesTransferred++;
        }
        //mCurRefHassImage.get().mReadLock.unlock();
        mCurRefHassImage.close();
        mTransferringImage.set(false);
    }

    private boolean enqueue(RefCountedAutoCloseable<HassImage> refHassImage) {
        return mImageQueue.offer(refHassImage);
    }
}
