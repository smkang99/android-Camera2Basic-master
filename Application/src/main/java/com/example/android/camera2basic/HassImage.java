package com.example.android.camera2basic;

import android.location.Location;
import android.media.Image;

import java.nio.ByteBuffer;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

//import static com.example.android.camera2basic.LED.ROOT_PATH;

/**
 * Created by Allan Suen on 1/26/2017.
 */

public class HassImage implements AutoCloseable {
    public static final int TYPE_RAW = 0;
    public static final int TYPE_RAW10 = 1;
    public static final int TYPE_YUV = 2;
    public static final int TYPE_RGB = 3;
    public static final int TYPE_PPM = 4;
    public static final int TYPE_GRAYSCALE = 5;
    public static final int TYPE_DISPONLY = 6;
    public static final int TYPE_ENC = 7;

    public static final int META_DATA_SIZE = 86;

    public ByteBuffer mPixels;
    public int mWidth;
    public int mHeight;
    public int mType;
    public long mTimestamp;
    public boolean mMainAlsEnabled;
    public ALS mMainAls;
    public boolean mSlaveAlsEnabled;
    public ALS mSlaveAls;
    public boolean mGpsEnabled;
    public Location mLocation;
    public Image mImage;
    public float mLongExposureTime;
    public float mShortExposureTime;

    private final ReentrantReadWriteLock mBufferLock = new ReentrantReadWriteLock();
    public final ReentrantReadWriteLock.ReadLock mReadLock = mBufferLock.readLock();
    public final ReentrantReadWriteLock.WriteLock mWriteLock = mBufferLock.writeLock();

    /**
     * Create HassImage object
     * @param width image width
     * @param height image height
     * @param type image type
     */
    public HassImage(int width, int height, int type) {
        mType = type;
        mWidth = width;
        mHeight = height;
        mMainAls = new ALS();
        mSlaveAls = new ALS();
        switch (mType) {
            case TYPE_RAW:
                mPixels = ByteBuffer.allocateDirect(mWidth * mHeight * 2);
                break;
            case TYPE_ENC:
                mPixels = ByteBuffer.allocateDirect(mWidth * mHeight * 2 + META_DATA_SIZE);
                break;
            case TYPE_RAW10:
                mPixels = ByteBuffer.allocateDirect(mWidth * mHeight * 10 / 8);
                break;
            case TYPE_YUV:
                mPixels = ByteBuffer.allocateDirect(mWidth * mHeight * 2 - 2);
                break;
            case TYPE_RGB:
            case TYPE_GRAYSCALE:
            case TYPE_DISPONLY:
                mPixels = ByteBuffer.allocateDirect(mWidth * mHeight * 4);
                break;
            case TYPE_PPM:
                mPixels = ByteBuffer.allocateDirect(mWidth * mHeight * 3);
                break;
        }
    }

    /**
     * Create HassImage object
     * @param width image width
     * @param height image height
     * @param type image type
     */
    public HassImage(int width, int height, int type, ByteBuffer pixels) {
        mType = type;
        mWidth = width;
        mHeight = height;
        mMainAls = new ALS();
        mSlaveAls = new ALS();
        mPixels = pixels;
    }

    /**
     * Create HassImage object
     * @param pixels image pixel data
     * @param width image width
     * @param height image height
     */
    public HassImage(byte[] pixels, int width, int height) {
        mPixels = ByteBuffer.allocateDirect(pixels.length);
        mPixels.put(pixels);
        mPixels.rewind();
        mWidth = width;
        mHeight = height;
    }

    /**
     * Close Image object if it exist
     */
    @Override
    public void close() {
        try {
            if (mImage != null) {
                mImage.close();
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        } finally {
            mImage = null;
        }
    }

    /**
     * ALS static nested class
     */
    public static final class ALS {
        public int red;
        public int green;
        public int blue;
        public int clear;
        public int infrared;
    }
}
