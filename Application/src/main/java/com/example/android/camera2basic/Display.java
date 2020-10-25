package com.example.android.camera2basic;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.view.TextureView;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.android.camera2basic.Camera2BasicFragment;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Observable;
import java.util.Observer;

import static android.R.attr.data;
import static android.graphics.Bitmap.createBitmap;
import static org.xmlpull.v1.XmlPullParser.TEXT;

/**
 * Created by Allan Suen on 11/29/2016.
 */

public class Display implements Observer {
    private static final int TOAST = 0;
    private static final int STATUS = 1;
    private static final int BITMAP = 2;
    private static final int RGB = 3;
    private static final int IMAGE_DATA = 4;

    private Activity mActivity;
    private ImageView mImageView;
    private TextView mTextStatus;
    private Bitmap mBitmap;
    private int mFramesDisplayed;

    private Handler mBackgroundHandler;

    /**
     * Observer receiving object from ImageProcessing/GpuImageProcessing
     * @param o ImageProcessing/GpuImageProcessing object
     * @param arg HassImage object
     */
    @Override
    public void update(Observable o, Object arg) {
        RefCountedAutoCloseable<HassImage> refHassImage = (RefCountedAutoCloseable<HassImage>)arg;
        switch (refHassImage.get().mType) {
            case HassImage.TYPE_RGB:
        	case HassImage.TYPE_GRAYSCALE:
            case HassImage.TYPE_DISPONLY:
                refHassImage.getAndRetain();
                displayHassImage(refHassImage);
                break;
            case HassImage.TYPE_RAW:
            case HassImage.TYPE_ENC:
            	break;
            default:
        	    break;
        }
    }

    /**
     * Create Display object
     * @param activity activity with UI thread
     */
    public Display(Activity activity) {
        mActivity = activity;
        mImageView = (ImageView)mActivity.findViewById(R.id.texture);
        mTextStatus = (TextView)mActivity.findViewById(R.id.textStatus);
        mBitmap = createBitmap(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, Bitmap.Config.ARGB_8888);

        mBackgroundHandler = new Handler(Looper.getMainLooper()) {
            @Override
            public void handleMessage(Message msg) {
                switch (msg.what) {
                    case TOAST:

                        break;
                    case STATUS:
                        mTextStatus.setText((String)msg.obj);
                        break;
                    case BITMAP:
                        mImageView.setImageBitmap((Bitmap)msg.obj);
                        mFramesDisplayed++;
                        break;
                    case RGB:
                        mBitmap.copyPixelsFromBuffer(ByteBuffer.wrap((byte[])msg.obj));
                        mImageView.setImageBitmap(mBitmap);
                        mFramesDisplayed++;
                        break;
                    case IMAGE_DATA:
                        RefCountedAutoCloseable<HassImage> refHassImage = (RefCountedAutoCloseable<HassImage>)msg.obj;
                        HassImage hassImage = refHassImage.get();
                        //hassImage.mReadLock.lock();
                        mBitmap.copyPixelsFromBuffer(hassImage.mPixels);
                        hassImage.mPixels.rewind();
                        //hassImage.mReadLock.unlock();
                        refHassImage.close();
                        mImageView.setImageBitmap(mBitmap);
                        mFramesDisplayed++;
                        break;
                }
                super.handleMessage(msg);
            }
        };
    }

    /**
     * Counter keeps track of frames displayed per second and reset counter
     * @return counter
     */
    public int getFrameCountAndReset() {
        int framesDisplayed = mFramesDisplayed;
        mFramesDisplayed = 0;
        return framesDisplayed;
    }

    /**
     * Display system status
     * @param text
     */
    public void displayStatus(String text) {
        mBackgroundHandler.obtainMessage(STATUS, text).sendToTarget();
    }

    private void displayBitmap(Bitmap bmp) {
        mBackgroundHandler.obtainMessage(BITMAP, bmp).sendToTarget();
    }

    private void displayRgb(byte[] rgbBytes) {
        mBackgroundHandler.obtainMessage(RGB, rgbBytes).sendToTarget();
    }

    private void displayHassImage(RefCountedAutoCloseable<HassImage> refHassImage) {
        mBackgroundHandler.obtainMessage(IMAGE_DATA, refHassImage).sendToTarget();
    }
}
