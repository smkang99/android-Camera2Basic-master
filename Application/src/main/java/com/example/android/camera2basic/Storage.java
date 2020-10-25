package com.example.android.camera2basic;

import android.util.Log;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.location.Location;
import android.media.ExifInterface;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;

import com.example.android.camera2basic.Camera2BasicFragment;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.Channels;
import java.nio.channels.WritableByteChannel;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Observable;
import java.util.Observer;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.Random;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.ImageFormat.NV21;
import static android.graphics.ImageFormat.RAW10;
import static android.location.Location.convert;

/**
 * Created by Allan Suen on 11/29/2016.
 */

public class Storage implements Observer {
    private static final int RGB2JPEG = 0;
    private static final int RGB2PPM = 1;
    private static final int RAW2BIN = 2;
    private static final int RAW10 = 3;

    private static final int JPEG_QUALITY = 50;

    private Bitmap mBitmap;
    private String mSdCardDir;
    private int mFramesSaved;
    private boolean mEnableRecording;
    private int mTimeLapse;
    private long mPreviousTimestamp;

    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;

    private ArrayBlockingQueue<RefCountedAutoCloseable<HassImage>> mImageQueue;

    static {
        System.loadLibrary("imgproc-lib");
    }

    /**
     * Observer receiving object from ImageProcessing/GpuImageProcessing
     * @param o ImageProcessing/GpuImageProcessing object
     * @param arg HassImage object
     */
    @Override
    public void update(Observable o, Object arg) {
        if (!isSdCardDetected()) return;

        RefCountedAutoCloseable<HassImage> refHassImage = (RefCountedAutoCloseable<HassImage>)arg;
        if (enqueue(refHassImage)) {
            HassImage hassImage = refHassImage.getAndRetain();
            switch (hassImage.mType) {
                case HassImage.TYPE_RGB:
                    saveRgb2Jpeg();
                    break;
                case HassImage.TYPE_RAW:
                case HassImage.TYPE_ENC:
                    saveRaw2Bin();
                    break;
                case HassImage.TYPE_RAW10:
                    saveRaw10();
                    break;
                case HassImage.TYPE_DISPONLY:
                    break;
            }
        }
    }

    /**
     * Creates Storage object
     * @param sdCardDir path to SD card
     */
    public Storage(String sdCardDir) {
        mSdCardDir = sdCardDir;
        mBitmap = createBitmap(Camera2BasicFragment.HD_WIDTH, Camera2BasicFragment.HD_HEIGHT, Bitmap.Config.ARGB_8888);
        mImageQueue = new ArrayBlockingQueue<RefCountedAutoCloseable<HassImage>>(2, true);
    }

    /**
     * Return path to SD card from external media paths
     * @param externalMediaDirs external media paths
     * @return path to SD card
     */
    public static String getSdCardDir(File[] externalMediaDirs) {
        for (File dir : externalMediaDirs) {
            if (Environment.isExternalStorageRemovable(dir)) {
                String folder_path = null;
                String main_folder_path = dir.getAbsolutePath();
                int folder_number = 0;
                int folder_max_num = 9999;

                while(folder_number < folder_max_num) {
                    String sub_folder_path = String.format("%04d",folder_number);
                    folder_path = main_folder_path + "/RAW" + sub_folder_path;

                    File newDir = new File(folder_path);
                    if(!newDir.exists()){
                        newDir.mkdir();
                        break;
                    }
                    else{
                        folder_number++;
                    }
                }

                return (folder_path);
            }
        }
        return null;
    }

    /**
     * Start background thread for Storage class
     */
    public void startBackgroundThread() {
        if (mBackgroundThread == null) {
            mBackgroundThread = new HandlerThread("StorageThread");
            mBackgroundThread.start();
            mBackgroundHandler = new Handler(mBackgroundThread.getLooper()) {
                @Override
                public void handleMessage(Message msg) {
                    if (mImageQueue.size() >= 1) {
                        RefCountedAutoCloseable<HassImage> refHassImage = mImageQueue.poll();
                        HassImage hassImage = refHassImage.get();

                        long time_diff = hassImage.mTimestamp - mPreviousTimestamp;
                        long target_diff = (long)(mTimeLapse*1000);

                        if((time_diff >= target_diff) || (time_diff <= 0) || (mPreviousTimestamp == 0)){
                            switch (msg.what) {
                                case RGB2JPEG:
                                    rgb2Jpeg(hassImage);
                                    break;
                                case RGB2PPM:
                                    rgb2Ppm(hassImage);
                                    break;
                                case RAW2BIN:
                                    raw2Bin(hassImage);
                                    break;
                                case RAW10:
                                    raw10(hassImage);
                                    break;
                            }
                            mPreviousTimestamp = hassImage.mTimestamp;
                        }
                        hassImage.close();
                        refHassImage.close();
                    }
                    super.handleMessage(msg);
                }
            };
        }
    }

    /**
     * Stop background thread for Storage class
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

    /**
     * Return whether SD card is detected
     * @return SD card status
     */
    public boolean isSdCardDetected() {
        return (mSdCardDir != null);
    }

    /**
     * Return whether Recording is enabled
     * @return Recording status
     */
    public void setEnableRecording(boolean bEnableRecording) {
        mEnableRecording = bEnableRecording;
    }

    /**
     * Return whether Recording is enabled
     * @return Recording status
     */
    public boolean isEnableRecording() {
        return ((mSdCardDir != null) & (mEnableRecording == true));
    }

    /**
     * Return whether Recording is enabled
     * @return Recording status
     */
    public void setTimeLapse(int iTimeLapse) {
        mTimeLapse = iTimeLapse;
    }

    /**
     * Return whether Recording is enabled
     * @return Recording status
     */
    public void initTimeLapse() {
        mPreviousTimestamp = 0;
    }

    /**
     * Counter keeps track of frames saved to SD per second and reset counter
     * @return counter
     */
    public int getFrameCountAndReset() {
        int framesSaved = mFramesSaved;
        mFramesSaved = 0;
        return framesSaved;
    }

    private boolean saveRgb2Jpeg() {
        mBackgroundHandler.obtainMessage(RGB2JPEG).sendToTarget();
        return true;
    }

    private boolean saveRgb2Ppm() {
        mBackgroundHandler.obtainMessage(RGB2PPM).sendToTarget();
        return true;
    }

    private boolean saveRaw2Bin() {
        mBackgroundHandler.obtainMessage(RAW2BIN).sendToTarget();
        return true;
    }

    private boolean saveRaw10() {
        mBackgroundHandler.obtainMessage(RAW10).sendToTarget();
        return true;
    }

    private boolean enqueue(RefCountedAutoCloseable<HassImage> refHassImage) {
        switch (refHassImage.get().mType) {
            case HassImage.TYPE_RGB:
            case HassImage.TYPE_RAW:
            case HassImage.TYPE_ENC:
            case HassImage.TYPE_RAW10:
                return mImageQueue.offer(refHassImage);
            default:
                return false;
        }
    }

    private void rgb2Jpeg(HassImage hassImage) {
        String path = "IMG_" + milliseconds2FilenameString(hassImage.mTimestamp) + ".jpg";
        File file = new File(mSdCardDir, path);
        try (FileOutputStream fos = new FileOutputStream(file)) {
            //hassImage.mReadLock.lock();
            mBitmap.copyPixelsFromBuffer(hassImage.mPixels);
            hassImage.mPixels.rewind();
            //hassImage.mReadLock.unlock();
            if (mBitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, fos)) {
                mFramesSaved++;
            }

            setExif(file.getAbsolutePath(), hassImage);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void rgb2Ppm(HassImage hassImage) {
        String path = "IMG_" + milliseconds2FilenameString(hassImage.mTimestamp) + ".ppm";
        File file = new File(mSdCardDir, path);
        try (FileOutputStream fos = new FileOutputStream(file)) {
            fos.write("P6\n".getBytes(StandardCharsets.UTF_8));
            String imageSize = Camera2BasicFragment.HD_WIDTH + " " + Camera2BasicFragment.HD_HEIGHT + "\n";
            fos.write(imageSize.getBytes(StandardCharsets.UTF_8));
            fos.write("255\n".getBytes(StandardCharsets.UTF_8));
            WritableByteChannel channel = Channels.newChannel(fos);
            //hassImage.mReadLock.lock();
            channel.write(hassImage.mPixels);
            hassImage.mPixels.rewind();
            //hassImage.mReadLock.unlock();
            mFramesSaved++;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void raw2Bin(HassImage hassImage) {
        String path;

        if(hassImage.mGpsEnabled) {
            long timestamp_gps = hassImage.mLocation.getTime();
            path = "IMG_" + seconds2FilenameString(timestamp_gps) + "_" + timeonly2FilenameString(hassImage.mTimestamp) + ".bin";
        }
        else {
            path = "IMG_" + milliseconds2FilenameString(hassImage.mTimestamp) + ".bin";
        }

        File file = new File(mSdCardDir, path);
        try (FileOutputStream fos = new FileOutputStream(file)) {
            fos.write(hassImage.mWidth & 0xFF);
            fos.write((hassImage.mWidth >> 8) & 0xFF);
            fos.write(hassImage.mHeight & 0xFF);
            fos.write((hassImage.mHeight >> 8) & 0xFF);
            WritableByteChannel channel = Channels.newChannel(fos);
            //hassImage.mReadLock.lock();
            channel.write(hassImage.mPixels);
            hassImage.mPixels.rewind();
            //hassImage.mReadLock.unlock();

            mFramesSaved++;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void raw10(HassImage hassImage) {
        String path = "IMG_" + milliseconds2FilenameString(hassImage.mTimestamp) + ".raw10";
        File file = new File(mSdCardDir, path);
        try (FileOutputStream fos = new FileOutputStream(file)) {
            WritableByteChannel channel = Channels.newChannel(fos);
            //hassImage.mReadLock.lock();
            channel.write(hassImage.mPixels);
            hassImage.mPixels.rewind();
            //hassImage.mReadLock.unlock();
            mFramesSaved++;

            String jpegPath = "ALS_" + milliseconds2FilenameString(hassImage.mTimestamp) + ".jpg";
            File jpeg = new File(mSdCardDir, jpegPath);
            setExif(jpeg.getAbsolutePath(), hassImage);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private String milliseconds2FilenameString(long time) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US);
        return sdf.format(new Date(time));
    }

    private String seconds2FilenameString(long time) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
        return sdf.format(new Date(time));
    }

    private String timeonly2FilenameString(long time) {
        SimpleDateFormat sdf = new SimpleDateFormat("HHmmssSSS", Locale.US);
        return sdf.format(new Date(time));
    }

    private String milliseconds2ExifString(long time) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy:MM:dd HH:mm:ss", Locale.US);
        return sdf.format(new Date(time));
    }

    private void setExif(String filename, HassImage hassImage) {
        ExifInterface exif = null;
        try {
            exif = new ExifInterface(filename);
            exif.setAttribute(ExifInterface.TAG_DATETIME, milliseconds2ExifString(hassImage.mTimestamp));
            if (hassImage.mMainAlsEnabled || hassImage.mSlaveAlsEnabled) {
                StringBuilder builder = new StringBuilder();
                builder.append("Hass ALS: R-");
                builder.append(hassImage.mMainAls.red);
                builder.append(", G-");
                builder.append(hassImage.mMainAls.green);
                builder.append(", B-");
                builder.append(hassImage.mMainAls.blue);
                builder.append(", C-");
                builder.append(hassImage.mMainAls.clear);
                builder.append(", I-");
                builder.append(hassImage.mMainAls.infrared);
                if (hassImage.mSlaveAlsEnabled) {
                    builder.append(" Sunlight ALS: R-");
                    builder.append(hassImage.mSlaveAls.red);
                    builder.append(", G-");
                    builder.append(hassImage.mSlaveAls.green);
                    builder.append(", B-");
                    builder.append(hassImage.mSlaveAls.blue);
                    builder.append(", C-");
                    builder.append(hassImage.mSlaveAls.clear);
                    builder.append(", I-");
                    builder.append(hassImage.mSlaveAls.infrared);
                }
                exif.setAttribute(ExifInterface.TAG_USER_COMMENT, builder.toString());
            }
            if (hassImage.mGpsEnabled && hassImage.mLocation != null) {
                Date now = new Date(hassImage.mLocation.getTime());
                SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd");
                String date = dateFormat.format(now);
                exif.setAttribute(ExifInterface.TAG_GPS_DATESTAMP, date);
                double latitude = hassImage.mLocation.getLatitude();
                if (latitude < 0) {
                    exif.setAttribute(ExifInterface.TAG_GPS_LATITUDE_REF, "S");
                } else {
                    exif.setAttribute(ExifInterface.TAG_GPS_LATITUDE_REF, "N");
                }
                String latitudeDms = Location.convert(Math.abs(latitude), Location.FORMAT_SECONDS);
                String[] latitudeSplit = latitudeDms.split(":");
                int latitudeSeconds = (int)(Double.valueOf(latitudeSplit[2]) * 10000);
                String latitudeRational = latitudeSplit[0] + "/1," + latitudeSplit[1] + "/1," + latitudeSeconds + "/10000";
                exif.setAttribute(ExifInterface.TAG_GPS_LATITUDE, latitudeRational);
                double longitude = hassImage.mLocation.getLongitude();
                if (longitude < 0) {
                    exif.setAttribute(ExifInterface.TAG_GPS_LONGITUDE_REF, "W");
                } else {
                    exif.setAttribute(ExifInterface.TAG_GPS_LONGITUDE_REF, "E");
                }
                String longitudeDms = Location.convert(Math.abs(longitude), Location.FORMAT_SECONDS);
                String[] longitudeSplit = longitudeDms.split(":");
                int longitudeSeconds = (int)(Double.valueOf(longitudeSplit[2]) * 10000);
                String longitudeRational = longitudeSplit[0] + "/1," + longitudeSplit[1] + "/1," + longitudeSeconds + "/10000";
                exif.setAttribute(ExifInterface.TAG_GPS_LONGITUDE, longitudeRational);
                double altitude = hassImage.mLocation.getAltitude();
//                exif.setAttribute(ExifInterface.TAG_GPS_ALTITUDE_REF, "0");
//                exif.setAttribute(ExifInterface.TAG_GPS_ALTITUDE, "");
                float speed = hassImage.mLocation.getSpeed();
                exif.setAttribute(ExifInterface.TAG_GPS_SPEED_REF, "K");
                int speedMeterPerHour = (int)(speed * 3600);
                String speedRational = speedMeterPerHour + "/1000";
                exif.setAttribute(ExifInterface.TAG_GPS_SPEED, speedRational);
                float bearing = hassImage.mLocation.getBearing();
//                exif.setAttribute(ExifInterface.TAG_GPS_DEST_BEARING_REF, "T");
//                exif.setAttribute(ExifInterface.TAG_GPS_DEST_BEARING, "");

                // Storing altitude and bearing in status for now
                String status = "Altitude: " + altitude + "m, Bearing: " + bearing + "°";
                exif.setAttribute(ExifInterface.TAG_GPS_STATUS, status);
            }
            exif.saveAttributes();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
