package com.example.android.camera2basic;

import android.app.AlarmManager;
import android.app.PendingIntent;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.camera2.CameraManager;
import android.location.LocationManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.os.Process;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.Toast;

//import com.sony.isdc.musarc.camera.CameraContext;
import android.app.ActivityManager;
import android.app.ActivityManager.MemoryInfo;
import android.content.Context;

import java.math.BigInteger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static android.R.attr.checked;
import static android.R.attr.data;
import static android.R.attr.start;
import static android.R.attr.visibility;
import static android.R.id.content;
import static android.preference.PreferenceManager.getDefaultSharedPreferences;
import static com.example.android.camera2basic.Commander.Result.TYPE_INT;
import static com.example.android.camera2basic.HassSensor.read;
import static com.example.android.camera2basic.HassSensor.write;

/**
 * Created by Allan Suen on 11/29/2016.
 */

public class Commander {
    public static final int GET_CAMERA_HW_VERSION = 0;
    public static final int GET_CAMERA_SW_VERSION = 1;
    public static final int ENABLE_CAMERA = 2;
    public static final int DISABLE_CAMERA = 3;
    public static final int RESET_CAMERA = 4;
    public static final int SET_SENSOR_REGISTER = 5;
    public static final int GET_SENSOR_REGISTER = 6;
    public static final int SET_GAMMA = 7;
    public static final int GET_GAMMA = 8;
    public static final int GET_FILTER_COUNT = 9;
    public static final int GET_RECON_MATRIX = 10;
    public static final int GET_PIXEL_PATTERN = 11;
    public static final int GET_ALPHA_COEFFICIENT = 12;
    public static final int GET_NOISE_REDUCTION_COEFFICIENT = 13;
    public static final int SET_HASS_MODE = 14;
    public static final int TAKE_PICTURE = 15;
    public static final int ENABLE_VIDEO = 16;
    public static final int DISABLE_VIDEO = 17;
    public static final int ENABLE_DISPLAY = 18;
    public static final int DISABLE_DISPLAY = 19;
    public static final int ENABLE_STORAGE = 20;
    public static final int DISABLE_STORAGE = 21;
    public static final int GET_CAMERA_FRAME_RATE = 22;
    public static final int GET_DISPLAY_FRAME_RATE = 23;
    public static final int GET_STORAGE_FRAME_RATE = 24;
    public static final int SET_STATUS_DISPLAY = 25;
    public static final int DISPLAY_TOAST = 26;
    //public static final int ENABLE_DOUBLE_BUFFER_ISP = 27;
    //public static final int DISABLE_DOUBLE_BUFFER_ISP = 28;
    public static final int ENABLE_GREEN_LED = 29;
    public static final int DISABLE_GREEN_LED = 30;
    public static final int ENABLE_BACK_LIGHT = 31;
    public static final int DISABLE_BACK_LIGTH = 32;
    public static final int ENABLE_STATUS = 33;
    public static final int DISABLE_STATUS = 34;
    public static final int ENABLE_HIDDEN_OPTIONS = 35;
    public static final int DISABLE_HIDDEN_OPTIONS = 36;
    public static final int SET_SENSOR_FORMAT = 37;
    public static final int ENABLE_RAW_STORAGE = 38;
    public static final int DISABLE_RAW_STORAGE = 39;
    public static final int ENABLE_APP_CONTROLS = 40;
    public static final int DISABLE_APP_CONTROLS = 41;
    public static final int ENABLE_THROTTLE_FRAME_RATE = 42;
    public static final int DISABLE_THROTTLE_FRAME_RATE = 43;
    public static final int SET_THROTTLE_FRAME_RATE = 44;
    public static final int GET_TEMPERATURE_ZONE_22 = 45;
    public static final int SET_SHORT_EXPOSURE = 46;
    public static final int GET_SHORT_EXPOSURE = 47;
    public static final int SET_ALPHA_COEFFICIENT = 48;
    public static final int SET_NOISE_REDUCTION_COEFFICIENT = 49;
    public static final int ENABLE_ALS = 50;
    public static final int DISABLE_ALS = 51;
    public static final int GET_ALS = 52;
    public static final int SET_LONG_EXPOSURE = 53;
    public static final int GET_LONG_EXPOSURE = 54;
    public static final int ENABLE_IR_LED = 55;
    public static final int DISABLE_IR_LED = 56;
    public static final int SET_IR_LED_INTENSITY = 57;
    public static final int ENABLE_CONSTANT_FREQUENCY = 58;
    public static final int DISABLE_CONSTANT_FREQUENCY = 59;
    public static final int UPDATE_OVERHEAT_PROTECTION = 60;
    public static final int GET_OVERHEAT_PROTECTION_STATUS = 61;
    public static final int ENABLE_SYSTEM_CHECK = 62;
    public static final int DISABLE_SYSTEM_CHECK = 63;
    public static final int GET_USB_FRAME_RATE = 64;
    public static final int ENABLE_AUTO_EXPOSURE = 65;
    public static final int DISABLE_AUTO_EXPOSURE = 66;
    public static final int ENABLE_GPU = 67;
    public static final int DISABLE_GPU = 68;
    public static final int ENABLE_DUAL_CAMERA_ALS = 69;
    public static final int DISABLE_DUAL_CAMERA_ALS = 70;
    public static final int SEND_DUAL_CAMERA_ALS_DATA = 71;
    public static final int STOP_DUAL_CAMERA_ALS_DATA = 72;
    public static final int SET_FILTER_COUNT = 73;
    public static final int SET_PIXEL_PATTERN = 74;
    public static final int SET_RECON_MATRIX = 75;
    public static final int SET_HASS_CONFIGURATION = 76;
    public static final int GET_HASS_CONFIGURATION = 77;
    public static final int SET_HASS_OFFSET = 78;
    public static final int GET_HASS_OFFSET = 79;
    public static final int ENABLE_SNAPSHOT_STREAM = 80;
    public static final int DISABLE_SNAPSHOT_STREAM = 81;
    public static final int ENABLE_USB3 = 82;
    public static final int DISABLE_USB3 = 83;
    public static final int SERIAL_DISCOVERY = 84;
    public static final int RESET_STATE = 85;
    public static final int SET_ALS_REGISTER = 86;
    public static final int GET_ALS_REGISTER = 87;
    public static final int SET_GAIN = 88;
    public static final int GET_GAIN = 89;
    public static final int EXECUTE_OVER_SERIAL = 90;
    public static final int SET_VMAX = 91;
    public static final int GET_VMAX = 92;
    public static final int RESTART_CAMERA_STREAM = 93;
    public static final int SHUTDOWN_APP = 94;
    public static final int ENABLE_GPS = 95;
    public static final int DISABLE_GPS = 96;
    public static final int ENABLE_SERIAL = 97;
    public static final int DISABLE_SERIAL = 98;
    public static final int SET_USB_MODE = 99;
    public static final int GET_VIDEO_MODE = 100;
    public static final int SET_SCALING_GOVERNOR_MODE = 101;
    public static final int SET_AUTOEXPOSURESPEEDLONG = 102;
    public static final int GET_AUTOEXPOSURESPEEDLONG = 103;
    public static final int SET_DISPLAY_LSB = 104;
    public static final int SET_AUTOEXPOSURESPEEDSHORT = 105;
    public static final int GET_AUTOEXPOSURESPEEDSHORT = 106;
    public static final int SET_AUTOEXPOSURETARGET = 107;
    public static final int GET_AUTOEXPOSURETARGET = 108;
    public static final int SET_AUTOEXPOSURERANGE = 109;
    public static final int GET_AUTOEXPOSURERANGE = 110;
    public static final int ENABLE_AUTO_START_STREAMING = 111;
    public static final int DISABLE_AUTO_START_STREAMING = 112;
    public static final int SET_LENS_INFORMATION = 113;
    public static final int SET_SENSOR_INFORMATION = 114;
    public static final int APPLICATION_FORCE_RESTART = 115;
    public static final int GET_CAMERA_FRAME_RATE_READONLY = 116;
    public static final int HEAT_BEAT = 117;
    public static final int SET_AVERAGEFRAMENUM = 118;
    public static final int SET_TIMELAPSE_TIME = 119;
    public static final int GET_HPF = 120;
    public static final int ENABLE_FOCUS_POSITION_DETECTOR_MODE = 121;
    public static final int DISABLE_FOCUS_POSITION_DETECTOR_MODE = 122;
    public static final int MEMORY_LEAK_CHECK = 123;

//    // Action
//    public static final int ACTION_GET = 0;
//    public static final int ACTION_SET = 1;
//    public static final int ACTION_TURN_ON = 2;
//    public static final int ACTION_TURN_OFF = 3;
//    public static final int ACTION_RESET = 4;
//    public static final int ACTION_CAPTURE = 5;
//    public static final int ACTION_START = 6;
//    public static final int ACTION_STOP = 7;
//    public static final int ACTION_ADD = 8;
//    public static final int ACTION_DELETE = 9;

    private static Commander mOneManArmy;

    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;

    private Activity mActivity;
    private Result mResult;

    private Camera2BasicFragment mCamera;
    private GpuImageProcessing mGpuImageProc;
    private ImageProcessing mImageProc;
    private Display mDisplay;
    private Storage mStorage;
    private Status mStatus;
    private OverheatProtection mOverheatProtection;
    private USB mUsb;
    //#ifdef USE_SERIAL
    //private SlaveALS mSlaveAls;
    //private Serial mSerial;
    private GPS mGps;
    private int heatbeat;

    static {
        System.loadLibrary("imgproc-lib");
    }

    private native boolean reigsterNativeMethods();
    private native void writeUsb2Data(byte[] data);
    private native byte[] readUsb2Data(int size);

    private Commander(Activity activity) {
        mActivity = activity;
        mResult = new Result();
        //startBackgroundThread();

        mCamera = new Camera2BasicFragment();
        mGpuImageProc = new GpuImageProcessing();
        mImageProc = new ImageProcessing();
        mDisplay = new Display(mActivity);
        mStorage = new Storage(Storage.getSdCardDir(mActivity.getExternalMediaDirs()));
        mStatus = new Status();
        mOverheatProtection = new OverheatProtection();
        mUsb = USB.getDevice();
        //#ifdef USE_SERIAL
        //mSlaveAls = SlaveALS.getInstance();
        //mSerial = Serial.startDevice(mActivity);
        mGps = GPS.startDevice((LocationManager)mActivity.getSystemService(Context.LOCATION_SERVICE));

        heatbeat = 0;

        reigsterNativeMethods();
    }

    /**
     * Initialize and return Commander object
     * @param activity MainActivity
     * @return Commander
     */
    public static Commander startOneManArmy(Activity activity) {
        mOneManArmy = new Commander(activity);
        return mOneManArmy;
    }

    /**
     * Return Commander object
     * @return Commander
     */
    public static Commander getOneManArmy() {
        return mOneManArmy;
    }

    /**
     * Start background thread for Commander class
     */
    public void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CommandThread");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper()) {
            @Override
            public void handleMessage(Message msg) {
                execute(msg.what, msg.arg1, msg.arg2, null, null);
                super.handleMessage(msg);
            }
        };
    }

    /**
     * Stop background thread for Commander class
     */
    public void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Execute a command
     * @param command command
     * @param arg1 argument 1
     * @param arg2 argument 2
     * @param obj optional argument of any type
     * @param callback optional callback function
     * @return Result object
     */
    public Result execute(int command, int arg1, int arg2, final Object obj, Callback callback) {
        mResult.mCommand = command;
        mResult.mCode = Result.SUCCESS;
        int w=1920, h=1080;
        switch (command) {
            case GET_CAMERA_HW_VERSION:
                mResult.mType = Result.TYPE_STRING;
                mResult.mString = "N/A";
                break;
            case GET_CAMERA_SW_VERSION:
                mResult.mType = Result.TYPE_STRING;
                mResult.mString = ""; //BuildConfig.VERSION_NAME;
                break;
            case ENABLE_CAMERA:

                mCamera.openCamera(w,h);
                break;
            case DISABLE_CAMERA:
                mCamera.closeCamera();
                break;
            case RESET_CAMERA:
                mCamera.closeCamera();
                mCamera.openCamera(w,h);
                break;
            case SET_SENSOR_REGISTER:
                write(arg1, arg2);
                break;
            case GET_SENSOR_REGISTER:
                mResult.mType = TYPE_INT;
                mResult.mInt = read(arg1).intValue();
                break;
            case SET_GAMMA:
                mGpuImageProc.setGamma(Float.valueOf((String)obj));
                break;
            case GET_GAMMA:
                mResult.mType = Result.TYPE_FLOAT;
                mResult.mFloat = mGpuImageProc.getGamma();
                break;
            case SET_AUTOEXPOSURESPEEDSHORT:
                mGpuImageProc.setAutoExposureSpeedShort(arg1);
                savePreferencesInt(SettingsFragment.PREF_KEY_AUTOEXPOSURESPEEDSHORT, arg1);
                break;
            case SET_AUTOEXPOSURESPEEDLONG:
                mGpuImageProc.setAutoExposureSpeedLong(arg1);
                savePreferencesInt(SettingsFragment.PREF_KEY_AUTOEXPOSURESPEEDLONG, arg1);
                break;
            case GET_AUTOEXPOSURESPEEDSHORT:
                mResult.mType = Result.TYPE_FLOAT;
                mResult.mFloat = mGpuImageProc.getAutoExposureSpeedShort();
                break;
            case GET_AUTOEXPOSURESPEEDLONG:
                mResult.mType = Result.TYPE_FLOAT;
                mResult.mFloat = mGpuImageProc.getAutoExposureSpeedLong();
                break;
            case SET_AUTOEXPOSURETARGET:
                mGpuImageProc.setAutoExposureTarget(arg1);
                savePreferencesInt(SettingsFragment.PREF_KEY_AUTOEXPOSURETARGET, arg1);
                break;
            case SET_AUTOEXPOSURERANGE:
                mGpuImageProc.setAutoExposureRange(arg1);
                savePreferencesInt(SettingsFragment.PREF_KEY_AUTOEXPOSURERANGE, arg1);
                break;
            case SET_AVERAGEFRAMENUM:
                mGpuImageProc.setAverageFrameNum(arg1);
                savePreferencesInt(SettingsFragment.PREF_KEY_AVERAGEFRAMENUM, arg1);
                break;
            case GET_AUTOEXPOSURETARGET:
                mResult.mType = Result.TYPE_FLOAT;
                mResult.mFloat = mGpuImageProc.getAutoExposureTarget();
                break;
            case GET_AUTOEXPOSURERANGE:
                mResult.mType = Result.TYPE_FLOAT;
                mResult.mFloat = mGpuImageProc.getAutoExposureRange();
                break;
            case SET_DISPLAY_LSB:
                mGpuImageProc.setHassDisplayLsb(arg1);
                break;
            case ENABLE_FOCUS_POSITION_DETECTOR_MODE:
                mGpuImageProc.setHpfMode(true);
                break;
            case DISABLE_FOCUS_POSITION_DETECTOR_MODE:
                mGpuImageProc.setHpfMode(false);
                break;
            case GET_FILTER_COUNT:
                mResult.mType = TYPE_INT;
                int[] config = mGpuImageProc.getHassConfiguration();
                mResult.mInt = config[3];
                break;
            case GET_RECON_MATRIX:
                mResult.mType = Result.TYPE_FLOAT_2DARRAY;
                float[][] hassCoefficients = mGpuImageProc.getHassCoefficients();
                mResult.mData = hassCoefficients;
                break;
            case GET_PIXEL_PATTERN:
                mResult.mType = Result.TYPE_INT_2DARRAY;
                int[][] pattern = mGpuImageProc.getHassMosaic();
                mResult.mData = pattern;
                break;
//            case GET_ALPHA_COEFFICIENT:
//                mResult.mType = Result.TYPE_INT;
//                mResult.mInt = mGpuImageProc.getAlphaCoefficient();
//                break;
            case GET_NOISE_REDUCTION_COEFFICIENT:
                mResult.mType = TYPE_INT;
                mResult.mInt = mGpuImageProc.getNrTap();
                break;
            case SET_HASS_MODE:
                mGpuImageProc.setHassMode(arg1, arg2);
                if (arg1 == GpuImageProcessing.HASS_MODE_SPC) {
                    setEnabled(R.id.numberWavelength, true);
                }

                if (arg1 == GpuImageProcessing.HASS_MODE_RAW) {
                    mGpuImageProc.translate_1ch_to_3ch = false;
                }
                else {
                    mGpuImageProc.translate_1ch_to_3ch = true;
                }
                break;
            case TAKE_PICTURE:
                mCamera.takePicture();
                break;
            case SET_LENS_INFORMATION:
                mGpuImageProc.setLensInformation(arg1, arg2);
                break;
            case SET_SENSOR_INFORMATION:
                mGpuImageProc.setSensorInformation(arg1, arg2);
                break;
            case ENABLE_VIDEO:
                mStorage.initTimeLapse();
                if ( mStorage.isEnableRecording() &&
                        ((!mGps.isEnabled())) || (mGps.isEnabled() && mGps.isFixed()))
                {
                    LED.setCaptureLEDValue(200);
                }
                else {
                    LED.setCaptureLEDValue(0);
                }

                if (mCamera.isSnapshotStreamEnabled()) {
                    mCamera.startSnapshotStreaming();
                } else {
                    mCamera.startStreaming();
                }
                setButtonText(R.id.buttonStream, "Stop");
                setEnabled(R.id.buttonSnapshot, false);
                setEnabled(R.id.buttonSettings, false);
                mStatus.setMode(1);
                break;
            case DISABLE_VIDEO:
                LED.setCaptureLEDValue(0);
                mCamera.stopStreaming();
                setButtonText(R.id.buttonStream, "Stream");
                setEnabled(R.id.buttonSnapshot, true);
                setEnabled(R.id.buttonSettings, true);
                mStatus.setMode(0);
                break;
//            case ENABLE_DISPLAY:
//                mGpuImageProc.addObserver(mDisplay);
//                break;
//            case DISABLE_DISPLAY:
//                mGpuImageProc.deleteObserver(mDisplay);
//                break;
            case ENABLE_STORAGE:
                if (mStorage.isSdCardDetected()) {
                    mStorage.setEnableRecording(true);
                    mStorage.setTimeLapse(arg1);
                    mStorage.startBackgroundThread();
                    if (mCamera.isGpuEnabled()) {
                        mGpuImageProc.addObserver(mStorage);
                    } else {
                        mImageProc.addObserver(mStorage);
                    }
                    savePreferencesBoolean(SettingsFragment.PREF_KEY_SAVE_SD, true);
                } else {
                    execute(DISPLAY_TOAST, 0, 0, "SD Card not detected.");
                    mStorage.setEnableRecording(false);
                    mResult.mCode = Result.FAIL;
                    SharedPreferences sharedPreferences = getDefaultSharedPreferences(mActivity);
                    SharedPreferences.Editor editor = sharedPreferences.edit();
                    editor.putBoolean(SettingsFragment.PREF_KEY_SAVE_SD, false);
                    editor.commit();
                }
                break;
            case DISABLE_STORAGE:
                mStorage.setEnableRecording(false);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_SAVE_SD, false);
                break;
            case GET_CAMERA_FRAME_RATE:
                mResult.mInt = mCamera.getFrameCountAndReset();
                break;
            case GET_CAMERA_FRAME_RATE_READONLY:
                mResult.mInt = mCamera.getFrameCount();
                break;
            case GET_DISPLAY_FRAME_RATE:
                mResult.mInt = mDisplay.getFrameCountAndReset();
                break;
            case GET_STORAGE_FRAME_RATE:
                mResult.mInt = mStorage.getFrameCountAndReset();
                break;
            case SET_STATUS_DISPLAY:
                mDisplay.displayStatus((String)obj);
                break;
            case DISPLAY_TOAST:
                mActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(mActivity, (String)obj, Toast.LENGTH_LONG).show();
                    }
                });
                break;
//            case ENABLE_DOUBLE_BUFFER_ISP:
//                mGpuImageProc.enableDoubleBuffer(true);
//                break;
//            case DISABLE_DOUBLE_BUFFER_ISP:
//                mGpuImageProc.enableDoubleBuffer(false);
//                break;
            case ENABLE_GREEN_LED:
                LED.enableGreenLight(true);
                break;
            case DISABLE_GREEN_LED:
                LED.enableGreenLight(false);
                break;
            case ENABLE_BACK_LIGHT:
                //LED.enableBackLight(true);
                break;
            case DISABLE_BACK_LIGTH:
                //LED.enableBackLight(false);
                break;
            case ENABLE_STATUS:
                mCamera.getFrameCountAndReset();
                mDisplay.getFrameCountAndReset();
                mStorage.getFrameCountAndReset();
                mUsb.getFrameCountAndReset();
                mStatus.enableDisplay(true);
                setVisibility(R.id.textStatus, View.VISIBLE);
                break;
            case DISABLE_STATUS:
                setVisibility(R.id.textStatus, View.INVISIBLE);
                mStatus.enableDisplay(false);
                break;
            case ENABLE_HIDDEN_OPTIONS:
                //setVisibility(R.id.layoutHiddenOptions, View.VISIBLE);
                break;
            case DISABLE_HIDDEN_OPTIONS:
                //setVisibility(R.id.layoutHiddenOptions, View.INVISIBLE);
                break;
            case SET_SENSOR_FORMAT:
                //String state = mCamera.getState().toString();
                //mCamera.closeCamera();
                //mCamera.setImageFormat(arg1);
                //mCamera.openCamera();
//                if (state == "Recording") {
//                    mCamera.startRecording();
//                }
                break;
            case ENABLE_RAW_STORAGE:
                break;
            case DISABLE_RAW_STORAGE:
                break;
            case ENABLE_APP_CONTROLS:
                setEnabled(R.id.buttonStream, true);
                setEnabled(R.id.buttonSnapshot, true);
                setEnabled(R.id.buttonSettings, true);
                setEnabled(R.id.numberIrLed, true);
                setEnabled(R.id.numberGain, true);
                break;
            case DISABLE_APP_CONTROLS:
                setEnabled(R.id.buttonStream, false);
                setEnabled(R.id.buttonSnapshot, false);
                setEnabled(R.id.buttonSettings, false);
                setEnabled(R.id.numberIrLed, false);
                setEnabled(R.id.numberWavelength, false);
                setEnabled(R.id.numberGain, false);
                setEnabled(R.id.numberExposure, false);
                break;
            case ENABLE_THROTTLE_FRAME_RATE:
                mCamera.startThrottleFrameRate();
                break;
            case DISABLE_THROTTLE_FRAME_RATE:
                mCamera.stopThrottleFrameRate();
                break;
            case SET_THROTTLE_FRAME_RATE:
                mCamera.setThrottleFrameRate(arg1);
                break;
            case GET_TEMPERATURE_ZONE_22:
                mResult.mInt = Thermal.getZoneTemperature(22);
                mGpuImageProc.setTempareture(mResult.mInt);
                break;
            case SET_SHORT_EXPOSURE:
                //if (arg1 >= 1 && arg1 <= 1123)
                // otsuki for WDR16
                if (arg1 >= 2 && arg1 <= 141)
                {
                    Exposure.setShort(arg1);
                } else {
                    //execute(DISPLAY_TOAST, 0, 0, "Please set an exposure value to between 1~1123");
                    // otsuki for WDR16
                    execute(DISPLAY_TOAST, 0, 0, "Please set an exposure value to between 2~141");
                }
                break;
            case GET_SHORT_EXPOSURE:
                mResult.mType = TYPE_INT;
                mResult.mInt = Exposure.getShort();
                break;
//            case SET_ALPHA_COEFFICIENT:
//                mGpuImageProc.setAlphaCoefficient(arg1);
//                break;
            case SET_NOISE_REDUCTION_COEFFICIENT:
                mGpuImageProc.setNrTap(arg1);
                break;
            case ENABLE_ALS:
                ALSSensor.scheduleRead(67);
                break;
            case DISABLE_ALS:
                ALSSensor.shutdown();
                break;
            case GET_ALS:
                mResult.mType = mResult.TYPE_INT_ARRAY;
                mResult.mData = ALSSensor.getData();
                break;
            case SET_LONG_EXPOSURE:
                //if (arg1 >= 1 && arg1 <= 1123)
                // otsuki for WDR16
                //HDR_ON_OFF
                if(true){
                    if (arg1 >= 145 && arg1 <= 2398) {
                        Exposure.setLong(arg1);
                    } else {
                        execute(DISPLAY_TOAST, 0, 0, "Please set an exposure value to between 145~2398");
                    }
                }
                else{
                    if (arg1 >= 1 && arg1 <= 2398) {
                        Exposure.setLong(arg1);
                    } else {
                        execute(DISPLAY_TOAST, 0, 0, "Please set an long exposure value to between 1~2398");
                    }
                }
                break;
            case GET_LONG_EXPOSURE:
                mResult.mType = TYPE_INT;
                mResult.mInt = Exposure.getLong();
                break;
            case GET_HPF:
                mResult.mType = TYPE_INT;
                mResult.mInt = mGpuImageProc.getHpf();
                break;
            case ENABLE_IR_LED: {
                SharedPreferences sharedPreferences = getDefaultSharedPreferences(mActivity);
                int percentage = sharedPreferences.getInt(SettingsFragment.PREF_KEY_IR_LED, 0);
                IRLED.setIntensityPercentage(percentage);
                break;
            }
            case DISABLE_IR_LED:
                IRLED.turnOff();
                break;
            case SET_IR_LED_INTENSITY: {
                SharedPreferences sharedPreferences = getDefaultSharedPreferences(mActivity);
                SharedPreferences.Editor editor = sharedPreferences.edit();
                editor.putInt(SettingsFragment.PREF_KEY_IR_LED, arg1);
                editor.commit();
                IRLED.setIntensityPercentage(arg1);
                break;
            }
            case ENABLE_CONSTANT_FREQUENCY: {
                if (mOverheatProtection.isActive()) {
                    execute(DISPLAY_TOAST, 0, 0, "Manual throttle disabled due to heat");
                } else {
                    mCamera.setThrottleFrameRate(arg1);
                    mCamera.startThrottleFrameRate();
                    SharedPreferences sharedPreferences = getDefaultSharedPreferences(mActivity);
                    SharedPreferences.Editor editor = sharedPreferences.edit();
                    editor.putBoolean(SettingsFragment.PREF_KEY_CONSTANT_FREQUENCY, true);
                    editor.putString(SettingsFragment.PREF_KEY_FRAME_RATE, String.valueOf(arg1));
                    editor.commit();
                }
                break;
            }
            case DISABLE_CONSTANT_FREQUENCY: {
                mCamera.stopThrottleFrameRate();
                SharedPreferences sharedPreferences = getDefaultSharedPreferences(mActivity);
                SharedPreferences.Editor editor = sharedPreferences.edit();
                editor.putBoolean(SettingsFragment.PREF_KEY_CONSTANT_FREQUENCY, false);
                editor.commit();
                break;
            }
            case UPDATE_OVERHEAT_PROTECTION:
                mOverheatProtection.update(arg1);
                break;
            case GET_OVERHEAT_PROTECTION_STATUS:
                if (mOverheatProtection.isActive()) {
                    mResult.mString = "true";
                } else {
                    mResult.mString = "false";
                }
                break;
            case ENABLE_SYSTEM_CHECK:
                mStatus.scheduleUpdate(1000);
                break;
            case DISABLE_SYSTEM_CHECK:
                mStatus.shutdown();
                break;
            case GET_USB_FRAME_RATE:
                mResult.mInt = mUsb.getFrameCountAndReset();
                break;
            case ENABLE_AUTO_EXPOSURE:
                Exposure.enableAutoExposure();
                savePreferencesBoolean(SettingsFragment.PREF_KEY_AUTO_EXPOSURE, true);
                break;
            case DISABLE_AUTO_EXPOSURE:
                Exposure.disableAutoExposure();
                setEnabled(R.id.numberExposure, true);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_AUTO_EXPOSURE, false);
                break;
            case ENABLE_GPU:
                // Assume onSurfaceTextureAvailable has been called as this point
                mCamera.enableGpu(true);
                mGpuImageProc.startBackgroundThread();
                mGpuImageProc.addObserver(mDisplay);
                mCamera.addObserver(mGpuImageProc);
                setEnabled(R.id.numberNr, true);
                break;
            case DISABLE_GPU:
                // Assume onSurfaceTextureAvailable has been called as this point
                mCamera.enableGpu(false);
                mImageProc.startBackgroundThread();
                mImageProc.addObserver(mDisplay);
                mCamera.addObserver(mImageProc);
                setEnabled(R.id.numberNr, false);
                break;
            case ENABLE_AUTO_START_STREAMING:
                mCamera.enableAutoStartStreaming(true);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_AUTO_START_STREAMING, true);
                break;
            case DISABLE_AUTO_START_STREAMING:
                mCamera.enableAutoStartStreaming(false);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_AUTO_START_STREAMING, false);
                break;
            case ENABLE_DUAL_CAMERA_ALS:
                //#ifdef USE_SERIAL
                //if (mSerial.isAlive()) {
                //    execute(DISPLAY_TOAST, 0, 0, "Serial communication established.");
                //    mSerial.executeCommand(SEND_DUAL_CAMERA_ALS_DATA);
                //    ALSSensor.mSlaveScheduled = true;
                //}
                break;
            case DISABLE_DUAL_CAMERA_ALS:
                //#ifdef USE_SERIAL
                //if (mSerial.isAlive()) {
                //    mSerial.executeCommand(STOP_DUAL_CAMERA_ALS_DATA);
                //    ALSSensor.mSlaveScheduled = false;
                //}
                break;
            case SEND_DUAL_CAMERA_ALS_DATA:
                //#ifdef USE_SERIAL
                //mSlaveAls.scheduleRead(100);
                break;
            case STOP_DUAL_CAMERA_ALS_DATA:
                //#ifdef USE_SERIAL
                //mSlaveAls.shutdown();
                break;
            case SET_FILTER_COUNT:
                config = mGpuImageProc.getHassConfiguration();
                config[3] = arg1;
                mGpuImageProc.setHassConfiguration(config);
                break;
            case SET_PIXEL_PATTERN:
                mGpuImageProc.setHassMosaic((int[][])obj);
                break;
            case SET_RECON_MATRIX:
                mGpuImageProc.setHassCoefficients((float[][])obj);
                break;
            case SET_HASS_CONFIGURATION:
                mGpuImageProc.setHassConfiguration((int[])obj);
                break;
            case GET_HASS_CONFIGURATION:
                mResult.mData = mGpuImageProc.getHassConfiguration();
                break;
            case SET_HASS_OFFSET:
                mGpuImageProc.setHassOffset(arg1, arg2);
                break;
            case GET_HASS_OFFSET:
                break;
            case ENABLE_SNAPSHOT_STREAM:
                mCamera.enableSnapshotStream(true);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_HIGH_SPEED_STREAM, false);
                break;
            case DISABLE_SNAPSHOT_STREAM:
                mCamera.enableSnapshotStream(false);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_HIGH_SPEED_STREAM, true);
                break;
            case ENABLE_USB3:
                USB.getDevice().setMode(USB.MODE_ACCESSORY);
                if (mCamera.isGpuEnabled()) {
                    mGpuImageProc.addObserver(mUsb);
                } else {
                    mImageProc.addObserver(mUsb);
                }
                mGpuImageProc.setDisplayOff(1);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_USB3, true);
                break;
            case DISABLE_USB3:
                USB.getDevice().setMode(USB.MODE_ADB);
                mGpuImageProc.setDisplayOff(0);
                savePreferencesBoolean(SettingsFragment.PREF_KEY_USB3, false);
                break;
            case RESET_STATE:
                deleteAllObserver();
                stopAllThreads();
                break;
            case SET_ALS_REGISTER:
                ALSSensor.write(arg1, arg2);
                break;
            case GET_ALS_REGISTER:
                mResult.mType = TYPE_INT;
                mResult.mInt = ALSSensor.read(arg1).intValue();
                break;
            case SET_GAIN:
                write(HassSensor.GAIN, arg1);
                break;
            case GET_GAIN:
                mResult.mType = TYPE_INT;
                mResult.mInt = read(HassSensor.GAIN).intValue();
                break;
            case EXECUTE_OVER_SERIAL:
                //#ifdef USE_SERIAL
                //mSerial.executeCommand(command, arg1, arg2);
                break;
            case GET_VMAX:
                mResult.mType = TYPE_INT;
                int vmax = read(HassSensor.VMAX_L).intValue();
                vmax |= (read(HassSensor.VMAX_M).intValue() << 8);
                vmax |= (read(HassSensor.VMAX_H).intValue() << 16);
                mResult.mInt = vmax;
                break;
            case SET_VMAX:
                write(HassSensor.VMAX_L, arg1 & 0xFF);
                write(HassSensor.VMAX_M, (arg1 >> 8) & 0xFF);
                write(HassSensor.VMAX_H, (arg1 >> 16) & 0x03);
                break;
            case RESTART_CAMERA_STREAM:
                mCamera.restartStreaming();
                break;
            case SHUTDOWN_APP:
                USB.getDevice().setMode(USB.MODE_ADB);
                System.exit(0);
                break;
            case ENABLE_GPS:
                if (mGps.isProviderEnabled()) {
                    mGps.scheduleUpdate();
                    savePreferencesBoolean(SettingsFragment.PREF_KEY_GPS, true);
                } else {
                    execute(DISPLAY_TOAST, 0, 0, "GPS not enabled.");
                    mResult.mCode = Result.FAIL;
                    SharedPreferences sharedPreferences = getDefaultSharedPreferences(mActivity);
                    SharedPreferences.Editor editor = sharedPreferences.edit();
                    editor.putBoolean(SettingsFragment.PREF_KEY_GPS, false);
                    editor.commit();
                }
                break;
            case DISABLE_GPS:
                mGps.shutdown();
                savePreferencesBoolean(SettingsFragment.PREF_KEY_GPS, false);
                break;
            case ENABLE_SERIAL:
                //#ifdef USE_SERIAL
                //mSerial.connect();
                break;
            case DISABLE_SERIAL:
                //#ifdef USE_SERIAL
                //mSerial.disconnect();
                break;
            case SET_USB_MODE:
                USB.getDevice().setMode(arg1);
                break;
            case GET_VIDEO_MODE:
                mResult.mType = TYPE_INT;
                if (mCamera.isStreaming()) {
                    mResult.mInt = 1;
                } else {
                    mResult.mInt = 0;
                }
                break;
            case SET_SCALING_GOVERNOR_MODE:
                mCamera.setScalingGovernorMode(arg1);
                break;
            case APPLICATION_FORCE_RESTART:
                Log.e(toString(), "Capture Stop!!! force restart" + heatbeat);
                mStatus.setMode(0);
            {
                final Intent intent = mActivity.getIntent();
                final PendingIntent appStarter = PendingIntent.getActivity(mActivity.getApplicationContext(), 0, intent, 0);

                final AlarmManager alarmManager = (AlarmManager)mActivity.getSystemService(mActivity.ALARM_SERVICE);
                alarmManager.set(AlarmManager.RTC_WAKEUP, System.currentTimeMillis() + 1000, appStarter);

                Process.killProcess(Process.myPid());
            }
            break;
            case SET_TIMELAPSE_TIME:
                mStorage.setTimeLapse(arg1);
                savePreferencesInt(SettingsFragment.PREF_KEY_TIMELAPSE, arg1);
                break;
            case HEAT_BEAT:
                Log.i(toString(), "Heat beat " + heatbeat);
                heatbeat++;
                break;
            case MEMORY_LEAK_CHECK:
            {
                MemoryInfo mi = new MemoryInfo();
                {
                    ActivityManager activityManager = (ActivityManager) mActivity.getSystemService(Context.ACTIVITY_SERVICE);
                    activityManager.getMemoryInfo(mi);
                }

                long total_memory_size  = mi.availMem;
                long threathould = 200*1000000;
                Log.i(toString(), "MEMSIZE = " + total_memory_size + "(" + threathould + ")");

                mGpuImageProc.set_memory_size(total_memory_size);

                // reboot when memory leak occurred
                if((total_memory_size <= threathould) || mi.lowMemory){
                    rebootDevice();
                }
            }
            break;

        }
//        if (callback != null) {
//            callback.onExecute(mResult);
//        }
        return mResult;
    }

    /**
     * Execute a command
     * @param command command
     * @return Result object
     */
    public Result execute(int command) {
        //mBackgroundHandler.obtainMessage(command).sendToTarget();
        return execute(command, 0, 0, null, null);
    }

    /**
     * Execute a command
     * @param command command
     * @param callback optional callback function
     * @return Result object
     */
    public Result execute(int command, Callback callback) {
        //mBackgroundHandler.obtainMessage(command, callback).sendToTarget();
        return execute(command, 0, 0, null, callback);
    }

    /**
     * Execute a command
     * @param command command
     * @param arg1 argument 1
     * @param arg2 argument 2
     * @return Result object
     */
    public Result execute(int command, int arg1, int arg2) {
        //mBackgroundHandler.obtainMessage(command, arg1, arg2).sendToTarget();
        return execute(command, arg1, arg2, null, null);
    }

    /**
     * Execute a command
     * @param command command
     * @param arg1 argument 1
     * @param arg2 argument 2
     * @param obj optional argument of any type
     * @return
     */
    public Result execute(int command, int arg1, int arg2, Object obj) {
        //mBackgroundHandler.obtainMessage(command, arg1, arg2, obj).sendToTarget();
        return execute(command, arg1, arg2, obj, null);
    }

    private int nativeExecute(int command, int arg1, int arg2) {
        Result result;
        if (command == SET_GAMMA) {
            byte[] floatData = readUsb2Data(4);
            float gamma = ByteBuffer.wrap(floatData).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            result = execute(command, arg1, arg2, String.valueOf(gamma));
        } else {
            result = execute(command, arg1, arg2);
        }
        int ret = 0;
        switch (result.mType) {
            case TYPE_INT:
                ret = result.mInt;
                break;
            case Result.TYPE_STRING:
                byte[] stringData = result.mString.getBytes();
                writeUsb2Data(stringData);
                ret = stringData.length;
                break;
            case Result.TYPE_FLOAT:
                ByteBuffer floatBuffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
                floatBuffer.putFloat(result.mFloat);
                byte[] floatData = floatBuffer.array();
                writeUsb2Data(floatData);
                ret = floatData.length;
                break;
            case Result.TYPE_INT_ARRAY:
                int[] intArray = (int[])result.mData;
                ByteBuffer intBuffer = ByteBuffer.allocate(4 * intArray.length).order(ByteOrder.LITTLE_ENDIAN);
                for (int i : intArray) {
                    intBuffer.putInt(i);
                }
                byte[] intArrayData = intBuffer.array();
                writeUsb2Data(intArrayData);
                ret = intArrayData.length;
                break;
            case Result.TYPE_FLOAT_ARRAY:
                float[] floatArray = (float[])result.mData;
                ByteBuffer floatArrayBuffer = ByteBuffer.allocate(4 * floatArray.length).order(ByteOrder.LITTLE_ENDIAN);
                for (float f : floatArray) {
                    floatArrayBuffer.putFloat(f);
                }
                byte[] floatArrayData = floatArrayBuffer.array();
                writeUsb2Data(floatArrayData);
                ret = floatArrayData.length;
                break;
            case Result.TYPE_INT_2DARRAY:
                int[][] int2DArray = (int[][])result.mData;
                ByteBuffer int2DArrayBuffer = ByteBuffer.allocate(4 * int2DArray.length * int2DArray[0].length).order(ByteOrder.LITTLE_ENDIAN);
                for (int[] int1DArray : int2DArray) {
                    for (int i : int1DArray) {
                        int2DArrayBuffer.putInt(i);
                    }
                }
                byte[] int2DArrayData = int2DArrayBuffer.array();
                writeUsb2Data(int2DArrayData);
                ret = int2DArrayData.length;
                break;
            case Result.TYPE_FLOAT_2DARRAY:
                float[][] float2DArray = (float[][])result.mData;
                ByteBuffer float2DArrayBuffer = ByteBuffer.allocate(4 * float2DArray.length * float2DArray[0].length).order(ByteOrder.LITTLE_ENDIAN);
                for (float[] float1DArray : float2DArray) {
                    for (float f : float1DArray) {
                        float2DArrayBuffer.putFloat(f);
                    }
                }
                byte[] float2DArrayData = float2DArrayBuffer.array();
                writeUsb2Data(float2DArrayData);
                ret = float2DArrayData.length;
                break;
        }
        return ret;
    }

    /**
     * Abstract class for callback funt
     */
    public static abstract class Callback {
        public abstract void onExecute(Result result);
    }

    @Override
    public String toString() {
        return "Commander";
    }

    /**
     * Result object holding data of status of execute function
     */
    public static class Result {
        public static final int FAIL = 0;
        public static final int SUCCESS = 1;

        public static final int TYPE_INT = 0;
        public static final int TYPE_STRING = 1;
        public static final int TYPE_FLOAT = 2;
        public static final int TYPE_INT_ARRAY = 3;
        public static final int TYPE_FLOAT_ARRAY = 4;
        public static final int TYPE_INT_2DARRAY = 5;
        public static final int TYPE_FLOAT_2DARRAY = 6;
        public static final int TYPE_BOOLEAN = 7;

        public int mCommand;
        public int mCode;
        public int mType;
        public int mInt;
        public String mString;
        public float mFloat;
        public boolean mBoolean;
        public Object mData;
    }

    private static void enableViewAndChildren(View view, boolean enabled) {
        view.setEnabled(enabled);
        if (view instanceof ViewGroup) {
            ViewGroup viewGroup = (ViewGroup) view;
            for (int i = 0; i < viewGroup.getChildCount(); i++) {
                View child = viewGroup.getChildAt(i);
                enableViewAndChildren(child, enabled);
            }
        }
    }

    private void deleteAllObserver() {
        mGpuImageProc.deleteObservers();
        mImageProc.deleteObservers();
        mCamera.deleteObservers();
    }

    private void stopAllThreads() {
        mStorage.stopBackgroundThread();
        mGpuImageProc.stopBackgroundThread();
        mImageProc.stopBackgroundThread();
    }

    private void setEnabled(final int id, final boolean enabled) {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mActivity.findViewById(id).setEnabled(enabled);
            }
        });
    }

    private void setVisibility(final int id, final int visibility) {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mActivity.findViewById(id).setVisibility(visibility);
            }
        });
    }

    private void setChecked(final int id, final boolean checked) {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ((CheckBox) mActivity.findViewById(id)).setChecked(checked);
            }
        });
    }

    private void setButtonText(final int id, final String text) {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ((Button) mActivity.findViewById(id)).setText(text);
            }
        });
    }

    private void savePreferencesInt(String name, int value){
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mActivity);
        SharedPreferences.Editor editor = sharedPreferences.edit();
        editor.putString(name, String.valueOf(value));
        editor.commit();
    }

    private void savePreferencesBoolean(String name, boolean value){
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mActivity);
        SharedPreferences.Editor editor = sharedPreferences.edit();
        editor.putBoolean(name, value);
        editor.commit();
    }

    private void rebootDevice() {
        try {
            java.lang.Process proc = Runtime.getRuntime().exec(new String[]{ "reboot" });
            proc.waitFor();
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

}

