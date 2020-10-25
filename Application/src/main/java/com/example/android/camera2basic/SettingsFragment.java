package com.example.android.camera2basic;

import android.Manifest;
import android.Manifest;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.content.pm.PackageManager;
import android.content.res.Resources;
import android.content.res.XmlResourceParser;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.preference.EditTextPreference;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceGroup;
import android.preference.TwoStatePreference;
import android.support.annotation.NonNull;
import android.support.annotation.RequiresApi;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.widget.Button;

import java.lang.reflect.Field;
import java.util.List;

import static android.R.attr.key;
import static android.provider.Settings.ACTION_LOCATION_SOURCE_SETTINGS;

/**
 * Created by Allan Suen on 6/5/2017.
 */

public class SettingsFragment extends PreferenceFragment implements OnSharedPreferenceChangeListener {
    public static final String PREF_KEY_GPU = "prefGpu";
    public static final String PREF_KEY_GPU_MODE = "prefGpuMode";
    public static final String PREF_KEY_WAVELENGTH = "prefWavelength";
    public static final String PREF_KEY_SAVE_SD = "prefSaveSd";
    public static final String PREF_KEY_ALS = "prefAls";
    public static final String PREF_KEY_SERIAL = "prefSerial";
    public static final String PREF_KEY_AUTO_EXPOSURE = "prefAutoExposure";
    public static final String PREF_KEY_CONSTANT_FREQUENCY = "prefConstFreq";
    public static final String PREF_KEY_FRAME_RATE = "prefFrameRate";
    public static final String PREF_KEY_USB3 = "prefUsb3";
    public static final String PREF_KEY_HIGH_SPEED_STREAM = "prefHighSpeedStream";
    public static final String PREF_KEY_GPS = "prefGps";
    public static final String PREF_KEY_AUTO_START_STREAMING = "prefAutoStartStreaming";
    public static final String PREF_KEY_FOCUS_POSITION_DETECTOR = "prefFocusPositionDetector";

    public static final String PREF_KEY_FILTER_COUNT = "prefFilterCount";
    public static final String PREF_KEY_FILTER_ARRAY = "prefFilterArray";
    public static final String PREF_KEY_NR_TAP = "prefNrTap";
    public static final String PREF_KEY_ALPHA = "prefAlpha";
    public static final String PREF_KEY_RECON_MATRIX = "prefReconMatrix";
    public static final String PREF_KEY_FILTER_SIZE = "prefFilterSize";
    public static final String PREF_KEY_FILTER_OFFSET_X = "prefFilterOffsetX";
    public static final String PREF_KEY_FILTER_OFFSET_Y = "prefFilterOffsetY";
    public static final String PREF_KEY_SHRINK_TAP = "prefShrinkTap";
    public static final String PREF_KEY_SHRINK_RATIO = "prefShrinkRatio";
    public static final String PREF_KEY_RECON_COLOR_NUM = "prefReconColorNum";

    public static final String PREF_KEY_GAIN = "prefGain";
    public static final String PREF_KEY_EXPOSURE = "prefExposure";
    public static final String PREF_KEY_IR_LED = "prefIrLed";
    public static final String PREF_KEY_GAMMA = "prefGamma";
    public static final String PREF_KEY_AUTOEXPOSURESPEEDLONG = "prefAutoExposureSpeedLong";
    public static final String PREF_KEY_AUTOEXPOSURESPEEDSHORT = "prefAutoExposureSpeedShort";
    public static final String PREF_KEY_AUTOEXPOSURETARGET = "prefAutoExposureTarget";
    public static final String PREF_KEY_AUTOEXPOSURERANGE = "prefAutoExposureRange";
    public static final String PREF_KEY_AVERAGEFRAMENUM = "prefAverageFrameNum";
    public static final String PREF_KEY_DISPLAY_LSB = "prefDisplayLSB";
    public static final String PREF_KEY_TIMELAPSE = "prefTimelapse";
    public static final String PREF_KEY_LENS_NUMBER = "prefLensNumber";
    public static final String PREF_KEY_OPTICAL_FILTER_NUMBER = "prefCameraSerialNumber";
    public static final String PREF_KEY_SENSOR_ID_NUM = "prefSensorIDNum";
    public static final String PREF_KEY_SENSOR_TYPE_NUM = "prefSensorTypeNum";

    private static final String PREF_KEY_SENSOR_ID = "prefSensorId";
    private static final String PREF_KEY_SENSOR_TYPE = "prefSensorType";
    private static final String PREF_KEY_START_X = "prefStartX";
    private static final String PREF_KEY_START_Y = "prefStartY";
    private static final String PREF_KEY_DIRECTION_X = "prefDirectionX";
    private static final String PREF_KEY_DIRECTION_Y = "prefDirectionY";
    private static final String PREF_KEY_PARAMETER_ID = "prefParameterId";

    private static final int PERMISSION_REQUEST_STORAGE = 1;
    private static final int PERMISSION_REQUEST_GPS = 2;

    private SharedPreferences mSharedPreferences;
    private boolean mExternalStoragePermission;
    private boolean mGpsPermission;

    /**
     * SettingsFragment initialization
     * @param savedInstanceState ignored
     */
    @RequiresApi(api = Build.VERSION_CODES.M)
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Load the preferences from an XML resource
        //addPreferencesFromResource(R.xml.preferences);
        mSharedPreferences = getPreferenceManager().getSharedPreferences();
        mExternalStoragePermission = isExternalStoragePermissionGranted();
        mGpsPermission = isGpsPermissionGranted();
    }

    /**
     * SettingsFragment onResume event
     */
    @Override
    public void onResume() {
        super.onResume();
        updatePreferenceGroupSummary(getPreferenceScreen());
        mSharedPreferences.registerOnSharedPreferenceChangeListener(this);
    }

    /**
     * SettingsFragment onPause event
     */
    @Override
    public void onPause() {
        super.onPause();
        mSharedPreferences.unregisterOnSharedPreferenceChangeListener(this);
        updateParameterGenerator();
    }

    /**
     * SettingsFragment onSharedPreferenceChanged event
     * @param sharedPreferences of the application
     * @param key of the shared preference
     */
    @RequiresApi(api = Build.VERSION_CODES.M)
    @Override
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        Preference pref = findPreference(key);
        updatePreferenceSummary(pref);
        updatePreferenceValue(pref);
    }

    /**
     * SettingsFragment onRequestPermissionResult event
     * @param requestCode type of request
     * @param permissions ignored
     * @param grantResults permission result
     */
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        switch (requestCode) {
            case PERMISSION_REQUEST_STORAGE:
                if (grantResults.length == 2
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED
                        && grantResults[1] == PackageManager.PERMISSION_GRANTED) {
                    mExternalStoragePermission = true;
                    TwoStatePreference saveSd = (TwoStatePreference)findPreference(PREF_KEY_SAVE_SD);
                    saveSd.setChecked(true);
                } else {
                    mExternalStoragePermission = false;
                }
                break;
            case PERMISSION_REQUEST_GPS:
                if (grantResults.length == 1
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    mGpsPermission = true;
                } else {
                    mGpsPermission = false;
                }
                break;
        }
    }

    private void updatePreferenceSummary(Preference preference) {
        String key = preference.getKey();
        switch (key) {
            case PREF_KEY_GAMMA:
            case PREF_KEY_AUTOEXPOSURESPEEDLONG:
            case PREF_KEY_AUTOEXPOSURESPEEDSHORT:
            case PREF_KEY_AUTOEXPOSURERANGE:
            case PREF_KEY_AUTOEXPOSURETARGET:
            case PREF_KEY_AVERAGEFRAMENUM:
            case PREF_KEY_TIMELAPSE:
            case PREF_KEY_LENS_NUMBER:
            case PREF_KEY_OPTICAL_FILTER_NUMBER:
            case PREF_KEY_SENSOR_ID_NUM:
            case PREF_KEY_SENSOR_TYPE_NUM:
            case PREF_KEY_SENSOR_ID:
            case PREF_KEY_SENSOR_TYPE:
            case PREF_KEY_DISPLAY_LSB:
                preference.setSummary(mSharedPreferences.getString(key, ""));
                break;
            case PREF_KEY_START_X:
                String startX = getResources().getString(R.string.pref_summary_start_x);
                preference.setSummary(startX + mSharedPreferences.getString(key, "0"));
                break;
            case PREF_KEY_START_Y:
                String startY = getResources().getString(R.string.pref_summary_start_y);
                preference.setSummary(startY + mSharedPreferences.getString(key, "0"));
                break;
            case PREF_KEY_GPU_MODE:
            case PREF_KEY_FRAME_RATE:
            case PREF_KEY_PARAMETER_ID:
                preference.setSummary(((ListPreference)preference).getEntry());
                break;
        }
    }

    private void updatePreferenceGroupSummary(PreferenceGroup group) {
        for (int i = 0; i < group.getPreferenceCount(); i++) {
            Preference pref = group.getPreference(i);
            if (pref instanceof PreferenceGroup) {
                updatePreferenceGroupSummary((PreferenceGroup)pref);
            } else {
                updatePreferenceSummary(pref);
            }
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private void updatePreferenceValue(Preference preference) {
        String key = preference.getKey();
        switch (key) {
            case PREF_KEY_SAVE_SD:
                TwoStatePreference saveSd = (TwoStatePreference)preference;
                if (saveSd.isChecked()) {
                    if (!mExternalStoragePermission) {
                        saveSd.setChecked(false);
                        requestExternalStoragePermission();
                    }
                }
                break;
            case PREF_KEY_GPS:
                TwoStatePreference gps = (TwoStatePreference)preference;
                if (gps.isChecked()) {
                    if (!GPS.getDevice().isProviderEnabled()) {
                        gps.setChecked(false);
                        requestGpsProvider();
                    } else if (!mGpsPermission) {
                        gps.setChecked(false);
                        requestGpsPermission();
                    } else if (!GPS.getDevice().isFixed()){
                        gps.setChecked(false);
                        displayGpsFixDialog();
                    }
                }
                break;
            case PREF_KEY_USB3:
                TwoStatePreference usb = (TwoStatePreference)preference;
                break;
        }
    }

    private void updateParameterGenerator() {
        int xmlId = 0;
        //Class xmlClass = R.xml.class;
        String parameterFile = mSharedPreferences.getString(PREF_KEY_PARAMETER_ID, "parameter_set_default");
        //try {
        //    Field xmlField = xmlClass.getField(parameterFile);
        //    xmlId = xmlField.getInt(null);
        //} catch (NoSuchFieldException e) {
          //  e.printStackTrace();
        //} //catch (IllegalAccessException e) {
          //  e.printStackTrace();
       // }
        if (xmlId != 0) {
            XmlResourceParser xml = getResources().getXml(xmlId);
          //  InternalParameterSet parameterSet = new InternalParameterSet(xml);
            ///////ReconstructionParameters reconParams = new ReconstructionParameters(parameterSet.getParameters());
          //  parameterSet.saveToSharedPreferences(mSharedPreferences.edit());
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private boolean isExternalStoragePermissionGranted() {
        return (getContext().checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED
                && getContext().checkSelfPermission(Manifest.permission.READ_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED);
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private void requestExternalStoragePermission() {
        if (shouldShowRequestPermissionRationale(Manifest.permission.WRITE_EXTERNAL_STORAGE)
                || shouldShowRequestPermissionRationale(Manifest.permission.READ_EXTERNAL_STORAGE)) {
            // Show rationale to request permission
        }
        requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.READ_EXTERNAL_STORAGE}, PERMISSION_REQUEST_STORAGE);
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private boolean isGpsPermissionGranted() {
        return (getContext().checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED);
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private void requestGpsPermission() {
        if (shouldShowRequestPermissionRationale(Manifest.permission.ACCESS_FINE_LOCATION)) {
            // Show rationale to request permission
        }
        requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, PERMISSION_REQUEST_GPS);
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private void requestGpsProvider() {
        AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
        builder.setMessage("Turn on location services in settings.")
                .setCancelable(false)
                .setPositiveButton("Settings",
                        new DialogInterface.OnClickListener(){
                            public void onClick(DialogInterface dialog, int id){
                                startActivity(new Intent(ACTION_LOCATION_SOURCE_SETTINGS));
                            }
                        });
        builder.setNegativeButton("Cancel",
                new DialogInterface.OnClickListener(){
                    public void onClick(DialogInterface dialog, int id) {
                        dialog.cancel();
                    }
                });
        AlertDialog alert = builder.create();
        alert.show();
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    private void displayGpsFixDialog() {
        final ProgressDialog dialog = new ProgressDialog(getContext());
        dialog.setMessage("Waiting for GPS fix.\nThis may take up to 5 minutes...");
        dialog.setIndeterminate(true);
        dialog.setCancelable(false);
        dialog.setButton(DialogInterface.BUTTON_NEGATIVE, "Cancel",
                new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        GPS gps = GPS.getDevice();
                        gps.cancelWait();
                        gps.shutdown();
                        dialog.cancel();
                    }
                });
        dialog.setOnDismissListener(new DialogInterface.OnDismissListener() {
            @Override
            public void onDismiss(DialogInterface dialog) {
                if (GPS.getDevice().isFixed()) {
                    Commander.getOneManArmy().execute(Commander.DISPLAY_TOAST, 0, 0, "GPS fix acquired.");
                    TwoStatePreference saveSd = (TwoStatePreference)findPreference(PREF_KEY_GPS);
                    saveSd.setChecked(true);
                }
            }
        });
        dialog.show();

        new Thread(new Runnable() {
            @Override
            public void run() {
                GPS gps = GPS.getDevice();
                gps.scheduleUpdate();
                gps.waitForFix();

                dialog.dismiss();
            }
        }).start();
    }
}