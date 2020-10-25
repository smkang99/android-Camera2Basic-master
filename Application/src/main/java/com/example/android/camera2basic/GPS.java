package com.example.android.camera2basic;

import android.content.Context;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;

import static android.location.LocationManager.GPS_PROVIDER;
import static android.location.LocationManager.NETWORK_PROVIDER;
import static java.security.AccessController.getContext;

/**
 * Created by Abhinav Mathur on 10/31/2017.
 */

public class GPS implements LocationListener {
    private static GPS mGps;

    private LocationManager mLocationManager;
    private Location mLocation;
    private boolean mScheduled;
    private boolean mFixed;
    private Monitor mFixMonitor;

    private HandlerThread mBackgroundThread;

    private GPS(LocationManager locationManager) {
        mLocationManager = locationManager;
        mScheduled = false;
        mFixed = false;
        mFixMonitor = new Monitor();
    }

    /**
     * Creates a new GPS instance object
     * @param locationManager : LocationManager object.
     * @return : Returns a GPS class object.
     */
    public static GPS startDevice(LocationManager locationManager) {
        if (mGps == null) {
            mGps = new GPS(locationManager);
        }
        return mGps;
    }

    /**
     * Returns the GPS class object.
     * @return : GPS class object.
     */
    public static GPS getDevice() {
        return mGps;
    }

    /**
     * Start location listener function. onLocationChanged function is called when there is
     * a change in GPS location. Current location parameters are updated accordingly.
     */
    public void scheduleUpdate() {
        if (mScheduled) return;

        startBackgroundThread();
        try {
            mLocationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER, 0, 0, this, mBackgroundThread.getLooper());
        } catch (SecurityException e) {
            e.printStackTrace();
        }
        mScheduled = true;
    }

    /**
     * Stop location listener
     */
    public void shutdown() {
        stopBackgroundThread();

        if (!mScheduled) return;
        try {
            mLocationManager.removeUpdates(this);
        } catch (SecurityException e) {
            e.printStackTrace();
        }
        mScheduled = false;
    }

    /**
     * Request a single location update
     */
    public void requestUpdate() {
        if (mScheduled) return;

        startBackgroundThread();
        try {
            mLocationManager.requestSingleUpdate(LocationManager.GPS_PROVIDER, this, mBackgroundThread.getLooper());
        } catch (SecurityException e) {
            e.printStackTrace();
        }
    }

    /**
     * Check if GPS data is being updated
     * @return boolean update status
     */
    public boolean isEnabled() {
        return mScheduled;
    }

    /**
     * Check if GPS received a fix
     */
    public boolean isFixed() {
        return mFixed;
    }

    /**
     * Pause thread until fix is obtained
     */
    public void waitForFix() {
        mFixMonitor.waitForSignal();
    }

    /**
     * Cancel acquiring fix and resume thread
     */
    public void cancelWait() {
        mFixMonitor.signal();
    }

    /**
     * Check if GPS settings is enabled
     * @return boolean enable status
     */
    public boolean isProviderEnabled() {
        return mLocationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
    }

    /**
     * Callback when location is changed
     * @param location current location
     */
    @Override
    public void onLocationChanged(Location location) {
        mLocation = location;
        mFixed = true;
        mFixMonitor.signal();
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {

    }

    @Override
    public void onProviderEnabled(String provider) {

    }

    @Override
    public void onProviderDisabled(String provider) {
        mFixed = false;
    }

    /**
     * Return the currently available location
     * @return Location Location object
     */
    public Location getLocation() {
        Location location = mLocation;
        if (!isCurrentLocationValid()) {
            try {
                location = mLocationManager.getLastKnownLocation(GPS_PROVIDER);
            } catch (SecurityException e) {
                e.printStackTrace();
            }
        }
        return location;
    }

    /**
     * Start background thread for GPS class
     */
    private void startBackgroundThread() {
        if (mBackgroundThread == null) {
            mBackgroundThread = new HandlerThread("GPS");
            mBackgroundThread.start();
        }
    }

    /**
     * Stop background thread for GPS class
     */
    private void stopBackgroundThread() {
        if (mBackgroundThread != null) {
            mBackgroundThread.quitSafely();
            try {
                mBackgroundThread.join();
                mBackgroundThread = null;
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Return the latitude in DMS format
     * @param latitude value of latitude
     * @return String latitude in DMS format
     */
    public static String toLatitudeDms(double latitude) {
        StringBuilder builder = new StringBuilder();

        if (latitude < 0) {
            builder.append("S ");
        } else {
            builder.append("N ");
        }

        String latitudeDegrees = Location.convert(Math.abs(latitude), Location.FORMAT_SECONDS);
        String[] latitudeSplit = latitudeDegrees.split(":");
        builder.append(latitudeSplit[0]);
        builder.append("°");
        builder.append(latitudeSplit[1]);
        builder.append("'");
        builder.append(latitudeSplit[2]);
        builder.append("\"");

        return builder.toString();
    }

    /**
     * Return the longitude in DMS format
     * @param longitude value of longitude
     * @return String longitude in DMS format
     */
    public static String toLongitudeDms(double longitude) {
        StringBuilder builder = new StringBuilder();

        if (longitude < 0) {
            builder.append("W ");
        } else {
            builder.append("E ");
        }

        String longitudeDegrees = Location.convert(Math.abs(longitude), Location.FORMAT_SECONDS);
        String[] longitudeSplit = longitudeDegrees.split(":");
        builder.append(longitudeSplit[0]);
        builder.append("°");
        builder.append(longitudeSplit[1]);
        builder.append("'");
        builder.append(longitudeSplit[2]);
        builder.append("\"");

        return builder.toString();
    }

    private boolean isCurrentLocationValid() {
        if (mLocation == null || mLocation.getLongitude() == 0 || mLocation.getLatitude() == 0)
            return false;
        else
            return true;
    }
}