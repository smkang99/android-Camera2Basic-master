package com.example.android.camera2basic;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.TimeUnit;

/**
 * Created by Allan Suen on 7/27/2017.
 */

public class Exposure {
    private static AtomicBoolean mSettingExposure = new AtomicBoolean(false);
    private static boolean mScheduled = false;
    private static ScheduledExecutorService mScheduledExecutor = Executors.newSingleThreadScheduledExecutor();
    private static ScheduledFuture mScheduledFuture;
    private static int mLatestSHS1;
    private static int mLatestSHS2;

    //static {
    //    System.loadLibrary("imgproc-lib");
    //}

    //private native static int GetSHS1();
    //private native static int GetSHS2();
    //private native static boolean DoneAE();
    //private native static void SetAEreq(int req);

    public static void enableAutoExposure() {
        if (mScheduled) return;

        mScheduledFuture = mScheduledExecutor.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                auto();
            }
        }, 0, 200, TimeUnit.MILLISECONDS);
        mScheduled = true;
    }

    public static void disableAutoExposure() {
        if (!mScheduled) return;

        mScheduledFuture.cancel(true);
        //mScheduledExecutor.shutdownNow();
        //mScheduledExecutor = null;

        mScheduled = false;
    }

    /**
     * Return short exposure
     * @return short exposure
     */
    public static int getShort() {
        return mLatestSHS1;
    }

    /**
     * Return long exposure
     * @return long exposure
     */
    public static int getLong() {
        return mLatestSHS2;
    }

    /**
     * Set short exposure
     * @param shs1 short exposure
     * @return operation success
     */
    public static boolean setShort(int shs1) {
        Collection<Register> registers = new ArrayList<>();
        registers.add(new Register(HassSensor.SHS1_L, shs1 & 0xFF));
        registers.add(new Register(HassSensor.SHS1_M, (shs1 >> 8) & 0xFF));
        registers.add(new Register(HassSensor.SHS1_H, (shs1 >> 16) & 0xFF));

        mLatestSHS1 = shs1;
        return HassSensor.write(registers);
    }

    /**
     * Set long exposure
     * @param shs2 long exposure
     * @return operation success
     */
    public static boolean setLong(int shs2) {
        Collection<Register> registers = new ArrayList<>();
        registers.add(new Register(HassSensor.SHS2_L, shs2 & 0xFF));
        registers.add(new Register(HassSensor.SHS2_M, (shs2 >> 8) & 0xFF));
        registers.add(new Register(HassSensor.SHS2_H, (shs2 >> 16) & 0xFF));

        mLatestSHS2 = shs2;
        return HassSensor.write(registers);
    }

    /**
     * Set exposure values
     * @param shs1 short exposure
     * @param shs2 long exposure
     * @return operation success
     */
    public static boolean set(int shs1, int shs2) {
        return (setShort(shs1) && setLong(shs2));
    }

    /**
     * Automatically update exposure values
     */
    public static void auto() {

        if (!mScheduled) return;

        if (mSettingExposure.compareAndSet(false, true)) {
            //if(DoneAE())
            if(false)
            {
                int shortExposure = 0 ; //GetSHS1();
                int longExposure = 0; //GetSHS2();
                set(shortExposure, longExposure);
                //SetAEreq((int)1);
            }
            mSettingExposure.set(false);
        }
    }

    /**
     * Automatically update exposure values in a background thread
     */
    public static void autoInBackground() {
        if (!mScheduled) return;

        mScheduledExecutor.execute(new Runnable() {
            @Override
            public void run() {
                auto();
            }
        });
    }
}
