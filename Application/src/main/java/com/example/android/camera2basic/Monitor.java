package com.example.android.camera2basic;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import static java.lang.System.currentTimeMillis;

/**
 * Created by Allan Suen on 11/2/2017.
 */

public class Monitor {
    private boolean mWait;
    private ReentrantLock mLock;
    private final Condition mCondition;

    /**
     * Create Monitor object
     */
    public Monitor() {
        mWait = false;
        mLock = new ReentrantLock();
        mCondition = mLock.newCondition();
    }

    /**
     * Suspend thread and wait for condition to signal
     */
    public void waitForSignal() {
        mLock.lock();
        mWait = true;
        try {
            while (mWait) {
                mCondition.await();
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mLock.unlock();
        }
    }

    /**
     * Suspend thread and wait for condition to signal
     * @param millis time to wait in milliseconds
     */
    public boolean waitForSignal(int millis) {
        mLock.lock();
        mWait = true;
        boolean signaled = true;
        long startTime = System.currentTimeMillis();
        try {
            while (mWait) {
                if (!mCondition.await(millis, TimeUnit.MILLISECONDS)) {
                    // Check if spurrious wake up or time elapsed
                    long elapsed = System.currentTimeMillis() - startTime;
                    if (elapsed >= millis) {
                        mWait = false;
                        signaled = false;
                    }
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mLock.unlock();
        }
        return signaled;
    }

    /**
     * Signal condition and allow suspended thread to continue
     */
    public void signal() {
        mLock.lock();
        try {
            mWait = false;
            mCondition.signalAll();
        } catch (IllegalMonitorStateException e) {
            e.printStackTrace();
        } finally {
            mLock.unlock();
        }
    }
}
