package com.example.android.camera2basic;
/**
 * A wrapper for an {@link AutoCloseable} object that implements reference counting to allow
 * for resource management.
 */
public class RefCountedAutoCloseable<T extends AutoCloseable> implements AutoCloseable {
    private T mObject;
    private long mRefCount = 0;

    /**
     * Wrap the given object.
     *
     * @param object an object to wrap.
     */
    public RefCountedAutoCloseable(T object) {
        if (object == null) throw new NullPointerException();
        mObject = object;
    }

    /**
     * Increment the reference count and return the wrapped object.
     *
     * @return the wrapped object, or null if the object has been released.
     */
    public synchronized T getAndRetain() {
        if (mRefCount < 0) {
            return null;
        }
        mRefCount++;
        return mObject;
    }

    /**
     * Return the wrapped object.
     *
     * @return the wrapped object, or null if the object has been released.
     */
    public synchronized T get() {
        return mObject;
    }

    /**
     * Decrement the reference count and release the wrapped object if there are no other
     * users retaining this object.
     */
    @Override
    public synchronized void close() {
        mRefCount--;
        if (mRefCount <= 0) {
            try {
                mObject.close();
            } catch (Exception e) {
                throw new RuntimeException(e);
            } finally {
                mRefCount = 0;
            }
        }
    }

    /**
     * Object is considered in use if reference count is greater than 0.
     */
    public synchronized boolean inUse() {
        return (mRefCount > 0);
    }
}
