package com.example.android.camera2basic;

/**
 * Created by Allan Suen on 6/21/2017.
 */

public class Register {
    public int mAddress;
    public int mData;

    /**
     * Create Register object
     * @param address register address
     * @param data register data
     */
    public Register(int address, int data) {
        mAddress = address;
        mData = data;
    }
}
