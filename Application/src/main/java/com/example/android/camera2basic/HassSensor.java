package com.example.android.camera2basic;

import java.math.BigInteger;
import java.util.List;

/**
 * Created by Allan Suen on 6/21/2017.
 */

public class HassSensor extends Sensor {
    public static final int SHS1_L = 0x3020;
    public static final int SHS1_M = 0x3021;
    public static final int SHS1_H = 0x3022;

    public static final int SHS2_L = 0x3024;
    public static final int SHS2_M = 0x3025;
    public static final int SHS2_H = 0x3026;

    public static final int GAIN = 0x3014;

    public static final int VMAX_L = 0x3018;
    public static final int VMAX_M = 0x3019;
    public static final int VMAX_H = 0x301A;

    private static final int READ_ACTION = 0x00;
    private static final int WRITE_ACTION = 0x01;

    /**
     * This function is used to read Hass Sensor register
     * @param registers: Receives a list of Register object with addresses to be read.
     * @return: Returns register read values. A null return value represents an error condition.
     */
    public static List<BigInteger> read(Iterable<Register> registers) {
        return read(READ_ACTION, registers);
    }

    /**
     * This function is used to read Hass Sensor register
     * @param register: Receives a Register object with addresses to be read.
     * @return: Returns register read value. A null return value represents an error condition.
     */
    public static BigInteger read(Register register) {
        return read(READ_ACTION, register);
    }

    /**
     * This function is used to read Hass Sensor register
     * @param address : Receives address of a specific register to be read.
     * @return: Returns register read value. A null return value represents an error condition.
     */
    public static BigInteger read(int address) {
        return read(READ_ACTION, address);
    }

    /**
     * This function is used to write to Hass Sensor register
     * @param registers: Receives a list of Register object with address and data to be written.
     * @return: Returns boolean true for success, false for failure.
     */
    public static boolean write(Iterable<Register> registers) {
        return write(WRITE_ACTION, registers);
    }

    /**
     * This function is used to write to Hass Sensor register
     * @param register : Receives a Register object with address and data to be written.
     * @return: Return boolean true for success, false for failure.
     */
    public static boolean write(Register register) {
        return write(WRITE_ACTION, register);
    }

    /**
     * This function is used to write to Hass Sensor register
     * @param address : Receives address to be written to.
     * @param data : Receives data to be written to.
     * @return : Returns boolean true for success, false for failure.
     */
    public static boolean write(int address, int data) {
        return write(WRITE_ACTION, address, data);
    }
}

