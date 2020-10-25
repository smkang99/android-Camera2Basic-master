package com.example.android.camera2basic;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static android.R.attr.data;
import static java.lang.Long.parseLong;

/**
 * Created by Abhinav Mathur on 4/20/2017.
 * SensorRegister class contains APIs to be able to read/write sensor registers.
 * The sensor needs to be powered up during this process.
 * This class and the underlying sensor driver supports 2 byte address, 1 byte data accesses.
 */

public class Sensor {
    private static final String SENSOR_READ_PATH = "/sys/kernel/debug/imx290/poke";
    private static final String SENSOR_WRITE_PATH = "/sys/kernel/debug/imx290/peek";
    private static final String SENSOR_STATUS_PATH = "/sys/kernel/debug/imx290/status";

    /**
     * Performs the actual sensor read.
     * This function sends the command to the kernel. Command is in the format
     * echo Address > /sys/kernel/debug/imx290/peek.
     * This supports 2 byte address and reads back a byte of data.
     * @param sensorAction : Sets the action here. Acceptable value are READ_ACTION : 0x00 or
     *                     WRITE_ACTION : 0x01.
     * @param registers : Receives a list of Register object with addresses to be read.
     * @return : Returns register read values. A null return value represents an error condition.
     */
    public static synchronized List<BigInteger> read(int sensorAction, Iterable<Register> registers) {
        List<BigInteger> dataList = new ArrayList<>();
        try (
                FileWriter fw = new FileWriter(SENSOR_WRITE_PATH);
                BufferedWriter bw = new BufferedWriter(fw);
                FileReader fr = new FileReader(SENSOR_READ_PATH);
                BufferedReader br = new BufferedReader(fr)
        ) {
            for (Register register : registers) {
                String command = "0x" + Integer.toHexString(sensorAction & 0xFF)
                        + String.format("%04X", register.mAddress & 0xFFFF);
                bw.write(command);
                bw.flush();
                BigInteger data = new BigInteger(br.readLine());
                dataList.add(data);
            }
            return dataList;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Performs the actual sensor read for a single Register object
     * @param sensorAction : Sets the action here. Acceptable value are READ_ACTION : 0x00 or
     *                     WRITE_ACTION : 0x01.
     * @param register : Receives a Register object with addresses to be read.
     * @return : Returns register read values. A null return value represents an error condition.
     */
    public static BigInteger read(int sensorAction, Register register) {
        return read(sensorAction, new ArrayList<Register>(Arrays.asList(register))).get(0);
    }

    /**
     * Performs the actual sensor read for a single register address
     * @param sensorAction : Sets the action here. Acceptable value are READ_ACTION : 0x00 or
     *                     WRITE_ACTION : 0x01.
     * @param address : Receives the address of register to be read from.
     * @return : Returns register read values. A null return value represents an error condition.
     */
    public static BigInteger read(int sensorAction, int address) {
        return read(sensorAction, new Register(address, 0));
    }

    /**
     * Performs the actual sensor write.
     * This function sends the command to the kernel. Command is in the format
     * echo Data-WriteBits-Address > sys/kernel/debug/imx290/peek.
     * This supports 2 byte address and a byte of data.
     * @param sensorAction : Sets the action here. Acceptable value are READ_ACTION : 0x00 or
     *                     WRITE_ACTION : 0x01.
     * @param registers : Receives a list of Register objects with address and data to be written.
     * @return : Return boolean true for success, false for failure.
     */
    public static synchronized boolean write(int sensorAction, Iterable<Register> registers) {
        try (
                FileWriter fw = new FileWriter(SENSOR_WRITE_PATH);
                BufferedWriter bw = new BufferedWriter(fw);
        ) {
            for (Register register : registers) {
                String command = String.format("0x%02X%02X%04X",
                        register.mData & 0xFF,
                        sensorAction & 0xFF,
                        register.mAddress & 0xFFFF);
                bw.write(command);
                bw.flush();
            }
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Performs the actual sensor write
     * @param sensorAction : Sets the action here. Acceptable value are READ_ACTION : 0x00 or
     *                     WRITE_ACTION : 0x01.
     * @param register : Receives a REgister object with address and data to be written.
     * @return : Return boolean true for success, false for failure.
     */
    public static boolean write(int sensorAction, Register register) {
        return write(sensorAction, new ArrayList<Register>(Arrays.asList(register)));
    }

    /**
     * Performs the actual sensor write
     * @param sensorAction : Sets the action here. Acceptable value are READ_ACTION : 0x00 or
     *                     WRITE_ACTION : 0x01.
     * @param address : Receives address to be written to.
     * @param data : Receives data to be written to.
     * @return : Return boolean true for success, false for failure.
     */
    public static boolean write(int sensorAction, int address, int data) {
        return write(sensorAction, new Register(address, data));
    }

    /**
     * Used to read the status of the read or write transaction from the driver.
     * @return : Returns 0 for not busy, 1 for busy and -1 for error condition.
     */
    public static synchronized int status() {
        int status = -1;
        try (
                FileReader fr = new FileReader(SENSOR_READ_PATH);
                BufferedReader br = new BufferedReader(fr);
        ) {
            String la = br.readLine();
            status = Integer.parseInt(la);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return status;
    }
}
