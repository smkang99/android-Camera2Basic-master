package com.example.android.camera2basic;

import android.util.Log;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by Abhinav Mathur on 2/1/2017.
 * GPIO class containing functions set direction and value
 * of a pre-exported GPIO pin. The class also exposes functions
 * to read the direction and value of the pin.
 */

public class GPIO {

    public static int HIGH = 1;
    public static int LOW = 0;
    private int pin;
    private String PATH = "/sys/class/gpio";

    public GPIO(int pin) {
        this.pin = pin;
    }

    private void setDirection(String direction) {
        try (
                FileWriter fw = new FileWriter(PATH + "/gpio" + pin + "/direction", false);
                BufferedWriter bw = new BufferedWriter(fw)
        ) {
            bw.write(direction);
        } catch (IOException e) {
            Log.e(toString(), "Error: " + e.getMessage());
        }
    }

    public void setValue(int value) {
        try (
                FileWriter fw = new FileWriter(PATH + "/gpio" + pin + "/value", false);
                BufferedWriter bw = new BufferedWriter(fw)
        ) {
            bw.write(Integer.toString(value));
        } catch (IOException e) {
            Log.e(toString(), "Error: " + e.getMessage());
        }
    }

    private String returnDirection() {
        String line = "";
        try (
                FileReader fr = new FileReader(PATH + "/gpio" + pin + "/direction");
                BufferedReader br = new BufferedReader(fr)
        ) {
            line = br.readLine();
        } catch (IOException e) {
            Log.e(toString(), "Error: " + e.getMessage());
        }
        return line;
    }

    private int returnValue() {
        String line = "";
        try (
                FileReader fr = new FileReader(PATH + "/gpio" + pin + "/value");
                BufferedReader br = new BufferedReader(fr)
        ) {
            line = br.readLine();
        } catch (IOException e) {
            Log.e("GPIO", "Error: " + e.getMessage());
        }
        if (line.isEmpty()) {
            return -1;
        } else {
            return Integer.parseInt(line);
        }
    }

    /* Public functions */

    /**
     * Sets GPIO pin to value of 1
     */
    public void setHigh() { setValue(GPIO.HIGH); }

    /**
     * Sets GPIO pin to value of 0
     */
    public void setLow() {
        setValue(GPIO.LOW);
    }

    /**
     * Sets direction of GPIO pin to be an output
     */
    public void setOut() {
        setDirection("out");
    }

    /**
     * Sets direction of GPIO pin to be an input
     */
    public void setIn() {
        setDirection("in");
    }

    /**
     * Return state of the GPIO pin.
     * @return: Values sent back are 0 or 1.
     */
    public int getValue() { return returnValue(); }

    /**
     * Retruns direction set for the GPIO pin.
     * @return: Values sent back are "in" for input or "out" for output.
     */
    public String getDirection() { return returnDirection(); }

    @Override
    public String toString() {
        return "GPIO";
    }
}