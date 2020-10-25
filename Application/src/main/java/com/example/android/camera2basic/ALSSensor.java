package com.example.android.camera2basic;

public class ALSSensor {
    public static void scheduleRead(int i) {
    }

    public static void shutdown() {
    }

    public static Object getData() {
        return null;
    }

    public static void write(int arg1, int arg2) {
    }

    public static Number read(int arg1) {
        return new Number() {
            @Override
            public int intValue() {
                return 0;
            }

            @Override
            public long longValue() {
                return 0;
            }

            @Override
            public float floatValue() {
                return 0;
            }

            @Override
            public double doubleValue() {
                return 0;
            }
        };
    }
}
