#include <jni.h>
#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <fstream>
#include <iterator>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>

//#include <CL/cl.h>

//#include "exercise.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "MuSARc", __VA_ARGS__))

#define I2C_RDWR 0x0707
int file;
int addr;

struct i2c_msg_sony {
    __u16 addr;
    __u16 flags;
    __u16 len;
    __u8 *buf;
};

struct i2c_rdwr_ioctl_data {
    struct i2c_msg_sony *msgs;  /* ptr to array of simple messages */
    int nmsgs;             /* number of messages to exchange */
};

struct i2c_msg_sony msg;
unsigned int buf;
struct i2c_rdwr_ioctl_data msgset;

void I2C_test();
void I2C_read(int regAddress, char* readData, int numBytes);
void I2C_write(int regAddress, char* writeData, int numBytes);
AAssetManager *amgr;

extern "C"
JNIEXPORT jstring JNICALL
Java_com_sony_isdc_musarc_MainActivity_stringFromJNI(JNIEnv *env, jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}


/* Initializes the I2C for sensor data writes.
        Is responsible for turning on Power to the sensor board and un-resetting it.
        Set's the slave address of the device and creates a handle to the device.
        Returns -1 if unable to get handel to i2c device.
*/
extern "C"
int Java_com_sony_isdc_musarc_I2C_I2Cinit(
        JNIEnv *env,
        jobject /* this */, short slaveAddress)
{
    // Init Hardware
    // Follow sequence
    /*  VREG_LVS1A_1P8
        - set enable 1*/
    system("echo 1 > /sys/kernel/debug/regulator/pm8994_lvs1/enable");
    /*   VREG_L18
       - est enable 1*/
    system("echo 1 > /sys/kernel/debug/regulator/pm8994_l18/enable");
    /*  CAM1_RST GPIO104
       - export
       - set value 1*/
    system("echo 1 > /sys/class/gpio/gpio104/value");

    char *filename = "/dev/i2c-8";
    int res = 0;
    int RegAddress = 0x3014;
    unsigned int Data;
    int Bytes = 0;

    if ((file = open(filename, O_RDWR)) < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        perror("Failed to open the i2c bus");
        //return (*env)->NewStringUTF(env, "Failed to open the i2c bus");
        //exit(1);
    }
    addr = slaveAddress; /* The I2C address */
    return file;
}

/* Performs the actual read of the I2C bus.
       The address/data is sent out LSB first, so need to repackage the address/data.
       Handles 2 byte address, 1 byte of data.
       Returns the readData to the caller.
*/
extern "C"
int Java_com_sony_isdc_musarc_I2C_I2Cread(
        JNIEnv *env,
        jobject /* this */, int regAddress, int numBytes)
{
    int res = 0;
    short address;
    char readData;
    int readDataInt;

    // Reverse Address
    address = regAddress << 8;
    address |= regAddress >> 8;

    //Populate the i2c msg structure to do a simple write
    msg.addr = addr; //Slave Address
    msg.flags = 0;// Specify Read(1) or Write (0)
    msg.len = 2; //Bytes to Write
    msg.buf = (__u8*)&buf;
    buf = address;
    msgset.msgs = &msg;
    msgset.nmsgs = 1;
    res = ioctl(file, I2C_RDWR, &msgset);

    msg.addr = addr; //Slave Address
    msg.flags = I2C_M_RD; // Specify Read(1) or Write (0)
    msg.len = numBytes; //Bytes to Read
    msg.buf = (__u8*)&buf;
    msgset.msgs = &msg;
    msgset.nmsgs = 1;
    res = ioctl(file, I2C_RDWR, &msgset);
    readData = *msg.buf;
    readDataInt = (int)readData;
    return(readData);

}

/* Performs the actual write onto the I2C bus.
       The data is sent out LSB first, so need to repackage the data.
       Handles 2 byte address, 1 byte of data.
       Returns -1 if write fails.
*/
extern "C"
int Java_com_sony_isdc_musarc_I2C_I2Cwrite(
        JNIEnv *env,
        jobject /* this */,int regAddress, char writeData, int numBytes)
{
    int res = 0;
    unsigned short address;
    int Data = writeData;
    // Reverse Address
    address = regAddress << 8;
    address |= regAddress >> 8;

    //Populate the i2c msg structure to do a simple write
    msg.addr = addr; //Slave Address
    msg.flags = 0; // Specify Read(1) or Write (0)
    msg.len = numBytes + 2; // 2 bytes are overhead
    msg.buf = (__u8*)&buf; // Data to write
    buf = (Data << 16) | address;
    msgset.msgs = &msg;
    msgset.nmsgs = 1;
    res = ioctl(file, I2C_RDWR, &msgset);
    return res;
}

extern "C"
void Java_com_sony_isdc_musarc_SensorRegister_SensorSendCommand(
        JNIEnv *env,
        jobject /* this */, jstring command)
{
    char * cmd;
    cmd = (char *) env->GetStringUTFChars(command , NULL ) ;
    system(cmd);
}

extern "C"
void Java_com_sony_isdc_musarc_ALSSensor_ALSSensorSendCommand(
        JNIEnv *env,
        jobject /* this */, jstring command)
{
    char * cmd;
    cmd = (char *) env->GetStringUTFChars(command , NULL ) ;
    system(cmd);
}

extern "C"
void Java_com_sony_isdc_musarc_IRLEDControl_IRLEDSendCommand(
        JNIEnv *env,
        jobject /* this */, jstring command)
{
    char * cmd;
    cmd = (char *) env->GetStringUTFChars(command , NULL ) ;
    system(cmd);
}