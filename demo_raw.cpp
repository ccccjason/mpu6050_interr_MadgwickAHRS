#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"

Madgwick filter;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
unsigned long microsPerReading, microsPrevious;
struct timeval tms;

float convertRawAcceleration(int aRaw)
{
    // since we are using 2G range
    // -2g maps to a raw value of -32768
    // +2g maps to a raw value of 32767

    float a = (aRaw * 2.0) / 32768.0;
    return a;
}

float convertRawGyro(int gRaw)
{
    // since we are using 250 degrees/seconds range
    // -250 maps to a raw value of -32768
    // +250 maps to a raw value of 32767

    float g = (gRaw * 250.0) / 32768.0;
    return g;
}

void setup()
{
    // initialize device
    printf("Initializing I2C devices...\n");
    accelgyro.initialize();
    accelgyro.setRate(4); // 1khz / (1 + 4) = 200 Hz

    // verify connection
    printf("Testing device connections...\n");
    printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" :
           "MPU6050 connection failed\n");

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 200;
    timerclear(&tms);
    gettimeofday(&tms, NULL);
    microsPrevious = tms.tv_usec;

    accelgyro.setIntDataReadyEnabled(true); // enable data ready interrupt

    accelgyro.setZAccelOffset(-2339);
    accelgyro.setZAccelOffset(683);
    accelgyro.setZAccelOffset(1666);
    accelgyro.setXGyroOffset(32);
    accelgyro.setYGyroOffset(-72);
    accelgyro.setZGyroOffset(5);
}

void loop()
{
    int16_t aix, aiy, aiz;
    int16_t gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, heading;
    unsigned long microsNow;

    gettimeofday(&tms, NULL);
    microsNow = tms.tv_usec;

    //if (microsNow - microsPrevious >= microsPerReading)
    if (accelgyro.getIntDataReadyStatus() ==
        1) { // wait for data ready status register to update all data registers

        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

        // convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);

        // update the filter, which computes orientation
        filter.updateIMU(gx, gy, gz, ax, ay, az);

        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();

        printf("Orientation: heading=%f  pitch=%f  roll=%f\n", heading, pitch, roll);

        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
    }
}

int main()
{
    setup();

    for (;;) {
        loop();
    }
}

