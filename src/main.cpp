#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>


MPU6050 mpu6050(Wire1);
long timer = 0;

#define ANGLE_OFFSET_X 180
#define ANGLE_OFFSET_Y 0
#define ANGLE_OFFSET_Z 0

static float roll;
static float pitch;
static float yaw;


void setup()
{
    Serial.begin(115200);
    Wire1.setSCL(27);
    Wire1.setSDA(26);
    Wire1.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
}

void loop()
{
    mpu6050.update();

    if(millis() - timer > 500){
        roll = mpu6050.getAngleY();
        pitch = mpu6050.getAngleX();
        yaw = mpu6050.getAngleZ();
        Serial.printf("R: %.3f, P: %.3f, Y: %.3f\n", roll, pitch, yaw);
        timer = millis();
    }

}
