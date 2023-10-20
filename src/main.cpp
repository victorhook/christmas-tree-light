#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>
//#include <Adafruit_NeoPixel.h>
#include "crsf.h"


MPU6050 mpu6050(Wire1);
long timer = 0;

#define ANGLE_OFFSET_X 180
#define ANGLE_OFFSET_Y 0
#define ANGLE_OFFSET_Z 0

#define CHANNEL_ARM 4          // AUX 1
#define CHANNEL_RAISE_WINGS 6  // AUX 3
#define CHANNEL_SHOOT_SNOW 7   // AUX ??

#define MIN_VALUE_ARM         1600
#define MAX_VALUE_ARM         2000
#define MIN_VALUE_RAISE_WINGS 1600
#define MAX_VALUE_RAISE_WINGS 2000
#define MIN_VALUE_SHOOT_SNOW  1600
#define MAX_VALUE_SHOOT_SNOW  2000

#define WINGS_PULSE_LOW  1000
#define WINGS_PULSE_HIGH 1750

#define within_range(value, low, high) ( (value >= low) && (value <= high) )

// PINS
#define PIN_SERVO_WINGS 7
#define PIN_SERVO_SNOW  6
#define PIN_RGB_LED     8
#define PIN_CRSF_RX     5

// IMU
static float roll;
static float pitch;
static float yaw;

// RC
static rx_state_t state;
static rc_input_t rc_scaled;

// States
static bool armed = false;
static bool wings_deployed = false;
static bool snow_released = false;

// Servos
static Servo servo_wings;
static Servo servo_snow;

// RGB LED
#define NBR_OF_RGB_PIXELS 1
//Adafruit_NeoPixel pixels(NBR_OF_RGB_PIXELS, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

#define usbSerial Serial
#define crsfSerial Serial2

// Functions
static void arm();
static void disarm();
static void raise_wings();
static void lower_wings();
static void shoot_snow();
static void withdraw_snow();

static void handle_new_crsf_packet();


void setup()
{
    usbSerial.begin(115200);
    crsfSerial.setRX(PIN_CRSF_RX);
    crsfSerial.begin(420000);
    Wire1.setSCL(27);
    Wire1.setSDA(26);
    Wire1.begin();
    mpu6050.begin();
    //mpu6050.calcGyroOffsets(true);

    crsf_init();

    servo_wings.attach(PIN_SERVO_WINGS, WINGS_PULSE_LOW, WINGS_PULSE_HIGH);
    servo_snow.attach(PIN_SERVO_SNOW);
    disarm();
    lower_wings();

    //pixels.begin();
    //pixels.fill(0xff0000);
    //pixels.show();

    usbSerial.println("Starting!");
}

void loop()
{
    mpu6050.update();
    /*
    if(millis() - timer > 500){
        roll = mpu6050.getAngleY();
        pitch = mpu6050.getAngleX();
        yaw = mpu6050.getAngleZ();
        Serial.printf("R: %.3f, P: %.3f, Y: %.3f\n", roll, pitch, yaw);
        timer = millis();
    }
    */
    while (crsfSerial.available())
    {
        uint8_t byte = crsfSerial.read();

        if (crsf_parse_byte(byte))
        {
            // New packet!
            crsf_get_last_state(&state);
            crsf_scale_rc_channels(&state.last_packet, &rc_scaled);

            handle_new_crsf_packet();
            
        }
    }
}

// -- //

static void handle_new_crsf_packet()
{
    //for (int i = 0; i < 16; i++)
    //{
    //    usbSerial.printf("%4d ", rc_scaled.channels[i]);
    //}
    //int16_t pulse_us = constrain(rc_scaled.channels[1], 1000, 2000);
    //usbSerial.printf(" -> %d\n", pulse_us);

    // ARMING
    if (!armed)
    {
        if (within_range(rc_scaled.channels[CHANNEL_ARM], MIN_VALUE_ARM, MAX_VALUE_ARM))
        {
            arm();
        }
    }
    else
    {
        if (!within_range(rc_scaled.channels[CHANNEL_ARM], MIN_VALUE_ARM, MAX_VALUE_ARM))
        {
            disarm();
        }
    }

    // DEPOLOY WINGS
    if (!wings_deployed)
    {
        if (within_range(rc_scaled.channels[CHANNEL_RAISE_WINGS], MIN_VALUE_RAISE_WINGS, MAX_VALUE_RAISE_WINGS))
        {
            raise_wings();
        }
    }
    else
    {
        if (!within_range(rc_scaled.channels[CHANNEL_RAISE_WINGS], MIN_VALUE_RAISE_WINGS, MAX_VALUE_RAISE_WINGS))
        {
            lower_wings();
        }
    }
    
    // RELEASE SNOW
    if (!snow_released)
    {
        if (within_range(rc_scaled.channels[CHANNEL_SHOOT_SNOW], MIN_VALUE_SHOOT_SNOW, MAX_VALUE_SHOOT_SNOW))
        {
            shoot_snow();
        }
    }
    else
    {
        if (!within_range(rc_scaled.channels[CHANNEL_SHOOT_SNOW], MIN_VALUE_SHOOT_SNOW, MAX_VALUE_SHOOT_SNOW))
        {
            withdraw_snow();
        }
    }
}

// -- //

static void arm()
{
    usbSerial.println("ARMING!");
    armed = true;
}

static void disarm()
{
    usbSerial.println("DISARMING!");
    armed = false;
}

static void raise_wings()
{
    Serial.println("RAISING WINGS!");
    wings_deployed = true;
    float pulse_us = WINGS_PULSE_LOW;
    float p = 0.05;
    while (pulse_us < (WINGS_PULSE_HIGH-1))
    {
        float err = (float) (WINGS_PULSE_HIGH - pulse_us);
        pulse_us += err * p;
        servo_wings.writeMicroseconds((int16_t) pulse_us);
        delayMicroseconds(1200);
        usbSerial.printf("%f, %f\n", pulse_us, WINGS_PULSE_HIGH);
    }
    usbSerial.println("WINGS RAISED!");
}

static void lower_wings()
{
    Serial.println("LOWERING WINGS!");
    wings_deployed = false;
    int16_t pulse_us = WINGS_PULSE_HIGH;
    while (pulse_us > (WINGS_PULSE_LOW+1))
    {
        delayMicroseconds(1200);
        pulse_us -= 1;
        servo_wings.writeMicroseconds(pulse_us);
    }
    usbSerial.println("WINGS LOWERED!");
}

static void shoot_snow()
{
    Serial.println("SHOOTING SNOW!");
    snow_released = true;
}

static void withdraw_snow()
{
    Serial.println("WITHDRAWING SNOW!");
    snow_released = false;
}
