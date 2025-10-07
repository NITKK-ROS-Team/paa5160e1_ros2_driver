/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <Arduino.h>

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

#include <CRC8.h> // Arduino CRC library (Rob Tillaart)

QwiicOTOS myOtos;
const int ledPin = 13;

static const char SERIAL_HEADER = 'X';

union float2byte {
    float f;
    uint8_t b[4];
};

static void get_time(int32_t &sec, int32_t &msec)
{
    unsigned long ms = millis();
    sec = ms / 1000;
    msec = ms % 1000;
}

void setup()
{
    pinMode(ledPin, OUTPUT);

    Serial.begin(115200);
    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        delay(1000);
    }

    myOtos.calibrateImu();
    myOtos.resetTracking();
}

void loop()
{
    while (Serial.available())
    {
        int ch = Serial.read();
        if (ch == 'r' || ch == 'R')
        {
            myOtos.resetTracking();
            digitalWrite(ledPin, LOW);
            delay(30);
            digitalWrite(ledPin, HIGH);
        }
    }

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    // Send binary packet: 'X' header, sec, msec, x, y, yaw, CRC8, then newline
    {
        int32_t sec = 0, msec = 0;
        get_time(sec, msec);

        float2byte fx, fy, fyaw;
        fx.f = myPosition.x;
        fy.f = myPosition.y;
        fyaw.f = myPosition.h;

        CRC8 crc(CRC8_DALLAS_MAXIM_POLYNOME,
                 CRC8_DALLAS_MAXIM_INITIAL,
                 CRC8_DALLAS_MAXIM_XOR_OUT,
                 CRC8_DALLAS_MAXIM_REV_IN,
                 CRC8_DALLAS_MAXIM_REV_OUT);
        crc.restart();
        crc.add((uint8_t)SERIAL_HEADER);
        crc.add(reinterpret_cast<const uint8_t*>(&sec), 4);
        crc.add(reinterpret_cast<const uint8_t*>(&msec), 4);
        crc.add(fx.b, 4);
        crc.add(fy.b, 4);
        crc.add(fyaw.b, 4);
        uint8_t crc8 = crc.calc();

        Serial.write(static_cast<uint8_t>(SERIAL_HEADER));
        Serial.write(reinterpret_cast<uint8_t*>(&sec), 4);
        Serial.write(reinterpret_cast<uint8_t*>(&msec), 4);
        Serial.write(fx.b, 4);
        Serial.write(fy.b, 4);
        Serial.write(fyaw.b, 4);
        Serial.write(crc8);
        Serial.write('\n');
    }
}
