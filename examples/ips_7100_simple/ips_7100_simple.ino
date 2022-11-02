/*

Module: ips_7100_simple.ino

Function:
    Simple example for MCCI Catena IPS-7100 library.

Copyright and License:
    See accompanying LICENSE file for copyright and license information.

Author:
    Pranau R, MCCI Corporation   October 2022

*/

#include <MCCI_Catena_IPS-7100.h>

#include <Arduino.h>
#include <Wire.h>

/****************************************************************************\
|
|   Manifest constants & typedefs.
|
\****************************************************************************/

using namespace McciCatenaIps7100;

/****************************************************************************\
|
|   Variables.
|
\****************************************************************************/

cIPS7100 gIps {Wire};

/****************************************************************************\
|
|   Code.
|
\****************************************************************************/

/*
Name:   setup()
Function:
        Initilize sensor and serial communication.
Definition:
        void setup(
            void
            );
Description:
        To initiate a serial connection between board and display and to check the connectivity of 7100 Particle Sensor.
Returns:
        No explicit result.
*/

void setup()
    {
    // Initiate USB serial at 115200 baud
    Serial.begin(115200);
    while (! Serial);
    Serial.println("\n****  7100 Particle Sensor I2C Example ****");
    // Wait on IPS boot
    delay(1000);

    while(!gIps.begin())
        {
        Serial.println("7100 Particle Sensor not connected!\n");
        delay(1000);
        }

    Serial.println("7100 Particle Sensor connected successfully!\n");

    delay (2000);
    }

/*
Name:   loop()
Function:
        To get particle readings and display it in serial monitor.
Definition:
        void loop (
            void
            );
Returns:
        No explicit result.
*/

void loop()
    {
    // Get new IPS sensor readings
    // Not meant to run more than once per second
    gIps.updateData();

    // Print sensor status
    int status = gIps.getStatus();
    Serial.print("STATUS  : ");
    Serial.println(status);

    // Print Vref value
    int vref = gIps.getVref();
    Serial.print("Vref    : ");
    Serial.println(vref);

    // Print PM10 via USB serial
    Serial.print("PM10    : ");
    Serial.println(gIps.getPM50Data());

    // Print PM1.0 via USB serial
    Serial.print("PM0.1   : ");
    Serial.println(gIps.getPM01Data());

    // Print PM1.0 via USB serial
    Serial.print("PM0.3   : ");
    Serial.println(gIps.getPM03Data());

    // Print PM1.0 via USB serial
    Serial.print("PM0.5   : ");
    Serial.println(gIps.getPM05Data());

    // Print PM1.0 via USB serial
    Serial.print("PM1.0   : ");
    Serial.println(gIps.getPM10Data());

    // Print PM2.5 via USB serial
    Serial.print("PM2.5   : ");
    Serial.println(gIps.getPM25Data());

    // Print PM10 via USB serial
    Serial.print("PM5.0    : ");
    Serial.println(gIps.getPM50Data());

    // Print PM10 via USB serial
    Serial.print("PM10    : ");
    Serial.println(gIps.getPM100Data());

    // Print PC1.0 via USB serial
    Serial.print("PC0.1   : "); 
    Serial.println(gIps.getPC01Data());

    // Print PC1.0 via USB serial
    Serial.print("PC0.3   : "); 
    Serial.println(gIps.getPC03Data());

    // Print PC1.0 via USB serial
    Serial.print("PC0.5   : "); 
    Serial.println(gIps.getPC05Data());

    // Print PC1.0 via USB serial
    Serial.print("PC1.0   : "); 
    Serial.println(gIps.getPC10Data());

    // Print PC1.0 via USB serial
    Serial.print("PC2.5   : "); 
    Serial.println(gIps.getPC25Data());

    // Print PC1.0 via USB serial
    Serial.print("PC5.0   : "); 
    Serial.println(gIps.getPC50Data());

    // Print PC1.0 via USB serial
    Serial.print("PC10   : "); 
    Serial.println(gIps.getPC100Data());

    Serial.println("");
    Serial.println("##########################################");
    Serial.println("");

    delay(3000);
    }