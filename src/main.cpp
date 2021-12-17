#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "i2c.h"
#include "i2c_BMP280.h"
#include "SD.h"

#define chipSelect 10
#define parachuteAltitudeDifference 1
#define serial  false
#define calibrationBorder   1.0
bool isCalibrated = false;

bool parachuteStatus = 0;

class Buzzer
{
public:
    int pin;
    byte tone;
    bool state = false;
    Buzzer(int _pin)
    {
        pinMode(_pin, OUTPUT);
        pin = _pin;
    }
    void quiet()
    {
        printTone(0);
    }
    void squeak()
    {
        printTone(255);
    }
    void cracklingSqueak()
    {
        printTone(120);
    }
    void softSqueak()
    {
        printTone(1);
    }
    void printTone(byte _tonePWM)
    {
        tone = _tonePWM;
        if(_tonePWM>0)
        {
            state = true;
            analogWrite(pin,_tonePWM);
        }
        else
        {
            state = false;
            digitalWrite(pin,LOW);
        }
    }
};

Buzzer buzzer(5);
BMP280 bmp280;
Adafruit_MPU6050 mpuAccel;

float heightCalibration()
{
    float difference, averagedMeters, meters, t_temperature;
    do
    {
        bmp280.awaitMeasurement();
        bmp280.getTemperature(t_temperature);
        bmp280.getAltitude(meters);
        averagedMeters = (averagedMeters * 4 + meters)/5;
        difference = averagedMeters - meters;
        bmp280.triggerMeasurement();
    }
    while(!(difference >= -calibrationBorder && difference <= calibrationBorder));

    return averagedMeters;
}

void setup() 
{
    if(serial)
    {
        Serial.begin(9600);
    }
    pinMode(2, OUTPUT);

    delay(2000);

    if(SD.begin(chipSelect)) 
    {
        buzzer.squeak();
        delay(250);
        buzzer.quiet();
        delay(100);
        buzzer.squeak();
        delay(250);
        buzzer.quiet();
    }
    else
    {
        while(1) {}
    }

    delay(2000);

    if (bmp280.initialize()) 
    {
        buzzer.squeak();
        delay(500);
        buzzer.quiet(); 
    }
    else
    {
        while (1) {}
    }

    const float averagedGroundAltitude = heightCalibration();
    static float averageAltitude = averagedGroundAltitude;

    delay(2000);

    if(mpuAccel.begin())
    {
        buzzer.squeak();
        delay(250);
        buzzer.quiet();
        delay(100);
        buzzer.squeak();
        delay(250);
        buzzer.quiet();
    }
    else
    {
        while (1) {}
    }

    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
}

void loop()
{ 
    sensors_event_t a, g, temp;
    mpuAccel.getEvent(&a, &g, &temp);

    //OUTPUT: ACC X, Y, Z, ROTATION Z, TILT X, Y, ALTITUDE, AVGALTITUDE, TEMPERATURE
    /* BMP280: temperature, pascal, altitude, averageAltitude */
    float temperature, pascal, altitude;
    static float max_alt = 0, averageAltitude;

    bmp280.awaitMeasurement();
    bmp280.getTemperature(temperature);
    bmp280.getPressure(pascal);
    bmp280.getAltitude(altitude);

    averageAltitude = (averageAltitude * 4 + altitude)/5;

    bmp280.triggerMeasurement();
  
    if(serial)
    {
        Serial.print(a.acceleration.x);
        Serial.print(",\t");
        Serial.print(a.acceleration.y); //ACCELERATION Y (VERTICALLY) (M/S^2)
        Serial.print(",\t");
        Serial.print(a.acceleration.z);
        Serial.print(",\t");

        Serial.print(g.gyro.z); //ROTATION AROUND ITS AXIS (RAD/S)
        Serial.print(",\t");

        Serial.print(altitude);
        Serial.print(",\t");
        Serial.print(averageAltitude);
        Serial.print(",\t");
        Serial.println(temperature);
    }
  
    File file = SD.open("Flight.txt", FILE_WRITE);
  
    if(file)
    {
        file.print(a.acceleration.x);
        file.print(",\t");
        file.print(a.acceleration.y); //ACCELERATION Y (VERTICALLY) (M/S^2)
        file.print(",\t");
        file.print(a.acceleration.z);
        file.print(",\t");

        file.print(g.gyro.z); //ROTATION AROUND ITS AXIS (RAD/S)
        file.print(",\t");

        file.print(altitude);
        file.print(",\t");
        file.print(averageAltitude);
        file.print(",\t");
        file.println(temperature);

        file.close();
    }


    if(parachuteStatus == 0)
        {
            if(averageAltitude > max_alt)
            {
                max_alt = averageAltitude;
            }
            else
            {
                if(max_alt - averageAltitude > parachuteAltitudeDifference)
                {
                File file2 = SD.open("PS.txt", FILE_WRITE); // PARACHUTE STATUS
                buzzer.squeak();
                digitalWrite(2, HIGH);
                parachuteStatus = 1;
                Serial.println("PE"); //PARACHUTE EJECTION
                file2.println("Parachute Ejection Confirmed");
                file2.close();
                }
            } 
        }
    delay(100);
}