//#include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <Wire.h>

#define HDCCONFIGADDRESS 0x02
#define HDC1080ADDRESS 0x40
#define HUMIDITY 0x01
#define TEMPERATURE 0x00

void collectData_t(void * queue);
void readGPS_t(void * queue);
void readHDC1080_temperature_t(void * queue);

//NOTE: a squid is a slang term for a "show off" on a street bike
class squid
{
    public:
        double latlng[4];        //Latitude/longitude at the start and end of special event
        double bikeYaw[100];        //WheelieAngle
        double bikeRoll[100];       //WheelieRoll
        unsigned long stuntTime;    //Time spent in stunt

        void clearData()
        {
            for(int i = 0; i < 4; i++)
                latlng[i] = 0;
            for(int i = 0; i < 100; i++)
            {
                bikeYaw[i] = 0;
                bikeRoll[i] = 0;
            }
        }
        //When object is created the value 0 is used to show no value has been entered
        squid()
        {
            for(int i = 0; i < 4; i++)
                latlng[i] = 0;
            for(int i = 0; i < 100; i++)
            {
                bikeYaw[i] = 0;
                bikeRoll[i] = 0;
            }
        }
};

class dataQueues
{
    public:
    QueueHandle_t gpsReadings;
    QueueHandle_t accelReadings;
};

TinyGPSPlus gps;

void setup()
{                 
    Serial.begin(9600);
    Wire.begin();
    QueueHandle_t gpsReadings;
    gpsReadings = xQueueCreate(1, ((sizeof(double)) * 2));
    dataQueues * collectDataQueues_ptr = new dataQueues;
    collectDataQueues_ptr->gpsReadings = gpsReadings;
    xTaskCreate(readGPS_t, "Read the air530 grove GPS data", 8096, (void*)gpsReadings, 2, NULL);
    xTaskCreate(collectData_t, "Collect data from the reading tasks", 8096, (void*)collectDataQueues_ptr, 1, NULL);
    xTaskCreate(readHDC1080_temperature_t, "Read temperature from HDC1080", 2048, (void*)gpsReadings, 3, NULL);
}

//This is an example function of reading from an I2C device while using the GPS
//using the Wire.h library
void readHDC1080_temperature_t(void * queue)
{
  uint8_t binaryTemp[2];
  uint32_t tempCelcius;
  uint8_t config[3];
  config;
  config[0] = HDCCONFIGADDRESS;
  config[1] = 0x06;
  config[2] = 0x0;
  Wire.beginTransmission(HDC1080ADDRESS);
  Wire.write(config, 3);
  Wire.endTransmission();
  vTaskDelay(5);

  while(1)
  {
    Wire.beginTransmission(HDC1080ADDRESS);
    Wire.write(TEMPERATURE);
    Wire.endTransmission();
    vTaskDelay(5);
    Wire.requestFrom(HDC1080ADDRESS, 2);
    while(Wire.available())
    {
      binaryTemp[0] = Wire.read(); // read from the starting register
      binaryTemp[1] = Wire.read();
    }

    tempCelcius = binaryTemp[0] << 8 | binaryTemp[1];
    //This bit shift is used to help with data accuracy. It will be shifted back after computations
    tempCelcius = tempCelcius << 8;
    //Formula in data sheet to convert the binary to a temp in celcius
    //Adjustments are likely possible and maybe necessary to get more accurate farenheit readings
    tempCelcius = tempCelcius*165;
    tempCelcius = tempCelcius/65536;
    //Shift back
    tempCelcius = tempCelcius >> 8;
    tempCelcius = (tempCelcius-40);

    Serial.print("Temperature is: ");
    Serial.print(tempCelcius);
    Serial.println(" degrees celcius.");
    vTaskDelay(1000);
  }  
}

void collectData_t(void * queue)
{
    dataQueues * queueStruct = (dataQueues*)queue; 
    QueueHandle_t gpsReadings = queueStruct->gpsReadings;
    squid rider;
    //float test = 0;
    unsigned long startTime;
    unsigned long endTime;
    while(1)
    {
        //Looking for odd behaviour from the accelerometer(s) until something is found
        //Possibilities: Wheelie, Stoppie, Crash

        //get lat/long data and time for start
        startTime = millis();
        if(xQueueReceive(gpsReadings, &rider.latlng[0], 500) == pdTRUE)
        {
            Serial.print("Rider stunt begins at: ");
            Serial.print(rider.latlng[0], 6);
            Serial.print(",");
            Serial.println(rider.latlng[1], 6);
        }
        //Repeatedly get accelerometer information here and move on when ending circumstances met
        //Timeout, Wheelie end, Stoppie end, crash movement stopped etc.
        //NOTE: wheelies have potential to go for minutes at a time and thus the timeout cannot be too soon.

        //get lat/long data and time for end of event
        endTime = millis();
        rider.stuntTime = endTime - startTime;
        if(xQueueReceive(gpsReadings, &rider.latlng[2], 500) == pdTRUE)
        {
            Serial.print("Rider stunt ends at: ");
            Serial.print(rider.latlng[2], 6);
            Serial.print(",");
            Serial.println(rider.latlng[3], 6);
        }
        Serial.print("Time elapsed: ");
        Serial.println(rider.stuntTime);
        //Send the data to the microSD where another task will upload data from the microSD to the web server

        vTaskDelay(250);
    }
}

//For testing purposes the AirGrove is set up in this code to use the second grove port on the vanduino
//The grove connection is labeled GP13/GP12
void readGPS_t(void * queue)
{
    QueueHandle_t gpsReadings = (QueueHandle_t)queue;
    //This will be changed to regular serial once I can't help it anymore or I get better debugging tools
    SoftwareSerial SoftSerial(13, 12);
    SoftSerial.begin(9600);
    double latlng[1];
    
    while(1)
    {
        if (SoftSerial.available())                     // if date is coming from software serial port ==> data is coming from SoftSerial shield
        {
            while(SoftSerial.available())               // reading data into char array
            {
                if(gps.encode(SoftSerial.read()))
                {
                    latlng[0] = gps.location.lat();
                    latlng[1] = gps.location.lng();
                    xQueueSend(gpsReadings, &latlng, 0);
                }
            } 
        }
        vTaskDelay(100);
    }
}

void loop()
{
    
}
