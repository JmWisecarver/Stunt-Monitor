//#include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"

void collectData_t(void * queue);
void readGPS_t(void * queue);

//NOTE: a squid is a slang term for a "show off" on a street bike
class squid
{

    public:
        double latlngtme[8];        //Latitude/longitude/minutes/seconds at the start and end of special event
        double bikeYaw[100];        //Wheelieheight
        double bikeRoll[100];       //WheelieRoll

        void clearData()
        {
            for(int i = 0; i < 8; i++)
                latlngtme[i] = 0;
            for(int i = 0; i < 100; i++)
            {
                bikeYaw[i] = 0;
                bikeRoll[i] = 0;
            }
        }
        //When object is created the value 0 is used to show no value has been entered
        squid()
        {
            for(int i = 0; i < 8; i++)
                latlngtme[i] = 0;
            for(int i = 0; i < 100; i++)
            {
                bikeYaw[i] = 0;
                bikeRoll[i] = 0;
            }
        }
};


TinyGPSPlus gps;

void setup()
{                 
    Serial.begin(9600);
    QueueHandle_t gpsReadings;
    gpsReadings = xQueueCreate(1, ((sizeof(double)) * 4));
    xTaskCreate(readGPS_t, "Read the air530 grove GPS data", 8096, (void*)gpsReadings, 2, NULL);
    xTaskCreate(collectData_t, "Collect data from the reading tasks", 8096, (void*)gpsReadings, 1, NULL);
}

void collectData_t(void * queue)
{
    QueueHandle_t gpsReadings = (QueueHandle_t)queue;
    squid rider;
    float test = 0;
    while(1)
    {
        //Looking for odd behaviour from the accelerometer until something is found
        //Possibilities: Wheelie, Stoppie, Crash

        //get lat/long data and time for start
        if(xQueueReceive(gpsReadings, &rider.latlngtme[0], 0) == pdTRUE)
        {
            //Serial.println(rider.latlngtme[0], 6);
            //Serial.println(rider.latlngtme[1], 6);
            Serial.println(rider.latlngtme[2]);
            Serial.println(rider.latlngtme[3]);
        }
        //Repeatedly get accelerometer information here and move on when ending circumstances met


        //get lat/long data and time for end
        if(xQueueReceive(gpsReadings, &rider.latlngtme[2], 0) == pdTRUE)
        {
            //Serial.println(rider.latlngtme[4], 6);
            //Serial.println(rider.latlngtme[5], 6);
            Serial.println(rider.latlngtme[6]);
            Serial.println(rider.latlngtme[7]);
        }

        //get data from google maps API for the currrent roadname

        vTaskDelay(1000);
    }
}

void readGPS_t(void * queue)
{
    QueueHandle_t gpsReadings = (QueueHandle_t)queue;
    //This will be changed to regular serial once I can't help it anymore or I get better debugging tools
    SoftwareSerial SoftSerial(13, 12);
    SoftSerial.begin(9600);
    double latlngtme[3];
    latlngtme[0] = (12.93847);
    latlngtme[1] = 1.1111;

    while(1)
    {
        if (SoftSerial.available())                     // if date is coming from software serial port ==> data is coming from SoftSerial shield
        {
            while(SoftSerial.available())               // reading data into char array
            {
                if(gps.encode(SoftSerial.read()))
                {
                    latlngtme[0] = gps.location.lat();
                    latlngtme[1] = gps.location.lng();
                    latlngtme[2] = gps.time.minute();
                    latlngtme[3] = gps.time.second();

                    xQueueSend(gpsReadings, &latlngtme, 0);
                }
            } 
        }
        vTaskDelay(100);
    }
}

void loop()
{
    
}
