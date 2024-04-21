// #include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

#define ARDUINOJSON_USE_DOUBLE 1

// MPU6050 addresses
// #define MPU6050_ADDRESS_AD0_LOW 0x68 // address pin low (GND), default for InvenSense evaluation board

// Mutexes and Semaphores
SemaphoreHandle_t i2cMutex;

void collectData_t(void *queue);
void readGPS_t(void *queue);
void readMPU6050_t(void *queue);
void communicateWithServer_t(void * queue);
double getRelativeDegrees(double base);

// NOTE: a squid is a slang term for a "show off" on a street bike
class squid
{
public:
    double beginLat;  // Latitude/longitude at the start and end of special event
    double beginLng;
    double endLat;
    double endLng;

    double averageBikeYaw;     // WheelieAngle
    int stuntTime; // Time spent in stunt

    void clearData()
    {
        
        beginLat = 0;
        beginLng = 0;
        endLat = 0;
        endLng = 0;
        averageBikeYaw = 0;
        stuntTime = 0;
    }
    // When object is created the value 0 is used to show no value has been entered
    squid()
    {
        beginLat = 0;
        beginLng = 0;
        endLat = 0;
        endLng = 0;
        averageBikeYaw = 0;
        stuntTime = 0;
    }
};

class dataQueues
{
public:
    QueueHandle_t gpsReadings;
    QueueHandle_t accelReadings;
    QueueHandle_t riderData;
};

typedef struct AccelData
{
    double AccelX;
    double AccelY;
    double AccelZ;
    double GyroX;
    double GyroY;
    double GyroZ;
    double Temp;
    double AngleX;
    double AngleY;
    double AngleZ;

} AccelData;

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

void setup()
{
    Serial.begin(9600);
    Wire.begin(23, 22);

    i2cMutex = xSemaphoreCreateMutex();
    QueueHandle_t gpsReadings;
    QueueHandle_t accelReadings;
    QueueHandle_t riderData;
    riderData = xQueueCreate(1, sizeof(squid));
    gpsReadings = xQueueCreate(1, ((sizeof(double)) * 2));
    accelReadings = xQueueCreate(1, ((sizeof(double)) * 3));
    dataQueues *collectDataQueues_ptr = new dataQueues;
    collectDataQueues_ptr->gpsReadings = gpsReadings;
    collectDataQueues_ptr->accelReadings = accelReadings;
    collectDataQueues_ptr->riderData = riderData;
    xTaskCreate(readGPS_t, "Read the air530 grove GPS data", 8096, (void *)gpsReadings, 2, NULL);
    xTaskCreate(collectData_t, "Collect data from the reading tasks", 8096, (void *)collectDataQueues_ptr, 4, NULL);
    xTaskCreate(readMPU6050_t, "Read MPU6050 data", 2048, (void *)accelReadings, 3, NULL);
    xTaskCreate(communicateWithServer_t, "Handle communications with HTTP server", 16192, (void *)riderData, 5, NULL);
}

//Make degrees more human readable to a range of -180 to 180
double getRelativeDegrees(double degrees)
{
    if(degrees > 180)
    {
        degrees = degrees - 360;
    }
    return degrees;
}

void communicateWithServer_t(void * queue)
{
    QueueHandle_t riderDataQueue = (QueueHandle_t)queue;
    unsigned long lastTime = 0;
    unsigned long timerDelay = 5000;
    const char * ssid = "AndroidAP235F";
    const char* password = "umbl7415";
    String serverName = "http://stunt-monitor.lithium720.pw/receive.php";
    //Connect to phones hotspot
    WiFi.begin(ssid, password);
    Serial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED)    {
        vTaskDelay(500);
        Serial.print(".");
    }
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    squid rider;
    while(1)
    {
        if(xQueueReceive(riderDataQueue, &rider, 0) == pdTRUE)
        {
            JsonDocument doc;
            double lat_begin = rider.beginLat;
            double lng_begin = rider.beginLng;
            double lat_end = rider.endLat;
            double lng_end = rider.endLng;

            Serial.println(rider.beginLat);
            Serial.println(rider.beginLng);
            Serial.println(rider.endLat);
            Serial.println(rider.endLng);
            Serial.println(rider.stuntTime);
            doc["Time_in_stunt"] = rider.stuntTime;
            doc["Rider_lat_begin"] = lat_begin;
            doc["Rider_lng_begin"] = lng_begin;
            doc["Rider_lat_end"] = lat_end;
            doc["Rider_lng_end"] = lng_end;
            doc["Avg_Wheelie_angle"] = rider.averageBikeYaw;

            String jsonDoc;
            serializeJson(doc, jsonDoc);
            Serial.println(jsonDoc);
            if ((millis() - lastTime) > timerDelay)
                {
                if(WiFi.status() == WL_CONNECTED)
                {
                    WiFiClient client;
                    HTTPClient http;

                    http.begin(client, serverName);
                    int httpResponseCode = http.POST(jsonDoc);

                    http.addHeader("Content-Type", "application/json");

                    Serial.print("HTTP Response code: ");
                    Serial.println(httpResponseCode);
                    String payload = http.getString();
                        Serial.println(payload);

                    http.end();
                }
                else
                {
                    Serial.println("WiFi Disconnected");
                }
            }
        }
        vTaskDelay(500);
    }
    
}

void collectData_t(void *queue)
{
    dataQueues *queueStruct = (dataQueues *)queue;
    QueueHandle_t gpsReadings = queueStruct->gpsReadings;
    QueueHandle_t accelReadings = queueStruct->accelReadings;
    QueueHandle_t riderData = queueStruct->riderData;
    

    AccelData *accelData_ptr = nullptr;
    squid rider;
    //The angle of which the device is mounted on the y-axis
    double basePosition = 0;
    double latLng[2] = {0, 0};
    bool baseSet = false;
    //Get the base position
    while(baseSet == false)
    {
        if (xQueueReceive(accelReadings, &accelData_ptr, 500) == pdTRUE)
        {
            basePosition = getRelativeDegrees(accelData_ptr->AngleY);
            baseSet = true;
        }
    }
    Serial.printf("Base position is:");
    Serial.println(basePosition);
    unsigned long startTime;
    unsigned long endTime;
    while (1)
    {
        // Looking for odd behaviour from the accelerometer(s) until something is found
        // Possibilities: Wheelie, Stoppie, Crash

        // get lat/long data and time for start

        double angleYData = 0;
        int loop = 0;
        double averageYAngle = 0;
        // Repeatedly get accelerometer information here and move on when ending circumstances met
        //Wheelie or stoppie must be ~38 degrees in either direction
        while((angleYData < 35 + basePosition) && (angleYData > -35 + basePosition))
        {
            if (xQueueReceive(accelReadings, &accelData_ptr, 500) == pdTRUE)
            {
                angleYData = getRelativeDegrees(accelData_ptr->AngleY);
                averageYAngle = angleYData;
                delete accelData_ptr; // free the memory
                accelData_ptr = nullptr;
            }
        }
        loop++;
        //A stunt has begun
        startTime = millis();
        if (xQueueReceive(gpsReadings, &latLng, 500) == pdTRUE)
            {
                rider.beginLat = latLng[0];
                rider.beginLng = latLng[1];
                Serial.print("Rider stunt begins at: ");
                Serial.print(rider.beginLat, 6);
                Serial.print(",");
                Serial.println(rider.beginLng, 6);
            }
        else
            {
                rider.beginLat = 0;
                rider.beginLng = 0;
            }
        // Timeout, Wheelie end, Stoppie end, crash movement stopped etc.
        // NOTE: wheelies have potential to go for minutes at a time and thus the timeout cannot be too soon.
        while((angleYData >= 35 + basePosition) || (angleYData <= -35 + basePosition))
        {
            if (xQueueReceive(accelReadings, &accelData_ptr, 500) == pdTRUE)
            {
                angleYData = getRelativeDegrees(accelData_ptr->AngleY);
                loop++;
                averageYAngle = angleYData + averageYAngle;
                delete accelData_ptr; // free the memory
                accelData_ptr = nullptr;
                Serial.println(angleYData);
            }
        }
        averageYAngle = averageYAngle/loop;

        // get lat/long data and time for end of event
        endTime = millis();
        rider.stuntTime = endTime - startTime;
        rider.averageBikeYaw = averageYAngle;
        if (xQueueReceive(gpsReadings, &latLng, 500) == pdTRUE)
        {
            rider.endLat = latLng[0];
            rider.endLng = latLng[1];
            Serial.print("Rider stunt ends at: ");
            Serial.print(rider.endLat, 6);
            Serial.print(",");
            Serial.println(rider.endLng, 6);
        }
        else
        {
            rider.endLat = 0;
            rider.endLng = 0;
        }
        Serial.print("Time elapsed: ");
        Serial.println(rider.stuntTime);
        Serial.print("Average wheelie angle: ");
        Serial.println(averageYAngle);
        xQueueSend(riderData, &rider ,500);
        // Send the data directly to server until we get storage figured out
        
        // Send the data to the microSD where another task will upload data from the microSD to the web server

        vTaskDelay(1000);
    }
}

// For testing purposes the AirGrove is set up in this code to use the second grove port on the vanduino
// The grove connection is labeled GP13/GP12
void readGPS_t(void *queue)
{
    QueueHandle_t gpsReadings = (QueueHandle_t)queue;
    // This will be changed to regular serial once I can't help it anymore or I get better debugging tools
    SoftwareSerial SoftSerial(13, 12);
    SoftSerial.begin(9600);
    double latlng[1];

    while (1)
    {
        if (SoftSerial.available()) // if date is coming from software serial port ==> data is coming from SoftSerial shield
        {
            while (SoftSerial.available()) // reading data into char array
            {
                if (gps.encode(SoftSerial.read()))
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

// read the mpu6050 data and send to a queue
void readMPU6050_t(void *queue)
{
    vTaskDelay(5000);
    // initialize device
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
        if (!mpu.begin())
        {
            Serial.println("Failed to initialize MPU6050...");
            while (1)
            {
                vTaskDelay(10);
            }
        }

        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        Serial.print("MPU6050 initialized for range ");
        switch (mpu.getAccelerometerRange())
        {
        case MPU6050_RANGE_2_G:
            Serial.println("+-2G");
            break;
        case MPU6050_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case MPU6050_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case MPU6050_RANGE_16_G:
            Serial.println("+-16G");
            break;
        default:
            break;
        }

        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        Serial.print("MPU6050 initialized for range ");
        switch (mpu.getGyroRange())
        {
        case MPU6050_RANGE_250_DEG:
            Serial.println("+- 250 deg/s");
            break;
        case MPU6050_RANGE_500_DEG:
            Serial.println("+- 500 deg/s");
            break;
        case MPU6050_RANGE_1000_DEG:
            Serial.println("+- 1000 deg/s");
            break;
        case MPU6050_RANGE_2000_DEG:
            Serial.println("+- 2000 deg/s");
            break;
        default:
            break;
        }

        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.print("Filter bandwidth set to: ");
        switch (mpu.getFilterBandwidth())
        {
        case MPU6050_BAND_260_HZ:
            Serial.println("260 Hz");
            break;
        case MPU6050_BAND_184_HZ:
            Serial.println("184 Hz");
            break;
        case MPU6050_BAND_94_HZ:
            Serial.println("94 Hz");
            break;
        case MPU6050_BAND_44_HZ:
            Serial.println("44 Hz");
            break;
        case MPU6050_BAND_21_HZ:
            Serial.println("21 Hz");
            break;
        case MPU6050_BAND_10_HZ:
            Serial.println("10 Hz");
            break;
        case MPU6050_BAND_5_HZ:
            Serial.println("5 Hz");
            break;
        }

        xSemaphoreGive(i2cMutex);
    }

    QueueHandle_t accelQueue = (QueueHandle_t)queue;

    double xAngle = 0;
    double yAngle = 0;
    double zAngle = 0;

    Serial.println("MPU6050 is ready to go!");

    vTaskDelay(100);

    AccelData *accelData_ptr = nullptr;
    while (1)
    {
        /* Get new sensor events with the readings */
        sensors_event_t a, g, temp;
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
        {
            mpu.getEvent(&a, &g, &temp);
            xSemaphoreGive(i2cMutex);
        }

        // create a struct pointer to send to the queue
        accelData_ptr = new AccelData;

        accelData_ptr->AccelX = a.acceleration.x;
        accelData_ptr->AccelY = a.acceleration.y;
        accelData_ptr->AccelZ = a.acceleration.z;
        accelData_ptr->GyroX = g.gyro.x;
        accelData_ptr->GyroY = g.gyro.y;
        accelData_ptr->GyroZ = g.gyro.z;
        accelData_ptr->Temp = temp.temperature;

        // ====== Untested code below ======

        // calculate the angle with min and max values of the accelerometer
        int xAngle = map(accelData_ptr->AccelX, -500, 500, -90, 90);
        int yAngle = map(accelData_ptr->AccelY, -500, 500, -90, 90);
        int zAngle = map(accelData_ptr->AccelZ, -500, 500, -90, 90);

        xAngle = RAD_TO_DEG * (atan2(-a.acceleration.y, -a.acceleration.z) + PI);
        yAngle = RAD_TO_DEG * (atan2(-a.acceleration.x, -a.acceleration.z) + PI);
        zAngle = RAD_TO_DEG * (atan2(-a.acceleration.x, -a.acceleration.y) + PI);

        accelData_ptr->AngleX = xAngle;
        accelData_ptr->AngleY = yAngle;
        accelData_ptr->AngleZ = zAngle;

        // ====== Untested code above ======

        if (uxQueueSpacesAvailable(accelQueue) == 0)
        {
            // free the memory for all the items in the queue
            AccelData *temp_ptr = nullptr;
            while (xQueueReceive(accelQueue, &temp_ptr, 0) == pdTRUE)
            {
                delete temp_ptr;
            }
            xQueueReset(accelQueue);
        }

        xQueueSend(accelQueue, &accelData_ptr, 0);

        // Make sure to delete the memory in the consumer task

        vTaskDelay(100);
    }
}

void loop()
{
}
