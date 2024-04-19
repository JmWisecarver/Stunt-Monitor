// #include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define HDCCONFIGADDRESS 0x02
#define HDC1080ADDRESS 0x40
#define HUMIDITY 0x01
#define TEMPERATURE 0x00

// MPU6050 addresses
#define MPU6050_ADDRESS_AD0_LOW 0x68 // address pin low (GND), default for InvenSense evaluation board

// Mutexes and Semaphores
SemaphoreHandle_t i2cMutex;

void collectData_t(void *queue);
void readGPS_t(void *queue);
void readHDC1080_temperature_t(void *queue);
void readMPU6050(void *queue);

// NOTE: a squid is a slang term for a "show off" on a street bike
class squid
{
public:
    double latlng[4];        // Latitude/longitude at the start and end of special event
    double bikeYaw[100];     // WheelieAngle
    double bikeRoll[100];    // WheelieRoll
    unsigned long stuntTime; // Time spent in stunt

    void clearData()
    {
        for (int i = 0; i < 4; i++)
            latlng[i] = 0;
        for (int i = 0; i < 100; i++)
        {
            bikeYaw[i] = 0;
            bikeRoll[i] = 0;
        }
    }
    // When object is created the value 0 is used to show no value has been entered
    squid()
    {
        for (int i = 0; i < 4; i++)
            latlng[i] = 0;
        for (int i = 0; i < 100; i++)
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

typedef struct AccelData
{
    double AccelX;
    double AccelY;
    double AccelZ;
    double GyroX;
    double GyroY;
    double GyroZ;
    double Temp;

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
    gpsReadings = xQueueCreate(1, ((sizeof(double)) * 2));
    accelReadings = xQueueCreate(1, ((sizeof(double)) * 3));
    dataQueues *collectDataQueues_ptr = new dataQueues;
    collectDataQueues_ptr->gpsReadings = gpsReadings;
    collectDataQueues_ptr->accelReadings = accelReadings;
    xTaskCreate(readGPS_t, "Read the air530 grove GPS data", 8096, (void *)gpsReadings, 2, NULL);
    xTaskCreate(collectData_t, "Collect data from the reading tasks", 8096, (void *)collectDataQueues_ptr, 1, NULL);
    xTaskCreate(readHDC1080_temperature_t, "Read temperature from HDC1080", 2048, (void *)gpsReadings, 3, NULL);
    xTaskCreate(readMPU6050, "Read MPU6050 data", 2048, (void *)accelReadings, 3, NULL);
}

// This is an example function of reading from an I2C device while using the GPS
// using the Wire.h library
void readHDC1080_temperature_t(void *queue)
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

    while (1)
    {
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
        {
            Wire.beginTransmission(HDC1080ADDRESS);
            Wire.write(TEMPERATURE);
            Wire.endTransmission();
            vTaskDelay(5);
            Wire.requestFrom(HDC1080ADDRESS, 2);
            while (Wire.available())
            {
                binaryTemp[0] = Wire.read(); // read from the starting register
                binaryTemp[1] = Wire.read();
            }
            xSemaphoreGive(i2cMutex);
        }

        tempCelcius = binaryTemp[0] << 8 | binaryTemp[1];
        // This bit shift is used to help with data accuracy. It will be shifted back after computations
        tempCelcius = tempCelcius << 8;
        // Formula in data sheet to convert the binary to a temp in celcius
        // Adjustments are likely possible and maybe necessary to get more accurate farenheit readings
        tempCelcius = tempCelcius * 165;
        tempCelcius = tempCelcius / 65536;
        // Shift back
        tempCelcius = tempCelcius >> 8;
        tempCelcius = (tempCelcius - 40);

        Serial.print("Temperature is: ");
        Serial.print(tempCelcius);
        Serial.println(" degrees celcius.");
        vTaskDelay(1000);
    }
}

void collectData_t(void *queue)
{
    dataQueues *queueStruct = (dataQueues *)queue;
    QueueHandle_t gpsReadings = queueStruct->gpsReadings;
    QueueHandle_t accelReadings = queueStruct->accelReadings;

    AccelData *accelData_ptr = nullptr;
    squid rider;
    // float test = 0;
    unsigned long startTime;
    unsigned long endTime;
    while (1)
    {
        // Looking for odd behaviour from the accelerometer(s) until something is found
        // Possibilities: Wheelie, Stoppie, Crash

        // get lat/long data and time for start
        startTime = millis();
        if (xQueueReceive(gpsReadings, &rider.latlng[0], 500) == pdTRUE)
        {
            Serial.print("Rider stunt begins at: ");
            Serial.print(rider.latlng[0], 6);
            Serial.print(",");
            Serial.println(rider.latlng[1], 6);
        }

        // Repeatedly get accelerometer information here and move on when ending circumstances met
        if (xQueueReceive(accelReadings, &accelData_ptr, 100) == pdTRUE)
        {
            // added temporary print statements to see if the data is being read correctly
            Serial.print("AccelX: ");
            Serial.println(accelData_ptr->AccelX);
            Serial.print(" AccelY: ");
            Serial.println(accelData_ptr->AccelY);
            Serial.print(" AccelZ: ");
            Serial.println(accelData_ptr->AccelZ);
            Serial.print(" GyroX: ");
            Serial.println(accelData_ptr->GyroX);
            Serial.print(" GyroY: ");
            Serial.println(accelData_ptr->GyroY);
            Serial.print(" GyroZ: ");
            Serial.println(accelData_ptr->GyroZ);
            Serial.print(" Temp: ");
            Serial.println(accelData_ptr->Temp);

            delete accelData_ptr; // free the memory
            accelData_ptr = nullptr;
        }
        // Timeout, Wheelie end, Stoppie end, crash movement stopped etc.
        // NOTE: wheelies have potential to go for minutes at a time and thus the timeout cannot be too soon.

        // get lat/long data and time for end of event
        endTime = millis();
        rider.stuntTime = endTime - startTime;
        if (xQueueReceive(gpsReadings, &rider.latlng[2], 500) == pdTRUE)
        {
            Serial.print("Rider stunt ends at: ");
            Serial.print(rider.latlng[2], 6);
            Serial.print(",");
            Serial.println(rider.latlng[3], 6);
        }
        Serial.print("Time elapsed: ");
        Serial.println(rider.stuntTime);
        // Send the data to the microSD where another task will upload data from the microSD to the web server

        vTaskDelay(250);
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
void readMPU6050(void *queue)
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

        if (uxQueueSpacesAvailable(accelQueue) == 0)
        {
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
