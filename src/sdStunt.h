// sdStunt.h
#ifndef SD_STUNT_H
#define SD_STUNT_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class SDStunt {
public:
    // Constructor
    SDStunt(uint8_t csPin);

    // Initializes the SD card
    bool begin();

    // Writes data to a specified file
    bool writeDataSD(const char* filename, const char* data, bool overwrite = false);

    // Reads data from a specified file
    bool readDataSD(const char* filename, String& outData);

    // Renames any existing .json to .bak, this is called in setup
    void backupJsonFiles();


private:
    uint8_t _csPin;  // Chip Select pin for the SD card
};

#endif // SD_STUNT_H
