// sdStunt.cpp

#include "sdStunt.h"

SDStunt::SDStunt(uint8_t csPin) : _csPin(csPin) {}

// Initialize SD card
bool SDStunt::begin() {
    if (!SD.begin(_csPin, SPI, 4000000)) {
        Serial.print("SD card initialization failed on CS pin: ");
        Serial.println(_csPin);
        return false;
    }
    Serial.println("SD Card initialized.");
    return true;
}


// Write data to SD card
bool SDStunt::writeDataSD(const char* filename, const char* data, bool overwrite) {
    String filePath = "/";
    filePath += filename;

    const char* mode = overwrite ? "w" : "a";  // "w" is write (overwrite), "a" is append
    File file = SD.open(filePath.c_str(), mode);
    if (!file) {
        Serial.print("Failed to open file: ");
        Serial.println(filePath);
        return false;
    }

    if (file.println(data)) {
        file.close();
        return true;
    } else {
        file.close();
        Serial.println("Failed to write data");
        return false;
    }
}

bool SDStunt::readDataSD(const char* filename, String& outData) {
    File file = SD.open(filename);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
    }

    outData = ""; // Clear previous data
    while (file.available()) {
        outData += (char)file.read();
    }
    file.close();
    return true;

}

void SDStunt::backupJsonFiles() {
    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open root directory.");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        String fileName = file.name();
        // Ensure it's a file, not a directory, and ends with '.json'
        if (!file.isDirectory() && fileName.endsWith(".json")) {
            // Correct the length subtraction to remove ".json"
            String newFileName = fileName.substring(0, fileName.length() - 5) + ".bak";
            if (SD.exists(newFileName)) {
                SD.remove(newFileName); // Remove the existing backup if it exists
            }
            if (SD.rename(fileName.c_str(), newFileName.c_str())) {
                Serial.print("Successfully backed up: ");
                Serial.println(fileName);
            } else {
                Serial.print("Failed to backup file: ");
                Serial.println(fileName);
            }
        }
        file = root.openNextFile();
    }
    root.close(); // Close the root directory to free up resources
}




