#ifndef TEMP_MANAGER_H
#define TEMP_MANAGER_H

#include <Arduino.h>

class TempManager {
  public:
    TempManager(uint8_t tempPin);
    void begin();
    void update();
    float getCurrentTemperature() const;
    float getOvertemperature() const;

  private:
    uint8_t tempSensorPin;
    float temperature = 0;
    float overtemperature = 0;
    unsigned long previousMillis = 0;
    static const unsigned long INTERVAL = 1000; // 1초 주기
    static const float TEMP_THRESHOLD;
};

#endif

