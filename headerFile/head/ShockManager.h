#ifndef SHOCKMANAGER_H
#define SHOCKMANAGER_H

#include <Arduino.h>

class ShockManager 
{
  private:
    uint8_t pin;
    unsigned long lastWindowTime;
    static const unsigned long windowDuration = 200; // 0.2초
    int buffer[5];
    int bufferIndex;
    int countInWindow;
    bool lastSensorState;
    float latest_average_shock;

  public:
    ShockManager(uint8_t sensorPin);
    void begin();
    void update();
    float getLatestAverageShock();
};

#endif
