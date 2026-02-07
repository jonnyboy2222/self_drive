#ifndef ALCOHOL_MANAGER_H
#define ALCOHOL_MANAGER_H

#include <Arduino.h>

class AlcoholManager
{
  public:
    AlcoholManager(uint8_t alcoholPin, uint8_t switchPin);
    void begin();
    bool isSwitchPressed();
    void startMeasuring();
    bool isMeasuring();
    bool update();  // 측정 결과가 true면 통과, false면 거절

  private:
    uint8_t alcoholSensorPin;
    uint8_t switchPin;

    static const int THRESHOLD = 200;
    static const unsigned long MEASURE_DURATION = 5000;
    static const unsigned long SAMPLE_INTERVAL = 100;

    bool measuring = false;
    unsigned long startTime = 0;
    unsigned long lastSampleTime = 0;
    unsigned long sum = 0;
    int count = 0;
};

#endif
