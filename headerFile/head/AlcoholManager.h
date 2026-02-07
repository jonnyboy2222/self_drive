#ifndef ALCOHOLMANAGER_H
#define ALCOHOLMANAGER_H

#include <Arduino.h>

// 핀 번호는 전역 define 또는 extern 선언 가능
#define ALCOHOL_SENSOR_PIN A0
#define SWITCH_PIN 2

class AlcoholManager
{
  private:
    const int THRESHOLD = 200;
    const unsigned long MEASUREDURATION = 5000;
    const unsigned long SAMPLEINTERVAL = 100;

    bool measuring = false;
    unsigned long startTime = 0;
    unsigned long lastSampleTime = 0;
    unsigned long sum = 0;
    int count = 0;

  public:
    void begin();
    bool isSwitchPressed();
    void startMeasuring();
    bool isMeasuring();
    bool update();
};

#endif