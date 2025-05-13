#ifndef OBSTACLE_MANAGER_H
#define OBSTACLE_MANAGER_H

#include <Arduino.h>

class ObstacleManager 
{
  public:
    ObstacleManager(uint8_t trig, uint8_t echo, uint8_t buzzer);
    void begin();
    void update();
    void setReversing(bool state);
    float getAvgDistance();
    
  private:
    uint8_t trigPin;
    uint8_t echoPin;
    uint8_t buzzerPin;

    bool isbacking = false;

    static const int SAMPLESIZE = 10;
    float samples[SAMPLESIZE] = {0};
    int sampleindex = 0;
    float total_distance = 0;
    float avg_distance = 0;
    unsigned long pre_measuretime = 0;
    const unsigned long checktime_interval = 15;

    unsigned long prebeeptime = 0;
    bool buzzerstate = false;
    int beepfreq = 0;
    int beepinterval = 0;

    void avgDistance();
    void setBeepfreqByDistance();
    void controlBuzzer();
};

#endif
