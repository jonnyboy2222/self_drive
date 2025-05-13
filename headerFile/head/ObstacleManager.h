// === ObstacleManager.h ===
#ifndef OBSTACLE_MANAGER_H
#define OBSTACLE_MANAGER_H

class ObstacleManager 
{
  public:
    void begin();
    void update();
    void setReversing(bool state);

  private:
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