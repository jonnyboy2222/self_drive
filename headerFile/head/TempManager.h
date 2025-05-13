#ifndef TEMP_MANAGER_H
#define TEMP_MANAGER_H

class TempManager {
  public:
    void begin();
    void update();
    float getCurrentTemperature() const;
    float getOvertemperature() const;

  private:
    float temperature = 0;
    float overtemperature = 0;
    unsigned long previousMillis = 0;
    static const unsigned long INTERVAL = 1000; // 1초 주기
    static const float TEMP_THRESHOLD;
};

#endif
