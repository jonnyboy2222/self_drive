// #ifndef SHOCKMANAGER_H
// #define SHOCKMANAGER_H
// #include <Arduino.h>
// class ShockManager
// {
//   private:
//     uint8_t pin;
//     unsigned long lastWindowTime;
//     static const unsigned long windowDuration = 200; // 0.2초
//     int buffer[5];
//     int bufferIndex;
//     int countInWindow;
//     bool lastSensorState;
//     float latest_average_shock;
//   public:
//     ShockManager(uint8_t sensorPin);
//     void begin();
//     void update();
//     float getLatestAverageShock();
// };
// #endif
#ifndef SHOCKMANAGER_H
#define SHOCKMANAGER_H
#include <Arduino.h>
class ShockManager
{
  private:
    uint8_t pin;
    unsigned long lastWindowTime;
    static const unsigned long windowDuration = 1000; // 1초 (1000ms)
    int buffer[5];
    int bufferIndex;
    int countInWindow;
    bool lastSensorState;
    float latest_average_shock;
    int one_sec_sum;
  public:
    ShockManager(uint8_t sensorPin);
    void begin();
    void update();
    float getLatestAverageShock();  // 마지막 평균 충격 값 반환
    unsigned long getTimestamp();  // 타임스탬프 리턴
    int getCountInWindow();        // 1초 동안 감지된 충격 횟수 반환
};
#endif