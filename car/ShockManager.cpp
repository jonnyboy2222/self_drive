// #include "ShockManager.h"
// #include "Arduino.h"
// ShockManager::ShockManager(uint8_t sensorPin)
//   : pin(sensorPin),
//     lastWindowTime(0),
//     bufferIndex(0),
//     countInWindow(0),
//     lastSensorState(LOW),
//     latest_average_shock(0.0)
// {
//   for (int i = 0; i < 5; i++) {
//     buffer[i] = 0;
//   }
// }
// void ShockManager::begin()
// {
//   pinMode(pin, INPUT);
//   lastWindowTime = millis();
// }
// void ShockManager::update()
// {
//   bool currentSensorState = digitalRead(pin);
//   // 상승 에지 감지 (LOW → HIGH)
//   if (currentSensorState == HIGH && lastSensorState == LOW)
//   {
//     countInWindow++;
//   }
//   lastSensorState = currentSensorState;
//   unsigned long currentTime = millis();
//   if (currentTime - lastWindowTime >= windowDuration)
//   {
//     buffer[bufferIndex] = countInWindow;
//     bufferIndex = (bufferIndex + 1) % 5;
//     if (bufferIndex == 0)
//     {
//       int sum = 0;
//       for (int i = 0; i < 5; i++)
//       {
//         sum += buffer[i];
//       }
//       latest_average_shock = sum / 5.0;
//     }
//     countInWindow = 0;
//     lastWindowTime = currentTime;
//   }
// }
// float ShockManager::getLatestAverageShock()
// {
//   return latest_average_shock;
// }
#include "ShockManager.h"
#include "Arduino.h"
ShockManager::ShockManager(uint8_t sensorPin)
  : pin(sensorPin),
    lastWindowTime(0),
    bufferIndex(0),
    countInWindow(0),
    lastSensorState(LOW),
    latest_average_shock(0.0)
{
  for (int i = 0; i < 5; i++) {
    buffer[i] = 0;
  }
}
void ShockManager::begin()
{
  pinMode(pin, INPUT);
  lastWindowTime = millis();
}
void ShockManager::update()
{
  bool currentSensorState = digitalRead(pin);
  // 상승 에지 감지 (LOW → HIGH)
  // if (currentSensorState == HIGH && lastSensorState == LOW) or (currentSensorState == LOW && lastSensorState == HIGH)
  if (currentSensorState != lastSensorState)
  {
    countInWindow++;
  }
  lastSensorState = currentSensorState;
  unsigned long currentTime = millis();
  if (currentTime - lastWindowTime >= windowDuration)
  {
    buffer[bufferIndex] = countInWindow;
    bufferIndex = (bufferIndex + 1) % 5;
    if (bufferIndex == 0)
    {
      int sum = 0;
      for (int i = 0; i < 5; i++)
      {
        sum += buffer[i];
      }
      // latest_average_shock = sum / 5.0;
      one_sec_sum = sum;
    }
    countInWindow = 0;
    lastWindowTime = currentTime;
  }
}
float ShockManager::getLatestAverageShock()
{
  // return latest_average_shock;
  return one_sec_sum;
}
unsigned long ShockManager::getTimestamp()
{
  return millis(); // 현재 타임스탬프 리턴 (밀리초 단위)
}
int ShockManager::getCountInWindow()
{
  return countInWindow; // 1초 동안 감지된 충격 횟수 반환
}