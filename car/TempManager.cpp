#include "TempManager.h"

const float TempManager::TEMP_THRESHOLD = 37.0;

TempManager::TempManager(uint8_t tempPin)
  : tempSensorPin(tempPin)
{}

void TempManager::begin() {
  pinMode(tempSensorPin, INPUT);
}

void TempManager::update() {
  unsigned long currentMillis = millis();
  unsigned long curMillis2 = millis();
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    if (cnt == 10) {
      temperature = temp_sum / 10;
      cnt = 0;
      temp_sum = 0;
    }

    if (temperature > TEMP_THRESHOLD) {
      overtemperature = temperature;
    }
  }

  if (curMillis2 - prevMillis2 >= CHECK_INTV) {
    prevMillis2 = curMillis2;

    cnt += 1;

    int temp_val = analogRead(tempSensorPin);
    float voltage = temp_val * (5.0 / 1024.0);
    temp = voltage * 100;  // LM35 기준 섭씨 온도 환산

    temp_sum += temp;
  }
}

float TempManager::getCurrentTemperature() const {
  return temperature;
}

float TempManager::getOvertemperature() const {
  return overtemperature;
}
