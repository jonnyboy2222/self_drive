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
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    int adc_value = analogRead(tempSensorPin);
    float voltage = adc_value * (5.0 / 1024.0);
    temperature = voltage * 100;  // LM35 기준 섭씨 온도 환산

    if (temperature > TEMP_THRESHOLD) {
      overtemperature = temperature;
    }
  }
}

float TempManager::getCurrentTemperature() const {
  return temperature;
}

float TempManager::getOvertemperature() const {
  return overtemperature;
}
