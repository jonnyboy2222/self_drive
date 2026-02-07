#include "TempManager.h"
#include <Arduino.h>

#define TEMP_SENSOR_PIN A0
const float TempManager::TEMP_THRESHOLD = 37.0;

void TempManager::begin() {
  pinMode(TEMP_SENSOR_PIN, INPUT);
}

void TempManager::update() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    int adc_value = analogRead(TEMP_SENSOR_PIN);
    float voltage = adc_value * (5.0 / 1024.0);
    temperature = voltage * 100;  // LM35 기준 섭씨 온도 환산

    if (temperature > TEMP_THRESHOLD) {
      overtemperature = temperature;
    }

    Serial.print("Temperature: ");
    Serial.println(temperature);
  }
}

float TempManager::getCurrentTemperature() const {
  return temperature;
}

float TempManager::getOvertemperature() const {
  return overtemperature;
}
