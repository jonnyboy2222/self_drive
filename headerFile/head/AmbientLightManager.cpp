#include "AmbientLightManager.h"
#include <Arduino.h>

#define LIGHT_SENSOR_PIN A2
#define HEADLIGHT_LED_PIN 12

const unsigned long AmbientLightManager::light_measuretime_interval = 50;

void AmbientLightManager::begin() {
  pinMode(HEADLIGHT_LED_PIN, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  digitalWrite(HEADLIGHT_LED_PIN, LOW);

  light_sample_index = 0;
  total_lights = 0;
  avg_light = 0;
  now_light_measuretime = 0;
  pre_light_measuretime = 0;
  for (int i = 0; i < LIGHTSAMPLESIZE; i++) {
    light_samples[i] = 0;
  }
}

void AmbientLightManager::update() {
  now_light_measuretime = millis();
  if (now_light_measuretime - pre_light_measuretime >= light_measuretime_interval) {
    int light = analogRead(LIGHT_SENSOR_PIN);
    int map_light = map(constrain(light, 50, 1020), 50, 1020, 255, 0);

    total_lights -= light_samples[light_sample_index];
    light_samples[light_sample_index] = map_light;
    total_lights += map_light;

    light_sample_index = (light_sample_index + 1) % LIGHTSAMPLESIZE;

    if (light_sample_index == 0) {
      avg_light = total_lights / LIGHTSAMPLESIZE;
      Serial.print("avg light: ");
      Serial.println(avg_light);
    }

    if (avg_light > LIGHT_THRESHOLD) {
      digitalWrite(HEADLIGHT_LED_PIN, HIGH);
    } else {
      digitalWrite(HEADLIGHT_LED_PIN, LOW);
    }

    pre_light_measuretime = now_light_measuretime;
  }
}
