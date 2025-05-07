// === AmbientLightManager 클래스 ===
class AmbientLightManager 
{
  private:
    const int LIGHT_SENSOR_PIN = A0;    // 조도센서 핀
    const int LEFT_HEADLIGHT_LED_PIN = 12;       
    const int RIGHT_HEADLIGHT_LED_PIN = 6;

    const static int LIGHTSAMPLESIZE = 10;
    int light_samples[LIGHTSAMPLESIZE] = {0};

    int light_sample_index = 0;
    int total_lights = 0;
    float avg_light = 0;
    unsigned long now_light_measuretime = 0;
    unsigned long pre_light_measuretime = 0;
    unsigned long light_measuretime_interval = 50;

    const int LIGHT_THRESHOLD = 21;

  public:
    void begin() 
    {
      pinMode(LEFT_HEADLIGHT_LED_PIN, OUTPUT);
      pinMode(RIGHT_HEADLIGHT_LED_PIN, OUTPUT);
      pinMode(LIGHT_SENSOR_PIN, INPUT);
      digitalWrite(LEFT_HEADLIGHT_LED_PIN, LOW);
      digitalWrite(RIGHT_HEADLIGHT_LED_PIN, LOW);
    }

    void result_avg_Light() 
    {
      int light = analogRead(LIGHT_SENSOR_PIN);
      int map_light = map(constrain(light, 50, 1020), 50, 1020, 255, 0);

      total_lights -= light_samples[light_sample_index];
      light_samples[light_sample_index] = map_light;
      total_lights += light_samples[light_sample_index];

      light_sample_index = (light_sample_index + 1) % LIGHTSAMPLESIZE;

      if (light_sample_index == 0) 
      {
        avg_light = total_lights / LIGHTSAMPLESIZE;
      }
    }

    void led_state_byThreshold() 
    {
      if (avg_light > LIGHT_THRESHOLD) 
      {
        digitalWrite(LEFT_HEADLIGHT_LED_PIN, HIGH);
        digitalWrite(RIGHT_HEADLIGHT_LED_PIN, HIGH);
      }
      else 
      {
        digitalWrite(LEFT_HEADLIGHT_LED_PIN, LOW);
        digitalWrite(RIGHT_HEADLIGHT_LED_PIN, LOW);
      }
    }

    void update() 
    {
      now_light_measuretime = millis();
      if (now_light_measuretime - pre_light_measuretime >= light_measuretime_interval) 
      {
        result_avg_Light();
        led_state_byThreshold();
        pre_light_measuretime = now_light_measuretime;
        Serial.println(avg_light);
      }
    }
};

AmbientLightManager ambientLightManager;

void setup() 
{
  Serial.begin(9600);
  ambientLightManager.begin();
}

void loop() 
{
  ambientLightManager.update();
}