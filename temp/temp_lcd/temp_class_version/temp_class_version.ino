#include <LiquidCrystal.h>

class TempManager
{
  private:
    const int TEMP_SENSEOR_PIN= A0;
    const float TEMP_THRESHOLD = 35.0;
    unsigned long pre_time = 0;
    const unsigned long INTERVAL = 500;
    float temperature = 0;
    LiquidCrystal LCD;

  public:
    TempManager(uint8_t rs, uint8_t en, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
    : LCD(rs, en, d4, d5, d6, d7) {}

    void begin() 
    {
      LCD.begin(16, 2);
      LCD.print("                ");
    }

    void measure_Temperature()
    {
        int adc_value = analogRead(TEMP_SENSEOR_PIN);
        float voltage = adc_value * (5.0 / 1024.0);
        temperature = voltage * 100;
        Serial.println(temperature);
    }

    void update_Lcd()
    {
      LCD.clear();
      LCD.setCursor(0, 0);
      if (temperature >= TEMP_THRESHOLD) 
      {
        LCD.print("Warning Temp!");
        LCD.setCursor(0, 1);
        LCD.print("Now: ");
        LCD.print(temperature);
        LCD.print(" C");
      } 
    }

    void update()
    {
      unsigned long now_time = millis();

      if (now_time - pre_time >= INTERVAL) 
      {
        pre_time = now_time;
        measure_Temperature();
        update_Lcd();
      }
    }
};

TempManager tempManager(12,11,5,4,3,2);
void setup()
{
  Serial.begin(9600);
  tempManager.begin();
}

void loop() 
{
  tempManager.update();
}
