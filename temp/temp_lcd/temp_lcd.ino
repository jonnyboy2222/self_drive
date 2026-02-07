#include <LiquidCrystal.h>

const int TEMP_PIN= A0;
const float THRESHOLD = 45.0;
unsigned long pre_time = 0;
const unsigned long INTERVAL = 3000;

float temperature = 0;

LiquidCrystal LCD(12, 11, 5, 4, 3, 2);

void setup() 
{
  Serial.begin(9600);
  LCD.begin(16, 2);
  LCD.print("Temp Monitor Ready");
}

////////////////////////온도 측정 및 환산 함수//////////////////////////
void measure_Temperature()
{
    int adc_value = analogRead(TEMP_PIN);
    float voltage = adc_value * (5.0 / 1024.0);
    temperature = voltage * 100;
    Serial.println(temperature);
}

////////////////////lcd에 경고 띄우기///////////////////////////////
void update_Lcd()
{
  LCD.clear();
  LCD.setCursor(0, 0);
  if (temperature >= THRESHOLD) 
  {
    LCD.print("Warning Temp!");
    LCD.setCursor(0, 1);
    LCD.print("Now: ");
    LCD.print(temperature);
    LCD.print(" C");
  } 
}

void loop() 
{
  unsigned long now_time = millis();

  if (now_time - pre_time >= INTERVAL) 
  {
    pre_time = now_time;
    measure_Temperature();
    update_Lcd();
  }
}