#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>

#define LCD_TYPE LiquidCrystal_PCF8574
LCD_TYPE lcd(0x27);
#define LCD_COLS (20)
#define LCD_ROWS (4)
char lcd_buf[LCD_COLS + 1] = "";
char lcd_bufdbg[LCD_COLS + 1] = "";

float curr_kt = 0.f, max_kt = 0.f, avg_kt = 0.f;
uint16_t analogCounts = 0;

// See https://www.adafruit.com/product/1733
#define NO_WIND_VOLTS (0.4f)
#define MAX_WIND_VOLTS (2.0f)
#define MAX_WIND_MPERSEC (32.4f)
#define VOLTS_PER_COUNT (2.048f / 1024.f)
#define SENSOR_PIN (A1)
#define MPERSEC_TO_KT (1.94384f)

void setup()
{
  // put your setup code here, to run once:
  int error;

  Serial.begin(115200);
  Serial.println("LCD...");

  // wait on Serial to be available on Leonardo
  while (!Serial)
    ;

  Serial.println("Probing for PCF8574 on address 0x27...");

  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  analogReference(EXTERNAL);

  if (error == 0)
  {
    Serial.println(": LCD found.");
    lcd.begin(LCD_COLS, LCD_ROWS); // initialize the lcd

    // lcd.createChar(1, dotOff);
    // lcd.createChar(2, dotOn);
  }
  else
  {
    Serial.println(": LCD not found.");
  } // if
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.print("Garden Anemometer");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Curr:  Max:  Avg:kt");
}

void loop()
{
  static uint8_t loop = 0;
  static uint8_t samp_counter = 0;
  static uint8_t doMax = 0;
  static float instant_v = 0., instantWind_v = 0.f, instant_kt;
  // put your main code here, to run repeatedly:
  static float expWeight = 1.f, expWeightSlow = 1.f;
  if (samp_counter == 0)
    analogCounts = 0;

  analogCounts += analogRead(SENSOR_PIN);
  samp_counter++;

  // There seems to be .1V of noise from anemometer, even when stopped
  // if (instant_kt > 5.0f && instant_v > 0.5f)
  if (samp_counter == 10)
  {

    instant_v = (analogCounts * VOLTS_PER_COUNT * .1f); //.1 for averaging the 10 readings
    instantWind_v = (instant_v - NO_WIND_VOLTS);

    instant_kt = instantWind_v * (MAX_WIND_MPERSEC / (MAX_WIND_VOLTS - NO_WIND_VOLTS));
    instant_kt *= MPERSEC_TO_KT;
    curr_kt = curr_kt + expWeight * (instant_kt - curr_kt);
    //    avg_kt = expWeight * curr_kt + (1.f - expWeight) * avg_kt;
    avg_kt = avg_kt + expWeightSlow * (instant_kt - avg_kt);
    /*
     * See https://en.wikipedia.org/wiki/Exponential_smoothing#Time_constant
     *
     * expWeight = 1 - e^(-DeltaT/tau)
     * or
     * tau = -DeltaT / ln(1 - expWeight)
     * where DeltaT = sampling interval
     * tau = time to return to true value after step function
     * or, traditionally,
     * expWeight = 0.9 !! :)
     */
    expWeight = 0.5f;
    expWeightSlow = .05f;

    if (loop > 200 && doMax == 0)
      doMax = 1;
    if (curr_kt > max_kt && doMax)
      max_kt = curr_kt;

    snprintf(lcd_buf, sizeof(lcd_buf),
             "% 3.1f % 3.1f % 3.1f ",
             curr_kt, max_kt, avg_kt);
    Serial.println(lcd_buf);
    lcd.setCursor(0, 1);
    lcd.print(lcd_buf);

    snprintf(lcd_bufdbg, sizeof(lcd_bufdbg),
             "% 4d % 4.3fv W% 6.3fv",
             analogCounts, instant_v, instantWind_v);
    lcd.setCursor(0, 2);
    lcd.print(lcd_bufdbg);

    samp_counter = 0;
  }

  loop++;
  delay(100); // 10Hz
}