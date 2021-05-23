#include "Arduino.h"
#include <MQUnifiedsensor.h>
#include "SdsDustSensor.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
int rxPin = 0;
int txPin = 1;
SdsDustSensor sds(rxPin, txPin);
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin A3                //Analog input 0 of your arduino
#define type "MQ-131"         //MQ131
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ131CleanAir 15 //RS / R0 = 15 ppm
MQUnifiedsensor MQ131(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

#define COPIN A0
#define NH3PIN A1
#define NO2PIN A2
void setup()
{
  Serial.begin(9600);
  sds.begin();
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("AIR QUALITY IDX");
  delay(1000);
  Serial.println(sds.queryFirmwareVersion().toString());       // prints firmware version
  Serial.println(sds.setActiveReportingMode().toString());     // ensures sensor is in 'active' reporting mode
  Serial.println(sds.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended

  pinMode(COPIN, INPUT);
  pinMode(NH3PIN, INPUT);
  pinMode(NO2PIN, INPUT);
  MQ131.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ131.setA(23.943);
  MQ131.setB(-1.11); // Configurate the ecuation values to get O3 concentration
  MQ131.init();
  float calcR0 = 0;
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("MQT initial setup");
  delay(1000);
  lcd.clear();
  for (int i = 1; i <= 10; i++)
  {
    MQ131.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ131.calibrate(RatioMQ131CleanAir);
    lcd.print(".");
  }
  MQ131.setR0(calcR0 / 10);
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.println("  done!.");
  delay(500);
  if (isinf(calcR0))
  {
    lcd.clear();
    lcd.print("ERROR 333"); //code: 333 -> Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply
    while (1)
      ;
  }
  if (calcR0 == 0)
  {
    lcd.println("ERROR 335"); //code 335 -> Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply
    while (1)
      ;
  }
}

void loop()
{

  float CO = analogRead(COPIN);
  float NO2 = analogRead(NO2PIN);
  float NH3 = analogRead(NH3PIN);
  const float max_volts = 5.0;
  const float max_analog_steps = 1023.0;
  CO = CO *  (max_volts / max_analog_steps);
  NO2 = NO2 * (max_volts / max_analog_steps);
  NH3 = NH3 * (max_volts / max_analog_steps);
  PmResult pm = sds.readPm();
  if (pm.isOk())
  {
    lcd.setCursor(0, 0);
    lcd.print("PM2.5= ");
    lcd.print(pm.pm25);
  }
  else
  {
    // notice that loop delay is set to 0.5s and some reads are not available
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("ERROR SDS");
    delay(500);
  }
  lcd.setCursor(0, 1);
  MQ131.update();                    // Update data, the arduino will be read the voltage on the analog pin
  float result = MQ131.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  lcd.print("PPM: ");
  lcd.print(result);
  delay(500);
}
