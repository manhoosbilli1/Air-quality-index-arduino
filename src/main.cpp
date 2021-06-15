#include "Arduino.h"
#include <MQUnifiedsensor.h>
#include "SdsDustSensor.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
SdsDustSensor sds(Serial3);
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

String ptr;
float CO = 0.02;
float NO2 = 1.44;
float NH3 = 0.02;
float PPM = 2.46;
float PM25 = 42.0;


String encode(float CO, float NO2, float NH3, float PPM, float PM25)
{
  ptr = '@';
  ptr += String(CO, 2);
  ptr += ',';
  ptr += String(NO2, 2);
  ptr += '!';
  ptr += String(NH3, 2);
  ptr += '#';
  ptr += String(PPM, 2);
  ptr += '&';
  ptr += String(PM25, 2);
  ptr += '$';
  return ptr;
}
void setup()
{
  Serial.begin(9600);
  sds.begin();           //serial3 = sds
  Serial2.begin(9600);   //serial2 = esp
  Serial2.println('s');
  Serial2.println("Hello World");
  Serial2.println("Arduino booting");
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
  Serial2.println("Arduino booted");
}

void loop()
{

  CO = analogRead(COPIN);
  NO2 = analogRead(NO2PIN);
  NH3 = analogRead(NH3PIN);
  const float max_volts = 5.0;
  const float max_analog_steps = 1023.0;
  CO = CO *  (max_volts / max_analog_steps);
  NO2 = NO2 * (max_volts / max_analog_steps);
  NH3 = NH3 * (max_volts / max_analog_steps);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("NH3:");
  lcd.print(NH3);
  lcd.setCursor(0,1);
  lcd.print("C0:");
  lcd.print(CO);
  lcd.print(" NO2:");
  lcd.print(NO2);
  delay(3000);
  lcd.clear();
  PmResult pm = sds.readPm();
  if (pm.isOk())
  {
    lcd.setCursor(0, 0);
    lcd.print("PM2.5: ");
    lcd.print(pm.pm25);
    PM25 = pm.pm25;
  }
  else
  {
    // notice that loop delay is set to 0.5s and some reads are not available
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("ERROR SDS");
    delay(500);
    lcd.clear();
  }
  lcd.setCursor(0, 1);
  MQ131.update();                    // Update data, the arduino will be read the voltage on the analog pin
  PPM = MQ131.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  lcd.print("PPM: ");
  lcd.print(PPM);
  String package = encode(CO,NO2,NH3,PPM,PM25); 
  Serial.println(package);
  Serial2.println(package);
  delay(3000);
  lcd.clear();
}

