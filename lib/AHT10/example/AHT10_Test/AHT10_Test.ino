#include <Wire.h>
#include <Thinary_AHT10.h>

AHT10Class AHT10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  Wire.begin();
  if(AHT10.begin(eAHT10Address_Low))
    Serial.println("Инициализация AHT10 OK");
  else
    Serial.println("Ошибка инициализации AHT10");
}

void loop() {

  Serial.println("----------------------------------------------------");
  Serial.println(String("")+"Влажность(%RH):\t\t"+AHT10.GetHumidity()+"%");
  Serial.println(String("")+"Температура(℃):\t\t"+AHT10.GetTemperature()+"℃");
  Serial.println(String("")+"Точка росы(℃):\t\t"+AHT10.GetDewPoint()+"℃");

  delay(1000);
}
