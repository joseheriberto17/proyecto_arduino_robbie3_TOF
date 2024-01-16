#include <Wire.h>
#include <VL6180X.h>

#define SCALING 2

VL6180X sensor1;
VL6180X sensor2;

void setup() {

  Serial.begin(115200);
  Wire.begin(); 

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);

  digitalWrite(2,LOW);
  digitalWrite(3,LOW);

   delay(100);

  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);

  delay(100);


  sensor1.setAddress(0x08);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setScaling(SCALING);
  sensor1.setTimeout(500);

  delay(100);

  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);

  delay(100);

  sensor2.setAddress(0x0A);
  sensor2.init();
  sensor2.configureDefault();
  sensor2.setScaling(SCALING);
  sensor2.setTimeout(500);

  delay(100);
}

void loop() {
  Serial.print("(Scaling = ");
  Serial.print(sensor1.getScaling());
  Serial.print("x) ");

  Serial.print("Sensor1: ");
  Serial.print(sensor1.readRangeSingleMillimeters());
  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print(" (Address: ");
  Serial.print(sensor1.getAddress());
  Serial.print(") ");

  
  Serial.print("Sensor2: ");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print(" (Address: ");
  Serial.print(sensor2.getAddress());
  Serial.print(") ");

  Serial.println();
}
