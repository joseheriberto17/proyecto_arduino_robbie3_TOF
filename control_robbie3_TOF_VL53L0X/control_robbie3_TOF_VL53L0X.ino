/*
  nombre: control_Robbie3_TOF_VL53L0X
  autor: jose heriberto maquez diaz

  librerias: VL53L0X de pololu
  hardware: 
    -varios modulo VL53L0x generico
    -arduino


  descripcion: este codigo permite que un arduino con interfaz i2c 
  y suficientes pines de habilitacion para los sensores pueda 
  controlar ,secuencialmente varios vl53l0x para que el robot 
  robbie3 obtener un perpectiva del entorno.
*/

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;
VL53L0X sensor6;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor1 and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY

void medir_distancia_TOF(VL53L0X &sensor)
{
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
}

void inicio_sensor_TOF(VL53L0X &sensor,uint8_t pin, uint8_t adress)
{
  digitalWrite(pin,HIGH);

  delay(100);
  sensor.setAddress(adress);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }
}


void setup()
{
  // inicio de perifericos

  Serial.begin(115200);
  Wire.begin();

  // inicio de los pines de Habilitadores, Total: 6 sensores 
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(3,OUTPUT);

  digitalWrite(8,LOW);
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);

  // inicio de los sensor TOF
  inicio_sensor_TOF(sensor1,8,0x01);
  inicio_sensor_TOF(sensor2,7,0x02);
  inicio_sensor_TOF(sensor3,6,0x03);
  inicio_sensor_TOF(sensor4,5,0x04);
  inicio_sensor_TOF(sensor5,4,0x05);
  inicio_sensor_TOF(sensor6,3,0x06);
  
  

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor1.setMeasurementTimingBudget(20000);
  sensor2.setMeasurementTimingBudget(20000);
  sensor3.setMeasurementTimingBudget(20000);
  sensor4.setMeasurementTimingBudget(20000);
  sensor5.setMeasurementTimingBudget(20000);
  sensor6.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);
  sensor3.setMeasurementTimingBudget(200000);
  sensor4.setMeasurementTimingBudget(200000);
  sensor5.setMeasurementTimingBudget(200000);
  sensor6.setMeasurementTimingBudget(200000);
#endif
}

void loop()
{
  // medicion de todos los sensores.
  medir_distancia_TOF(sensor1);
  Serial.print(" ");
  medir_distancia_TOF(sensor2);
  Serial.print(" ");
  medir_distancia_TOF(sensor3);
  Serial.print(" ");
  medir_distancia_TOF(sensor4);
  Serial.print(" ");
  medir_distancia_TOF(sensor5);
  Serial.print(" ");
  medir_distancia_TOF(sensor6);
  
  Serial.println();
}

