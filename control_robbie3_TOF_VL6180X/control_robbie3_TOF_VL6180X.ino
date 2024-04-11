/*
  nombre: control_Robbie3_TOF_VL6180X
  autor: jose heriberto maquez diaz

  librerias: VL6180X de pololu
  hardware: 
    -6 modulo VL6180X marca pololu
    -raspberry pico W


  descripcion: este codigo permite que un arduino con interfaz i2c 
  y con suficientes pines de habilitacion para los sensores pueda 
  controlar ,secuencialmente varios vl6180x para que el robot 
  robbie3 obtener un perpectiva del entorno.
*/

#include <Wire.h>
#include <VL6180X.h>

#define SCALING 1

// sensores TOF VL6180X
VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;
VL6180X sensor4;
VL6180X sensor5;
VL6180X sensor6;

// variable para el valor de distancia para cada sensor
uint16_t value_sensor1;
uint16_t value_sensor2;
uint16_t value_sensor3;
uint16_t value_sensor4;
uint16_t value_sensor5;
uint16_t value_sensor6;

// variable para el manejo del tiempo
unsigned long valorPrevio;
unsigned long valorActual;
unsigned long valorDiff;

// variable para el mensaje de datos
char data[80];

// variable para la comunicacion i2c a un dispositivo esclavo.
uint8_t size_datos;
const int16_t I2C_SLAVE = 0x08;
uint8_t num = 0;

uint16_t medir_distancia_TOF(VL6180X &sensor)
{
  if (sensor.timeoutOccurred()) 
  { 
    Serial.print(" TIMEOUT ");
    Serial.print(sensor.getAddress());
    Serial.print(" ");
    return 0;
  }
  else
  {
    return sensor.readRangeSingleMillimeters();
  }
  
}

void inicio_sensor_TOF(VL6180X &sensor,uint8_t pin, uint8_t adress)
{
  digitalWrite(pin,HIGH);

  delay(100);
  sensor.setAddress(adress);
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
  
}


void setup()
{
  // tiempo
  valorPrevio=millis();

  // inicio de perifericos
  Serial.begin(115200);
  Wire.begin();
//  Wire.setClock(400000);

  // inicio de los pines de Habilitadores, Total: 6 sensores 
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);

  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);

  // inicio de los sensor TOF
  inicio_sensor_TOF(sensor1,6,0x09);
  inicio_sensor_TOF(sensor2,7,0x10);
  inicio_sensor_TOF(sensor3,8,0x11);
  inicio_sensor_TOF(sensor4,10,0x12);
  inicio_sensor_TOF(sensor5,11,0x13);
  inicio_sensor_TOF(sensor6,12,0x14);
}
  

void loop()
{
  // tiempo
  valorActual=millis();
  valorDiff = valorActual - valorPrevio; 
  valorPrevio = valorActual;

  // medicion de todos los sensores.
  value_sensor1 = medir_distancia_TOF(sensor1);
  value_sensor2 = medir_distancia_TOF(sensor2);
  value_sensor3 = medir_distancia_TOF(sensor3);
  value_sensor4 = medir_distancia_TOF(sensor4);
  value_sensor5 = medir_distancia_TOF(sensor5);
  value_sensor6 = medir_distancia_TOF(sensor6);

  sprintf(data,"%u %u %u %u %u %u",value_sensor1,value_sensor2,value_sensor3,value_sensor4,value_sensor5,value_sensor6);

  Serial.println(data);

  size_datos = strlen(data); 

  // enviar datos al periferico i2c
  Wire.beginTransmission(I2C_SLAVE);
  Wire.write(size_datos);
  num = Wire.endTransmission();

  Wire.beginTransmission(I2C_SLAVE);
  Wire.write(data);
  num = Wire.endTransmission();

  memset(data, 0, sizeof(data));
}
