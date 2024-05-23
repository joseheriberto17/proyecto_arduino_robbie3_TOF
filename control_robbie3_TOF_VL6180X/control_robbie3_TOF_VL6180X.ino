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

  1) deshabilita todos los sensores VL6180x
  2) habilita y le asigna una nueva direcion i2c a cada sensor uno por uno.
  3) raspberry pico W registra la distancia que le solicita acada sensor uno por uno.
  4) cuando ya tiene las distacia de todos los sensores los envia al MCU STM32.
  5) repite el ciclo desde el paso 3 hasta que se desconecte la alimentacion.
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
    return 0; // si no hay respuesta de sensor llamado se asigna el valor como cero.
  }
  else
  {
    return sensor.readRangeSingleMillimeters();
  }
  
}
// funcion que habilita, configura por defecto el sensor y asigna una nueva direccion i2c.
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

  // inicio de perifericos
  Serial.begin(115200);
  Wire.begin();

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
  // medicion de  distancia de todos los sensores.
  value_sensor1 = medir_distancia_TOF(sensor1);
  value_sensor2 = medir_distancia_TOF(sensor2);
  value_sensor3 = medir_distancia_TOF(sensor3);
  value_sensor4 = medir_distancia_TOF(sensor4);
  value_sensor5 = medir_distancia_TOF(sensor5);
  value_sensor6 = medir_distancia_TOF(sensor6);
  
  // crea una cadena de caracteres con las variable de distancia de cada sensor.
  sprintf(data,"%u %u %u %u %u %u",value_sensor1,value_sensor2,value_sensor3,value_sensor4,value_sensor5,value_sensor6);
  Serial.println(data);

  size_datos = strlen(data); 

  // enviar el numero de bits que tiene que leer el MCU STM32 para que reciba la proxima transmicion de datos. 
  Wire.beginTransmission(I2C_SLAVE);
  Wire.write(size_datos);
  num = Wire.endTransmission();

  //  enviar la cadena de caracteres con las variable de distancia al MCU STM32.
  Wire.beginTransmission(I2C_SLAVE);
  Wire.write(data);
  num = Wire.endTransmission();

  // formatea la variable data.
  memset(data, 0, sizeof(data));
}
