// Space Glove (prueba)

/* DEFINICIÓN DE PINES */

/* BUS I2C 1 */
#define SDA_1 21
#define SCL_1 22

/* BUS I2C 2 */
#define SDA_2 33
#define SCL_2 32

/* LED DE PLACA */
#define LED_PLACA 2

/* Lbrerias usadas */
#include <Arduino.h>
#include <Adafruit_MPU6050.h> // SSENSOR MPU6050
#include <Adafruit_Sensor.h>  // LIBRERIA USADA PARA EL SENSOR ADAFRUIT CON TODOS LOS METODOS
#include <Wire.h>             // LIBRERIA PARA I2C
#include "libs/wifisetup.h"        // INICIALIZACIÓN DE LA CONEXIÓN WIFI
#include <Servo.h>            // LIBRERIA PARA EL SERVO (OPCIONAL)
#include <ArduinoJson.h>      // JSON
#include <SD.h> 
#include <SPI.h> 

/* DECLARACION DE VARIABLES */
const char *config_file = "/config/diccionario.txt";

struct Ori {
  String orientiacion;
};

/* INICIALIZACION DE VARIABLES */
TaskHandle_t tarea1;
TaskHandle_t tarea2; // VARIABLES PARA EL USO DE NUCLEOS
TaskHandle_t tarea3;

/* SENSOR MPU6050 */
Adafruit_MPU6050 sensor;  // MPU6050 (DEDO1)
Adafruit_MPU6050 sensor2; // MPU6060 (DED02)
Adafruit_MPU6050 sensor3; // MPU6050 (DED03)


/* BUSES I2C */
TwoWire I2Cuno = TwoWire(0); // Primer bus I2C
TwoWire I2Cdos = TwoWire(1); // Segundo bus I2C

/* VARIABLES PARA LA ESTRUCTUA */ 
Ori orientacion;



// void OTA_nucleo(void *parameter)
// {
//   ArduinoOTA.handle(); // METODO QUE ACTUALIZA EL SENSOR (CACHA LA INFORMACIÓN)
// }

/* INICIALIZACION DE FUNCIONES */
/* CARGAR ORIENTACIONES */
void cargarOrientacion(const char *config_file, Ori &orientacion){
  File archivo = SD.open(config_file);
  
}



/* INICIALIZACIÓN DE SENSORES */
void incializarSensores()
{
  /* INICIALIZACIÓN DE SENSOR DED01 */
  sensor.setAccelerometerRange(MPU6050_RANGE_2_G); // RANGO DE ACELEROMETRO
  sensor.setGyroRange(MPU6050_RANGE_500_DEG); // RANGO DEL GIROSCOPIO
  sensor.setFilterBandwidth(MPU6050_BAND_5_HZ); 

  /* INICIALIZACIÓN DE SENSOR DED02 */
  sensor2.setAccelerometerRange(MPU6050_RANGE_2_G);
  sensor2.setGyroRange(MPU6050_RANGE_500_DEG);
  sensor2.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED01) */
void obtenerSensor1()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED01*/
  sensors_event_t a, g, temp;
  sensor.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;
Serial.println("");
  Serial.println("[1]");
    Serial.println("------------------------------");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
    Serial.println("------------------------------");
  
}

/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED02) */
void obtenerSensor2()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED02*/
  sensors_event_t a, g, temp;
  sensor2.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;

  /* IMPRIME LOS VALORES DE DED02 */ 
   Serial.println("[2]");
   Serial.println("------------------------------");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
    Serial.println("------------------------------");
}


/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED03) */
void obtenerSensor3()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED02*/
  sensors_event_t a, g, temp;
  sensor3.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;

  /* IMPRIME LOS VALORES DE DED02 */ 
   Serial.println("[3]");
   Serial.println("------------------------------");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
    Serial.println("------------------------------");
}


/* FUNCIÓN DE INICIO DE LA PLACA */
void setup()
{
  pinMode(LED_PLACA, OUTPUT); // LED PLACA
  I2Cuno.begin(SDA_1, SCL_1, 40000UL); // INCIIALIZACIÓN DE BUS 1
  I2Cdos.begin(SDA_2, SCL_2, 40000UL); // INICIALIZACIÓN DE BUS 2
  Serial.begin(115200); // APERTURA DE MONITOR SERIAL.
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Prueba de MPU6050 (4)");
  if (!sensor.begin(0x68, &I2Cuno) || !sensor2.begin(0x69, &I2Cuno) || !sensor3.begin(0x68, &I2Cdos)) // INICIO DE SENSORES.
  {
    Serial.println("No se pudo conectar con alguno de los sensores");
    for (;;) // BUCLE DE LED
    {
      digitalWrite(LED_PLACA, HIGH);
      delay(500);
      digitalWrite(LED_PLACA, LOW);
      delay(500);
    }
    while (1)
    {
      delay(10);
    }
  }
  digitalWrite(LED_PLACA, HIGH);
  Serial.println("Sensores conectados");
  incializarSensores(); // FUNCIÓN QUE INICIALIZA LOS PARAMETROS DE SENSORES, COMO LOS GRADOS DE MOVIMIENTO ETC.
  delay(2000);
}


/* FUNCIÓN EN CONSTANTE REPETICIÓN */
void loop()
{
  obtenerSensor1(); // OBTIENE VALORES DEL SENSOR 1 (DED01)
  obtenerSensor2(); // OBTIENE VALORES DEL SENSOR 2 (DED02)
  obtenerSensor3();
  delay(2000);
}
