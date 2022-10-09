// Space Glove (prueba)

/* DEFINICIÓN DE PINES */

/* BUS I2C 1 */
#define SDA_1 21 // ROJO
#define SCL_1 22 // NEGRO

/* BUS I2C 2 */
#define SDA_2 33 // ROJO 
#define SCL_2 32 // NEGRO

/* LED DE PLACA */
#define LED_PLACA 2

/* Lbrerias usadas */
#include <Arduino.h>
#include <Adafruit_MPU6050.h> // SSENSOR MPU6050
#include <Adafruit_Sensor.h>  // LIBRERIA USADA PARA EL SENSOR ADAFRUIT CON TODOS LOS METODOS
#include <Wire.h>             // LIBRERIA PARA I2C

// /* DECLARACION DE VARIABLES */
// const char *config_file = "/config/diccionario.txt";

// struct Ori {
//   String orientiacion;
// };


/* SENSOR MPU6050 */
Adafruit_MPU6050 sensor;  // MPU6050 (DEDO1)
Adafruit_MPU6050 sensor2; // MPU6060 (DED02)
Adafruit_MPU6050 sensor3; // MPU6050 (DED03)
Adafruit_MPU6050 sensor4; // MPU6060 DED04
Adafruit_MPU6050 sensor5; 
Adafruit_MPU6050 sensor6;


/* BUSES I2C */
TwoWire I2Cuno = TwoWire(0); // Primer bus I2C
TwoWire I2Cdos = TwoWire(1); // Segundo bus I2C

// /* VARIABLES PARA LA ESTRUCTUA */ 
// Ori orientacion;



// void OTA_nucleo(void *parameter)
// {
//   ArduinoOTA.handle(); // METODO QUE ACTUALIZA EL SENSOR (CACHA LA INFORMACIÓN)
// }

/* INICIALIZACION DE FUNCIONES */
// /* CARGAR ORIENTACIONES */
// void cargarOrientacion(const char *config_file, Ori &orientacion){
//   File archivo = SD.open(config_file);
//   StaticJsonDocument<200> json;
//   StaticJsonBuffer<200> json(1024);
// }

boolean vsensor1, vsensor2, vsensor3, vsensor4;
const int numSamples = 120;
int samplesRead = numSamples;
const float accelerationThreshold = 17;

/* INICIALIZACIÓN DE SENSORES */
void incializarSensores()
{
  /* INICIALIZACIÓN DE SENSOR DED01 */
  sensor.setAccelerometerRange(MPU6050_RANGE_16_G); // RANGO DE ACELEROMETRO
  sensor.setGyroRange(MPU6050_RANGE_500_DEG); // RANGO DEL GIROSCOPIO
  sensor.setFilterBandwidth(MPU6050_BAND_5_HZ); 

  /* INICIALIZACIÓN DE SENSOR DED02 */
  sensor2.setAccelerometerRange(MPU6050_RANGE_16_G);
  sensor2.setGyroRange(MPU6050_RANGE_500_DEG);
  sensor2.setFilterBandwidth(MPU6050_BAND_5_HZ);

    /* INICIALIZACIÓN DE SENSOR DED02 */
  sensor3.setAccelerometerRange(MPU6050_RANGE_16_G);
  sensor3.setGyroRange(MPU6050_RANGE_500_DEG);
  sensor3.setFilterBandwidth(MPU6050_BAND_5_HZ);
    /* INICIALIZACIÓN DE SENSOR DED02 */

  sensor4.setAccelerometerRange(MPU6050_RANGE_16_G);
  sensor4.setGyroRange(MPU6050_RANGE_500_DEG);
  sensor4.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED01) */
void obtenerSensor1()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED01*/
  sensors_event_t a, g, temp;
  sensor.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;
  Serial.print(a.acceleration.x, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.y, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.z, 3);
  Serial.print(" ");
  Serial.print(g.gyro.x, 3);
  Serial.print(" ");
  Serial.print(g.gyro.y, 3);
  Serial.print(" ");
  Serial.print(g.gyro.z, 3);
  Serial.print(" ");
  Serial.print("1");
  Serial.println();
 

}

/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED02) */
void obtenerSensor2()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED02*/
  sensors_event_t a, g, temp;
  sensor2.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;
  Serial.print(a.acceleration.x, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.y, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.z, 3);
  Serial.print(" ");
  Serial.print(g.gyro.x, 3);
  Serial.print(" ");
  Serial.print(g.gyro.y, 3);
  Serial.print(" ");
  Serial.print(g.gyro.z, 3);
  Serial.print(" ");
  Serial.print("2");
  Serial.println();
 
}


/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED03) */
void obtenerSensor3()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED02*/
  sensors_event_t a, g, temp;
  sensor3.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;
  Serial.print(a.acceleration.x, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.y, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.z, 3);
  Serial.print(" ");
  Serial.print(g.gyro.x, 3);
  Serial.print(" ");
  Serial.print(g.gyro.y, 3);
  Serial.print(" ");
  Serial.print(g.gyro.z, 3);
  Serial.print(" ");
  Serial.print("3");
  Serial.println();
 
}

void obtenerSensor4()
{
  /* OBTIENE LOS VALORES DEL SENSOR DED02*/
  sensors_event_t a, g, temp;
  sensor4.getEvent(&a, &g, &temp);
  float coord_y = a.acceleration.y;
  Serial.print(a.acceleration.x, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.y, 3);
  Serial.print(" ");
  Serial.print(a.acceleration.z, 3);
  Serial.print(" ");
  Serial.print(g.gyro.x, 3);
  Serial.print(" ");
  Serial.print(g.gyro.y, 3);
  Serial.print(" ");
  Serial.print(g.gyro.z, 3);
  Serial.print(" ");
  Serial.print("4");
  Serial.println();
 
}


/* FUNCIÓN DE INICIO DE LA PLACA */
void setup()
{
  pinMode(LED_PLACA, OUTPUT); // LED PLACA
  I2Cuno.begin(SDA_1, SCL_1, 40000UL); // INCIIALIZACIÓN DE BUS 1
  I2Cdos.begin(SDA_2, SCL_2, 40000UL); // INICIALIZACIÓN DE BUS 2
  Serial.begin(115200); // APERTURA DE MONITOR SERIAL.
  while(!Serial);
  vsensor1 = sensor.begin(0x68, &I2Cuno);
  vsensor2 = sensor2.begin(0x69, &I2Cuno);
  vsensor3 = sensor3.begin(0x69, &I2Cdos);
  vsensor4 = sensor4.begin(0x68, &I2Cdos);
  if (!vsensor1 || !vsensor2 || !vsensor3 || !vsensor4) // INICIO DE SENSORES.
  {
    Serial.println("No se pudo conectar con alguno de los sensores");
    Serial.println(vsensor1);
    Serial.println(vsensor2);
    Serial.println(vsensor3);
    Serial.println(vsensor4);
    while (1)
    {
      delay(10);
      digitalWrite(LED_PLACA, HIGH);
      delay(500);
      digitalWrite(LED_PLACA, LOW);
      delay(500);
    }
  }
  digitalWrite(LED_PLACA, HIGH);
  incializarSensores(); // FUNCIÓN QUE INICIALIZA LOS PARAMETROS DE SENSORES, COMO LOS GRADOS DE MOVIMIENTO ETC.
  Serial.println("aX aY aZ gX gY gZ sN");
}


/* FUNCIÓN EN CONSTANTE REPETICIÓN */
void loop()
{
  float aX, aY, aZ, gX, gY, gZ;
  while(samplesRead == numSamples){
    sensors_event_t a, g, t;
    sensor4.getEvent(&a, &g, &t);
    aX = a.acceleration.x;
    aY= a.acceleration.y;
    aZ = a.acceleration.z;

    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);
    if(aSum >= accelerationThreshold){
      samplesRead = 0;
      break;
    }
  }
  while (samplesRead < numSamples){
    samplesRead++;
    obtenerSensor1();
    obtenerSensor2();
    obtenerSensor3();
    obtenerSensor4();
    if(samplesRead == numSamples){
      Serial.println();
    }
  }
  
}
