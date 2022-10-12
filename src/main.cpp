// Space Glove (prueba)

/* DEFINICIÓN DE PINES */

/* BUS I2C 1 */
#define SDA_1 21 // ROJO
#define SCL_1 22 // NEGRO

/* BUS I2C 2 */
#define SDA_2 33 // ROJO
#define SCL_2 32 // NEGRO

#define BOTON_CAPTURADOR 34

/* LED DE PLACA */
#define LED_PLACA 2
#define READINGS_PER_SAMPLE 40
#define THRESHOLD 20

/* Lbrerias usadas */
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h> // LIBRERIA PARA I2C
#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>a
#include "data/model.h"

/* SENSOR MPU6050 */

Adafruit_MPU6050 sensores[6];
float ax;
float ay;
float az;
float baseAx;
float baseAy;
float baseAz;

/* BUSES I2C */
TwoWire I2Cuno = TwoWire(0); // Primer bus I2C
TwoWire I2Cdos = TwoWire(1); // Segundo bus I2C

boolean vsensor1, vsensor2, vsensor3, vsensor4;

const float accelerationThreshold = 18;
int numSamples = 40;
int samplesRead = numSamples;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model *tflModel = nullptr;
tflite::MicroInterpreter *tflInterpreter = nullptr;
TfLiteTensor *tflInputTensor = nullptr;
TfLiteTensor *tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 2048;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char *GESTURES[] = {
    "nueve",
    "ocho"};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

/* INICIALIZACIÓN DE SENSORES */
void incializarSensores()
{
  for (int i = 1; i <= 4; i++)
  {
    sensores[i].setAccelerometerRange(MPU6050_RANGE_16_G); // RANGO DE ACELEROMETRO
    sensores[i].setGyroRange(MPU6050_RANGE_250_DEG);       // RANGO DEL GIROSCOPIO
    sensores[i].setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

/* FUNCION QUE OBTIENE LOS VALORES DEL SENSOR (DED01) */
void obtenerSensores(int n)
{
  /* OBTIENE LOS VALORES DEL SENSOR DED01*/
  sensors_event_t a, g, temp;
  sensores[n].getEvent(&a, &g, &temp);
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
  Serial.print(n);
  Serial.println();
}

void err_Conexion()
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

void run_inference()
{
  sensors_event_t a, g, temp;
  for (int i = 0; i < READINGS_PER_SAMPLE; i++)
  {
    sensores[1].getEvent(&a, &g, &temp);
    ax = a.acceleration.x - baseAx;
    ay = a.acceleration.y - baseAy;
    az = a.acceleration.z - baseAz;
    tflInputTensor->data.f[i * 3 + 0] = (ax + 8.0) / 16.0;
    tflInputTensor->data.f[i * 3 + 1] = (ay + 8.0) / 16.0;
    tflInputTensor->data.f[i * 3 + 2] = (az + 8.0) / 16.0;
    delay(10);
  }

  TfLiteStatus invokeStatus = tflInterpreter->Invoke();

  for (int i = 0; i < NUM_GESTURES; i++)
  {
    Serial.print(GESTURES[i]);
    Serial.print(": ");
    Serial.println(tflOutputTensor->data.f[i], 6);
  }
  Serial.println();
}
void detectMovement()
{
  sensors_event_t a, g, temp;
  sensores[1].getEvent(&a, &g, &temp);
  if (abs(a.acceleration.x - baseAx) + abs(a.acceleration.y - baseAy) + abs(a.acceleration.z - baseAz) > THRESHOLD)
  {
    run_inference();
  }
  else
  {
    delay(5);
  }
}
void calibrate_sensor()
{
  float totX, totY, totZ;
  sensors_event_t a, g, temp;

  for (int i = 0; i < 10; i++)
  {
    sensores[1].getEvent(&a, &g, &temp);
    totX = totX + a.acceleration.x;
    totY = totY + a.acceleration.y;
    totZ = totZ + a.acceleration.z;
  }
  baseAx = totX / 10;
  baseAy = totY / 10;
  baseAz = totZ / 10;
}
/* FUNCIÓN DE INICIO DE LA PLACA */
void setup()
{
  pinMode(BOTON_CAPTURADOR, INPUT);
  pinMode(18, INPUT);
  pinMode(LED_PLACA, OUTPUT);          // LED PLACA
  I2Cuno.begin(SDA_1, SCL_1, 40000UL); // INCIIALIZACIÓN DE BUS 1
  I2Cdos.begin(SDA_2, SCL_2, 40000UL); // INICIALIZACIÓN DE BUS 2
  Serial.begin(115200);                // APERTURA DE MONITOR SERIAL.
  while (!Serial)
    ;
  vsensor1 = sensores[1].begin(0x68, &I2Cuno);
  vsensor2 = sensores[2].begin(0x69, &I2Cuno);
  vsensor3 = sensores[3].begin(0x69, &I2Cdos);
  vsensor4 = sensores[4].begin(0x68, &I2Cdos);
  if (!vsensor1 || !vsensor2 || !vsensor3 || !vsensor4) // INICIO DE SENSORES.
  {
    err_Conexion();
  }
  digitalWrite(LED_PLACA, HIGH);
  incializarSensores(); // FUNCIÓN QUE INICIALIZA LOS PARAMETROS DE SENSORES, COMO LOS GRADOS DE MOVIMIENTO ETC.
  Serial.println("aX aY aZ gX gY gZ sN");
  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION)
  {
    Serial.println("Model schema mismatch!");
    while (1)
      ;
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
}

/* FUNCIÓN EN CONSTANTE REPETICIÓN */
void loop()
{
  detectMovement();
}