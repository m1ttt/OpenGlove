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

/* Lbrerias usadas */
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h> // LIBRERIA PARA I2C
#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include "data/model.h"

// /* DECLARACION DE VARIABLES */
// const char *config_file = "/config/diccionario.txt";

// struct Ori {
//   String orientiacion;
// };

/* SENSOR MPU6050 */

Adafruit_MPU6050 sensores[6];

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
const int numSamples = 75;
int samplesRead = numSamples;

const float accelerationThreshold = 18;

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
    "cerrado",
    "giros"};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

/* INICIALIZACIÓN DE SENSORES */
void incializarSensores()
{
  /* INICIALIZACIÓN DE SENSOR DED01 */
  sensores[1].setAccelerometerRange(MPU6050_RANGE_2_G); // RANGO DE ACELEROMETRO
  sensores[1].setGyroRange(MPU6050_RANGE_500_DEG);      // RANGO DEL GIROSCOPIO
  sensores[1].setFilterBandwidth(MPU6050_BAND_5_HZ);

  /* INICIALIZACIÓN DE SENSOR DED02 */
  sensores[2].setAccelerometerRange(MPU6050_RANGE_2_G);
  sensores[2].setGyroRange(MPU6050_RANGE_500_DEG);
  sensores[2].setFilterBandwidth(MPU6050_BAND_5_HZ);

  /* INICIALIZACIÓN DE SENSOR DED02 */
  sensores[3].setAccelerometerRange(MPU6050_RANGE_2_G);
  sensores[3].setGyroRange(MPU6050_RANGE_500_DEG);
  sensores[3].setFilterBandwidth(MPU6050_BAND_5_HZ);
  /* INICIALIZACIÓN DE SENSOR DED02 */

  sensores[4].setAccelerometerRange(MPU6050_RANGE_2_G);
  sensores[4].setGyroRange(MPU6050_RANGE_500_DEG);
  sensores[4].setFilterBandwidth(MPU6050_BAND_5_HZ);
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
  float aX, aY, aZ, gX, gY, gZ;
  int sexoAnal_Record = digitalRead(BOTON_CAPTURADOR);
  int sexoAnal_Record2 = digitalRead(18);
  if (sexoAnal_Record == HIGH)
  {
    samplesRead = 0;
    while (samplesRead < numSamples)
    {
      for (int i = 1; i <= 4; i++)
      {
        sensors_event_t a, g, temp;
        sensores[1].getEvent(&a, &g, &temp);
        aX = a.acceleration.x;
        aY = a.acceleration.y;
        aX = a.acceleration.z;
        gX = g.gyro.x;
        gY = g.gyro.y;
        gZ = g.gyro.z;
      }
      //   // // normalize the IMU data between 0 to 1 and store in the model's // input tensor
      tflInputTensor->data.f[samplesRead * 6 + 0] = (aX);
      tflInputTensor->data.f[samplesRead * 6 + 1] = (aY);
      tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ);
      tflInputTensor->data.f[samplesRead * 6 + 3] = (gX);
      tflInputTensor->data.f[samplesRead * 6 + 4] = (gY);
      tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ);
      samplesRead++;
      if (samplesRead == numSamples)
      {
        TfLiteStatus invokeStatus = tflInterpreter->Invoke();
        if (invokeStatus != kTfLiteOk)
        {
          Serial.println("Invoke failed!");
          while (1)
            ;
          return;
        }
        for (int i = 0; i < NUM_GESTURES; i++)
        {
          Serial.print(GESTURES[i]);
          Serial.print(": ");
          Serial.println(tflOutputTensor->data.f[i], 6);

          Serial.println();
        }
      }
    }
  }
  else if (sexoAnal_Record2 == HIGH)
  {
    samplesRead = 0;
    while (samplesRead < numSamples)
    {
      for (int i = 1; i <= 4; i++)
      {
        obtenerSensores(i);
      }
      samplesRead++;
      if (samplesRead == numSamples)
      {
        Serial.println("");
        break;
      }
    }
  }
  else
  {
  }
}