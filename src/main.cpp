// Space Glove (prueba)

/* DEFINICIÓN DE PINES */

/* BUS I2C 1 */
#define SDA_1 21 // ROJO
#define SCL_1 22 // NEGRO

/* BUS I2C 2 */
#define SDA_2 33 // ROJO
#define SCL_2 32 // NEGRO

// #define SENSOR_MENIQUE 27
#define SENSOR_PULGAR 14

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



/* SENSOR MPU6050 */

Adafruit_MPU6050 sensores[4];
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



void obtenerSensores2(int n)
{
    sensors_event_t a, g, temp;
    sensores[n].getEvent(&a, &g, &temp);
    ax = a.acceleration.x - baseAx;
    ay = a.acceleration.y - baseAy;
    az = a.acceleration.z - baseAz;
}

void err_Conexion()
{
    Serial.println("No se pudo conectar con alguno de los sensores");
    Serial.println(vsensor1);
    Serial.println(vsensor2);
    Serial.println(vsensor3);
    for(;;){
        digitalWrite(LED_PLACA, HIGH);
        delay(500);
        digitalWrite(LED_PLACA, LOW);
        delay(500);
    }
}

void run_inference()
{
    sensors_event_t a, g, temp;
    for (int i = 0; i < 42; i++)
    {
        for (int a = 1; a <= 3; a++)
        {
            obtenerSensores2(a);
            tflInputTensor->data.f[i * 3 + 0] = (ax + 8.0) / 16.0;
            tflInputTensor->data.f[i * 3 + 1] = (ay + 8.0) / 16.0;
            tflInputTensor->data.f[i * 3 + 2] = (az + 8.0) / 16.0;
        }
    }

    TfLiteStatus invokeStatus = tflInterpreter->Invoke();

    float salidaA = tflOutputTensor->data.f[0];
    float salidaB = tflOutputTensor->data.f[1];
    float salidaY = tflOutputTensor->data.f[3];
    if (salidaA >= 0.70)
    {
        Serial.print("Letra A");
        Serial.print(salidaA);

    }
    else if (salidaB >= 0.70)
    {
        Serial.print("Letra B");

    }
    else if (salidaY >= 0.70)
    {
        Serial.print("Letra Y");

    }
    Serial.println();
}
void detectMovement()
{

    run_inference();
}


void iniciarlizarPines()
{

    pinMode(LED_PLACA, OUTPUT); // LED PLACA
    pinMode(SENSOR_PULGAR, OUTPUT);
    digitalWrite(SENSOR_PULGAR, HIGH);
    pinMode(25, OUTPUT);
    I2Cuno.begin(SDA_1, SCL_1, 40000UL); // INCIIALIZACIÓN DE BUS 1
    I2Cdos.begin(SDA_2, SCL_2, 40000UL); // INICIALIZACIÓN DE BUS 2
    Serial.begin(115200);
}

void inicializarTensorFlow()
{
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

void inicializarPuertosI2C()
{
    // APERTURA DE MONITOR SERIAL.
    while (!Serial)
        ;
    vsensor2 = sensores[1].begin(0x68, &I2Cuno);
    vsensor3 = sensores[2].begin(0x69, &I2Cdos);
    vsensor4 = sensores[3].begin(0x68, &I2Cdos);
    if (!vsensor2 || !vsensor3 || !vsensor4) // INICIO DE SENSORES.
    {
        err_Conexion();
    }
}
/* FUNCIÓN DE INICIO DE LA PLACA */
void setup()
{
    iniciarlizarPines();
    inicializarPuertosI2C();
    digitalWrite(LED_PLACA, HIGH);
    incializarSensores(); // FUNCIÓN QUE INICIALIZA LOS PARAMETROS DE SENSORES, COMO LOS GRADOS DE MOVIMIENTO ETC.
    inicializarTensorFlow();
}

/* FUNCIÓN EN CONSTANTE REPETICIÓN */
void loop()
{
    detectMovement();
}