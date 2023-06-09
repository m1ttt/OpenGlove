// Space Glove (prueba)

/* DEFINICIÓN DE PINES */

/* BUS I2C 1 */
#define SDA_1 21 // ROJO
#define SCL_1 22 // NEGRO

/* BUS I2C 2 */
#define SDA_2 33 // ROJO
#define SCL_2 32 // NEGRO

// #define SENSOR_MENIQUE 27
#define SENSOR_PULGAR 14 // SENSOR PULGAR

#define LED_PLACA 2 // LED DE LA PLACA

/* Lbrerias usadas */

#include <Arduino.h>
#include <Adafruit_MPU6050.h> // LIBRERIA PARA MPU6050
#include <Wire.h> // LIBRERIA PARA I2C
#include <TensorFlowLite_ESP32.h> // LIBRERIA PARA TENSORFLOW
#include <tensorflow/lite/micro/all_ops_resolver.h> // LIBRERIA PARA TENSORFLOW
#include <tensorflow/lite/micro/micro_error_reporter.h> // LIBRERIA PARA TENSORFLOW
#include <tensorflow/lite/micro/micro_interpreter.h> // LIBRERIA PARA TENSORFLOW
#include <tensorflow/lite/schema/schema_generated.h> // LIBRERIA PARA TENSORFLOW
#include "data/model.h" // MODELO DE TENSORFLOW

/* SENSOR MPU6050 */

Adafruit_MPU6050 sensores[4]; // ARRAY DE SENSORES
float ax;                    // ACELERACION EN X
float ay;                   // ACELERACION EN Y
float az;                 // ACELERACION EN Z
float baseAx;            // ACELERACION EN X
float baseAy;          // ACELERACION EN Y
float baseAz;       // ACELERACION EN Z

/* BUSES I2C */
TwoWire I2Cuno = TwoWire(0); // Primer bus I2C 
TwoWire I2Cdos = TwoWire(1); // Segundo bus I2C

boolean vsensor1, vsensor2, vsensor3, vsensor4; // VARIABLES DE VERIFICACIÓN DE CONEXIÓN DE SENSORES

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter; // error reporter

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver; // resolver

const tflite::Model *tflModel = nullptr; // model
tflite::MicroInterpreter *tflInterpreter = nullptr; // interpreter
TfLiteTensor *tflInputTensor = nullptr; // input tensor
TfLiteTensor *tflOutputTensor = nullptr; // output tensor
 
// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 2048; // 8kB
byte tensorArena[tensorArenaSize] __attribute__((aligned(16))); // tensor arena

/* INICIALIZACIÓN DE SENSORES */
void incializarSensores() 
{
    for (int i = 1; i <= 4; i++) 
    {
        sensores[i].setAccelerometerRange(MPU6050_RANGE_16_G); // RANGO DEL ACELEROMETRO
        sensores[i].setGyroRange(MPU6050_RANGE_250_DEG);       // RANGO DEL GIROSCOPIO
        sensores[i].setFilterBandwidth(MPU6050_BAND_21_HZ);   // BANDA DE PASO DEL FILTRO
    }
}

void obtenerSensores2(int n) 
{ // OBTENER DATOS DE LOS SENSORES
    sensors_event_t a, g, temp; // VARIABLES DE EVENTOS DE LOS SENSORES
    sensores[n].getEvent(&a, &g, &temp); // OBTENER EVENTOS DE LOS SENSORES
    ax = a.acceleration.x - baseAx; // OBTENER ACELERACION EN X
    ay = a.acceleration.y - baseAy; // OBTENER ACELERACION EN Y
    az = a.acceleration.z - baseAz; // OBTENER ACELERACION EN Z
}

void err_Conexion() 
{ // VERIFICAR CONEXIÓN DE LOS SENSORES
    Serial.println("No se pudo conectar con alguno de los sensores"); // ERROR DE CONEXIÓN
    Serial.println(vsensor1); // VERIFICAR CONEXIÓN DEL SENSOR 1
    Serial.println(vsensor2); // VERIFICAR CONEXIÓN DEL SENSOR 2
    Serial.println(vsensor3); // VERIFICAR CONEXIÓN DEL SENSOR 3 
    for (;;) 
    { // BUCLE INFINITO
        digitalWrite(LED_PLACA, HIGH); // ENCENDER LED
        delay(500);
        digitalWrite(LED_PLACA, LOW); // APAGAR LED
        delay(500);
    }
}


void run_inference()
{
    sensors_event_t a, g, temp; // VARIABLES DE EVENTOS DE LOS SENSORES
    for (int i = 0; i < 42; i++) 
 // 42 = 7 * 3 * 2 (7 muestras, 3 ejes, 2 sensores)
    {
        for (int a = 1; a <= 3; a++) 
        // 3 sensores
        {
            obtenerSensores2(a); // OBTENER DATOS DE LOS SENSORES
            tflInputTensor->data.f[i * 3 + 0] = (ax + 8.0) / 16.0; // NORMALIZAR DATOS
            tflInputTensor->data.f[i * 3 + 1] = (ay + 8.0) / 16.0; // NORMALIZAR DATOS
            tflInputTensor->data.f[i * 3 + 2] = (az + 8.0) / 16.0; // NORMALIZAR DATOS
        }
    }

    TfLiteStatus invokeStatus = tflInterpreter->Invoke(); // INVOKE

    float salidaA = tflOutputTensor->data.f[0]; // OBTENER SALIDA
    float salidaB = tflOutputTensor->data.f[1]; // OBTENER SALIDA
    float salidaY = tflOutputTensor->data.f[3];    

    if (salidaA >= 0.70) 
    // 0.70 = 70% de certeza
    {
        Serial.print("Letra A"); // IMPRIMIR SALIDA
        Serial.print(salidaA); // IMPRIMIR SALIDA
    }
    else if (salidaB >= 0.70)
    // 0.70 = 70% de certeza
    {
        Serial.print("Letra B");
    }
    else if (salidaY >= 0.70)
    // 0.70 = 70% de certeza
    {
        Serial.print("Letra Y");
    }
    Serial.println();
}
void detectMovement() 
{
    // OBTENER DATOS DE LOS SENSORES
    run_inference(); // CORRER INFERENCIA
}

void iniciarlizarPines()
{
// INICIALIZACIÓN DE PINES
    pinMode(LED_PLACA, OUTPUT); // LED PLACA
    pinMode(SENSOR_PULGAR, OUTPUT); // SENSOR PULGAR
    digitalWrite(SENSOR_PULGAR, HIGH); // SENSOR PULGAR
    pinMode(25, OUTPUT); // SENSOR PULGAR
    I2Cuno.begin(SDA_1, SCL_1, 40000UL); // INCIIALIZACIÓN DE BUS 1
    I2Cdos.begin(SDA_2, SCL_2, 40000UL); // INICIALIZACIÓN DE BUS 2
    Serial.begin(115200); // INICIALIZACIÓN DE MONITOR SERIAL
}

void inicializarTensorFlow()
{
    tflModel = tflite::GetModel(model); // CARGAR MODELO
    if (tflModel->version() != TFLITE_SCHEMA_VERSION) 
    // VERIFICAR VERSIÓN DEL MODELO
    {

        Serial.println("Model schema mismatch!");
        while (1)
            ;
    }

    // Create an interpreter to run the model
    tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter); // CREAR INTERPRETE

    // Allocate memory for the model's input and output tensors
    tflInterpreter->AllocateTensors(); // ASIGNAR MEMORIA
 
    // Get pointers for the model's input and output tensors
    tflInputTensor = tflInterpreter->input(0); // ENTRADA
    tflOutputTensor = tflInterpreter->output(0); // SALIDA
}

void inicializarPuertosI2C()
{
    // APERTURA DE MONITOR SERIAL.
    while (!Serial)
        ;
    vsensor2 = sensores[1].begin(0x68, &I2Cuno); // INICIALIZACIÓN DE SENSORES.
    vsensor3 = sensores[2].begin(0x69, &I2Cdos); // INICIALIZACIÓN DE SENSORES.
    vsensor4 = sensores[3].begin(0x68, &I2Cdos); // INICIALIZACIÓN DE SENSORES.
    if (!vsensor2 || !vsensor3 || !vsensor4) // INICIO DE SENSORES.
    {
        err_Conexion(); // FUNCIÓN DE ERROR DE CONEXIÓN.
    }
}
/* FUNCIÓN DE INICIO DE LA PLACA */
void setup() 
// FUNCIÓN DE INICIO DE LA PLACA
{
    // INICIALIZACIÓN DE PINES
    iniciarlizarPines(); // FUNCIÓN QUE INICIALIZA LOS PINES DE LA PLACA
    inicializarPuertosI2C(); // FUNCIÓN QUE INICIALIZA LOS PUERTOS I2C
    incializarSensores(); // FUNCIÓN QUE INICIALIZA LOS PARAMETROS DE SENSORES, COMO LOS GRADOS DE MOVIMIENTO ETC.
    inicializarTensorFlow(); // FUNCIÓN QUE INICIALIZA TENSORFLOW
    digitalWrite(LED_PLACA, HIGH); // ENCENDER LED
   
}

/* FUNCIÓN EN CONSTANTE REPETICIÓN */
void loop()
{
    detectMovement(); // FUNCIÓN QUE DETECTA EL MOVIMIENTO
}
