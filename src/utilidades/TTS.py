
import serial
from gtts import gTTS
import playsound
PuertoSerie = serial.Serial('/dev/cu.usbserial-0001', 115200)
# Creamos un buble sin fin
while True:
    # leemos hasta que encontarmos el final de linea
    sArduino = PuertoSerie.readline()
    sArduino1 = sArduino.decode('latin1')
    texto = sArduino1
    print(sArduino1)
    