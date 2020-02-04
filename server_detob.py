import socket
import struct
import pickle
import zlib
import cv2
import os
# from Calibrate import Calibrate
from DeteccionObj import DeteccionObj
import threading
import time
import Parametros
from multiprocessing import Value

import math
import numpy as np

# para probar el codigo en la misma máquina usar de HOST a localhost
# Igual para el archivo client.py

# ESTE ARCHIVO ES EL QUE FUNCIONA COMO SERVER

# ref = Value('d', 0.0)
#
# def lee_de_teclado():
#     while 1:
#         ref.value = input("indicar referencia")
#         print(ref.value)



print("\033[H\033[J")  # esto limpia toda la consola

HOST ="" #
PORT = 9009 #

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

s.listen(10)
while 1:
    try:
        conn, addr = s.accept()
        break
    except KeyboardInterrupt:
        s.close()
        exit(0)


data_cal = conn.recv(bytes.__sizeof__(b'Cliente Listo '))
if data_cal == b'Cliente Listo ':
    conn.sendall("Conexion Establecida...\n".encode())
else:
    assert()

#
# conn.sendall("Preparing Camera...".encode())
# time.sleep(0.5)
# conn.sendall("Begin".encode())
# time.sleep(0.5)


flag = b'1'
while flag == b'1':
    flag = conn.recv(1024)
    struct.pack('?', flag)


#Ahora empieza a guardar una imagen en el archivo: IM_ESTADO
# la RPi deberia levantar esta imagen cada 1 seg y analizarla
# FILE_OUTPUT = "IM_ESTADO.jpg"
cap = cv2.VideoCapture(0)
time.sleep(0.5)


cap.set(3, 320)
cap.set(4, 240)


# if os.path.isfile(FILE_OUTPUT):
#     os.remove(FILE_OUTPUT)

img_counter = 0
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
m=1
Dt = 0.25  # un delay de 200 ms
# k = 0
# while os.path.isfile("Caract_planta_finales_si_{}".format(k)):
#     k+=1

# ang_file = open("Caract_planta_finales_si_{}".format(k), "w+") # aca voy a guardar el angulo y el tiempo para la etapa de caracterizacion
ts = time.time()
i = 0
k = 1

while m:
    try:
        # d = time.time()
        ret, frame = cap.read()
        result, frame = cv2.imencode('.jpg', frame, encode_param) #pone la imagen en formato jpg
        #    data = zlib.compress(pickle.dumps(frame, 0))
        data = pickle.dumps(frame, 0) # lo arma como un vector de 1 x length(data)
        size = len(data)

        # print("{}: {}".format(img_counter, size))
        conn.sendall(struct.pack(">L", size) + data) #struct.pack(">L", size) es un encabezado que coloca antes del mensaje para
                                                    # que el cliente sepa el tamaño del mensaje
        # img_counter += 1



        frame = pickle.loads(data, fix_imports=True, encoding="bytes") # hace el proceso opuesto al de pickle.dumps para
                                                                    # tener frame en forma de matriz que pueda pasarselo al
                                                                    # Deteccion Obj.py
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)               # Pasa la imagen de bytes a una imagen en HSV (creo), no se bien para que hace eso
        x, y, angulo = DeteccionObj(frame, True,0,0)                # detecta el ángulo, el argumento True es para que lo muestre en pantalla
        # print(x, y, angulo)

        # ang_file.write("%d\n" % (angulo))  # tiempo transcurrido, angulo, velocidad.
        # delay = (i + 1) * Dt - (time.time() - ts)
        # if (delay)<0:
        #     time.sleep(0.05)
        #     pass
        # else:
        #     time.sleep(delay) # duermo el sistema un tiempo Dt
        # i+=1
        #
        # while flag2:
        #     print("IM HERE")
        #     flag2 = conn.recv(1024)
        time.sleep(0.1)                        # este tiempo se lo puse para que el servidor entregue imagenes más lento
                                                # que la capacidad de procesar la imagen por el cliente. de esta manera espero
                                                # que no se acumulen imagenes en el buffer. Probe hacer esto mucho mas rapido
                                                # pero el cliente queda congelado muy rápido, por lo que supuse que era un
                                                # problema de exceso de info en el buffer.

        # print("tiempo:" +str(time.time() - d))


    except KeyboardInterrupt:
        m=0
        s.close()
        conn.close()
        # ang_file.close()



