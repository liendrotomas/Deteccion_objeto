#Tengo un espacio de 320x240 px.
# supongo que el robot requiere un area de 15 px de radio para realizar el giro.
# por lo que en primer lugar tengo que definir areas de 30x30 px en la imagen de la camara
#320/30 ≈ 10
# 240/30 = 8

#defino entonces que tengo n = 8 lineas y m = 10 columnas, por la forma de la imagen
# la esquina superior izquierda es el punto (0,0) y la esquina inferior derecha es la (320,240)

# la numeracion es natural. es decir el casillero 1 es el punto(15,15) el dos es el (30,15)
# def gril():
# import numpy as np
n = 4 # numero de casillas en sentido vertical
m = 5 # numero de casillas en sentido horizontal

size_x = 320
size_y = 240
#
limits_x = []
limits_y = []

grilla = []
for i in range(n):
    grilla.append([0] * m)

for i in range(2):
    limits_x.append([0] * (m+1))
for i in range(2):
    limits_y.append([0] * (n+1))

for i in range (0, n):
    for j in range(0, m):
        grilla[i][j] = (30 + 60 * j, 30 + 60 * i)

for j in range (0,m+1):
    limits_x[0][j] = (60 * j, 0)
    limits_x[1][j] = (60 * j, 240)
for j in range(0, n + 1):
    limits_y[0][j] = (0, 60 * j)
    limits_y[1][j] = (300, 60 * j)
        # print(grilla[i][j])
        # print (grilla)
# return grilla
# print((grilla[1][0])[0]+90)
# print(len(limits_x[1]))