#ESte es el codigo de deteccion de angulo

import numpy as np
import cv2
from Parametros import l_s, l_h, l_v, u_v, u_h, u_s,upper_red_2 , lower_red_2
from math import atan2
from Grilla_detob import grilla, limits_x, limits_y
from mapeo import mapeo
from Grilla_detob import n, m

def ShowBlue2(im_blue):
    COLORS = ["BLUE"]  # , "YELLOW"], "GREEN"]

    hsv = cv2.cvtColor(im_blue, cv2.COLOR_BGR2HSV)
    # defino los límites HSV de la máscara
    lower_blue = np.array([l_h[1], l_s[1], l_v[1]])#(hsv_blue_l)#
    upper_blue = np.array([u_h[1], u_s[1], u_v[1]])#(hsv_blue_u)#
    # creo máscara
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    # Para evitar que se detecten puntos muy chicos, se pone un tamaño mínimo por debajo del cual
    # se ignoran los puntos
    kernel = np.ones((2, 2), np.uint8)
    # print(len(mask_blue[0]))
    mask_blue2 = cv2.erode(mask_blue, kernel)
    mask_blue = cv2.morphologyEx(mask_blue2, cv2.MORPH_OPEN, kernel)
    for i in range(0, len(grilla)):
        for j in range(0, len(grilla[1])):
            color = (100, 100, 100)
            cv2.circle(mask_blue, (grilla[i][j]), 1, color, thickness=2, lineType=0)

    for i in range(0, len(limits_x[1])):
        cv2.line(mask_blue, (limits_x[0][i]), limits_x[1][i], (0, 0, 255), thickness=1, lineType=0)

    for i in range(0, len(limits_y[1])):
        cv2.line(mask_blue, (limits_y[0][i]), limits_y[1][i], (0, 0, 255), thickness=1, lineType=0)

    cv2.imwrite("mask.jpg", mask_blue, [cv2.IMWRITE_JPEG_QUALITY, 90])

    # print(mask_blue[165][249])
    # Deteccion de contornos
    # [_,contours_blue,_] = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # cX = 0
    # cY = 0
    # color_ctr = (255, 0, 0)
    # max_area = 0
    # best_cnt = 0

    # n = 4  # numero de casillas en sentido vertical
    # m = 5  # numero de casillas en sentido horizontal
    #

    grilla_aux = []
    for i in range(n):
        grilla_aux.append([0] * m)
    print(grilla_aux)
    for a in range(0, len(mask_blue)):
        for b in range(0, len(mask_blue[0])):
            if (mask_blue[a][b] > 200):
                i_index, j_index = mapeo(a,b)
                # print(i_index, j_index)
                grilla_aux[i_index][j_index] = 1
    return grilla_aux

def DeteccionObj_detob(frame, show, h, k):
    frame = cv2.imread("IM_detob.jpg")
    # COLORS = ["BLUE"]#"RED", "BLUE", "GREEN"]  # ,"YELLOW"]
    frame = cv2.blur(frame, (3, 3))

    grilla_aux2 = ShowBlue(frame)

    return grilla_aux2


def DeteccionObj(frame, show, h, k):
    pos_y = 0
    pos_x = 0
    L=9999 # en caso de que no se detecte un angulo se quiere una salida que indique un error

    COLORS = ["RED", "BLUE", "GREEN"]  # ,"YELLOW"]
    frame = cv2.blur(frame, (3, 3))
    # aca realizo la mascara de cada color. las funciones Show"color" estan mas abajo y devuelven el centro de masa
    # (coordenadas x e y)
    # de cada area contenida en el contorno.
    for color in COLORS:
        if color == "BLUE":
            # p1x, p1y = ShowBlue(frame)
            pass
        elif color == "RED":
            p2x, p2y = ShowRed(frame)
        elif color == "GREEN":
            p3x, p3y = ShowGreen(frame)

    # acá viene la parte de la deteccion del ángulo
    if (p2x - p3x) == 0: # esto es solo para que este definida la tangente
        pass
    else:
        cv2.line(frame, (p2x, p2y), (p2x+50, p2y), (0, 0, 255), thickness=5, lineType=0)     #linea horizontal
        cv2.line(frame, (p2x, p2y), (p3x, p3y), (0, 255, 0), thickness=5, lineType=0)     #linea del robot, va del rojo al verde
        # cv2.imwrite("VEL", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        # ang1 = atan2((p3y), (p1x - p2x)) # angulo con la horizontal
        ang1 = 0 # el ángulo de la horizontal
        ang2 = atan2((p3y - p2y), (p3x - p2x))
        L = int(np.round(np.rad2deg(ang2-ang1)))
        pos_x = int(round(0.5*(p2x + p3x)))
        pos_y = int(round(0.5*(p2y + p3y)))

        for i in range(0, len(grilla)):
            for j in range(0, len(grilla[1])):
                color = (250 , 0, 0)
                if i == h and j == k:
                    color = (0, 255, 255)

                cv2.circle(frame, (grilla[i][j]), 1, color, thickness=2, lineType=0)

        for i in range(0, len(limits_x[1])):
            cv2.line(frame, (limits_x[0][i]), limits_x[1][i], (0, 0, 255), thickness=1, lineType=0)

        for i in range(0, len(limits_y[1])):
            cv2.line(frame, (limits_y[0][i]), limits_y[1][i], (0, 0, 255), thickness=1, lineType=0)

        # print(pos_x, pos_y)
        if show:
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
    # if L == 9999:
        # return '\0'
    # else:
    return pos_x, pos_y, L

def ShowRed(im_red):
    COLORS = ["RED"]  # , "YELLOW"], "GREEN"]
    hsv = cv2.cvtColor(im_red, cv2.COLOR_BGR2HSV)
    # defino los límites HSV de la máscara
    lower_red = np.array([l_h[0], l_s[0], l_v[0]])#(hsv_red_l) #
    upper_red = np.array([u_h[0], u_s[0], u_v[0]])#(hsv_red_u)#
    # creo máscara
    # creo máscara
    mask_red_1 = cv2.inRange(hsv, lower_red, upper_red)
    mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask_red = mask_red_1 + mask_red_2
    # mask_red = cv2.inRange(hsv, lower_red, upper_red)
    # Para evitar que se detecten puntos muy chicos, se pone un tamaño mínimo por debajo del cual
    # se ignoran los puntos
    kernel = np.ones((2, 2), np.uint8)
    mask_red = cv2.erode(mask_red, kernel)
    # Deteccion de contornos
    [_, contours_red, _] = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cX = 0
    cY = 0
    # color_ctr = (255, 0, 0)
    max_area = 0
    best_cnt = 0

    for cnt in contours_red:
        area = cv2.contourArea(cnt)

        if area > max_area:
            max_area = area
            best_cnt = cnt
    # compute the center of the contour
    M = cv2.moments(best_cnt)
    if M["m00"] == 0:
        pass
    else:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        cv2.circle(im_red, (cX, cY), 3, (255, 255, 255), -1)


    # cv2.imshow("BLUE", im_blue)
    # cv2.imshow("Mask_RED", mask_red)
    # cv2.waitKey(1)

    return cX, cY

def ShowGreen(im_green):
    COLORS = ["GREEN"]  # , "YELLOW"], "GREEN"]

    hsv = cv2.cvtColor(im_green, cv2.COLOR_BGR2HSV)
    # defino los límites HSV de la máscara
    lower_green = np.array([l_h[2], l_s[2], l_v[2]])#(hsv_green_l)#
    upper_green = np.array([u_h[2], u_s[2], u_v[2]])#(hsv_green_u)#
    # creo máscara
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    # Para evitar que se detecten puntos muy chicos, se pone un tamaño mínimo por debajo del cual
    # se ignoran los puntos
    kernel = np.ones((2, 2), np.uint8)
    mask_green = cv2.erode(mask_green, kernel)
    # Deteccion de contornos
    [_,contours_green, _] = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cX = 0
    cY = 0
    # color_ctr = (255, 0, 0)
    max_area = 0
    best_cnt = 0

    for cnt in contours_green:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt
    # compute the center of the contour
    M = cv2.moments(best_cnt)
    if M["m00"] == 0:
        pass
    else:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        cv2.circle(im_green, (cX, cY), 3, (255, 255, 255), -1)


    # cv2.imshow("BLUE", im_blue)
    # cv2.imshow("Mask_GREEN", mask_green)
    # cv2.waitKey(1)

    return cX, cY


