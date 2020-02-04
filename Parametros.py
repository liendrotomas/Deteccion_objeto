# este codigoo es para la determinacion de los parametros.

# HSV esto a mano
# tengo que normalizar los colores obtenidos del GIMP para tener
# el h en el rango de 0 a 180,
#    s en el rango de 0 a 255,
#    v en el rango de 0 a 255
#
# l_h = [round(250/360*179), round(205/360*180), round(120/360*180)]
# l_s = [round(15/100*255),  round(30/100*255),  round(30/100*255)]
# l_v = [round(0/100*255),  round(0/100*255),  round(50/100*255)]
# u_h = [round(360/360*179), round(243/360*180), round(142/255*180)]
# u_s = [round(100/100*255),  round(100/100*255),  round(100/100*255)]
# u_v = [round(100/100*255), round(100/100*255), round(100/100*255)]
# lower_red_2 = 0
# upper_red_2 = 20
# print(l_h,l_s,l_v,u_h,u_s,u_v)
#

l_h = [round(300/360*180), round(205/360*180), round(110/360*180)]
l_s = [round(15/100*255),  round(30/100*255),  round(45/100*255)]
l_v = [round(50/100*255),  round(20/100*255),  round(40/100*255)]
u_h = [round(360/360*180), round(243/360*180), round(170/255*180)]
u_s = [round(100/100*255),  round(100/100*255),  round(100/100*255)]
u_v = [round(100/100*255), round(100/100*255), round(100/100*255)]
lower_red_2 = 0
upper_red_2 = 30
# print(l_h,l_s,l_v,u_h,u_s,u_v)