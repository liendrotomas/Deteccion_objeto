from take_photo_detob import take_ph
import os
from Grilla_detob import m, n

def generar_solucion(obj_i1,obj_j1,obj_i2,obj_j2):
    if obj_i1 > n-1 or obj_i2 > n-1:
        print("No se puede encontrar solución, se exceden los limites de la grilla.")
        return 0
    if obj_j1 > m-1 or obj_j2 > m-1:
        print("No se puede encontrar solución, se exceden los limites de la grilla.")
        return 0
    grilla2 = take_ph()
    mtsa_file = open("grilla_detob.txt", "+w")
    mtsa_file.write("const M = 3\n"
                    "const N = 4\n\n"
                    "set Controlables = {go[i:0..M][j:0..N]}\n"
                    "set Alphabet = {Controlables,arrived[i:0..M][j:0..N]}\n\n"
                    "Matriz = Elem[0][0],\n"
                    "Elem[i:0..M][j:0..N] = (when (i<M) go[i+1][j] -> arrived[i+1][j] -> Elem[i+1][j] |\n"
                    "						when (i>0) go[i-1][j] -> arrived[i-1][j] -> Elem[i-1][j] |\n"
                    "						when (j<N) go[i][j+1] -> arrived[i][j+1] -> Elem[i][j+1] |\n"
                    "						when (j>0) go[i][j-1] -> arrived[i][j-1] -> Elem[i][j-1]).\n\n"
                    "Robot = ({Controlables} -> Robot).\n"
                    "||Environment = (Matriz || Robot).\n\n" +
                    "fluent Arrived{}{}".format(obj_i1, obj_j1) + " = <arrived[%i][%i] ,{go[i:0..M][j:0..N]}\{go[%i][%i]}> initially 0\n"%(obj_i1,obj_j1,obj_i1,obj_j1)+

                    "fluent Arrived{}{}".format(obj_i2, obj_j2) + " = <arrived[%i][%i] ,{go[i:0..M][j:0..N]}\{go[%i][%i]}> initially 0\n\n"%(obj_i2,obj_j2,obj_i2,obj_j2)
                    )
    i_list = []
    j_list = []
    print(grilla2)
    for i in range(0, len(grilla2)):
        for j in range(0, len(grilla2[0])):
            if grilla2[i][j] == 1:
                mtsa_file.write("ltl_property Never{}{}".format(i, j) + " =  []!(arrived[%i][%i])" % (i, j) + "\n")
                i_list.append(i)
                j_list.append(j)
                if i_list[-1] == obj_i1 and j_list[-1] == obj_j1:
                    print("No se puede encontrar solucion.")
                    return Exception
                if i_list[-1] == obj_i2 and j_list[-1] == obj_j2:
                    print("No se puede encontrar solucion.")
                    return Exception
    mtsa_file.write("controllerSpec ControlSpec = {\n"
                    "		safety = {")
    while len(i_list):
        mtsa_file.write("Never{}{}".format(i_list.pop(), j_list.pop()) + ", ")

    mtsa_file.write("}\n"
                    "        assumption = {}\n" +
                    "        liveness = {"+"Arrived{}{}, Arrived{}{}".format(obj_i1, obj_j1,obj_i2, obj_j2) + "}\n"
                    "        controllable = {Controlables}\n"
                    "}\n\n"
                    "controller ||Controller = Environment~{ControlSpec}.\n"
                    "minimal ||UPDATE_CONTROLLER = (Environment || Controller).\n"
                    "    ")
    mtsa_file.close()

    os.system("java -jar multiCore.jar --file grilla_detob.txt")
    print("Solución generada")

    return 0
