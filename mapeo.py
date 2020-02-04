from Grilla_detob import grilla
def mapeo(i,j):
    h = -1
    k = -1
    # print(len(grilla))
    #
    # print(len(grilla[0]))
    n_flag = True
    for n in range(0, len(grilla)):
        # if j < (grilla[0][0])[1]+30:
        #     k = 0
        #     break
        if (grilla[n][0])[1]-31 < i and i < (grilla[n][0])[1]+31:
            k = n
            n_flag = False
            break
    if n_flag:
        if i < 240:
            k = n

    m_flag = True
    for m in range(0, len(grilla[0])):
        # print(m)

        # if i < (grilla[0][0])[0]+30:
        #     h = 0
        #     break
        print((grilla[0][m]))
        if (grilla[0][m])[0]-31 < j and j < (grilla[0][m])[0]+31:
            h = m
            m_flag = False
            break
    if m_flag:
        if j < 320:
            h = m
    if h == -1 or k == -1:
        print("Coordenadas fuera de rango")
        return Exception

    return k,h