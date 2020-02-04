# este codigo permite tomar fotos
import cv2
import time
from DeteccionObj_detob import DeteccionObj_detob
def take_ph():
    cap = cv2.VideoCapture(0)

    cap.set(3, 320)
    cap.set(4, 240)
    i = 1
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    FILE_OUTPUT = "IM_detob.jpg"
    while i < 10 :
        ret, frame = cap.read()
        i += 1
    print("Imagen tomada")
    print("Detectando obstáculos...")

    # result, frame = cv2.imencode('.jpg', frame, encode_param)
    # frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    # cv2.imwrite(FILE_OUTPUT, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    cap.release()
    cv2.destroyAllWindows()

    grilla2 = DeteccionObj_detob(frame, False, 0, 0)


    return grilla2
