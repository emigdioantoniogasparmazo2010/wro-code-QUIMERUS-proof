# wro-code-QUIMERUS-proof
official code of the QUIMERUS team (test code)

#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# =========================
# CONFIGURACIÓN
# =========================

ev3 = EV3Brick()

motor_izquierdo = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motor_derecho = Motor(Port.C)
pala = Motor(Port.D)

sensor_izq = ColorSensor(Port.S1)
sensor_der = ColorSensor(Port.S4)


robot = DriveBase(motor_izquierdo, motor_derecho, 48.7, 174.5)


robot.settings(
    straight_speed=600,
    straight_acceleration=650,
    turn_rate=140,
    turn_acceleration=200
)

# =========================
# CALIBRACIÓN
# =========================

BLANCO_IZQ = 100
NEGRO_IZQ = 18

BLANCO_DER = 100
NEGRO_DER = 33

UMBRAL_IZQ = (BLANCO_IZQ + NEGRO_IZQ) / 2
UMBRAL_DER = (BLANCO_DER + NEGRO_DER) / 2

UMBRAL_CRUCE_IZQ = 50
UMBRAL_CRUCE_DER = 55

KP = 0.6
KD = 1.4

KP_CRUCE = 1.2
KD_CRUCE = 0.7

VELOCIDAD_LINEA = 450
VELOCIDAD_CRUCE = 120


# =========================
# FUNCIONES BÁSICAS
# =========================

def avanzar(mm):
    
    robot.straight(mm)


def retroceder(mm):
    
    robot.straight(-mm)


def girar(grados):
    detener()
    wait(50)

    robot.turn(grados)
    wait(150)

def detener():
    robot.stop()


def esperar(ms):
    wait(ms)


def beep():
    ev3.speaker.beep()


# =========================
# PALA
# =========================

def subir_pala():
    pala.run_angle(200, -180)


def bajar_pala():
    pala.run_angle(300, 180)


# =========================
# SENSORES
# =========================

def leer_izq():
    return sensor_izq.reflection()


def leer_der():
    return sensor_der.reflection()


def ambos_en_negro():
    return leer_izq() < UMBRAL_CRUCE_IZQ and leer_der() < UMBRAL_CRUCE_DER


# =========================
# PID LÍNEA
# =========================

def seguir_linea_pid(tiempo_ms):
    error_anterior = 0
    tiempo = 0

    while tiempo < tiempo_ms:
        error = leer_izq() - leer_der()
        derivada = error - error_anterior

        correccion = (KP * error) + (KD * derivada)

        robot.drive(VELOCIDAD_LINEA, correccion)

        error_anterior = error
        wait(10)
        tiempo += 10

    detener()


# =========================
# CRUCES SUAVES
# =========================

def seguir_linea_hasta_cruces(numero_cruces):
    cruces = 0
    error_anterior = 0

    while cruces < numero_cruces:
        error = leer_izq() - leer_der()
        derivada = error - error_anterior

        correccion = (KP_CRUCE * error) + (KD_CRUCE * derivada)

        robot.drive(VELOCIDAD_CRUCE, correccion)

        if ambos_en_negro():
            cruces += 1
            beep()

            detener()
            wait(50)

            # salir recto del cruce
            robot.drive(70, 0)
            wait(300)

            error_anterior = 0

            # asegurar que ya salió del cruce
            while ambos_en_negro():
                robot.drive(70, 0)
                wait(10)

        error_anterior = error
        wait(10)

    detener()
    wait(150)


# =========================
# RUTA COMPETENCIA
# =========================

def ruta_competencia_1():

    # SALIDA INICIAL
   
    avanzar(130)
    girar(90)
    avanzar(200)
   

    # IR A MISIÓN 1
    seguir_linea_pid(1600)
    seguir_linea_hasta_cruces(1)
    girar(-88)
    esperar(1000)
    #1
    retroceder(60)
    bajar_pala()
    #esperar(50)
    avanzar(25)
    girar(97)
    esperar(1000)
    #2
    avanzar(650)
    #seguir_linea_pid(1280)
    girar(100)
    esperar(1000)
    #3
    retroceder(100)
    subir_pala()
    esperar(50)
    """
   # IR A MISION 2 y 3
    
    avanzar(450)
    retroceder(60)
    girar(-93)
    retroceder(140)
    bajar_pala()
    retroceder(780)
    avanzar(40)
    girar(45)
    retroceder(160)
    girar(-41)
    retroceder(400)
    girar(-15)
    retroceder(30)
    subir_pala()
    esperar(200)


    # IR A MISION 4 - BLANCO
    avanzar(20)
    seguir_linea_pid(2900)
    seguir_linea_hasta_cruces(1)
    girar(92)
    avanzar(48)
    girar(91)
    retroceder(278)
    bajar_pala()
    avanzar(275)
    #seguir_linea_hasta_cruces(1)
    girar(92)
    seguir_linea_pid(600)
    seguir_linea_hasta_cruces(1)
    girar(95)
    retroceder(325)
    girar(46.5)
    retroceder(200)
    subir_pala()
 
    
    # IR A MISION 5 VERDE
    avanzar(140)
    girar(-40)
    seguir_linea_hasta_cruces(1)
    girar(182)
    retroceder(280)
    bajar_pala()
    esperar(50)
    avanzar(30)
    seguir_linea_hasta_cruces(1)
    girar(183)
    retroceder(362)
    subir_pala()
    
    # IR A MISION 6 AMARILLO
    avanzar(80)
    seguir_linea_hasta_cruces(1)
    esperar(200)
    girar(-89.5)
    esperar(200)
    seguir_linea_hasta_cruces(1)
    girar(-96)
    retroceder(280)
    bajar_pala()
    esperar(200)

    #terminar mision 6  AMARILLO
    avanzar(273)
    girar(-91)
    avanzar(600)
    seguir_linea_hasta_cruces(1)
    girar(90)
    seguir_linea_hasta_cruces(2) 
    girar(-90)
    retroceder(110)
    subir_pala()

     # Ir a mision 7 AZUL
    avanzar(110)
    girar(-87.5)
    seguir_linea_pid(1200)
    
    seguir_linea_hasta_cruces(1)
    girar(-92)
    seguir_linea_hasta_cruces(1)
    girar(-97)
    esperar(200)
    retroceder(275)
    bajar_pala()
    esperar(200)
    

    #terminar mision 7  AZUL
    avanzar(265)
    
    girar(-92)
    seguir_linea_hasta_cruces(1)
    girar(90)
    seguir_linea_hasta_cruces(2)
    avanzar(550)
    seguir_linea_hasta_cruces(1)
    girar(88)
    seguir_linea_hasta_cruces(2)
    avanzar(125)
    girar(-80)

    retroceder(355)
    subir_pala()


    #seguir_linea_pid(2500)
    """
    detener()


# =========================
# PRUEBAS
# =========================

def ruta_prueba_pid():
    seguir_linea_pid(4000)


def ruta_prueba_cruces():
    seguir_linea_hasta_cruces(5)


# =========================
# MAIN
# =========================

def main():
    beep()
    wait(4000)
    beep()
    wait(1500)
    beep()
    wait(1000)
    beep()
    wait(500)

    ruta_competencia_1()


    beep()
    wait(500)
    beep()
    wait(500)
    beep()


main()
