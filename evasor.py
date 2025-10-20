"""
Algoritmo para evadir obstáculos durante la recolección de café.
Se detiene cuando llega a la zona de cultivo.
Sistema de navegación basado en conocimiento previo del terreno y uso de sensores.
"""
from ev3dev2.auto import *
from time import perf_counter, sleep
import math
"""
Inicialización de motores y sensores
"""
motor_izq = LargeMotor(OUTPUT_A)
motor_der = LargeMotor(OUTPUT_D)
#motores del brazo y garra (no usados en este módulo)
#motor_3 = LargeMotor(OUTPUT_C)
#motor_4 = LargeMotor(OUTPUT_D)
#sensor de color frontal para detectar la línea
ojo_frente = ColorSensor('in1')
ojo_frente.mode = 'COL-REFLECT'
#sensor ultrasónico para detectar obstáculos delante del robot
ojo_ultra = UltrasonicSensor('in2')
#configuramos el sensor ultrasónico para que use el modo de distancia en cm
ojo_ultra.mode = 'US-DIST-CM'
#sensor de color para detectar el color del café, montado en la garra
ojo_color = ColorSensor('in3')
#configuramos el sensor de color para que use el modo de color
ojo_color.mode = 'COL-COLOR'
#giroscopio
giroscopio = GyroSensor('in4')
giroscopio.mode = 'GYRO-ANG'
giroscopio.calibrate()
time.sleep(1)
arbol_a_recoger = 0 # 0,1,2

VEL_ALTA = 80
VEL_MEDIA = 30
VEL_REVERSA_MEDIA = -30
VEL_NEUTRA = 0
VEL_REVERSA = -80
PELOTAS = 0

def girar_izquierda():
    """Gira el robot hacia la izquierda."""
    print("Girando a la izquierda...")
    # 1. Reiniciar el ángulo del giroscopio a 0
    giroscopio.reset()
    motor_izq.run_forever(speed_sp=VEL_ALTA)
    motor_der.run_forever(speed_sp=VEL_REVERSA)
    giroscopio.wait_until_angle_changed_by(90)
    sleep(0.1)
    motor_izq.stop()
    motor_der.stop()

def girar_derecha():
    """Gira el robot hacia la derecha."""
    print("Girando a la derecha...")
    # 1. Reiniciar el ángulo del giroscopio a 0
    giroscopio.reset()
    motor_izq.run_forever(speed_sp=VEL_REVERSA)
    motor_der.run_forever(speed_sp=VEL_ALTA)
    giroscopio.wait_until_angle_changed_by(90)
    sleep(0.1)
    motor_izq.stop()
    motor_der.stop()

def avanza(): #funcion principal de avance 
    """Avanza hasta encontrar un obstáculo o la zona de recolección/deposito."""
    while ojo_ultra.distance_centimeters > 15 and ojo_frente.value() > 30:
    # Avanzar mientras no haya obstáculos cercanos o llegues a la línea de recoleccion
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
    #algo se detecto, procedo a detenerme
    motor_izq.stop()   
    motor_der.stop()
    #evadir obstaculo o llegamos a la zona de recoleccion
    if ojo_ultra.distance_centimeters <= 15:#obstaculo
        evadir_obstaculo()
    else: #llegue a la zona de recoleccion
        print("Zona alcanzada.")
        motor_izq.stop()
        motor_der.stop()
        sleep(0.5)
    if PELOTAS == 0: #Si no tengo pelotas es que voy a recoger
        posiciona_en_recoleccion() #Procede a recolectar
    else:
        posiciona_en_deposito() #EOC deposita las pelotas

def avanza_dist(distancia_cm):
    """Avanza una distancia específica en centímetros."""
    # Calcular el número de grados que deben girar los motores
    # Circunferencia de la rueda = 2 * pi * radio
    radio_rueda_cm = 2.8  # Radio de la rueda en cm (ajustar según el robot)
    circunferencia_cm = 2 * math.pi * radio_rueda_cm
    grados_a_girar = (distancia_cm / circunferencia_cm) * 360
    # Reiniciar los contadores de tacho
    motor_izq.reset()
    motor_der.reset()
    # Configurar los motores para avanzar la distancia deseada
    motor_izq.run_to_rel_pos(position_sp=grados_a_girar, speed_sp=VEL_ALTA, stop_action="brake")
    motor_der.run_to_rel_pos(position_sp=grados_a_girar, speed_sp=VEL_ALTA, stop_action="brake")
    # Esperar a que ambos motores terminen
    motor_izq.wait_while('running')
    motor_der.wait_while('running')
    sleep(0.5)

def evadir_obstaculo():  
    """
    Función para evadir obstáculos detectados por el sensor ultrasónico.
    El robot se detiene, evalúa la situación y realiza maniobras para evitar el obstáculo.
    """
    print("Iniciando maniobra de evasión de obstáculo...")
    
    sleep(0.5)  # Pequeña pausa para estabilizar
    # Girar a la izquierda para evadir el obstáculo
    girar_izquierda()
    sleep(0.5)
    #Avanzar todo a la izquierda
    while ojo_frente.value() > 30:  #mientras no detecte la linea
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
    #avanzo hasta detectar la linea
    motor_izq.stop()
    motor_der.stop()
    #ahora giro a la derecha y reviso
    girar_derecha()
    sleep(0.5)
    if ojo_ultra.distance_centimeters > 15: #si esta libre
        avanza()
    else: #2 - si no esta libre, giro a la derecha y avanzo 1 slot
        girar_derecha()
        sleep(0.5)
        avanza_dist(72)#avanza un slot de 72 cm
        girar_izquierda()
        sleep(0.5)
        if ojo_ultra.distance_centimeters > 15: #si esta libre
            avanza()
        else: #3 - si no esta libre, giro a la derecha y avanzo 1 slot
            girar_derecha()
            sleep(0.5)
            avanza_dist(72)#avanza un slot de 72 cm
            girar_izquierda()
            sleep(0.5)
            avanza() #avanza sin importar que haya, ya que es la ultima opcion

def calibra_angulo():
        """ajusta el angulo del robot
            asume que esta enfrente del arbol a 20cm"""
        #no sabemos si estamos muy a la izq o a la der
        #revisamos desde 45g la izq
        giroscopio.reset()
        motor_izq.run_forever(speed_sp=VEL_REVERSA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
        giroscopio.wait_until_angle_changed_by(45)
        #escaneamos hacia la derecha hasta encontrar la dist correcta
        while ojo_ultra.distance_centimeters < 19 or ojo_ultra.distance_centimeters >21:
            motor_izq.run_forever(speed_sp=VEL_MEDIA)
            motor_der.run_forever(speed_sp=VEL_REVERSA_MEDIA)
        motor_izq.stop()
        motor_der.stop()

def posiciona_en_recoleccion():
    """Posiciona el robot en la zona de recolección.
        nos colocamos en la esquina izquierda de la zona de recolección"""
    print("Posicionando en la zona de recolección...")
    # Aquí se pueden agregar las maniobras necesarias para posicionar el robot
    # en la zona de recolección, si es necesario.
    girar_izquierda()
    while ojo_frente.value() > 30:
    # Avanzar mientras no se llegue a la linea de limite
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
    #llegue, procedo a detenerme
    motor_izq.stop()   
    motor_der.stop()
    girar_derecha() #queda el robot mirando enfrente
    pos_arbol() #nos vamos al arbol que toque recoger

def deposita():
    print("deposita")

def sig_zona_depo(): #Reposiciona zona de deposito
    girar_izquierda()
    avanza_dist(70)
    girar_derecha()
    pos_zona_maduros()

def pos_zona_maduros():
    #buscamos la zona de deposito
    if ojo_ultra.distance_centimeters <= 15 and ojo_color.color == 5: #si es la caja para maduros(roja)
        deposita()  #deja todos los maduros
        girar_derecha() #date la vuelta
        girar_derecha()
        avanza() #otra tanda de recoleccion
    else: #no hay nada o es la caja incorrecta ve a la siguiente pos.
        sig_zona_depo()

def posiciona_en_deposito():
    """Posiciona el robot en la zona de deposito.
        nos colocamos en la esquina izquierda de la zona de deposito"""
    print("Posicionando en la zona de deposito...") 
    girar_derecha()
    while ojo_frente.value() > 30:
    # Avanzar mientras no se llegue a la linea de limite
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
    #llegue, procedo a detenerme
    motor_izq.stop()   
    motor_der.stop()
    girar_izquierda() #queda el robot mirando enfrente
    pos_zona_maduros() #nos vamos a buscar la zona de maduros


"""requiero brazo final para las llamadas"""
def recoger_alto():
    print("recoge")
def recoger_medio():
    print("recoge")
def recoger_bajo():
    print("recoge")

def pos_recoger(): #se coloca enfrente de la primer pelota
    girar_izquierda()
    avanza_dist(26)
    girar_derecha()


def pos_arbol(): #nos deja el robot ajustado en medio de cada arbol
    if arbol_a_recoger == 0:
        #ahora giramos 90 y nos colocamon enfrente del primer arbol
        girar_derecha()
        avanza_dist(49) #asume que el robot esta a un margen de 10cm del limite izq
        girar_izquierda()
        #estamos colocados enfrente del arbol, ajustamos posibles desviaciones
        #asumo margen de 10cm desde la linea al robot mas 10 de la linea al arbol
        if ojo_ultra.distance_centimeters > 18.5 and ojo_ultra.distance_centimeters < 21.5:
            print("OK")
        else:
            calibra_angulo()
    elif arbol_a_recoger == 1:
        #ahora giramos 90 y nos colocamon enfrente del segundo arbol
        girar_derecha()
        avanza_dist(166.4) #asume que el robot esta a un margen de 10cm del limite izq
        girar_izquierda()
        #estamos colocados enfrente del arbol, ajustamos posibles desviaciones
        #asumo margen de 10cm desde la linea al robot mas 10 de la linea al arbol
        if ojo_ultra.distance_centimeters > 18.5 and ojo_ultra.distance_centimeters < 21.5:
            print("OK")
        else:
            calibra_angulo()
    elif arbol_a_recoger == 2:
        #ahora giramos 90 y nos colocamon enfrente del tercer arbol
        girar_derecha()
        avanza_dist(283.8) #asume que el robot esta a un margen de 10cm del limite izq
        girar_izquierda()
        #estamos colocados enfrente del arbol, ajustamos posibles desviaciones
        #asumo margen de 10cm desde la linea al robot mas 10 de la linea al arbol
        if ojo_ultra.distance_centimeters > 18.5 and ojo_ultra.distance_centimeters < 21.5:
            print("OK")
        else:
            calibra_angulo()
    #ya estamos enfrente de algun arbol, pasamos a colocarnos en la semilla 1
    pos_recoger()

def run():
    avanza_dist(30) #avanza un poco para saltar la primer linea negra
    avanza() #comienza el avance normal
        