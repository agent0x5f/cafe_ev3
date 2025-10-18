"""
Algoritmo para evadir obstáculos durante la recolección de café.
Se detiene cuando llega a la zona de cultivo.
Sistema de navegación basado en conocimiento previo del terreno y uso de sensores.
"""
from ev3dev2.auto import *
from time import perf_counter, sleep
import math

motor_1 = LargeMotor(OUTPUT_A)
motor_2 = LargeMotor(OUTPUT_D)
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
