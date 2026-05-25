# importando funções necessárias para a programação
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# definindo portas de motores e sensores e a orientação do hub
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)
ultrassonic_sensor = UltrasonicSensor(Port.E)
force_sensor = ForceSensor(Port.F)
hub = PrimeHub()

while force_sensor.pressed() != True: # espera eu apertar o sensor de força pra iniciar
    wait(1)

# definindo variáveis
walking_speed = 0
white_before_green = True
obstacle = 0
left_ref = left_sensor.reflection()
right_ref = right_sensor.reflection()
left_white = left_sensor.color() == Color.WHITE
right_white = right_sensor.color() == Color.WHITE
left_green = left_sensor.color() == Color.GREEN
right_green = right_sensor.color() == Color.GREEN

lista_sensores = [
    left_ref,
    right_ref,
    left_white,
    right_white,
    left_green,
    right_green,
]

def line_bw(): # função para seguir preto e branco
    if hub.imu.rotation(Axis.Y) > 8:  
        walking_speed = 20 # descida da rampa
    elif hub.imu.rotation(Axis.Y) < -8: 
        walking_speed = 40 # subida da rampa
    else:
        walking_speed = 55 # velocidade normal

    if left_ref < 30 and right_ref > 80: # mtpreto branco
        if left_sensor.color() == Color.BLACK:
            left_motor.dc(55)
            right_motor.run_angle(55, 216)
            while right_sensor.color() != Color.BLACK: # gira até o sensor direito ver preto
                left_motor.dc(-30)
                right_motor.dc(30)
            left_motor.dc(70)
            right_motor.dc(-70)
            wait(350)
            left_motor.dc(-55)
            right_motor.run_angle(-55, 108)

    elif left_ref > 80 and right_ref < 30: # branco mtpreto
        if right_sensor.color() == Color.BLACK:
            left_motor.dc(55)
            right_motor.run_angle(55, 216)
            while left_sensor.color() != Color.BLACK: # gira até o sensor esquerdo ver preto
                left_motor.dc(30)
                right_motor.dc(-30)
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(350)
            left_motor.dc(-55)
            right_motor.run_angle(-55, 108)

    elif left_ref > 80 and right_ref > 80: # branco branco
        left_motor.dc(walking_speed)
        right_motor.dc(walking_speed)

    elif left_ref < 80 and right_ref > 80: # preto branco
        left_motor.dc(-15)
        right_motor.dc(30)

    elif left_ref > 80 and right_ref < 80: # branco preto
        left_motor.dc(30)
        right_motor.dc(-15)

    elif left_ref < 65 and right_ref < 65: # preto preto
        left_motor.dc(25)
        right_motor.dc(25)

def green(): # função para fazer a verificação do verde e os três possíveis casos de verde
    left_motor.dc(-55)
    right_motor.run_angle(-55, 108)
    if left_white and right_white:
        left_motor.dc(55)
        right_motor.run_angle(55, 144)
        if left_green and right_white:
            left_motor.dc(55)
            right_motor.run_angle(55, 288)
    else:
        left_motor.dc(55)
        right_motor.run_angle(55, 504)

while True: # loop principal
    if force_sensor.pressed() == True: # se eu apertar o sensor de força, não faz nada
        wait(1)
    else:
        lista_sensores
        line_bw()
        if left_green == True:
            left_motor.dc(55)
            right_motor.run_angle(55, 36)
            if left_green == True:
                green()

        elif right_green == True:
            left_motor.dc(55)
            right_motor.run_angle(55, 36)
            if right_green == True:
                green()
