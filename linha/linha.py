# importando funções necessárias para a programação
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# definindo portas de motores e sensores e a orientação do hub
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
mov_motors = DriveBase(left_motor, right_motor)
left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)
ultrassonic_sensor = UltrasonicSensor(Port.E)
force_sensor = ForceSensor(Port.F)
hub = PrimeHub()

while force_sensor.pressed() != True: # espera eu apertar o sensor de força pra iniciar
    wait(10)

# definindo variáveis
obstacle = 0

def line_bw(): # função para seguir preto e branco
    
    if hub.imu.rotation(Axis.Y) > 8:  
        walking_speed = 200 # descida da rampa
    elif hub.imu.rotation(Axis.Y) < -8: 
        walking_speed = 400 # subida da rampa
    else:
        walking_speed = 550 # velocidade normal

    if left_sensor.reflection() < 30 and right_sensor.reflection() > 40: # mtpreto branco
        if left_sensor.reflection() < 11:
            left_motor.speed(550)
            right_motor.run_angle(550, 216)
            while right_sensor.reflection() > 11: # gira até o sensor direito ver preto
                left_motor.speed(-300)
                right_motor.speed(300)
            left_motor.speed(700)
            right_motor.speed(-700)
            wait(350)
            left_motor.speed(-550)
            right_motor.run_angle(-550, 108)

    elif left_sensor.reflection() > 40 and right_sensor.reflection() < 30: # branco mtpreto
        if right_sensor.reflection() < 11:
            left_motor.speed(550)
            right_motor.run_angle(550, 216)
            while left_sensor.reflection() > 11: # gira até o sensor esquerdo ver preto
                left_motor.speed(300)
                right_motor.speed(-300)
            left_motor.speed(-700)
            right_motor.speed(700)
            wait(350)
            left_motor.speed(-550)
            right_motor.run_angle(-550, 108)

    elif left_sensor.reflection() > 40 and right_sensor.reflection() > 40: # branco branco
        left_motor.speed(walking_speed)
        right_motor.speed(walking_speed)

    elif left_sensor.reflection() < 40 and right_sensor.reflection() > 40: # preto branco
        left_motor.speed(-150)
        right_motor.speed(300)

    elif left_sensor.reflection() > 40 and right_sensor.reflection() < 40: # branco preto
        left_motor.speed(300)
        right_motor.speed(-150)

    elif left_sensor.reflection() < 30 and right_sensor.reflection() < 30: # preto preto
        left_motor.speed(250)
        right_motor.speed(250)

def green(): # função para fazer a verificação do verde e os três possíveis casos de verde
    left_motor.speed(-550)
    right_motor.run_angle(-550, 108)
    
    if left_sensor.color() == Color.WHITE and right_sensor.color() == Color.WHITE: # checa se tem linha antes do verde
        left_motor.speed(550)
        right_motor.run_angle(550, 144)
        
        if left_sensor.color() == Color.GREEN and right_sensor.color() == Color.WHITE:
            left_motor.speed(550)
            right_motor.run_angle(550, 288)
            
        elif left_sensor.color() == Color.WHITE and right_sensor.color() == Color.GREEN:
            left_motor.speed(550)
            right_motor.run_angle(550, 288)

        elif left_sensor.color() == Color.GREEN and right_sensor.color() == Color.GREEN:
            left_motor.speed(550)
            right_motor.run_angle(550, 288)
    
    else:
        left_motor.speed(550)
        right_motor.run_angle(550, 504)

while True: # loop principal
    if force_sensor.pressed() == True: # se eu apertar o sensor de força, não faz nada
        wait(10)
        
    else:
        line_bw()
        
        if left_sensor.color() == Color.GREEN:
            left_motor.speed(550)
            right_motor.run_angle(550, 36)
            if left_sensor.color() == Color.GREEN:
                green()

        elif right_sensor.color() == Color.GREEN:
            left_motor.speed(550)
            right_motor.run_angle(550, 36)
            if right_sensor.color() == Color.GREEN:
                green()
