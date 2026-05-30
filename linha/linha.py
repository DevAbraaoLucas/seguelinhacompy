# importando funções necessárias para a programação
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# definindo portas de motores e sensores
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)
ultrassonic_sensor = UltrasonicSensor(Port.E)
hub = PrimeHub()

# definindo variáveis
obstacle = 0

def line_follower(Kp): # função para seguir preto e branco
    if hub.imu.rotation(Axis.Y) > 8:  
        base_speed = 200 # descida da rampa
    elif hub.imu.rotation(Axis.Y) < -8: 
        base_speed = 400 # subida da rampa
    else:
        base_speed = 550 # velocidade normal

    if left_sensor.reflection() < 20 and right_sensor.reflection() > 40: # mtpreto branco
        if left_sensor.reflection() < 11:
            left_motor.run(550)
            right_motor.run_angle(550, 216)
            while right_sensor.reflection() > 11: # gira até o sensor direito ver preto
                left_motor.run(-300)
                right_motor.run(300)
            left_motor.run(700)
            right_motor.run(-700)
            wait(350)
            left_motor.run(-550)
            right_motor.run_angle(-550, 108)

    elif left_sensor.reflection() > 40 and right_sensor.reflection() < 20: # branco mtpreto
        if right_sensor.reflection() < 11:
            left_motor.run(550)
            right_motor.run_angle(550, 216)
            while left_sensor.reflection() > 11: # gira até o sensor esquerdo ver preto
                left_motor.run(300)
                right_motor.run(-300)
            left_motor.run(-700)
            right_motor.run(700)
            wait(350)
            left_motor.run(-550)
            right_motor.run_angle(-550, 108)

    else:
        error = left_sensor.reflection() - right_sensor.reflection()
        correction = Kp * error
    
        left_motor.run(base_speed + correction)
        right_motor.run(base_speed - correction)

def green(): # função para fazer a verificação do verde e os três possíveis casos de verde
    left_motor.run(-550)
    right_motor.run_angle(-550, 108)
    
    if left_sensor.color() == Color.WHITE and right_sensor.color() == Color.WHITE: # checa se tem linha antes do verde
        left_motor.run(550)
        right_motor.run_angle(550, 144)
        
        if left_sensor.color() == Color.GREEN and right_sensor.color() == Color.WHITE: # verde branco
            left_motor.run(550)
            right_motor.run_angle(550, 288)
            left_motor.run(-1000)
            right_motor.run_angle(1000, 400)
            while right_sensor.reflection() > 20:
                left_motor.run(-550)
                right_motor.run(550)
            left_motor.run(700)
            right_motor.run_angle(-700, 300)
            left_motor.run(-550)
            right_motor.run_angle(-550, 72)
            
        elif left_sensor.color() == Color.WHITE and right_sensor.color() == Color.GREEN: # branco verde
            left_motor.run(550)
            right_motor.run_angle(550, 288)
            left_motor.run(1000)
            right_motor.run_angle(-1000, 400)
            while left_sensor.reflection() > 20:
                left_motor.run(550)
                right_motor.run(-550)
            left_motor.run(-700)
            right_motor.run_angle(700, 300)
            left_motor.run(-550)
            right_motor.run_angle(-550, 72)

        elif left_sensor.color() == Color.GREEN and right_sensor.color() == Color.GREEN: # verde verde
            left_motor.run(550)
            right_motor.run_angle(550, 180)
            left_motor.run(1000)
            right_motor.run_angle(-1000, 1000)
            while left_sensor.reflection() > 20:
                left_motor.run(300)
                right_motor.run(-300)
            left_motor.run(-700)
            right_motor.run_angle(700, 300)
            left_motor.run(-550)
            right_motor.run_angle(-550, 72)
    
    else:
        left_motor.run(550)
        right_motor.run_angle(550, 504)

while True: # loop principal
    if left_sensor.color() == Color.GREEN:
        left_motor.run(550)
        right_motor.run_angle(550, 36)
        if left_sensor.color() == Color.GREEN:
            green()
    
    elif right_sensor.color() == Color.GREEN:
        left_motor.run(550)
        right_motor.run_angle(550, 36)
        if right_sensor.color() == Color.GREEN:
            green()
    
    else:
        line_follower(1)
