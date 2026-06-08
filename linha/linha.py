# importando funções necessárias para a programação
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# definindo portas de motores e sensores
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
drive = DriveBase(left_motor, right_motor, 31, 126)
left_sensor = ColorSensor(Port.C)
right_sensor = ColorSensor(Port.D)
ultrassonic_sensor = UltrasonicSensor(Port.E)
hub = PrimeHub(top_side = Axis.Z, front_side = Axis.Y, broadcast_channel = 49, observe_channels = [94])

hub.light.off()
hub.display.off() # desligando as luzes do hub pra economizar bateria

def line_follower(Kp, base_speed): # função para seguir preto e branco
    if left_sensor.reflection() < 20 and right_sensor.reflection() > 40: # mtpreto branco
            drive.drive(370, 0)
            wait(300)
            while right_sensor.reflection() > 25: # gira até o sensor direito ver preto
                left_motor.dc(-70)
                right_motor.dc(70)
            left_motor.dc(70)
            right_motor.dc(-70)
            wait(150)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(100)

    elif left_sensor.reflection() > 40 and right_sensor.reflection() < 20: # branco mtpreto
            drive.drive(370, 0)
            wait(300)
            while left_sensor.reflection() > 25: # gira até o sensor esquerdo ver preto
                left_motor.dc(70)
                right_motor.dc(-70)
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(150)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(100)

    else:
        error = left_sensor.reflection() - right_sensor.reflection()
        correction = Kp * error

        left_motor_vel = base_speed + correction
        right_motor_vel = base_speed - correction

        left_motor.dc(left_motor_vel)
        right_motor.dc(right_motor_vel)

def green(): # função para fazer a verificação do verde e os três possíveis casos de verde
    left_motor.dc(-60)
    right_motor.dc(-60)
    wait(250)
    
    if left_sensor.color() == Color.WHITE and right_sensor.color() == Color.WHITE: # checa se tem linha antes do verde
        left_motor.dc(55)
        right_motor.dc(55)
        wait(300)
        
        if left_sensor.color() == Color.GREEN and right_sensor.color() == Color.WHITE: # verde branco
            drive.drive(400, 0)
            wait(350)
            left_motor.dc(-100)
            right_motor.dc(100)
            wait(500)
            while right_sensor.reflection() > 20:
                left_motor.dc(-70)
                right_motor.dc(70)
            left_motor.dc(70)
            right_motor.dc(-70)
            wait(150)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(100)
            
        elif left_sensor.color() == Color.WHITE and right_sensor.color() == Color.GREEN: # branco verde
            drive.drive(400, 0)
            wait(350)
            left_motor.dc(100)
            right_motor.dc(-100)
            wait(500)
            while left_sensor.reflection() > 20:
                left_motor.dc(70)
                right_motor.dc(-70)
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(150)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(100)

        elif left_sensor.color() == Color.GREEN and right_sensor.color() == Color.GREEN: # verde verde
            drive.drive(400, 0)
            wait(400)
            left_motor.dc(100)
            right_motor.dc(-100)
            wait(1515)
            while left_sensor.reflection() > 25:
                left_motor.dc(75)
                right_motor.dc(-75)
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(150)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(50)
    
    else:
        left_motor.dc(90)
        right_motor.dc(90)
        wait(250)

def obstacle(side):
    hub.imu.reset_heading(0)

    if ultrassonic_sensor.distance() < 44:
        if side == 1: # desvia pra esquerda
            left_motor.dc(-100)
            right_motor.dc(-100)
            wait(111)
            left_motor.dc(-100)
            right_motor.dc(100)
            wait(555)
            left_motor.brake()
            right_motor.brake()
            wait(67)
            while True:
                drive.drive(100, 50)
                if right_sensor.reflection() < 25:
                    break
            drive.drive(700, 0)
            wait(400)
            while True:
                left_motor.dc(-100)
                right_motor.dc(80)
                if right_sensor.reflection() < 25:
                    break
            left_motor.dc(70)
            right_motor.dc(-70)
            wait(222)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(300)
            
        elif side == 2: # desvia pra direita
            left_motor.dc(-100)
            right_motor.dc(-100)
            wait(111)
            left_motor.dc(100)
            right_motor.dc(-100)
            wait(555)
            left_motor.brake()
            right_motor.brake()
            wait(67)
            while True:
                drive.drive(100, -50)
                if left_sensor.reflection() < 25:
                    break
            drive.drive(700, 0)
            wait(400)
            while True:
                left_motor.dc(80)
                right_motor.dc(-100)
                if left_sensor.reflection() < 25:
                    break
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(222)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(300)

def red_line():
    if left_sensor.color() == Color.RED or right_sensor.color() == Color.RED:
        drive.brake()
        wait(99999)

while True: # loop principal
#if hub.imu.rotation(Axis.X) < -15 or hub.imu.rotation(Axis.X) > 15:
#    line_follower(2, 50)
#else:
    line_follower(2, 75)
    
    if left_sensor.color() == Color.GREEN or right_sensor.color() == Color.GREEN:
        left_motor.dc(50)
        right_motor.dc(50)
        wait(50)
        if left_sensor.color() == Color.GREEN or right_sensor.color() == Color.GREEN:
            green()

    obstacle(2)

    red_line()
