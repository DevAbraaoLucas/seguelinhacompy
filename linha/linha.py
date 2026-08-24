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

timer = StopWatch()

last_error = 0

def curvas(speed): # se -speed: curva pra esquerda, se +speed: curva pra direita
    left_motor.dc(speed)
    right_motor.dc(-speed)

def guinada(side, degrees, speed):
    hub.imu.reset_heading(0)
    if side == 'E': # esquerda
        while True:
            curvas(-speed)
            if hub.imu.heading() <= -degrees: # curva pra esquerda
                drive.brake()
                break
    else: # direita
        while True:
            curvas(speed)
            if hub.imu.heading() >= degrees: # curva pra direita
                drive.brake()
                break

def line_follower(Kp, Kd, base_speed): # função para seguir preto e branco
    if left_sensor.reflection() < 14 and right_sensor.reflection() > 30: # mtpreto branco
            drive.drive(400, 0)
            wait(250)
            hub.imu.reset_heading(0)
            while right_sensor.reflection() > 12 or hub.imu.heading() <= -120: # gira até o sensor esquerdo ver preto
                left_motor.dc(-70)
                right_motor.dc(70)
                if hub.imu.heading() <= -120:
                    while True:
                        left_motor.dc(70)
                        right_motor.dc(-70)
                        if left_sensor.reflection() < 12 or hub.imu.heading() >= -6.7:
                            break
                    if hub.imu.heading() >= -6.7:
                        left_motor.dc(-100)
                        right_motor.dc(-100)
                        wait(167)
            left_motor.dc(70)
            right_motor.dc(-70)
            wait(167)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(67)

    elif left_sensor.reflection() > 30 and right_sensor.reflection() < 14: # branco mtpreto
            drive.drive(400, 0)
            wait(250)
            hub.imu.reset_heading(0)
            while left_sensor.reflection() > 12 or hub.imu.heading() >= 120: # gira até o sensor esquerdo ver preto
                left_motor.dc(70)
                right_motor.dc(-70)
                if hub.imu.heading() >= 120:
                    while True:
                        left_motor.dc(-70)
                        right_motor.dc(70)
                        if right_sensor.reflection() < 12 or hub.imu.heading() <= 6.7:
                            break
                    if hub.imu.heading() <= 6.7:
                        left_motor.dc(-100)
                        right_motor.dc(-100)
                        wait(167)
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(167)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(67)

    else:
        global last_error
        error = left_sensor.reflection() - right_sensor.reflection()
        p = Kp * error
        d = Kd * (error - last_error)
        correction = p + d
        last_error = error

        left_motor.dc(base_speed + correction)
        right_motor.dc(base_speed - correction)

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
            wait(400)
            guinada('E', 40, 100)
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
            wait(400)
            guinada('D', 40, 100)
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
            wait(444)
            guinada('D', 167, 100)
            while left_sensor.reflection() > 25:
                left_motor.dc(75)
                right_motor.dc(-75)
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(150)
            left_motor.dc(-80)
            right_motor.dc(-80)
            wait(67)
    
    else:
        left_motor.dc(90)
        right_motor.dc(90)
        wait(250)

def obstacle(side):
    if ultrassonic_sensor.distance() < 44:
        left_motor.dc(-100)
        right_motor.dc(-100)
        wait(111)
        drive.brake()

        if side == 1: # desvia pra esquerda
            guinada('E', 67, 80)
            left_motor.dc(100)
            right_motor.dc(100)
            wait(200)
            timer.reset()
            while True:
                left_motor.dc(100)
                right_motor.dc(100)
                wait(60)
                if right_sensor.reflection() < 12:
                    break
                if timer.time() < 2000:
                    left_motor.dc(100)
                    right_motor.dc(-70)
                    wait(60)
                else:
                    left_motor.dc(100)
                    right_motor.dc(-67)
                    wait(60)
                if right_sensor.reflection() < 12:
                    break
            drive.drive(600, 0)
            wait(300)
            while True:
                left_motor.dc(-100)
                right_motor.dc(80)
                if right_sensor.reflection() < 25:
                    break
            left_motor.dc(70)
            right_motor.dc(-70)
            wait(222)
            if timer.time() > 1500:
                left_motor.dc(-80)
                right_motor.dc(-80)
                wait(150)
            else:
                left_motor.dc(-80)
                right_motor.dc(-80)
                wait(100)
            
        elif side == 2: # desvia pra direita
            guinada('D', 67, 100)
            left_motor.dc(100)
            right_motor.dc(100)
            wait(200)
            timer.reset()
            while True:
                left_motor.dc(100)
                right_motor.dc(100)
                wait(60)
                if left_sensor.reflection() < 12:
                    break
                if timer.time() < 2000:
                    left_motor.dc(-70)
                    right_motor.dc(100)
                    wait(60)
                else:
                    left_motor.dc(-67)
                    right_motor.dc(100)
                    wait(60)
                if left_sensor.reflection() < 12:
                    break
            drive.drive(600, 0)
            wait(300)
            while True:
                left_motor.dc(80)
                right_motor.dc(-100)
                if left_sensor.reflection() < 25:
                    break
            left_motor.dc(-70)
            right_motor.dc(70)
            wait(222)
            if timer.time() > 1500:
                left_motor.dc(-80)
                right_motor.dc(-80)
                wait(150)
            else:
                left_motor.dc(-80)
                right_motor.dc(-80)
                wait(100)

def red_line():
    if left_sensor.color() == Color.RED or right_sensor.color() == Color.RED:
            drive.brake()
            wait(99999)

def resgate():
    lado_entrada = data
    while True:
        hub.ble.broadcast(00)
        left_motor.dc(-100)
        right_motor.dc(-100)
        if data == 01:
            left_motor.brake()
            right_motor.brake()
            hub.ble.broadcast(02)
            wait(500)
            break
    left_motor.dc(100)
    right_motor.dc(100)
    wait(1000)
    hub.ble.broadcast(03)
    left_motor.dc(-40)
    right_motor.dc(-40)
    wait(400)
    guinada(lado_entrada, 90, 100)

hub.ble.broadcast(0) # antes de começar a seguir linha, manda um sinal para o hub debaixo subir a garra
wait(500)

while True: # loop principal    
    data = hub.ble.observe(94)
    hub.ble.broadcast(1) # enquanto ta seguindo linha, o hub debaixo trava os motores da garra
    
    if hub.imu.tilt()[0] < -6.7:
        line_follower(2, 0, 90)
    elif hub.imu.tilt()[0] > 5:
        line_follower(2, 0, 50)
    else:
        line_follower(3, 0.367, 80)
    
    if left_sensor.color() == Color.GREEN or right_sensor.color() == Color.GREEN:
        left_motor.dc(50)
        right_motor.dc(50)
        wait(67)
        if left_sensor.color() == Color.GREEN or right_sensor.color() == Color.GREEN:
            green()

    obstacle(2) # 1 = esquerda; 2 = direita

    '''if 49 <= left_sensor.reflection() <= 53 and 49 <= right_sensor.reflection() <= 53:
        timer.reset()
        while 49 <= left_sensor.reflection() <= 53 and 49 <= right_sensor.reflection() <= 53:
            print(timer.time())
        if timer.time() > 100:
            break'''

    if data == 2: # fim do seguimento de linha
        timer.reset()
        while True:
            print(timer.time())
            line_follower(3, 0.367, 80)
            if left_sensor.reflection() < 50 or right_sensor.reflection() < 50 or timer.time() >= 1111 or data == 0:
                break
        print(timer.time())
        print(ultrassonic_sensor.distance())
        if ultrassonic_sensor.distance() < 600 and timer.time() > 1300:
            hub.ble.broadcast(3)
            break
        else:
            hub.ble.broadcast(4)

    red_line()
