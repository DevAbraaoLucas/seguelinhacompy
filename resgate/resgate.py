from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel = 94, observe_channels = [49])
#left_motor_garra = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
#right_motor_garra = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
#motor_caçamba = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
left_ultrassonic = UltrasonicSensor(Port.E)
right_ultrassonic = UltrasonicSensor(Port.F)

hub.display.off()
hub.light.off() # desligando as luzes do hub pra economizar bateria

def movimento_garra(speed, time):
    left_motor_garra.dc(speed)
    right_motor_garra.dc(speed)
    wait(time)

def lock_garra():
    left_motor_garra.brake()
    right_motor_garra.brake()

#while True:
#    print(left_ultrassonic.distance(), 'left')
#    print(right_ultrassonic.distance(), 'right')
#    print(soma)
    

def paredes_resgate():
    if 700 < soma < 730 or 1000 < soma < 1030:
        hub.ble.broadcast(2)
        print('resgate?')
        if data == 3:
            print('resgate')
        elif data == 4:
            wait(3000)
        else:
            wait(6.7)
        if left_ultrassonic.distance() < 100 and right_ultrassonic.distance() < 650:
            print('entrada esquerda')
        elif left_ultrassonic.distance() < 650 and right_ultrassonic.distance() < 100:
            print('entrada direita')

while True:
    hub.ble.broadcast(0)

    data = hub.ble.observe(49)

    soma = left_ultrassonic.distance() + right_ultrassonic.distance()

    '''if data == 0: # assim que começa a seguir linha ou sempre que reiniciar
        movimento_garra(67, 500)
    elif data == 1: # enquanto ta seguindo linha, trava os dois motores da garra
        lock_garra()'''
    
    paredes_resgate()
    hub.ble.broadcast(0)
