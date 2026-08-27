from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel = 94, observe_channels = [49])
left_motor_garra = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor_garra = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
#motor_caçamba = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
left_ultrassonic = UltrasonicSensor(Port.E)
right_ultrassonic = UltrasonicSensor(Port.F)

hub.display.off()
hub.light.off() # desligando as luzes do hub pra economizar bateria

timer = StopWatch()



data = hub.ble.observe(49)

def movimento_garra(speed, time):
    left_motor_garra.dc(speed)
    right_motor_garra.dc(speed)
    wait(time)

def lock_garra():
    left_motor_garra.brake()
    right_motor_garra.brake()

def paredes_resgate():
    global data
    if 700 < soma < 730 or 1000 < soma < 1030:
        hub.ble.broadcast(2)
        print('resgate?')
        if data == 3:
            '''if left_ultrassonic.distance() < 100 and right_ultrassonic.distance() < 650:
                hub.ble.broadcast('E')
            elif left_ultrassonic.distance() < 650 and right_ultrassonic.distance() < 100:
                hub.ble.broadcast('D')'''
            while True:
                data = hub.ble.observe(49)
                if data == 0:
                    movimento_garra(-67, 500)
                    break
                elif data == 00: # descer garra
                    print('RECEBI DESCE')
                    movimento_garra(67,500)
                    lock_garra()
                elif data == 10: # subir garra
                    print('SOBE GARRA')
                    movimento_garra(-100, 400)
                    lock_garra()
        elif data == 4:
            wait(1000)
        
while True:
    hub.ble.broadcast(0)

    data = hub.ble.observe(49)
    '''if data is None:
        timer.reset()
        while True:
            if data != None:
                break
            if timer.time() > 20000:
                hub.system.shutdown()'''

    soma = left_ultrassonic.distance() + right_ultrassonic.distance()

    if data == 0: # assim que começa a seguir linha ou sempre que reiniciar
        movimento_garra(-67, 500)
    elif data == 1: # enquanto ta seguindo linha, trava os dois motores da garra
        lock_garra()
    
    paredes_resgate()
