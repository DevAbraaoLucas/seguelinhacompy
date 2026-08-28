from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel = 94, observe_channels = [49])
motor_garra_esquerdo = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
motor_garra_direito = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
#motor_caçamba = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
ultrassonico_esquerdo = UltrasonicSensor(Port.E)
ultrassonico_direito = UltrasonicSensor(Port.F)

hub.display.off()
hub.light.off() # desligando as luzes do hub pra economizar bateria

timer = StopWatch()

dados = hub.ble.observe(49)

def movimento_garra(velocidade, tempo):
    motor_garra_esquerdo.dc(velocidade)
    motor_garra_direito.dc(velocidade)
    wait(tempo)

def travar_garra():
    motor_garra_esquerdo.brake()
    motor_garra_direito.brake()

def paredes_resgate():
    global dados
    if 700 < soma < 730 or 1000 < soma < 1030:
        hub.ble.broadcast(2)
        print('resgate?')
        if dados == 3:
            '''if ultrassonico_esquerdo.distance() < 100 and ultrassonico_direito.distance() < 650:
                hub.ble.broadcast('E')
            elif ultrassonico_esquerdo.distance() < 650 and ultrassonico_direito.distance() < 100:
                hub.ble.broadcast('D')'''
            while True:
                dados = hub.ble.observe(49)
                if dados == 0:
                    movimento_garra(-67, 500)
                    break
                elif dados == 20: # descer garra
                    print('RECEBI DESCE')
                    movimento_garra(67,500)
                    travar_garra()
                elif dados == 10: # subir garra
                    print('SOBE GARRA')
                    movimento_garra(-100, 400)
                    travar_garra()
        elif dados == 4:
            wait(1000)

while True:
    hub.ble.broadcast(0)

    dados = hub.ble.observe(49)
    '''if dados is None:
        timer.reset()
        while True:
            if dados != None:
                break
            if timer.time() > 20000:
                hub.system.shutdown()'''

    soma = ultrassonico_esquerdo.distance() + ultrassonico_direito.distance()

    if dados == 0: # assim que começa a seguir linha ou sempre que reiniciar
        movimento_garra(-67, 500)
    elif dados == 1: # enquanto ta seguindo linha, trava os dois motores da garra
        travar_garra()

    paredes_resgate()
