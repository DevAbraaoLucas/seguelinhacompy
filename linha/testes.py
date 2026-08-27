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

#Color.SILVER = Color(h = 210, s = 27, v = 65)
#cores = (Color.SILVER, Color.RED, Color.GREEN, Color.WHITE)
#left_sensor.detectable_colors(cores)
#right_sensor.detectable_colors(cores)

#while True:
#    print(left_sensor.hsv(), 'e')
#    print(right_sensor.hsv(), 'd')

# Color.SILVER = Color(h = 212, s = 29, v = 63)
#left_sensor.detectable_colors(Color.GRAY)

# left_sensor.lights.on(100)

while True:   
    print(left_sensor.reflection(), 'esquerda')
    print(right_sensor.reflection(), 'direita')
    left_motor.dc(80)
    right_motor.dc(80)
#    print(left_sensor.hsv(), 'e')
#    print(right_sensor.hsv(), 'd')
#    print(left_sensor.color(), 'esquerda')
#    print(right_sensor.color(), 'direita')
#    left_motor.dc(80)
#    right_motor.dc(80)
#    print(right_motor.speed())
#    print(hub.imu.tilt()[0])

# branco: h = 212, s = 28, v = 80
# fita prata esq: h = 212, s = 29, v = 63
# fita prata dir: h = 204, s 29, v = 66
# verde normal: h = 150, s = 73, v = 31
# verde escuro: h = 167, s = 43, v = 35
# vermelho: h = 260, s = 19, v = 77

'''if 49 <= left_sensor.reflection() <= 53 and 49 <= right_sensor.reflection() <= 53:
    timer.reset()
    while 49 <= left_sensor.reflection() <= 53 and 49 <= right_sensor.reflection() <= 53:
        print(timer.time())
    if timer.time() > 100:
        break'''
