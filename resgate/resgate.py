from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel = 94, observe_channels = [49])
left_motor_garra = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor_garra = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
motor_caçamba = Motor(Port.D, positive_direction=Direction.CLOCKWISE)

hub.display.off()
hub.light.off() # desligando as luzes do hub pra economizar bateria

def movimento_garra(speed, time):
    left_motor_garra.dc(speed)
    right_motor_garra.dc(speed)
    wait(time)
    left_motor_garra.brake()
    right_motor_garra.brake()

while True:    
    data = hub.ble.observe(49)
    
    if data == 0:
        movimento_garra(67, 500)
    else:
        wait(6.7)
