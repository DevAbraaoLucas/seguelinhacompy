from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel = 94, observe_channels = [49])

hub.display.off()
hub.light.off() # desligando as luzes do hub pra economizar bateria

while True:
    print(hub.battery.voltage())
    
    data = hub.ble.observe(49)

    if data == 12:
        hub.light.on(Color.RED)

    else:
        hub.light.on(Color.GREEN) 
