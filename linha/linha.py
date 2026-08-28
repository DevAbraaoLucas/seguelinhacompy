from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# definindo portas de motores e sensores
motor_esquerdo = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
motor_direito = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
dois_motores = DriveBase(motor_esquerdo, motor_direito, 31, 126)
sensor_esquerdo = ColorSensor(Port.C)
sensor_direito = ColorSensor(Port.D)
sensor_ultrassonico = UltrasonicSensor(Port.E)
hub = PrimeHub(top_side = Axis.Z, front_side = Axis.Y, broadcast_channel = 49, observe_channels = [94])

hub.light.off()
hub.display.off() # desligando as luzes do hub pra economizar bateria

timer = StopWatch()

ultimo_erro = 0

def parar(tempo):
    motor_esquerdo.brake()
    motor_direito.brake()
    wait(tempo)

def curvas(velocidade): # se -velocidade: curva pra esquerda, se +velocidade: curva pra direita
    motor_esquerdo.dc(velocidade)
    motor_direito.dc(-velocidade)

def guinada(lado, graus, velocidade):
    hub.imu.reset_heading(0)
    if lado == 'E': # esquerda
        while True:
            curvas(-velocidade)
            if hub.imu.heading() <= -graus: # curva pra esquerda
                dois_motores.brake()
                break
    else: # direita
        while True:
            curvas(velocidade)
            if hub.imu.heading() >= graus: # curva pra direita
                dois_motores.brake()
                break

def andar_reto(velocidade, tempo, Kp=2.5):
    hub.imu.reset_heading(0)
    timer_reto = StopWatch()
    while timer_reto.time() < tempo:
        erro = hub.imu.heading()  # heading alvo é 0
        turn_rate = -Kp * erro
        dois_motores.drive(velocidade, turn_rate)
    dois_motores.stop()

def seguir_linha(Kp, Kd, velocidade_base): # função para seguir preto e branco
    if sensor_esquerdo.reflection() < 14 and sensor_direito.reflection() > 30: # mtpreto branco
            dois_motores.drive(400, 0)
            wait(250)
            hub.imu.reset_heading(0)
            while sensor_direito.reflection() > 12 or hub.imu.heading() <= -120: # gira até o sensor esquerdo ver preto
                motor_esquerdo.dc(-70)
                motor_direito.dc(70)
                if hub.imu.heading() <= -120:
                    while True:
                        motor_esquerdo.dc(70)
                        motor_direito.dc(-70)
                        if sensor_esquerdo.reflection() < 12 or hub.imu.heading() >= -6.7:
                            break
                    if hub.imu.heading() >= -6.7:
                        motor_esquerdo.dc(-100)
                        motor_direito.dc(-100)
                        wait(167)
            motor_esquerdo.dc(70)
            motor_direito.dc(-70)
            wait(167)
            motor_esquerdo.dc(-80)
            motor_direito.dc(-80)
            wait(67)

    elif sensor_esquerdo.reflection() > 30 and sensor_direito.reflection() < 14: # branco mtpreto
            dois_motores.drive(400, 0)
            wait(250)
            hub.imu.reset_heading(0)
            while sensor_esquerdo.reflection() > 12 or hub.imu.heading() >= 120: # gira até o sensor esquerdo ver preto
                motor_esquerdo.dc(70)
                motor_direito.dc(-70)
                if hub.imu.heading() >= 120:
                    while True:
                        motor_esquerdo.dc(-70)
                        motor_direito.dc(70)
                        if sensor_direito.reflection() < 12 or hub.imu.heading() <= 6.7:
                            break
                    if hub.imu.heading() <= 6.7:
                        motor_esquerdo.dc(-100)
                        motor_direito.dc(-100)
                        wait(167)
            motor_esquerdo.dc(-70)
            motor_direito.dc(70)
            wait(167)
            motor_esquerdo.dc(-80)
            motor_direito.dc(-80)
            wait(67)

    else:
        global ultimo_erro
        erro = sensor_esquerdo.reflection() - sensor_direito.reflection()
        p = Kp * erro
        d = Kd * (erro - ultimo_erro)
        correcao = p + d
        ultimo_erro = erro

        motor_esquerdo.dc(velocidade_base + correcao)
        motor_direito.dc(velocidade_base - correcao)

def verde(): # função para fazer a verificação do verde e os três possíveis casos de verde
    motor_esquerdo.dc(-60)
    motor_direito.dc(-60)
    wait(250)

    if sensor_esquerdo.color() == Color.WHITE and sensor_direito.color() == Color.WHITE: # checa se tem linha antes do verde
        motor_esquerdo.dc(55)
        motor_direito.dc(55)
        wait(300)

        if sensor_esquerdo.color() == Color.GREEN and sensor_direito.color() == Color.WHITE: # verde branco
            dois_motores.drive(400, 0)
            wait(400)
            guinada('E', 40, 100)
            while sensor_direito.reflection() > 20:
                motor_esquerdo.dc(-70)
                motor_direito.dc(70)
            motor_esquerdo.dc(70)
            motor_direito.dc(-70)
            wait(150)
            motor_esquerdo.dc(-80)
            motor_direito.dc(-80)
            wait(100)

        elif sensor_esquerdo.color() == Color.WHITE and sensor_direito.color() == Color.GREEN: # branco verde
            dois_motores.drive(400, 0)
            wait(400)
            guinada('D', 40, 100)
            while sensor_esquerdo.reflection() > 20:
                motor_esquerdo.dc(70)
                motor_direito.dc(-70)
            motor_esquerdo.dc(-70)
            motor_direito.dc(70)
            wait(150)
            motor_esquerdo.dc(-80)
            motor_direito.dc(-80)
            wait(100)

        elif sensor_esquerdo.color() == Color.GREEN and sensor_direito.color() == Color.GREEN: # verde verde
            dois_motores.drive(400, 0)
            wait(444)
            guinada('D', 167, 100)
            while sensor_esquerdo.reflection() > 25:
                motor_esquerdo.dc(75)
                motor_direito.dc(-75)
            motor_esquerdo.dc(-70)
            motor_direito.dc(70)
            wait(150)
            motor_esquerdo.dc(-80)
            motor_direito.dc(-80)
            wait(67)

    else:
        motor_esquerdo.dc(90)
        motor_direito.dc(90)
        wait(250)

def obstaculo(lado):
    if sensor_ultrassonico.distance() < 44:
        motor_esquerdo.dc(-100)
        motor_direito.dc(-100)
        wait(111)
        dois_motores.brake()

        if lado == 1: # desvia pra esquerda
            guinada('E', 67, 80)
            motor_esquerdo.dc(100)
            motor_direito.dc(100)
            wait(200)
            timer.reset()
            while True:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60)
                if sensor_direito.reflection() < 12:
                    break
                if timer.time() < 2000:
                    motor_esquerdo.dc(100)
                    motor_direito.dc(-70)
                    wait(60)
                else:
                    motor_esquerdo.dc(100)
                    motor_direito.dc(-67)
                    wait(60)
                if sensor_direito.reflection() < 12:
                    break
            dois_motores.drive(600, 0)
            wait(300)
            while True:
                motor_esquerdo.dc(-100)
                motor_direito.dc(80)
                if sensor_direito.reflection() < 25:
                    break
            motor_esquerdo.dc(70)
            motor_direito.dc(-70)
            wait(222)
            if timer.time() > 1500:
                motor_esquerdo.dc(-80)
                motor_direito.dc(-80)
                wait(150)
            else:
                motor_esquerdo.dc(-80)
                motor_direito.dc(-80)
                wait(100)

        elif lado == 2: # desvia pra direita
            guinada('D', 67, 100)
            motor_esquerdo.dc(100)
            motor_direito.dc(100)
            wait(200)
            timer.reset()
            while True:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60)
                if sensor_esquerdo.reflection() < 12:
                    break
                if timer.time() < 2000:
                    motor_esquerdo.dc(-70)
                    motor_direito.dc(100)
                    wait(60)
                else:
                    motor_esquerdo.dc(-67)
                    motor_direito.dc(100)
                    wait(60)
                if sensor_esquerdo.reflection() < 12:
                    break
            dois_motores.drive(600, 0)
            wait(300)
            while True:
                motor_esquerdo.dc(80)
                motor_direito.dc(-100)
                if sensor_esquerdo.reflection() < 25:
                    break
            motor_esquerdo.dc(-70)
            motor_direito.dc(70)
            wait(222)
            if timer.time() > 1500:
                motor_esquerdo.dc(-80)
                motor_direito.dc(-80)
                wait(150)
            else:
                motor_esquerdo.dc(-80)
                motor_direito.dc(-80)
                wait(100)

def linha_vermelha():
    if sensor_esquerdo.color() == Color.RED or sensor_direito.color() == Color.RED:
            dois_motores.brake()
            wait(99999)

def resgate():
    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(1100)
    parar(1000)
    hub.ble.broadcast(20)
    wait(500)
    parar(1000)
    motor_esquerdo.dc(100)
    motor_direito.dc(100)
    wait(1000)
    parar(1000)
    hub.ble.broadcast(10)
    motor_esquerdo.dc(-40)
    motor_direito.dc(-40)
    wait(400)
    parar(1000)
    guinada(lado_entrada, 90, 100)

hub.ble.broadcast(0) # antes de começar a seguir linha, manda um sinal para o hub debaixo subir a garra
wait(500)

# ANTES DE COMEÇAR OS ROUNDS, NÃO ESQUECER EM HIPÓTESE ALGUMA:
# | verificar a leitura dos verdes
# | verificar a leitura de reflexões do sensor de cor
# | mexer no lado de desvio do obstáculo
# | verificar a distância do obstáculo
# | verificar a leitura do vermelho

while True: # loop principal
    dados = hub.ble.observe(94)
    hub.ble.broadcast(1) # enquanto ta seguindo linha, o hub debaixo trava os motores da garra

    if hub.imu.tilt()[0] < -6.7:
        seguir_linha(2, 0, 90)
    elif hub.imu.tilt()[0] > 5:
        seguir_linha(2, 0, 50)
    else:
        seguir_linha(3, 0.367, 80)

    if sensor_esquerdo.color() == Color.GREEN or sensor_direito.color() == Color.GREEN:
        motor_esquerdo.dc(50)
        motor_direito.dc(50)
        wait(67)
        if sensor_esquerdo.color() == Color.GREEN or sensor_direito.color() == Color.GREEN:
            verde()

    obstaculo(2) # 1 = esquerda; 2 = direita

    if dados == 2: # fim do seguimento de linha
        timer.reset()
        while True:
            seguir_linha(3, 0.367, 80)
            if sensor_esquerdo.reflection() < 50 or sensor_direito.reflection() < 50 or timer.time() >= 1111 or dados == 0:
                break
        if timer.time() >= 1100:
            hub.ble.broadcast(3)
            break
        else:
            hub.ble.broadcast(4)

    linha_vermelha()

parar(1000)

resgate()
