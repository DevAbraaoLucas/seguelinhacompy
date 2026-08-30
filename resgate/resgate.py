from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel = 94, observe_channels = [49])
motor_garra_esquerdo = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
motor_garra_direito = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
motor_cacamba = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
ultrassonico_esquerdo = UltrasonicSensor(Port.E)
ultrassonico_direito = UltrasonicSensor(Port.F)

hub.display.off()
hub.light.off() # desligando as luzes do hub pra economizar bateria

# Protocolo de sinais (hub.ble.broadcast / observe) - espelha o comentário em linha.py:
#   principal -> secundário (canal 49):
#     0  = reset / sobe a garra        1  = trava a garra (seguindo linha)
#     3  = confirmou chegada na entrada da sala   4 = ainda não chegou
#     10 = sobe a garra                20 = desce a garra (captura cega)
#     30 = abre e fecha a caçamba (entrega das vítimas)
#     40 = "tô seguindo a parede procurando a saída, confere o ultrassônico esquerdo agora"
#          (mandado toda hora dentro de seguir_parede(), só depois de resgate_concluido)
#   secundário -> principal (canal 94):
#     0   = ocioso                     2  = detectou as paredes da sala (fim da linha)
#     'E'/'D'/'M' = lado da entrada (só durante a entrada na sala)
#     'E'         = abertura na lateral esquerda (só depois de resgate_concluido, sem
#                   ambiguidade com o uso acima pq essa fase já passou) - procurar_saida()
#                   só acompanha a parede esquerda, por isso só usa esse lado
#     99  = resgate concluído (entrega feita)
# (achar a área de entrega é 100% local no principal agora, via sensor de cor de frente dele - não usa broadcast)

timer = StopWatch()

dados = hub.ble.observe(49)
resgate_concluido = False # checkpoint: vira True quando o principal avisar que entregou as bolinhas

def movimento_garra(velocidade, tempo):
    motor_garra_esquerdo.dc(velocidade)
    motor_garra_direito.dc(velocidade)
    wait(tempo)

def travar_garra():
    motor_garra_esquerdo.brake()
    motor_garra_direito.brake()

def movimento_cacamba(velocidade, tempo):
    motor_cacamba.dc(velocidade)
    wait(tempo)
    motor_cacamba.brake()

def checar_paredes_saida():
    if ultrassonico_esquerdo.distance() > 150:
        hub.ble.broadcast('S')

def entregar_bolinhas():
    # ajustar sentido (sinal de velocidade) e tempo de cada movimento pro mecanismo real.
    global resgate_concluido
    movimento_cacamba(80, 300)  # abre a caçamba
    wait(5000)  # tempo pras vítimas caírem
    movimento_cacamba(-80, 300)  # fecha a caçamba de novo
    resgate_concluido = True
    hub.ble.broadcast(99)
    wait(800) # segura o sinal de "concluído" tempo suficiente pro principal perceber,
              # antes do loop abaixo trocar de assunto pros sensores laterais

def resgate():
    global dados, resgate_concluido
    if 700 < soma < 730 or 1000 < soma < 1030:
        hub.ble.broadcast(2)
        if dados == 3:
            if ultrassonico_esquerdo.distance() < 100 and ultrassonico_direito.distance() < 650:
                hub.ble.broadcast('E')
            elif ultrassonico_esquerdo.distance() < 650 and ultrassonico_direito.distance() < 100:
                hub.ble.broadcast('D')
            else:
                hub.ble.broadcast('M')
            while True:
                dados = hub.ble.observe(49)
                if resgate_concluido == True:
                    hub.ble.broadcast(99)
                    if dados == 40:
                        checar_paredes_saida()
                if dados == 0:
                    movimento_garra(-67, 500)
                    break
                elif dados == 10: # subir garra
                    movimento_garra(-100, 400)
                    travar_garra()
                elif dados == 20: # descer garra
                    movimento_garra(67,500)
                    travar_garra()
                elif dados == 30: # abrir/fechar caçamba pra entregar
                    entregar_bolinhas()
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

    resgate()
