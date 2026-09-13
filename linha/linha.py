from pybricks.hubs import PrimeHub
from pybricks.iodevices import PUPDevice
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Color, Direction, Port, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task

hub = PrimeHub(top_side = Axis.Z, front_side = Axis.Y, broadcast_channel = 49, observe_channels = [94])
motor_esquerdo = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
motor_direito = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
dois_motores = DriveBase(motor_esquerdo, motor_direito, 31, 126)
sensor_esquerdo = ColorSensor(Port.C)
sensor_direito = ColorSensor(Port.D)
sensor_ultrassonico = UltrasonicSensor(Port.E)
sensor_cor_frente = ColorSensor(Port.F)

hub.light.off()
hub.display.off() # desligando as luzes do hub pra economizar bateria

# Protocolo de sinais (hub.ble.broadcast / observe):
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
#     'S'         = abertura na lateral esquerda (só depois de resgate_concluido, sem
#                   ambiguidade com o uso acima pq essa fase já passou) - procurar_saida()
#                   só acompanha a parede esquerda, por isso só usa esse lado
#     99  = resgate concluído (entrega feita)
# (achar a área de entrega é 100% local no principal agora, via sensor_cor_frente - não usa broadcast)

timer = StopWatch()

dados = hub.ble.observe(94)

ultimo_erro = 0
resgate_concluido = False
fim_resgate = False
veio_de_area_entrega = False # controla se a próxima quina da procurar_saida() é um giro de 45° garantido

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

def seguir_linha(Kp, Kd, velocidade_base, vel_min=-100): # função para seguir preto e branco
    if sensor_esquerdo.reflection() < 14 and sensor_direito.reflection() > 30: # mtpreto branco
            dois_motores.drive(400, 0)
            wait(250)
            hub.imu.reset_heading(0)
            timer.reset()
            while sensor_direito.reflection() > 12 or hub.imu.heading() <= -120: # gira até o sensor direito ver preto
                motor_esquerdo.dc(-70)
                motor_direito.dc(70)
                if timer.time() >= 8067:
                    motor_esquerdo.dc(100)
                    motor_direito.dc(100)
                    wait(140)
                    break
                if hub.imu.heading() <= -120:
                    while True:
                        motor_esquerdo.dc(70)
                        motor_direito.dc(-70)
                        if sensor_esquerdo.reflection() < 12 or hub.imu.heading() >= -6.7 or timer.time() >= 8067:
                            break
                    if hub.imu.heading() >= -6.7:
                        motor_esquerdo.dc(-100)
                        motor_direito.dc(-100)
                        wait(100)
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
            timer.reset()
            while sensor_esquerdo.reflection() > 12 or hub.imu.heading() >= 120: # gira até o sensor esquerdo ver preto
                motor_esquerdo.dc(70)
                motor_direito.dc(-70)
                if timer.time() >= 8067:
                    motor_esquerdo.dc(100)
                    motor_direito.dc(100)
                    wait(140)
                    break
                if hub.imu.heading() >= 120:
                    while True:
                        motor_esquerdo.dc(-70)
                        motor_direito.dc(70)
                        if sensor_direito.reflection() < 12 or hub.imu.heading() <= 6.7 or timer.time() >= 8067:
                            break
                    if hub.imu.heading() <= 6.7:
                        motor_esquerdo.dc(-100)
                        motor_direito.dc(-100)
                        wait(100)
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

        motor_esquerdo.dc(max(vel_min, min(100, velocidade_base + correcao)))
        motor_direito.dc(max(vel_min, min(100, velocidade_base - correcao)))

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
            wait(333)
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
            wait(333)
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
            wait(250)
            guinada('D', 167, 100)
            while sensor_esquerdo.reflection() > 25:
                motor_esquerdo.dc(75)
                motor_direito.dc(-100)
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
            guinada('E', 67, 100)
            motor_esquerdo.dc(100)
            motor_direito.dc(100)
            wait(200)
            timer.reset()
            while timer.time() < 2000:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60) # anda um pouco pra frente
                motor_esquerdo.dc(100)
                motor_direito.dc(-70)
                wait(60) # anda um pouco pro lado
            while True:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60) # anda um pouco pra frente
                if sensor_direito.reflection() < 25:
                    break
                motor_esquerdo.dc(100)
                motor_direito.dc(-100)
                wait(60) # anda um pouco pro lado
                if sensor_direito.reflection() < 25:
                    break
            dois_motores.drive(600, 0)
            wait(300)
            while True:
                motor_esquerdo.dc(-100)
                motor_direito.dc(80)
                if sensor_direito.reflection() < 20:
                    break
            motor_esquerdo.dc(70)
            motor_direito.dc(-70)
            wait(222)
            motor_esquerdo.dc(-100)
            motor_direito.dc(-100)
            wait(80)

        elif lado == 2: # desvia pra direita
            guinada('D', 67, 100)
            motor_esquerdo.dc(100)
            motor_direito.dc(100)
            wait(200)
            timer.reset()
            while timer.time() < 2000:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60) # anda um pouco pra frente
                motor_esquerdo.dc(-70)
                motor_direito.dc(100)
                wait(60) # anda um pouco pro lado
            while True:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60) # anda um pouco pra frente
                if sensor_esquerdo.reflection() < 25:
                    break
                motor_esquerdo.dc(-100)
                motor_direito.dc(100)
                wait(60) # anda um pouco pro lado
                if sensor_esquerdo.reflection() < 25:
                    break
            dois_motores.drive(600, 0)
            wait(300)
            while True:
                motor_esquerdo.dc(80)
                motor_direito.dc(-100)
                if sensor_esquerdo.reflection() < 20:
                    break
            motor_esquerdo.dc(-70)
            motor_direito.dc(70)
            wait(222)
            motor_esquerdo.dc(-100)
            motor_direito.dc(-100)
            wait(80)

def na_faixa_prata():
    return (40 < sensor_esquerdo.reflection() < 50
            and 40 < sensor_direito.reflection() < 50)

def fita_prata():
    if not na_faixa_prata():
        return False
    timer.reset()
    while na_faixa_prata():
        if timer.time() >= 150:            
            return True
        seguir_linha(3, 0.367, 55)
    return False

def linha_vermelha():
    if sensor_esquerdo.color() == Color.RED or sensor_direito.color() == Color.RED:
            dois_motores.brake()
            wait(99999)

def andar_reto(velocidade, tempo, Kp=1):
    hub.imu.reset_heading(0)
    timer.reset()
    while timer.time() < tempo:
        erro = hub.imu.heading()  # heading alvo é 0
        turn_rate = -Kp * erro
        motor_esquerdo.dc(velocidade + turn_rate)
        motor_direito.dc(velocidade - turn_rate)

        if sensor_esquerdo.reflection() < 15 and sensor_direito.reflection() < 15:
            motor_esquerdo.brake()
            motor_direito.brake()
            break

def andar_ate_parede(velocidade, distancia_parede, tempo_max=4000, Kp=1):
    # ajustar velocidade/distancia_parede/tempo_max.
    hub.imu.reset_heading(0)
    timer.reset()
    while sensor_ultrassonico.distance() > distancia_parede and timer.time() < tempo_max:
        erro = hub.imu.heading()
        motor_esquerdo.dc(velocidade + (-Kp * erro))
        motor_direito.dc(velocidade - (-Kp * erro))
    motor_esquerdo.brake()
    motor_direito.brake()

def varredura(lado_inicial):
    lado_livre = 'D' if lado_inicial == 'E' else 'E' # lado sem parede perto (achado na entrada)

    andar_reto(70, 3500)
    motor_esquerdo.dc(-45)
    motor_direito.dc(-45)
    hub.ble.broadcast(10) # sobe a garra
    wait(400)

    guinada(lado_livre, 90, 100) # vira pro lado sem parede perto
    hub.ble.broadcast(20)
    andar_reto(-100, 400)

    andar_reto(100, 5000) # até a parede lateral
    hub.ble.broadcast(10)
    motor_esquerdo.dc(-55)
    motor_direito.dc(-55)
    wait(400)

    hub.ble.broadcast(67)

    guinada('D', 90, 100) # alinha com a parede - pronto pra procurar_entrega()

def ir_para_entrega():
    global resgate_concluido
    parar(500)
    andar_reto(-100, 200)

    hub.ble.broadcast(20) # desce a garra
    wait(400)
    motor_esquerdo.dc(90)
    motor_direito.dc(45)
    wait(670)
    andar_reto(67, 3000)

    hub.ble.broadcast(10) # sobe a garra
    wait(200)
    motor_esquerdo.dc(-40)
    motor_direito.dc(-40)
    wait(1500)

    guinada('D', 90, 100) # ajustar lado/velocidade - vira de costas pra encostar a caçamba
    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(1000) # ajustar - encosta a parte de trás (caçamba) na área de resgate

    hub.ble.broadcast(30) # abre/fecha a caçamba (entregar_bolinhas() no secundário)
    timer.reset()
    while hub.ble.observe(94) != 99 and timer.time() < 7000:
        motor_esquerdo.dc(67)
        motor_direito.dc(67)
        wait(150)
        motor_esquerdo.dc(-100)
        motor_direito.dc(-100)
        wait(250)

    hub.ble.broadcast(67)

    resgate_concluido = True

def achou_entrega(): # checa se o sensor de cor da frente está vendo alguma área de entrega
    return sensor_cor_frente.color() == Color.GREEN or sensor_cor_frente.color() == Color.RED

def procurar_entrega():
    global resgate_concluido
    while True:
        andar_ate_parede(100, 50)
        parar(1)
        if achou_entrega():
            ir_para_entrega()
            break
        andar_reto(100, 1000)
        andar_reto(-100, 500)
        guinada('D', 90, 100) # ajustar lado do giro entre uma parede e outra

    resgate_concluido = True # não achou em nenhuma parede - segue mesmo assim

def atravessar_abertura():
    hub.imu.reset_heading(0)
    timer.reset()
    while timer.time() < 2000: # ajustar - tempo pra atravessar o vão da parede
        erro = hub.imu.heading()
        motor_esquerdo.dc(50 + erro)
        motor_direito.dc(50 - erro)
        if sensor_esquerdo.reflection() < 20 or sensor_direito.reflection() < 20:
            return True # achou preto = é a saída de verdade
    return False # não achou preto = era a entrada (fita prateada), não conta

def seguir_parede(velocidade=100, distancia_parede=60, tempo_max=8000, Kp=1):
    global dados
    hub.imu.reset_heading(0)
    timer.reset()
    while sensor_ultrassonico.distance() > distancia_parede and timer.time() < tempo_max:
        hub.ble.broadcast(40)
        dados = hub.ble.observe(94)
        if dados == 'S':
            hub.speaker.beep(5000)
            break
        erro = hub.imu.heading()
        motor_esquerdo.dc(velocidade + (-Kp * erro))
        motor_direito.dc(velocidade - (-Kp * erro))
        if sensor_esquerdo.reflection() < 15 and sensor_direito.reflection() < 15:
            motor_esquerdo.brake()
            motor_direito.brake()
            dados = 'P'
            break

def virar_na_parede():
    global veio_de_area_entrega
    if veio_de_area_entrega:
        guinada('D', 45, 100)
        veio_de_area_entrega = False
        return

    cor = sensor_cor_frente.color()
    if cor == Color.GREEN or cor == Color.RED:
        guinada('D', 45, 100) # beirada da área de resgate - acompanha a diagonal
        veio_de_area_entrega = True
    else:
        guinada('D', 90, 100) # quina de parede de verdade
        andar_reto(-100, 400)

def procurar_saida():
    global fim_resgate, dados

    andar_reto(100, 200)
    guinada('E', 90, 100)
    while True:
        andar_reto(100, 1)
        if sensor_ultrassonico.distance() <= 80:
            break
        if sensor_esquerdo.reflection() < 15 or sensor_direito.reflection() < 15:
            break
    if sensor_esquerdo.reflection() < 15:
        andar_reto(100, 100)
        motor_esquerdo.dc(-100)
        motor_direito.dc(40)
        wait(700)
        andar_reto(60, 200)
        fim_resgate = True
        hub.ble.broadcast(1)
    else:
        guinada('D', 45, 100)

    while True:
        if fim_resgate:
            break
        seguir_parede()
        parar(200)

        if dados == 'S': # quando tem saida na lateral esquerda
            andar_reto(100, 300)
            guinada('E', 90, 100)
            if atravessar_abertura():
                fim_resgate = True
                hub.ble.broadcast(1)
                break
            else: # era a entrada - volta pra dentro da sala
                andar_reto(-50, 2000)
                guinada('D', 90, 100) # volta a ficar paralelo à parede esquerda
                andar_reto(100, 1000)
                parar(1000)

        elif dados == 'P': # quando os dois sensores de cor veem preto
            parar(100)
            if sensor_esquerdo.reflection() < 15 and sensor_direito.reflection() < 15:
                # achou a fita preta da saída - sai do loop e termina o resgate
                fim_resgate = True
                hub.ble.broadcast(1)
                break
        
        else:
            virar_na_parede() # chegou numa quina/beirada sem achar abertura

def resgate():
    motor_esquerdo.dc(100)
    motor_direito.dc(100)
    wait(300)
    timer.reset()
    parar(1)
    while timer.time() < 1000:
        lado_entrada = hub.ble.observe(94)
    print(lado_entrada)

    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(100)

    hub.ble.broadcast(20) # desce a garra
    motor_esquerdo.dc(-60)
    motor_direito.dc(-60)
    wait(300) # volta pra entrada

    varredura(lado_entrada)

    procurar_entrega() # depois de varrer, procura a área de entrega e chama ir_para_entrega()

    procurar_saida() # depois de entregar, procura a saída

# ANTES DE COMEÇAR OS ROUNDS, NÃO ESQUECER EM HIPÓTESE ALGUMA:
# | verificar a leitura dos verdes
# | verificar a leitura de reflexões do sensor de cor
# | verificar a leitura da fita prata
# | mexer no lado de desvio do obstáculo
# | verificar a distância do obstáculo
# | verificar a leitura do vermelho

'''while True:
    print(sensor_esquerdo.reflection(), sensor_direito.reflection())
    print(sensor_esquerdo.color(), sensor_direito.color())
    seguir_linha(3, 0, 50)'''

hub.ble.broadcast(0) # antes de começar a seguir linha, manda um sinal para o hub debaixo subir a garra
wait(500)   

while True: # loop principal
    dados = hub.ble.observe(94)
    hub.ble.broadcast(1) # enquanto ta seguindo linha, o hub debaixo trava os motores da garra

    if hub.imu.tilt()[0] < -6.7:
        seguir_linha(1.9, 0, 70, vel_min=50)
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

    if fita_prata():
        hub.ble.broadcast(3)
        break

    linha_vermelha()

while fim_resgate != True: # loop de resgate
    resgate()

while True: # volta a seguir linha
    dados = hub.ble.observe(94)
    hub.ble.broadcast(1) # enquanto ta seguindo linha, o hub debaixo trava os motores da garra

    if hub.imu.tilt()[0] < -6.7:
        seguir_linha(2, 0, 70, vel_min=50)
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

    linha_vermelha()
