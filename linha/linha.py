from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Color, Direction, Port, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

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

def seguir_linha(Kp, Kd, velocidade_base): # função para seguir preto e branco
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
                        wait(67)
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
                        wait(67)
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
            wait(300)
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
            wait(300)
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
            guinada('E', 67, 80)
            motor_esquerdo.dc(100)
            motor_direito.dc(100)
            wait(200)
            timer.reset()
            while True:
                motor_esquerdo.dc(100)
                motor_direito.dc(100)
                wait(60) # anda um pouco pra frente
                if sensor_direito.reflection() < 25:
                    break
                if timer.time() < 2000:
                    motor_esquerdo.dc(100)
                    motor_direito.dc(-70)
                    wait(60) # anda um pouco pro lado
                else:
                    motor_esquerdo.dc(100)
                    motor_direito.dc(-60)
                    wait(60) # anda um pouco pro lado
                if sensor_direito.reflection() < 25:
                    break
            dois_motores.drive(600, 0)
            wait(267)
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
                wait(130)
            else:
                motor_esquerdo.dc(-80)
                motor_direito.dc(-80)
                wait(80)

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
                if sensor_esquerdo.reflection() < 25:
                    break
                if timer.time() < 2000:
                    motor_esquerdo.dc(-70)
                    motor_direito.dc(100)
                    wait(60)
                else:
                    motor_esquerdo.dc(-60)
                    motor_direito.dc(100)
                    wait(60)
                if sensor_esquerdo.reflection() < 25:
                    break
            dois_motores.drive(600, 0)
            wait(267)
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
                wait(130)
            else:
                motor_esquerdo.dc(-80)
                motor_direito.dc(-80)
                wait(80)

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
    motor_esquerdo.brake()
    motor_direito.brake()

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

def achou_entrega():
    cor = sensor_cor_frente.color()
    return cor == Color.GREEN or cor == Color.RED

def ir_para_entrega():
    global resgate_concluido
    motor_esquerdo.dc(60)
    motor_direito.dc(60)
    wait(300) # ajustar - avanço diagonal pra entrar bem na área de resgate

    guinada('D', 90, 100) # ajustar lado/velocidade - vira de costas pra encostar a caçamba
    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(200) # ajustar - encosta a parte de trás (caçamba) na área de resgate
    motor_esquerdo.brake()
    motor_direito.brake()

    hub.ble.broadcast(30) # abre/fecha a caçamba (entregar_bolinhas() no secundário)
    timer.reset()
    while hub.ble.observe(94) != 99 and timer.time() < 5000: # espera confirmação, com timeout de segurança
        motor_esquerdo.dc(67)
        motor_direito.dc(67)
        wait(67)
        motor_esquerdo.dc(-100)
        motor_direito.dc(-100)
        wait(67)

    resgate_concluido = True

def varredura(lado_inicial):
    lado_livre = 'D' if lado_inicial == 'E' else 'E' # lado sem parede perto (achado na entrada)

    hub.ble.broadcast(20) # desce a garra
    wait(500)
    andar_reto(100, 900) # ajustar tempo/velocidade pra ~45cm (sala 90x90)
    hub.ble.broadcast(10) # sobe a garra
    motor_esquerdo.dc(-55)
    motor_direito.dc(-55)
    wait(400)

    guinada(lado_livre, 90, 100) # vira pro lado sem parede perto
    motor_esquerdo.dc(-100) # garantir que vai se alinhar com a parede lateral
    motor_direito.dc(-100)  # (o giro sozinho não garante isso)
    hub.ble.broadcast(20)
    wait(500)
    andar_ate_parede(100, 90) # até a parede lateral
    hub.ble.broadcast(10)
    motor_esquerdo.dc(-55)
    motor_direito.dc(-55)
    wait(400)

    parar(500)

    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(900) # ajustar tempo pra ~45cm de ré
    parar(500)

    guinada(lado_inicial, 90, 100) # desfaz o giro anterior - volta a apontar pra dentro da sala
    hub.ble.broadcast(20)
    wait(500)
    andar_ate_parede(100, 90) # até a parede oposta à entrada
    hub.ble.broadcast(10)
    motor_esquerdo.dc(-55)
    motor_direito.dc(-55)
    wait(400)

    guinada(lado_inicial, 180, 100) # meia-volta
    hub.ble.broadcast(20)
    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(500)
    andar_ate_parede(100, 90)
    hub.ble.broadcast(10)
    motor_esquerdo.dc(-55)
    motor_direito.dc(-55)
    wait(400)

    guinada('D', 90, 100) # alinha com a parede - pronto pra procurar_entrega()

def procurar_entrega():
    global resgate_concluido
    for _ in range(4): # no máximo as 4 paredes - depois disso desiste pra não travar a rodada
        andar_ate_parede(100, 40)
        if achou_entrega():
            ir_para_entrega()
            return
        guinada('D', 90, 100) # ajustar lado do giro entre uma parede e outra

    resgate_concluido = True # não achou em nenhuma parede - segue mesmo assim

def atravessar_abertura():
    hub.imu.reset_heading(0)
    timer.reset()
    while timer.time() < 2000: # ajustar - tempo pra atravessar o vão da parede
        erro = hub.imu.heading()
        motor_esquerdo.dc(60 + erro)
        motor_direito.dc(60 - erro)
        if sensor_esquerdo.reflection() < 20 or sensor_direito.reflection() < 20:
            motor_esquerdo.brake()
            motor_direito.brake()
            return True # achou preto = é a saída de verdade
    motor_esquerdo.brake()
    motor_direito.brake()
    return False # não achou preto = era a entrada (fita prateada), não conta

def seguir_parede(velocidade=100, distancia_parede=40, tempo_max=4000, Kp=1):
    # ajustar velocidade/distancia_parede/tempo_max.
    hub.imu.reset_heading(0)
    timer.reset()
    while sensor_ultrassonico.distance() > distancia_parede and timer.time() < tempo_max:
        hub.ble.broadcast(40)
        if hub.ble.observe(94) == 'S':
            break
        erro = hub.imu.heading()
        motor_esquerdo.dc(velocidade + (-Kp * erro))
        motor_direito.dc(velocidade - (-Kp * erro))
    motor_esquerdo.brake()
    motor_direito.brake()

def virar_na_parede():
    global veio_de_area_entrega
    if veio_de_area_entrega:
        guinada('E', 45, 100)
        veio_de_area_entrega = False
        return

    cor = sensor_cor_frente.color()
    if cor == Color.GREEN or cor == Color.RED:
        guinada('E', 45, 100) # beirada da área de resgate - acompanha a diagonal
        veio_de_area_entrega = True
    else:
        guinada('E', 90, 100) # quina de parede de verdade

def procurar_saida():
    global fim_resgate
    while True:
        seguir_parede()

        if hub.ble.observe(94) == 'S':
            motor_esquerdo.dc(100)
            motor_direito.dc(100)
            wait(300) # ajustar - avanço pra atravessar a abertura
            guinada('E', 90, 100)
            if atravessar_abertura():
                fim_resgate = True
                break
            else:
                # era a entrada (sem fita preta) - volta pra dentro da sala e continua
                motor_esquerdo.dc(-100)
                motor_direito.dc(-100)
                wait(100) # ajustar - tempo pra voltar pra dentro da sala
                motor_esquerdo.brake()
                motor_direito.brake()
                guinada('D', 90, 100) # volta a ficar paralelo à parede esquerda
        else:
            virar_na_parede() # chegou numa quina/beirada sem achar abertura

def resgate():
    timer.reset()
    while timer.time() < 500:
        lado_entrada = hub.ble.observe(94)
        parar(1)
    print(lado_entrada)

    motor_esquerdo.dc(-100)
    motor_direito.dc(-100)
    wait(1100) # volta pra entrada
    parar(1000)

    varredura(lado_entrada)
    procurar_entrega()      # depois de varrer, procura a área de entrega e chama ir_para_entrega()
    procurar_saida()        # depois de entregar, procura a saída

hub.ble.broadcast(0) # antes de começar a seguir linha, manda um sinal para o hub debaixo subir a garra
#wait(500) # NAO ESQUECER DE TIRAR ISSO

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

    obstaculo(1) # 1 = esquerda; 2 = direita

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

while fim_resgate != True:
    resgate()

while True:
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

    linha_vermelha()
