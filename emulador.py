from time import sleep
from math import cos, pi
import random

class Drone:
    def __init__(self):
        #dimensões
        self.largura = float(30)
        self.largura_corredor = float(100)
        
        #sensores
        self.posicao = {"x": 0.0, "y": 0.0, "z": 0.0} #refere-se à posição do drone, note que esse valor não é igual ao dos sensores
        self.last_posicao = self.posicao.copy()
        self.angulo_roll = float(0) #angulo do roll -> movimento usado para ir para os lados / NOTA: É EM RADIANOS!!! -> entre 0 e pi/2 o drone vai pra esquerda
        self.angulo_pitch = float(0) #angulo do pitch -> movimento usado para ir para frente/trás / NOTA: É EM RADIANOS!!! -> entre 0 e pi/2 o drone vai pra frente
        self.lidar_r = float(35)
        self.lidar_l = self.largura_corredor - self.largura - self.lidar_r
        self.lidar_d = self.posicao["z"]
        self.camera = None

        #ações
        self.state_horizontal = None
        self.state_vertical = None
        self.side_velocity = 0.0
        self.vertical_velocity = 0.0
        self.pitch_velocity = 0.0
        self.forward_velocity = 0.0


    def cagada(self):
        if random.randint(0, 9) % 2 == 0:
            print("a camera do drone se apaixonou pelo furlas e mudou a direção do drone!\n")
            self.posicao["x"] += 15
            self.posicao["z"] += 20


    def calculo_lidar(self, lidar_type: str):


        if lidar_type not in ["horizontal", "vertical"]:
            raise ValueError(f"o método calculo_lidar foi programado para receber como parâmetro os valores 'horizontal' ou 'vertical'\nContudo foi passado {lidar_type}, ocasionando um erro.")
        

        #os calculos são um pouco difíceis de explicar... pergunta pro @lipedras      
        if lidar_type == "horizontal":

            self.lidar_r = ((self.largura_corredor/2 - self.posicao["x"])/ cos(self.angulo_roll)) - (self.largura/2)
            self.lidar_l = ((self.largura_corredor/2 - self.posicao["x"]) / cos(self.angulo_roll)) - (self.largura/2)
            self.lidar_d = self.posicao["z"] / cos(self.angulo_roll)

        if lidar_type == "vertical":
            self.lidar_d = self.posicao["z"] / cos(self.angulo_pitch)


    def stabilize(self):
        
        print(f"analisando estabilidade...")
        #caso1: o drone se movimenta horizontalmente e indevidamente
        if self.posicao["x"] != self.last_posicao["x"] and self.state_horizontal is None:
            print("movimento irregular do tipo 1 detectado")

            tamanho_da_cagada = self.last_posicao["x"] - self.posicao["x"]
            #caso1 - A -> a variação é pequena, logo a velocidade do ajuste deve ser suficiente para corrigir a posição em 1s
            if abs(tamanho_da_cagada) < 10.0:

                self.side_velocity = tamanho_da_cagada
                self.state_horizontal = "moving"
                if tamanho_da_cagada > 0:
                    self.angulo_roll = -pi / 6
                else:
                    self.angulo_roll =  pi/6


            #caso1 - B -> a variação é grande, logo a velocidade do ajuste deve ser suficiente para corrigir a posição em 2s
            if abs(tamanho_da_cagada) > 10.0:

                self.side_velocity = tamanho_da_cagada / 2.0
                self.state_horizontal = "moving"
                if tamanho_da_cagada > 0:
                    self.angulo_roll = -pi / 3
                else:
                    self.angulo_roll =  pi / 3


            #o laço a seguir é o responsável pela simulação do movimento, note que nele há um IF criando a condição de escape para cobrir as imprecisões do drone.
            print("corrigindo a posição...")
            while self.posicao["x"] != self.last_posicao["x"]:

                self.posicao["x"] += self.side_velocity
                self.calculo_lidar(lidar_type="horizontal")

                print(f"posicao: {self.posicao}")
                
                sleep(1)

                if abs(self.posicao["x"] - self.last_posicao["x"]) < 5:
                    break
            self.side_velocity = 0
            self.state_horizontal = None
            self.angulo_roll = 0
            self.calculo_lidar(lidar_type="horizontal")
            print(f"drone estabilizado!\nposicao: {self.posicao}\nlidars: d: {self.lidar_d} | l: {self.lidar_l} | r: {self.lidar_r}\n")
        sleep(2)

        #caso2: o drone se movimenta verticalmente e indevidamente
        if self.posicao["z"] != self.last_posicao["z"] and self.state_vertical is None:
            print("movimento irregular do tipo 2 detectado!")

            tamanho_da_cagada_v = self.last_posicao["z"] - self.posicao["z"]
            #caso2 - A -> a variação é pequena, logo a velocidade do ajuste deve ser suficiente para corrigir a posição em 1s
            if abs(tamanho_da_cagada_v) < 10.0:

                self.vertical_velocity = tamanho_da_cagada_v
                self.state_vertical = "moving"


            #caso2 - B -> a variação é grande, logo a velocidade do ajuste deve ser suficiente para corrigir a posição em 2s
            if abs(tamanho_da_cagada_v) > 10.0:

                self.vertical_velocity = tamanho_da_cagada_v / 2.0
                self.state_vertical = "moving"


            #o laço a seguir é o responsável pela simulação do movimento, note que nele há um IF criando a condição de escape para cobrir as imprecisões do drone.
            print("corrigindo a posição...")
            while self.posicao["z"] != self.last_posicao["z"]:

                
                self.posicao["z"] += self.vertical_velocity
                self.calculo_lidar(lidar_type="vertical")

                print(f"posicao: {self.posicao}")
                
                sleep(1)

                if abs(self.posicao["z"] - self.last_posicao["z"]) < 5:
                    break


            self.vertical_velocity = 0
            self.state_vertical = None
            self.calculo_lidar(lidar_type="vertical")


            print(f"drone estabilizado!\nposicao: {self.posicao}\nlidars: d: {self.lidar_d} | l: {self.lidar_l} | r: {self.lidar_r}\n")
        print("Drone estável!\n")


    def takeoff(self, altura: float):

        #definição de estado e velocidade // não é necessária nenhuma angulação para o take off, basta acelerar os motores.
        print("Estado -1: Taking Off\n")
        self.state_vertical = "moving"
        self.vertical_velocity = 15.0


        #note que não foi necessária uma condição de escape, no caso do drone não ficar precisamente com 150 de altura, pois o "<" já inclui essa margem.
        while self.posicao["z"] < altura:
            self.last_posicao = self.posicao.copy()
            self.posicao["z"] += self.vertical_velocity
            self.calculo_lidar(lidar_type="vertical")
            print(f"lidar_d: {self.lidar_d}\nposicao: {self.posicao}\n")
            sleep(1)
        
        self.last_posicao = self.posicao.copy()
        self.vertical_velocity = 0
        self.state_vertical = None


        self.cagada()
        self.stabilize()

        print(f"Take Off concluido! Altitude atual: {self.posicao['z']}")
        sleep(3)


    def parado(self):

        #nao sei mto bem o q mais seria feito nessa etapa...
        print("Estado 0: parado no ar")
        for i in range(0,5):
            self.cagada()
            self.stabilize()
            sleep(2)
        print("Etapa 0 concluida...")
        print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\n")       
        sleep(3)


    def para_frente(self):

        #definindo velocidades e estados // NOTA P/ GABS -> vamos add um modelo para o Y?
        print("Estado 1: movendo para frente")
        self.forward_velocity = 15.0
        self.angulo_pitch = pi / 4
        self.calculo_lidar(lidar_type="vertical")

        #da mesma forma que o loop de take off, aqui não foi necessário escape. Denota-se que aqui não há variação nos estados, nem fator de estabilização
        while self.posicao["y"] < 50.0:
            self.last_posicao = self.posicao.copy()
            self.posicao["y"] += self.forward_velocity
            print(f"lidar_d: {self.lidar_d}\nposicao: {self.posicao}\n")
            sleep(1)

        #redefinindo as velocidades e angulações
        self.forward_velocity = 0.0
        self.angulo_pitch = 0.0
        self.calculo_lidar(lidar_type="vertical")
        
        print("Etapa 1 concluida...")
        print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\n")
        sleep(3)

    
    def para_direita(self):

        #definindo velocidades e estados
        pass

drone_1 = Drone()
drone_1.takeoff(150.0)
drone_1.parado()
drone_1.para_frente()