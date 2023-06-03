from time import sleep
from math import cos, pi, degrees
import random
import numpy as np

#essa função de controle tira um percentual em relação ao objetivo e qaunto ainda precisa andar, movendo mais devagar ou em velocidade normal
def controle(obj, atual, inicial):
    maxi = (obj - inicial)/2
    pcent = 1 - ((atual - inicial)/(obj-inicial))
    if pcent < 0.1:
        pcent = 0.1
        print('Drone movendo em velocidade baixa')
        sleep(2)
    else:
        print('Drone movendo em velocidade normal')
        sleep(1)
    vel = maxi*pcent
    return vel  

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
        self.camera = 0

        #ações
        self.state_horizontal = None
        self.state_vertical = None
        self.side_velocity = 0.0
        self.vertical_velocity = 0.0
        self.pitch_velocity = 0.0
        self.forward_velocity = 0.0



    def atualizaCam(self):
        print("Atualizando a camera que tudo vê...")
        #leva em conta os possiveis valores de value do HSV 
        matriz_aleatoria = np.random.randint(1, 101, size =(5,5))
        for linha in matriz_aleatoria:
            for elemento in linha:
                print(elemento, end=' ')
            print()
        self.camera = np.mean(matriz_aleatoria) #o self.camera vai receber so a media pra ser realizado os calc
        sleep(1)


    def cagada(self):
        if random.randint(0, 9) % 2 == 0:
            print("a camera do drone se apaixonou pelo furlas e mudou a direção do drone!\n")
            self.posicao["x"] += random.randint(-50, 50)
            self.posicao["z"] += random.randint(-50, 50)
            self.atualizaCam()


    def calculo_lidar(self, lidar_type: str):


        if lidar_type not in ["horizontal", "vertical"]:
            raise ValueError(f"o método calculo_lidar foi programado para receber como parâmetro os valores 'horizontal' ou 'vertical'\nContudo foi passado {lidar_type}, ocasionando um erro.")
        

        #os calculos são um pouco difíceis de explicar... pergunta pro @lipedras      
        if lidar_type == "horizontal":

            self.lidar_r = abs(((self.largura_corredor/2 - self.posicao["x"])/ cos(self.angulo_roll)) - (self.largura/2))
            self.lidar_l = abs(((self.largura_corredor/2 + self.posicao["x"]) / cos(self.angulo_roll)) - (self.largura/2))
            self.lidar_d = abs(self.posicao["z"] / cos(self.angulo_roll))

        if lidar_type == "vertical":
            self.lidar_d = abs(self.posicao["z"] / cos(self.angulo_pitch))


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
                    while (self.camera%2) == 0: #lipe, tentei fzr com que caisse mais vzs so pra mostrar o print, mas n consegui :(
                        print("A camera perdeu a mao, corrigindo...")
                        sleep(0.5)
                        self.atualizaCam()
                    break
            self.side_velocity = 0
            self.state_horizontal = None
            self.angulo_roll = 0
            self.calculo_lidar(lidar_type="horizontal")
            print(f"drone roll estabilizado!\nposicao: {self.posicao}\nlidars: d: {self.lidar_d} | l: {self.lidar_l} | r: {self.lidar_r}\n")
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
                    while (self.camera%2)  == 0: #caso a camera nao esteja apontando a media da matriz correta, ela deve atualizar
                        print("A camera perdeu a mao, corrigindo...")
                        sleep(0.5)
                        self.atualizaCam()
                    break


            self.vertical_velocity = 0
            self.state_vertical = None
            self.calculo_lidar(lidar_type="vertical")


            print(f"Altura do drone estabilizada!\nposicao: {self.posicao}\nlidars: d: {self.lidar_d} | l: {self.lidar_l} | r: {self.lidar_r}\n")
        print("Drone estável!\n")


    def takeoff(self, altura: float):

        #definição de estado e velocidade // não é necessária nenhuma angulação para o take off, basta acelerar os motores.
        print("Estado -1: Taking Off\n")
        self.state_vertical = "moving"
        self.last_posicao = self.posicao.copy()

        #note que não foi necessária uma condição de escape, no caso do drone não ficar precisamente com 150 de altura, pois o "<" já inclui essa margem.
        while self.posicao["z"] < altura: 
            self.vertical_velocity = controle(altura, self.posicao['z'], self.last_posicao['z'])
            self.posicao["z"] += self.vertical_velocity 
            self.calculo_lidar(lidar_type="vertical")
            print(f"lidar_d: {self.lidar_d}\nposicao: {self.posicao}\n")
            self.atualizaCam()
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
        print("Estado 0 concluido!")
        print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\n")       
        sleep(3)


    def para_frente(self, obj):

        #definindo velocidades e estados // NOTA P/ GABS -> vamos add um modelo para o Y?
        print("Estado 1: movendo para frente")
        self.angulo_pitch = pi / 4
        self.last_posicao = self.posicao.copy()
        self.calculo_lidar(lidar_type="vertical")

        #da mesma forma que o loop de take off, aqui não foi necessário escape. Denota-se que aqui não há variação nos estados, nem fator de estabilização
        while self.posicao["y"] < obj:
            self.forward_velocity = controle(obj, self.posicao['y'], self.last_posicao['y'])
            self.posicao["y"] += self.forward_velocity
            print(f"lidar_d: {self.lidar_d}\nposicao: {self.posicao}\n")
            self.atualizaCam()
            sleep(1)

        #redefinindo as velocidades e angulações
        self.forward_velocity = 0.0
        self.angulo_pitch = 0.0
        self.calculo_lidar(lidar_type="vertical")

        self.cagada()
        self.stabilize()
        
        print("Estado 1 concluido!")
        print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\n")
        sleep(3)

    
    def para_direita(self, obj):

        #definindo velocidades e estados
        print("Estado 2: movendo para a direita")
        self.angulo_roll = - pi/18
        self.state_horizontal = "moving"
        self.last_posicao = self.posicao.copy()
        self.calculo_lidar(lidar_type="horizontal")

        #movendo
        while self.posicao["x"] < obj:
            self.side_velocity = controle(obj, self.posicao['x'], self.last_posicao['x'])
            self.posicao["x"] += self.side_velocity
            self.calculo_lidar(lidar_type="horizontal")
            print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\nposicao: {self.posicao}\n")
            sleep(1)
        
        #reset para o 0
        self.angulo_roll = 0
        self.side_velocity = 0
        self.state_horizontal = None
        self.calculo_lidar(lidar_type="horizontal")

        self.cagada()
        self.stabilize()

        print("Estado 2 concluido!")
        print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\nposicao: {self.posicao}\n")
        sleep(3)


    def flip(self):
        if abs(self.posicao["x"]) < 10:

            #definindo velocidades e estados
            print("iniciando o flip...")
            self.pitch_velocity = pi / 5
            self.state_vertical = "moving"
            self.vertical_velocity = - 10

            #movendo
            while self.angulo_pitch < 2 * pi:
                self.angulo_pitch += self.pitch_velocity
                self.last_posicao = self.posicao.copy()
                #self.posicao["z"] += self.vertical_velocity
                self.calculo_lidar(lidar_type="vertical")
                print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\nposicao: {self.posicao}\nangulação pitch: {degrees(self.angulo_pitch)}")
                sleep(0.1)
            
            #reset para 0
            self.pitch_velocity = 0
            self.angulo_pitch = 0
            self.vertical_velocity = 0
            self.state_vertical = None
            self.calculo_lidar(lidar_type="vertical")
            print("flip concluido!")
            print(f"lidars:\nd: {self.lidar_d}\nl: {self.lidar_l}\nr: {self.lidar_r}\nposicao: {self.posicao}\n")


        else:
            print("os sensores identificaram que o drone está próximo demais da parede para realizar o flip.")
            print("ponto estável settado para X: 0 ")
            self.last_posicao["x"] = 0.0
            self.stabilize()
            self.flip()


    def lan(self):
        print("Estado 4: landing...")

        # não é necessária nenhuma angulação, basta diminuir a ação os motores.
        self.state_vertical = "moving"
        self.last_posicao = self.posicao.copy()

        while self.posicao["z"] > 2: 
            self.vertical_velocity = controle(0, self.posicao['z'], self.last_posicao['z'])
            #caso a posição z for negativo, nao é realizado a conta e sai do loop 
            if (self.posicao["z"] + self.vertical_velocity) < 0:
                break
            self.posicao["z"] += self.vertical_velocity
            self.calculo_lidar(lidar_type="vertical")
            print(f"lidar_d: {self.lidar_d}\nposicao: {self.posicao}\n")
            self.atualizaCam()
            sleep(1)
        
        #setando tudo pra zero
        self.last_posicao = self.posicao.copy()
        self.vertical_velocity = 0
        self.side_velocity = 0.0
        self.state_horizontal = None
        self.state_vertical = None
        self.state_vertical = None
        self.camera = 0

        print(f"Landing concluido! Altitude atual: {self.posicao['z']}")

drone_1 = Drone()
drone_1.takeoff(150.0)
drone_1.parado()
drone_1.para_frente(50)
drone_1.para_direita(15)
drone_1.flip()
drone_1.lan()
print('\nTodos estados foram concluidos, pode guardar o drone, Lipinho.')