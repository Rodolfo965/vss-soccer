#!/usr/bin/env python3
from math import pow, atan2, sqrt, sin, cos
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool, Float32MultiArray

class RobotController:
    def __init__(self):
        self.update_rate = 10

        self.raio = 0.015
        self.distanciaR = 0.08

        self.kp_angular = 2.5
        self.ki_angular = 0.7
        self.kd_angular = 0.1

        self.omega=0
        self.omegaO=0
        self.w=0
        self.dtheta=0
        self.dx=0
        self.dy=0

        self.po=0.39

        self.erro_angular_acumulado = 0.0
        self.erro_angular_anterior = 0.0

        self.dt = 0.1

        self.ix = 0
        self.iy = 0
        self.fx = 0
        self.fy = 0

        # Inicializa um vetor para armazenar a posição anterior e atual
        self.posicoes_robo = [None, None]  # posicoes_robo[0] -> anterior, posicoes_robo[1] -> atual

        rospy.init_node('robot_controller', anonymous=True)
        self.vel_esquerda_pub = rospy.Publisher('cmd_vel_esquerda', Float32, queue_size=10)
        self.vel_direita_pub = rospy.Publisher('cmd_vel_direita', Float32, queue_size=10)

        self.bola_pose = Point()
        self.ponto_pose = Point()
        self.centro_pose = Point()
        self.robo_dentro = False

        rospy.Subscriber('/robo/posicao_robo', Point, self.position_callback_robo)
        rospy.Subscriber('/bola/posicao_bola', Point, self.position_callback_bola)
        rospy.Subscriber('/campo/objetivo', Point, self.position_callback_ponto)
        rospy.Subscriber('/campo/centro', Point, self.position_callback_centro)
        rospy.Subscriber('/campo/robo_em_campo', Bool, self.callback_roboDentro)
        rospy.Subscriber('/campo/posicao_campo', Float32MultiArray, self.position_callback_campo)

        self.rate = rospy.Rate(self.update_rate)

    def position_callback_campo(self, msg):
        if len(msg.data) == 4:
            self.ix = msg.data[0]
            self.iy = msg.data[1]
            self.fx = msg.data[2]
            self.fy = msg.data[3]
        else:
            rospy.logwarn("Dados recebidos do campo não têm o formato esperado.")

    
    def estimador(self):
        if self.posicoes_robo[0] is not None and self.posicoes_robo[1] is not None:
            self.dx = self.posicoes_robo[1].x - self.posicoes_robo[0].x
            self.dy = self.posicoes_robo[1].y - self.posicoes_robo[0].y
            self.dtheta = self.posicoes_robo[1].z - self.posicoes_robo[0].z

            self.w = self.dtheta / self.dt
            u = sqrt(pow((self.dx), 2) + pow((self.dy), 2))

            self.omegaO = self.po * (self.omega - self.w)

    def position_callback_robo(self, msg):
        self.posicoes_robo[0] = self.posicoes_robo[1]
        self.posicoes_robo[1] = msg

        #rospy.loginfo(f"Posição anterior do robô: {self.posicoes_robo[0].x,self.posicoes_robo[0].y,self.posicoes_robo[0].z,}")
        #rospy.loginfo(f"Posição atual do robô: {self.posicoes_robo[1].x,self.posicoes_robo[1].y,self.posicoes_robo[1].z,}")
        
        # Chama o estimador após atualizar a posição
        self.estimador()

    def position_callback_bola(self, msg):
        self.bola_pose = msg

    def position_callback_ponto(self, msg):
        self.ponto_pose = msg

    def position_callback_centro(self, msg):
        self.centro_pose = msg

    def callback_roboDentro(self, msg):
        self.robo_dentro = msg.data

    def distancia_euclidiana(self, alvo_pose):
        return sqrt(pow((alvo_pose.x - self.posicoes_robo[1].x), 2) + pow((alvo_pose.y - self.posicoes_robo[1].y), 2))

        
    def velocidades_rodas(self, alvo_pose):
       
        angulo_viragem = atan2(alvo_pose.y - self.posicoes_robo[1].y, alvo_pose.x - self.posicoes_robo[1].x)
       
        erro_theta = (angulo_viragem - self.posicoes_robo[1].z)
       
        erro_normalizado = atan2(sin(erro_theta), cos(erro_theta))

        self.erro_angular_acumulado += erro_normalizado * self.dt
       
      #  self.erro_angular_acumulado = max(min(self.erro_angular_acumulado, 1.0), -1.0)
       
        derivada_erro_angular = (erro_normalizado - self.erro_angular_anterior) / self.dt
       
        self.omega = (self.kp_angular * erro_normalizado + self.ki_angular * self.erro_angular_acumulado + self.kd_angular * derivada_erro_angular)
       
        self.erro_angular_anterior = erro_normalizado

        vel_linear = 0.8
        alinhamento = 0.25

        if abs(erro_normalizado) < alinhamento:
            va_esquerda = (vel_linear) / self.raio
            va_direita = (vel_linear) / self.raio   
        else:
            va_esquerda = (vel_linear - (self.omegaO * (self.distanciaR)))/ self.raio
            va_direita = (vel_linear + (self.omegaO * (self.distanciaR)))/ self.raio
            
        return va_esquerda, va_direita

    def mover_robo(self, alvo_pose, tolerancia):
        if self.posicoes_robo[1] is not None:
            distancia = self.distancia_euclidiana(alvo_pose)
            vel_esquerda, vel_direita = self.velocidades_rodas(alvo_pose)

            rospy.loginfo(f"Posição do Robô: ({self.posicoes_robo[1].x:.2f}, {self.posicoes_robo[1].y:.2f}, {self.posicoes_robo[1].z:.2f}), Alvo: ({alvo_pose.x:.2f}, {alvo_pose.y:.2f}), Distância: {distancia:.2f}")
            #rospy.loginfo(f"centro recebido: ({self.centro_pose.x}, {self.centro_pose.y})")
            #rospy.loginfo(f"omegas: ({self.omega}, {self.omegaO},{self.w})")
            #rospy.loginfo(f"velocidades: ({self.dx}, {self.dy},{self.dtheta})")
         
            if distancia > 150:
                self.vel_esquerda_pub.publish(vel_esquerda)
                self.vel_direita_pub.publish(vel_direita)
                rospy.loginfo(f"velocidade E: ({vel_esquerda:.2f}, velocidade D: ({vel_direita:.2f})")
            else:
                vel_esquerda_reduzida = vel_esquerda / 1
                vel_direita_reduzida = vel_direita / 1
                
                self.vel_esquerda_pub.publish(vel_esquerda_reduzida)
                self.vel_direita_pub.publish(vel_direita_reduzida)
               # rospy.loginfo(f"Distância menor que 500. Velocidades reduzidas: E: ({vel_esquerda_reduzida:.2f}), D: ({vel_direita_reduzida:.2f})")
                
                if distancia < tolerancia:
                    self.vel_esquerda_pub.publish(0.0)
                    self.vel_direita_pub.publish(0.0)
                    rospy.loginfo("Movimento concluído. Parando o robô.")
        else:
            self.vel_esquerda_pub.publish(0.0)
            self.vel_direita_pub.publish(0.0)
            rospy.loginfo("Coordenadas inválidas. Parando o robô.")

    def moveParaBola(self):
        self.mover_robo(self.bola_pose, tolerancia=10)

    def moveParaCampo(self):
        self.mover_robo(self.centro_pose, tolerancia=100)

def main():
    controller = RobotController()
    while not rospy.is_shutdown():
        if controller.robo_dentro:
            controller.moveParaBola()
        else:
            controller.moveParaCampo()
        controller.rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
