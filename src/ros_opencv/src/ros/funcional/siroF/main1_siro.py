#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

# Inicializa o CvBridge
bridge = CvBridge()

# Armazena as imagens recebidas
img_bola = None
img_robo = None

# Armazena as coordenadas recebidas
pos_bola = None
pos_robo = None

# Variáveis globais para desenhar e verificar objetos
ix, iy, fx, fy = -1, -1, -1, -1
px, py = -1, -1
cx, cy = 0, 0  # Variáveis para o ponto central
drawing = False
robo_em_campo = False  # Inicializa como booleano

# Inicializa publishers
pub_objetivo = rospy.Publisher('/campo/objetivo', Point, queue_size=10)
pub_centro = rospy.Publisher('/campo/centro', Point, queue_size=10)
pub_robo_em_campo = rospy.Publisher('/campo/robo_em_campo', Bool, queue_size=10)
pub_campo = rospy.Publisher('/campo/posicao_campo', Float32MultiArray, queue_size=10)

def image_callback_bola(msg):
    global img_bola
    try:
        img_bola = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Erro ao converter imagem da bola: %s", str(e))

def image_callback_robo(msg):
    global img_robo
    try:
        img_robo = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Erro ao converter imagem do robo: %s", str(e))

def pos_callback_bola(msg):
    global pos_bola
    pos_bola = (msg.x, msg.y)

def pos_callback_robo(msg):
    global pos_robo
    pos_robo = (msg.x, msg.y, msg.z)
    
def desenha_campo(event, x, y, flags, param):
    global ix, iy, fx, fy, cx, cy, px, py, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            fx, fy = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        fx, fy = x, y
        # Calcula o ponto central
        cx = (ix + fx) // 2
        cy = (iy + fy) // 2

    if event == cv2.EVENT_RBUTTONDOWN:
        px, py = x, y
        rospy.loginfo(f"Ponto recebido: ({px}, {py})")
        publish_objetivo(px, py)

def publish_objetivo(x, y):
    point_msg = Point()
    point_msg.x = x
    point_msg.y = y
    point_msg.z = 0
    pub_objetivo.publish(point_msg)

def publish_centro(x, y):
    point_msg = Point()
    point_msg.x = x
    point_msg.y = y
    point_msg.z = 0
    pub_centro.publish(point_msg)

def publish_robo_em_campo(robo_em_campo):
    object_msg = Bool()
    object_msg.data = robo_em_campo
    pub_robo_em_campo.publish(object_msg)

def verifica_objeto(frame, robox, roboy):
    global ix, iy, fx, fy, cx, cy, px, py, robo_em_campo

    if ix != -1 and iy != -1 and fx != -1 and fy != -1:
        # Verifica se o robô está dentro do campo e ajusta a cor do retângulo
        if robo_em_campo:
            cor_retangulo = (0, 255, 0)  # Verde quando o robô está dentro
        else:
            cor_retangulo = (0, 0, 255)  # Vermelho quando o robô está fora

        # Desenha o retângulo com a cor definida
        cv2.rectangle(frame, (ix, iy), (fx, fy), cor_retangulo, 2)
        
        coordenadas_campo = Float32MultiArray(data=[float(ix), float(fx), float(iy), float(fy)])
        pub_campo.publish(coordenadas_campo)

        # Desenha o ponto central do campo
        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
        publish_centro(cx, cy)

    if px != -1 and py != -1:
        cv2.circle(frame, (px, py), 5, (0, 0, 255), -1)

    if ix < robox < fx and iy < roboy < fy:
        robo_em_campo = True
    else:
        robo_em_campo = False

    publish_robo_em_campo(robo_em_campo)

    # Configurações de fonte
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_thickness = 2

    # Define o texto e a cor baseada na posição do robô
    if robo_em_campo:
        text = "Robo dentro do campo!"
        text_color = (0, 255, 0)
    else:
        text = "Robo fora do campo!"
        text_color = (0, 0, 255)

    # Tamanho do texto
    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]

    # Posição do texto para centralizar em relação ao campo
    campo_width = abs(fx - ix)
    text_x = ix + (campo_width - text_size[0]) // 2
    text_y = iy - 10  # Posiciona o texto 10 pixels acima do campo

    # Desenha o texto na imagem
    cv2.putText(frame, text, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

    return frame


def combine_images(img_robo, img_bola, pos_robo, pos_bola):
    global ix, iy, fx, fy, px, py

    # Verifica se as imagens foram recebidas
    if img_robo is None:
        rospy.logwarn("Imagem do robô não recebida.")
        return
    if img_bola is None:
        rospy.logwarn("Imagem da bola não recebida.")
        return

    # Cria uma imagem preta para o fundo do campo
    frame_campo = np.zeros_like(img_robo)  # Usa o mesmo tamanho da imagem do robô

    # Desenha o quadrado e o ponto na imagem do campo
    if ix != -1 and iy != -1 and fx != -1 and fy != -1:
        cv2.rectangle(frame_campo, (ix, iy), (fx, fy), (0, 255, 0), 2)
        cv2.circle(frame_campo, (cx, cy), 5, (255, 0, 0), -1)  # Desenha o ponto no centro do campo
    if px != -1 and py != -1:
        cv2.circle(frame_campo, (px, py), 5, (0, 0, 255), -1)

    # Adiciona o texto na imagem do campo
    if pos_robo is not None:
        frame_campo = verifica_objeto(frame_campo, pos_robo[0], pos_robo[1])

    # Funde as imagens (composição de alfa)
    alpha = 0.5
    fused_image = cv2.addWeighted(frame_campo, alpha, img_robo, 1 - alpha, 0)
    fused_image = cv2.addWeighted(fused_image, 1.0, img_bola, 0.5, 0)

    # Exibe a imagem fundida
    cv2.imshow("Campo do Robo", fused_image)

def main():
    rospy.init_node('fusion_node')

    # Subscrição dos tópicos de imagem
    rospy.Subscriber('/robo/frame_robo', Image, image_callback_bola)
    rospy.Subscriber('/bola/frame_bola', Image, image_callback_robo)

    # Subscrição dos tópicos de coordenadas (usando Point)
    rospy.Subscriber('/bola/posicao_bola', Point, pos_callback_bola)
    rospy.Subscriber('/robo/posicao_robo', Point, pos_callback_robo)

    cv2.namedWindow("Campo do Robo", cv2.WINDOW_AUTOSIZE)  # Cria a janela apenas uma vez

    cv2.setMouseCallback("Campo do Robo", desenha_campo)  # Define o callback do mouse para desenha_campo

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        combine_images(img_robo, img_bola, pos_robo, pos_bola)
        cv2.waitKey(1)  # Permite que as janelas de visualização sejam atualizadas
        rate.sleep()

    cv2.destroyAllWindows()  # Fecha todas as janelas ao finalizar o nó

if __name__ == '__main__':
    main()
