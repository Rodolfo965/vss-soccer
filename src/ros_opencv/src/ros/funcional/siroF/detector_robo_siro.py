#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

verde_baixo = np.array([35, 70, 20], np.uint8)
verde_alto = np.array([85, 255, 255], np.uint8)
amarelo_baixo = np.array([17, 100, 20], np.uint8)
amarelo_alto = np.array([40, 255, 255], np.uint8)

def detecta_robo(frame):
    
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    h, s, v = cv2.split(frameHSV)
    s = cv2.add(s, 50)
    v = cv2.add(v, -100)
    frameHSV = cv2.merge([h, s, v])

    mascaraAmarela = cv2.inRange(frameHSV, amarelo_baixo, amarelo_alto)
    mascaraVerde = cv2.inRange(frameHSV, verde_baixo, verde_alto)

    mascaraVerde = cv2.erode(mascaraVerde, None, iterations=1)
    mascaraVerde = cv2.dilate(mascaraVerde, None, iterations=2)
    mascaraAmarela = cv2.erode(mascaraAmarela, None, iterations=1)
    mascaraAmarela = cv2.dilate(mascaraAmarela, None, iterations=2)

    mascaraVerde = cv2.medianBlur(mascaraVerde, 13)
    mascaraAmarela = cv2.medianBlur(mascaraAmarela, 5)

    Frente_robo, _ = cv2.findContours(mascaraAmarela, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Tras_robo, _ = cv2.findContours(mascaraVerde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centro_frontal = None
    centro_traseiro = None
    ponto_mediox = 0
    ponto_medioy = 0
    theta = 0
    
    for c in Frente_robo:
        area = cv2.contourArea(c)
        if area > 700:
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = 1
            Frente_x = int(M["m10"] / M["m00"])
            Frente_y = int(M["m01"] / M["m00"])
            centro_frontal = (Frente_x, Frente_y)
            ContornoFrente = cv2.convexHull(c)
            cv2.drawContours(frame, [ContornoFrente], 0, (255, 0, 0), 3)

    for c in Tras_robo:
        area = cv2.contourArea(c)
        if area > 700:
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = 1
            Tras_x = int(M["m10"] / M["m00"])
            Tras_y = int(M["m01"] / M["m00"])
            centro_traseiro = (Tras_x, Tras_y)
            novoContorno = cv2.convexHull(c)
            cv2.drawContours(frame, [novoContorno], 0, (255, 0, 0), 3)

    if centro_frontal and centro_traseiro:
        ponto_mediox = int((centro_frontal[0] + centro_traseiro[0]) / 2)
        ponto_medioy = int((centro_frontal[1] + centro_traseiro[1]) / 2)

        vetor = np.array([centro_frontal[0] - centro_traseiro[0], centro_frontal[1] - centro_traseiro[1]])
        theta = np.arctan2(vetor[1], vetor[0])
        theta_deg = np.degrees(theta)

        if theta_deg < 0:
            theta_deg += 360

        tamanho_seta = 40
        seta_fim_x = int(ponto_mediox + tamanho_seta * np.cos(theta))
        seta_fim_y = int(ponto_medioy + tamanho_seta * np.sin(theta))
        cv2.arrowedLine(frame, (ponto_mediox, ponto_medioy), (seta_fim_x, seta_fim_y), (0, 0, 255), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, 'theta: {:.2f} graus'.format(theta_deg), (10, 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.circle(frame, (ponto_mediox + 10, ponto_medioy), 7, (0, 255, 255), -1)
        cv2.putText(frame, f'Jarvan I: {ponto_mediox}, {ponto_medioy}', (ponto_mediox + 10, ponto_medioy), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

    return frame, ponto_mediox, ponto_medioy, theta

def main():
    rospy.init_node('detector_robo')
    pub_frame_robo = rospy.Publisher('/robo/frame_robo', Image, queue_size=10)
    pub_ponto = rospy.Publisher('/robo/posicao_robo', Point, queue_size=10)
    bridge = CvBridge()

    def image_callback(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            frame, x, y, theta = detecta_robo(frame)
            
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub_frame_robo.publish(image_msg)

            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = theta
            pub_ponto.publish(point_msg)

        except CvBridgeError as e:
            rospy.logerr(f'Erro ao converter a imagem: {e}')

    rospy.Subscriber('/camera_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
