#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

#amarelo_baixo = np.array([20, 100, 20], np.uint8)
#amarelo_alto = np.array([40, 255, 255], np.uint8)

#vermelho_baixo1 = np.array([0, 100, 200], np.uint8)
#vermelho_baixo2 = np.array([175, 100, 20], np.uint8)
#vermelho_alto1 = np.array([8, 255, 255], np.uint8)
#vermelho_alto2 = np.array([179, 255, 255], np.uint8)

laranja_baixo = np.array([0, 143, 20], np.uint8)
laranja_alto = np.array([15, 255, 255], np.uint8)

def detecta_bola(frame):
    Bola_x = 0
    Bola_y = 0
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Ajustar a saturação e valor
    h, s, v = cv2.split(frameHSV)
    s = cv2.add(s, 30)
    v = cv2.add(v, -180)
    frameHSV = cv2.merge([h, s, v])
    
    mascaraLaranja=cv2.inRange(frameHSV,laranja_baixo,laranja_alto)
    #mascaraAmarela = cv2.inRange(frameHSV, amarelo_baixo, amarelo_alto)
    #mascaraVermelha1 = cv2.inRange(frameHSV, vermelho_baixo1, vermelho_alto1)
    #mascaraVermelha2 = cv2.inRange(frameHSV, vermelho_baixo2, vermelho_alto2)
    #mascaraVermelha = cv2.add(mascaraVermelha1, mascaraVermelha2)

    #mascaraVermelha = cv2.erode(mascaraVermelha, None, iterations=1)
    #mascaraVermelha = cv2.dilate(mascaraVermelha, None, iterations=2)
    #mascaraVermelha = cv2.medianBlur(mascaraVermelha, 13)

   # mascaraAmarela = cv2.erode(mascaraAmarela, None, iterations=1)
    #mascaraAmarela = cv2.dilate(mascaraAmarela, None, iterations=2)
    #mascaraAmarela = cv2.medianBlur(mascaraAmarela, 5)

    Bola, _ = cv2.findContours(mascaraLaranja, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in Bola:
        area = cv2.contourArea(c)
        if area > 100:
            M = cv2.moments(c)
            if M['m00'] == 0:
                M['m00'] = 1
            Bola_x = int(M['m10'] / M['m00'])
            Bola_y = int(M['m01'] / M['m00'])

            cv2.circle(frame, (Bola_x, Bola_y), 7, (0, 255, 0), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f'Bola {Bola_x},{Bola_y}', (Bola_x + 10, Bola_y), font, 0.75, (0, 255, 255), 1, cv2.LINE_AA)
            contornoBola = cv2.convexHull(c)
            cv2.drawContours(frame, [contornoBola], 0, (0, 0, 255), 3)

    return frame, Bola_x, Bola_y

def main():
    rospy.init_node('detector_bola')
    pub_frame_bola = rospy.Publisher('/bola/frame_bola', Image, queue_size=10)
    pub_ponto = rospy.Publisher('/bola/posicao_bola', Point, queue_size=10)
    bridge = CvBridge()

    def image_callback(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            frame, bolax, bolay = detecta_bola(frame)
            
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub_frame_bola.publish(image_msg)

            point_msg = Point()
            point_msg.x = bolax
            point_msg.y = bolay
            point_msg.z = 0
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