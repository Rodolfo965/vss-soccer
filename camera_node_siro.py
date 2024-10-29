#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node('camera_node')
    pub = rospy.Publisher('/camera_raw', Image, queue_size=10)
    cap = cv2.VideoCapture(2,cv2.CAP_V4L2)
    # Define a resolução desejada (largura x altura)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    bridge = CvBridge()
    rate = rospy.Rate(60)  # 10hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            continue
        
        try:
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f'Erro ao converter a imagem: {e}')

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
