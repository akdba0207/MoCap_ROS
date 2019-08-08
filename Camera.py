#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from geometry_msgs.msg import Point
from datetime import datetime


def callback(value):
    pass


def cameratracker():
    rospy.init_node('cameratracker', anonymous=True)
    pub = rospy.Publisher('cameradata', Point, queue_size=100)
    rate = rospy.Rate(100)  # 10hz
    pos = Point()
    posxp1 = 0
    posxp2 = 0
    cam = cv2.VideoCapture(1)  # 카메라 생성

    if cam.isOpened() == False:  # 카메라 생성 확인
        print
        'Can\'t open the CAM(%d)' % (1)
        exit()
    cam.set(3, 320)
    cam.set(4, 240)
    cam.set(5, 100)

    cv2.namedWindow('Trackbar')
    cv2.createTrackbar('Low H', 'Trackbar', 55, 179, callback)      #yello 22 37 123 255 83 255
    cv2.createTrackbar('High H', 'Trackbar', 7, 179, callback)     #red 0 7 121 255 128 255
    cv2.createTrackbar('Low S', 'Trackbar', 121, 255, callback)
    cv2.createTrackbar('High S', 'Trackbar', 255, 255, callback)
    cv2.createTrackbar('Low V', 'Trackbar', 160, 255, callback)
    cv2.createTrackbar('High V', 'Trackbar', 255, 255, callback)
    posx = 160
    posy = 120
    while not rospy.is_shutdown():
        ret, src = cam.read()
        if not ret:
            break

        src_hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        v1_min = cv2.getTrackbarPos('Low H', 'Trackbar')
        v1_max = cv2.getTrackbarPos('High H', 'Trackbar')
        v2_min = cv2.getTrackbarPos('Low S', 'Trackbar')
        v2_max = cv2.getTrackbarPos('High S', 'Trackbar')
        v3_min = cv2.getTrackbarPos('Low V', 'Trackbar')
        v3_max = cv2.getTrackbarPos('High V', 'Trackbar')
        finalimage = cv2.inRange(src_hsv, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        m = cv2.moments(finalimage)
        area = cv2.countNonZero(finalimage)
        if area < 300:
            posx = posx
            posy = posy
        else:
            posx = int(m["m10"] / m["m00"])
            posy = int(m["m01"] / m["m00"])

        posx = (posxp2+posxp1+posx)/3

        if posx <= 10 :
            posx = 10
        elif posx >= 310:
            posx = 310

        cv2.circle(src, (posx, posy), 5, (255, 255, 255), -1)
        cv2.putText(src, "centroid", (posx - 25, posy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow("cam", src)
        cv2.imshow("final image", finalimage)

        if cv2.waitKey(1) >= 0:
            break

        pos.x = -(posx-138)
        pos.y = -(posy-120)

        pub.publish(pos)
        rate.sleep()
        posxp1 = posx
        posxp2 = posxp1


    cam.release()
    cv2.destroyWindow('CAM_Window')


if __name__ == '__main__':
    try:
        cameratracker()
    except rospy.ROSInterruptException:
        pass
