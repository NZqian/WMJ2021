#!/usr/bin/env python
# coding=utf-8

import numpy as np
import cv2

file = open("../build/Record.txt")

cv2.namedWindow("Window")

for line in file.readlines():
    background = np.zeros((1024, 1024, 3), dtype=np.uint8)
    line = line[:-1]
    points = line.split(" ")
    oriPt = (-int(100 * float(points[0])), int(100 * float(points[1])) + 256)
    prePt = (-int(100 * float(points[2])), int(100 * float(points[3])) + 256)

    img = cv2.circle(background, oriPt, 10, (255, 0, 0), 3)
    img = cv2.circle(background, prePt, 10, (0, 255, 0), 3)

    cv2.imshow("Window", img)
    key = cv2.waitKey(0)
    if key == ord('q'):
        break


