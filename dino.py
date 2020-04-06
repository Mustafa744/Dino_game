import numpy as np
import cv2
from PIL import ImageGrab
from pynput.keyboard import Key, Controller
keyboard = Controller()

low_red = np.array([161, 155, 84])
high_red = np.array([179, 255, 255])

low_red = np.array([0,0,0])
high_red = np.array([0,0,255])

while 1:
    full_img=ImageGrab.grab(bbox=(100, 100, 640, 480)) #x, y, w, h
    full_frame = np.array(full_img)
    full_frame = cv2.cvtColor(full_frame, cv2.COLOR_RGB2BGR)

    img = ImageGrab.grab(bbox=(100, 100, 250, 345)) #x, y, w, h
    frame = np.array(img)
    frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    red_mask = cv2.inRange(hsv_frame, low_red, high_red)

    kernel = np.ones((3, 3), np.uint8)

    red_mask = cv2.erode(red_mask, kernel, iterations=1)
    red_mask = cv2.dilate(red_mask, kernel, iterations=9)
    cv2.imshow("frame2", red_mask)

    red = cv2.bitwise_and(frame, frame, mask=red_mask)

    im2, contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)>0 :

        a = max(contours, key = cv2.contourArea)

        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x))

        x, y, w, h = cv2.boundingRect(a)
        #cv2.circle(frame, (x,y), 10, (200,0,0), 5)
        cv2.circle(frame, (0,200), 10, (200,0,0), 5)


        print(y+h)
        if y>160:
            cv2.drawContours(frame, [a], -1, (0, 255, 0), 3)
            cv2.rectangle(full_frame, (x, y), (x + w+50, y + h+50), (0, 250, ), 3)
            if x<40 :
                cv2.circle(frame, (0, 200), 40, (200, 0, 0), 5)
                keyboard.press(Key.up)
                keyboard.release(Key.up)

    cv2.imshow("frame", full_frame)

    key = cv2.waitKey(1)
    if key == 27:
        cv2.destroyAllWindows()
        break
