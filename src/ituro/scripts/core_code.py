#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from threading import Thread
import time
from matplotlib import pyplot as plt
from ituro.msg import cv_cizgi

class lane_follower:
    def __init__(self):
        self.pub = rospy.Publisher('/ituro/cizgi', cv_cizgi, queue_size=10)
        self.stopped = False
        self.error_lst = []
        self.angle_lst = []
        self.frame = None
        self.lb = None
        self.up = None
        self.l_b = np.array([0, 33, 0])
        self.u_b = np.array([255, 255, 255])
        self.hsv_frame = None
        self.mask_1 = None
        self.inter_frame = None
        self.kernel = np.ones((3, 3), np.uint8)  # to be used in pre_process kernel 3x3
        self.mask = None
        self.dilated = None
        self.contours_blk = None
        self.b_box = None
        self.box = None
        self.x_min = None
        self.y_min = None
        self.y_dist_px = None
        self.w_min = None
        self.h_min = None
        self.enclosing_area = None
        self.set_point = None
        self.error = None
        self.normal_error = None
        self.normal_angle = None
        self.line_side = None
        self.angle = None
        self.sorted_contour = None
        self.c_x, self.c_y = None, None

    def publish(self, publish=True, error=None, distance=None, angle=None, line_side=None, encl=None):  # TODO adjust the publisher vs
        if publish:
            msg = cv_cizgi()
            msg.x = self.c_x
            msg.y = self.c_y
            msg.theta = angle
            self.pub.publish()
            print("error : " + str(error) + " || " + "angle : " + str(angle) + " || " + "line side : " + str(line_side) + " || " + "distance : " + str(distance) + " || " + "area : " + str(encl))

    def capture(self):  # TODO delete it
        _, self.frame = self.cap.read()
        self.frame = cv2.resize(self.frame, (640, 480), fx=0, fy=0, interpolation=cv2.INTER_CUBIC)
        return self.frame


    def pre_process(self, frame=None, rotate=False):
        if rotate:self.frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.frame = frame.copy()
        self.hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask_1 = cv2.inRange(self.hsv_frame, self.l_b, self.u_b)
        self.inter_frame = cv2.bitwise_and(self.frame, self.frame, mask=self.mask_1)
        self.inter_frame = cv2.cvtColor(self.inter_frame, cv2.COLOR_BGR2GRAY)
        self.inter_frame = cv2.GaussianBlur(self.inter_frame, (5, 5), 0)  # ksize happens to be 5*5=25
        self.mask = cv2.erode(self.inter_frame, self.kernel, iterations=5)
        self.dilated = cv2.dilate(self.mask, self.kernel, iterations=9)
        return self.dilated

    def contour_operations(self, frame=None):
        (self.contours_blk, _) = cv2.findContours(frame.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        self.contours_blk = list(self.contours_blk)
        #self.contours_blk.sort(key=cv2.minAreaRect)

        if len(self.contours_blk) > 0:
            self.sorted_contour = max(self.contours_blk, key=cv2.contourArea)  # It refers to the single argument function to customize the sort order. The function is applied to each item on the iterable.
            m = cv2.moments(self.sorted_contour)
            if m["m00"] != 0:  # center of the main path contour
                self.c_x = int(m["m10"] / m["m00"])
                self.c_y = int(m["m01"] / m["m00"])
            self.b_box = cv2.minAreaRect(self.sorted_contour)
        return self.b_box, (self.c_x, self.c_y)


    def process(self, b_box=None, frame=None, center=None):

        (self.x_min, self.y_min), (self.w_min, self.h_min), self.angle = b_box

        if self.angle < -45:
            self.angle = 90 + self.angle
        if self.w_min < self.h_min and self.angle > 0:
            self.angle = (90 - self.angle) * -1
        if self.w_min > self.h_min and self.angle < 0:
            self.angle = 90 + self.angle

        self.frame = frame
        self.error = self.frame.shape[0] / 2 - self.y_min
        # self.error_lst.append(self.error)
        # self.angle_lst.append(self.angle)

        self.y_dist_px = self.frame.shape[0] / 2 - self.y_min

        if abs(self.error) < 20:  # TODO ajust the value, vital !!!
            self.line_side = "correct"
        else:
            if self.error > 0:
                self.line_side = "right"
            elif self.error <= 0:
                self.line_side = "left"

        # cv2.line(self.frame, (int(self.x_min), int(self.y_min)), (int(self.x_min + 50), int(self.y_min)), (255, 0, 0), 3)
        cv2.line(self.frame, (self.frame.shape[1] // 2, self.frame.shape[0] // 2), ((self.frame.shape[1] + 40) // 2, self.frame.shape[0] // 2), (255, 0, 0), 3)
        cv2.circle(self.frame, (self.frame.shape[1] // 2, self.frame.shape[0] // 2), 3, (255, 255, 255), -1)  # center of the frame
        cv2.line(self.frame, (self.frame.shape[1] // 2, int(self.y_min)), (self.frame.shape[1] // 2, self.frame.shape[0] // 2), (0, 255, 255), 1)  # line b/ drone and the center of the contour
        self.box = cv2.boxPoints(self.b_box)
        self.box = np.int0(self.box)
        cv2.drawContours(self.frame, [self.box], 0, (0, 0, 255), 2)
        cv2.circle(self.frame, center, 3, (0,0,255), -1)  # center of the contour

        self.enclosing_area = self.w_min * self.h_min

        return self.frame, self.error, self.y_dist_px, self.angle, self.line_side, self.enclosing_area

    def display(self, publish=True, frame=None, angle=None, distance = None, error=None, line_side=None, encl=None):
        if publish:self.publish(error=error, distance=distance, angle=angle, line_side=line_side, encl=encl)  # TODO arrange
        cv2.imshow("main", frame)

class VideoGet:

    def __init__(self, source="IMG_2086.MOV"):
        self.stream = cv2.VideoCapture(source)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()
                cv2.waitKey(10)  # TODO delete it before main use

    def stop(self):
        self.stopped = True



def main():
    inst_1 = lane_follower()
    inst_2 = VideoGet(source=0).start()
    i = 0
    x_i = []
    y_area = []
    while inst_2.grabbed:
        try:
            i+=1
            x_i.append(i)
            #raw_frame = inst_1.capture()  -----without threading-----
            raw_frame = inst_2.frame
            raw_frame = cv2.resize(raw_frame, (640, 480), fx=0, fy=0, interpolation=cv2.INTER_CUBIC)
            dilated_frame = inst_1.pre_process(frame=raw_frame, rotate=False)
            b_box, (c_x, c_y) = inst_1.contour_operations(frame=dilated_frame)
            refined_frame, error, y_dist_px, angle,  line_side, enc = inst_1.process(b_box=b_box, frame=raw_frame, center=(c_x, c_y))
            y_area.append(enc)
            inst_1.display(publish=True, frame=refined_frame, angle=angle, error=error, line_side=line_side, distance=y_dist_px, encl=enc)
            if cv2.waitKey(20) == ord("q"): break
        except:
            break

    plt.plot(x_i, y_area)  # TODO delete it
    plt.show()
    cv2.destroyAllWindows()
    inst_1.cap.release()

if __name__ == "__main__":
    main()