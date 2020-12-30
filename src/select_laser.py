'''选择激光点'''

import cv2
import numpy as np

class SelectLaserData():
    def __init__(self,config):
        self.config = config
        self.min_laser_points = 5
        self.win_name = 'laser point'
        cv2.namedWindow(self.win_name,cv2.WINDOW_AUTOSIZE)
    def __call__(self, laser):
        '''假定激光在机器人4m范围内有效'''
        if self.config['select_laser']== 'auto':
            print('Description: red point is selected laser point ')
            laser_point = self.preprocess(laser)
            valid_point = self.find_laser_points(laser_point)
            line_points = self.fit_laser_line(valid_point)
            self.show(laser_point,valid_point,line_points, self.win_name)
        elif self.config['select_laser']=='manual':
            laser_point = self.preprocess(laser)
            canvas = self.show(laser_point,self.win_name,no_show=True)
            roi = cv2.selectROI(windowName=self.win_name, img=canvas, showCrosshair=True, fromCenter=False)
            filtered_point = self.filter_laser_point(laser_point,roi)
            valid_point = self.find_laser_points(filtered_point)
            line_points = self.fit_laser_line(valid_point)
            self.show(laser_point, valid_point, line_points, self.win_name)
        else:
            raise NotImplementedError

    def filter_laser_point(self,laser_point,roi):
        fliter_point = []
        return fliter_point

    def show(self,laser_point,valid_point=None,line_points=None,win_name = 'laser point',no_show = False):
        canvas = np.array((800,800,3),dtype=np.uint8)
        for p in laser_point:
            cv2.circle(canvas, (), 1, (255, 0, 0), -1)
        if valid_point is not None:
            for p in valid_point:
                cv2.circle(canvas, (), 1, (0, 0, 255), -1)
        if line_points is not None:
            cv2.line(canvas,laser_point[0],line_points[1],(0,255,0))
        if not no_show:
            cv2.imshow(win_name, canvas)
            cv2.waitKey(0 if self.config['one_by_one_show'] else 1)
        return canvas
    def fit_laser_line(self,valid_point):
        line_points = []
        return line_points

    def preprocess(self,laser):
        laser_point = []
        return laser_point
    def find_laser_points(self, laser_point):
        valid_point = []
        return valid_point