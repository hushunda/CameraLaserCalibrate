'''选择激光点'''

import cv2
import numpy as np
import math

class SelectLaserData():
    def __init__(self,config):
        self.config = config
        self.min_laser_points = 3
        self.win_name = 'laser point'
        cv2.namedWindow(self.win_name,cv2.WINDOW_AUTOSIZE)
    def __call__(self, laser):
        '''假定激光在机器人4m范围内有效'''
        return_points = []
        if self.config['select_laser']== 'auto':
            print('Description: red point is selected laser point ')
            laser_point = self.preprocess(laser)
            for lp in laser_point:
                valid_point = self.find_laser_points(lp)
                line_points = self.fit_laser_line(valid_point)
                self.show(lp,valid_point,line_points, self.win_name)
                # return_points.append(line_points)
                return_points.append(valid_point)
        elif self.config['select_laser']=='manual':
            laser_point = self.preprocess(laser)
            for lp in laser_point:
                canvas = self.show(lp,win_name=self.win_name,no_show=True)
                roi = cv2.selectROI(windowName=self.win_name, img=canvas, showCrosshair=True, fromCenter=False)
                filtered_point = self.filter_laser_point(lp,roi)
                # self.show(filtered_point, win_name=self.win_name)
                valid_point = self.find_laser_points(filtered_point)
                # self.show(valid_point, win_name=self.win_name)
                line_points = self.fit_laser_line(valid_point)
                self.show(lp, valid_point, line_points, self.win_name)
                # return_points.append(line_points)
                return_points.append(valid_point)
        else:
            raise NotImplementedError

        return return_points

    def filter_laser_point(self,laser_point,roi):
        fliter_point = []
        pixel_point = (laser_point.copy()+4)*100
        for i in range(len(pixel_point)):
            if self.point_in_box(pixel_point[i], roi):
                fliter_point.append(laser_point[i])
        return np.array(fliter_point)

    def point_in_box(self,point,box):
        box = [box[0],box[1],box[0]+box[2],box[1]+box[3]]
        if box[0]<point[0]<box[2] and box[1]<point[1]<box[3]:
            return True
        else:
            return False

    def show(self,laser_point,valid_point=None,line_points=None,win_name = 'laser point',no_show = False):
        canvas = np.zeros((800,800,3),dtype=np.uint8)

        for p in laser_point:
            if math.isinf(p[0]) or math.isinf(p[1]) or np.isnan(p[0]) or np.isnan(p[1]):
                continue
            p = (p+4)*100
            cv2.circle(canvas, (int(p[0]),int(p[1])), 1, (255, 255, 0), -1)
        if valid_point is not None:
            for p in valid_point:
                p = (p + 4) * 100
                cv2.circle(canvas, (int(p[0]),int(p[1])), 1, (0, 0, 255), -1)
        if line_points is not None and len(line_points)>0:
            line_points = [[int((y+4)*100) for y in x] for x in line_points]
            cv2.line(canvas,tuple(line_points[0]),tuple(line_points[1]),(0,255,0))
        if not no_show:
            cv2.imshow(win_name, canvas)
            cv2.waitKey(0 if self.config['one_by_one_show'] else 1)
        return canvas

    def fit_laser_line(self,valid_point,min_num_point = 10,min_valid_len = 4):
        if len(valid_point) <min_num_point:
            return []
        # 取板子上相邻num个点最近的点
        num = 1
        out_valid = []
        for i in range(0,len(valid_point),num):
            t = valid_point[i:i+num]
            out_valid.append(t[np.argmin(t[:,0])])
        valid_point = np.array(out_valid)
        kb= np.polyfit(valid_point[:,0],valid_point[:,1],1)
        dis = [self.get_distance_from_point_to_line(p,(0,kb[1]),(-kb[1]/kb[0],0)) for p in valid_point]
        if max(dis)<0.01:
            return [valid_point[0],valid_point[-1]]
        index = np.where(np.array(dis)<0.01)[0]
        if len(index) == 0:
            print('这条线不够')
            return []
        if index[-1]-index[0]>min_valid_len:
            return [valid_point[index[-1]], valid_point[index[0]]]
        print('这条线不够')
        return []

    def get_distance_from_point_to_line(self, point, line_point1, line_point2):
        # 对于两点坐标为同一点时,返回点与点的距离
        if line_point1 == line_point2:
            point_array = np.array(point)
            point1_array = np.array(line_point1)
            return np.linalg.norm(point_array - point1_array)
        # 计算直线的三个参数
        A = line_point2[1] - line_point1[1]
        B = line_point1[0] - line_point2[0]
        C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
            (line_point2[0] - line_point1[0]) * line_point1[1]
        # 根据点到直线的距离公式计算距离
        distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A ** 2 + B ** 2))
        return distance

    def preprocess(self, laser):
        laser_point = []
        for la in laser:
            theta = np.arange(len(la.ranges))*la.angle_increment+la.angle_min
            ranges = np.array(la.ranges)
            x = ranges*np.cos(theta)
            y = ranges*np.sin(theta)
            laser_point.append(np.vstack([x,y]).T)
        return laser_point

    def find_laser_points(self, laser_point,seg_max_dis = 0.02,max_dis = 1, min_angle = -40,max_angle = 40):
        '''只取出3米内的正前方的激光线,120度内'''
        dis = np.linalg.norm(laser_point,axis=1)
        theta = np.arccos(laser_point[:, 0] / dis)
        valid_point = laser_point[
            np.logical_and(
                np.logical_and(
                        np.logical_and(
                            np.deg2rad(min_angle)<theta,theta<np.deg2rad(max_angle)
                ),dis<max_dis),
            np.logical_not(np.isinf(dis)))]

        '''分割激光线段'''
        valid_point = np.array(valid_point)
        point_diff_dis = np.linalg.norm(valid_point[1:]-valid_point[:-1],axis=1)
        index = np.where(point_diff_dis>seg_max_dis)[0]
        if len(index)>0:
            seg_line = [valid_point[:index[0]+1]]
            for i in range(len(index)-1):
                if index[i+1]-index[i]<3:continue
                seg_line.append(valid_point[index[i]+1:index[i+1]+1])
            seg_line.append(valid_point[index[-1]+1:])
        else:
            return valid_point

        line = [seg_line[0]]
        for l in seg_line[1:]:
            if np.linalg.norm(line[-1][-1]-l[0])<seg_max_dis:
                np.append(line[-1],l)
            else:
                line.append(l)
        #
        len_line = [len(x) for x in line]
        valid_line_point = line[len_line.index(max(len_line))]
        return valid_line_point