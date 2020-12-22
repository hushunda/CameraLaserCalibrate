'''
相机和单线激光标定：
    上一步已经相机和标定板的坐标变换
    相机坐标和激光时间完全统一
    当前：
        1. 找出标定板对应的激光点
        2. 拟合标定板的直线，
        3. 求解最优化
            方法1 SVD
            方法2 http://people.csail.mit.edu/bkph/articles/Nearest_Orthonormal_Matrix.pdf
                　https://home.deec.uc.pt/~jpbar/Publication_Source/pami2012.pdf
            方法3 随机梯度下降法求解
'''
import math
import numpy as np
import cv2
import random
from scipy.optimize import root,leastsq


class Optimize():
    def __init__(self,method = 'svd'):
        self.AllMethod = ['svd','sgd']
        self.method = method
        assert self.method in self.AllMethod

    def run(self,**args):
        return getattr(self,self.method)(args)

    def svd(self,args):
        '''标定
            输入：激光点位置,板子的法向量，板子到圆点的距离
             nHp = -d --> AH = b
               [ h1, h4, h7]
            H =[ h2, h5, h8]
               [ h3, h6, h9]
        '''

        # 数据预处理
        laser_points = args['laser_points']
        R_cpt = args['R_cpt']
        T_cpt = args['T_cpt']
        # H初始化
        if 'H0' in args and args['H0'] is not None:
            H0 = args['H0']
        else:
            H0 = -np.eye(3)
        Nc = []
        d = []
        laser_3dpoints = []
        for p,R,T in zip(laser_points,R_cpt,T_cpt):
            n = R[:,2]
            for p_i in p:
                laser_3dpoints.append([p_i[0],p_i[1],1])
                Nc.append(n)
                d.append(-sum(n * T))


        # 第一步 最小二乘求解
        def func(H,Nc,D,laser_points):
            Nc = np.array(Nc)
            D = np.array(D)
            H = H.reshape(3,3)
            laser_points = np.array(laser_points)
            return sum(Nc*(H.dot(laser_points)))+D

        def loss(H,Nc,D,laser_points):
            err =[]
            for n,d,p in zip(Nc,D,laser_points):
                err.append(func(H,n,d,p))
            return err
        para = leastsq(loss,H0,args=(Nc,d,laser_3dpoints))
        H0 = para[0].reshape(3,3)

        # 检验最小二乘法的结果
        H_true = args['H_true']
        # [R_clt,T_clt] = args['H_true']
        for n,di,p in zip(Nc,d,laser_3dpoints):
            err = func(H0,n,di,p)
            rig = func(H_true,n,di,p)
            # rig = sum(n*(R_clt.dot([p[0],p[1],0])+T_clt[:,0]))+di
            if err>1e-3:
                print('leastsq is wrong!')
        # 第二步 计算出旋转和平移矩阵
        h3 = H0[:,2]
        Rlc = H0.copy()
        Rlc[:,2] = np.cross(Rlc[:,0],Rlc[:,1])
        Rlc = Rlc.T
        Tlc = -Rlc.dot(h3)
        # 第三步 SVD计算Rlc.(Rlc可能不是旋转矩阵)
        U,s,V =np.linalg.svd(Rlc)

        Rlc = U.dot(V.T)
        return Rlc,Tlc

    def sgd(self,args):
        print('sgd')
        print(args)

class LaserCameraCalibration():
    def __init__(self,laser_data_path, camera_pose_path ):
        self.laser_data_path = laser_data_path
        self.camera_pose_path = camera_pose_path
        self.calibrate_data = {}
        self.load_data()
        self.optimize_method = 'svd'
        self.optimize = Optimize(self.optimize_method)

    def run(self):
        '''
        1. 找到激光点
        2. 拟合激光点
        3. 计算R和T
        '''
        self.find_laser_points()
        self.fit_laser_line()
        self.optimize.run()

    @staticmethod
    def calibrate(self,laser_points=None, R_cpt=None, T_cpt=None):
        pass

    def compute(self, x):
        R = []
        T = x[3:]


    def fit_laser_line(self):
        pass

    def find_laser_points(self):
        pass

    def load_data(self):
        with open(self.camera_pose_path,'r') as f:
            for line in f.readlines():
                line = line.strip('\n')
                self.calibrate_data[line[0]] = line[1:]

def theta_to_rotate_matrix(angle):
    anglex,angley,anglez = np.deg2rad(angle)
    rx = np.array([[1, 0, 0],
                   [0, np.cos(anglex), np.sin(anglex)],
                   [0, -np.sin(anglex), np.cos(anglex)]],
                  np.float32)

    ry = np.array([[np.cos(angley), 0, np.sin(angley)],
                   [0, 1, 0],
                   [-np.sin(angley), 0, np.cos(angley), ]],
                  np.float32)

    rz = np.array([[np.cos(anglez), np.sin(anglez), 0],
                   [-np.sin(anglez), np.cos(anglez), 0],
                   [0, 0, 1]], dtype=np.float32)

    r = rx.dot(ry).dot(rz)
    return r

def test_optimize():
    ''' 测试优化算法 camera laser plane '''
    # 假定激光和相机的位姿
    R_clt = theta_to_rotate_matrix([5,15,89]) # 激光到相机坐标系的旋转矩阵
    T_clt = np.array([1.0,15.3,26.3])[:,None] # 激光到相机坐标系的平移矩阵

    # 生成标定板的位姿
    R_cpt = np.array([theta_to_rotate_matrix([random.random()*90,random.random()*90,random.random()*90]) for i in range(10)])
    T_cpt = np.array([[random.random()*100,random.random()*50,random.random()*50] for i in range(10)])

    # 激光在板子上的位置
    opt = Optimize('svd')
    P_l = compute_laser_points(R_cpt, T_cpt, R_clt, T_clt)
    # 验证激光点正确性
    for i,p in enumerate(P_l):
        R_cpt_inv = np.linalg.inv(R_cpt[i])
        T = T_cpt[i]
        Nc = R_cpt[i][:,2]
        d = - Nc.dot(T)
        for pi in p:
            pp = R_cpt_inv.dot(R_clt.dot(np.array(pi)[:,None])+T_clt)-R_cpt_inv.dot(np.array(T)[:,None])
            err = sum(Nc*(R_clt.dot(np.array(pi)[:,None])+T_clt)[:,0])+d
            if abs(pp[2])>1e-3 or abs(err)>1e-3:
                print('laser point is wrong!')


    # 初始化
    tlc = [3,16,30]
    Ti = np.eye(3)
    Ti[:,2] = -np.array(tlc)
    H0 = theta_to_rotate_matrix([15,0,25]).dot(Ti)

    H_true = R_clt.copy()
    H_true[:,2] = T_clt[:,0]
    RT = opt.run(laser_points=P_l, R_cpt=R_cpt, T_cpt=T_cpt, H0=H0, H_true = H_true)#[R_clt,T_clt])#
    print('-'*10+'label'+'-'*10)
    print(np.linalg.inv(R_clt))
    print(-np.linalg.inv(R_clt).dot(T_clt))
    print('-' * 10 + 'prediction' + '-' * 10)
    print(RT[0].tolist())
    print(RT[1].tolist())


def temp_test():
    A = Optimize()
    A.run(test = 'sajka')

def compute_laser_points(R_cpt, T_cpt, R_clt, T_clt):
    '''
    RR.dot(p)+RT+T
    :return:
    '''
    P_l = []
    R_clt_inv = np.linalg.inv(R_clt)
    for R,T in zip(R_cpt,T_cpt):
        R_new = R_clt_inv.dot(R)
        T_new = R_clt_inv.dot(T)[:,None]-R_clt_inv.dot(T_clt)
        # 求激光线的方向
        # N1 = np.array([0,0,1])[:,None]
        N2 = R_new[:,2]
        N3 = [1,-N2[0]/N2[1],0]
        # 取出激光上一点
        R_new_inv = np.linalg.inv(R_new)
        T_new_inv = R_new_inv.dot(T_new)
        if R_new_inv[2,0]!=0:
            st_point = [float(T_new_inv[2]/R_new_inv[2,0]),0,0]
        else:
            st_point = [0, float(T_new_inv[1] / R_new_inv[1, 1]), 0]
        # 再取出一个点
        end_point = (np.array(st_point)+np.array(N3)*1).tolist()
        P_l.append([st_point,end_point])
    return P_l

if __name__ == '__main__':
    # lc_calibate = LaserCameraCalibration()
    # lc_calibate.calibrate()
    test_optimize()
    # temp_test()
