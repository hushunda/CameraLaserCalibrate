'''
相机模型
'''
import cv2
import numpy as np
import pickle

def get_camera_model(model_name):
    return eval(model_name.capitalize())

class BaseCameraModel(object):
    def __init__(self,config):
        print('load %s camera mode'%config['camera_model'])
        self.load_camera_params(config)
        self.config = config

    def load_camera_params(self,config):
        '''载入相机内参'''
        if config['inter_params_path'] is not None:
            with open(config['inter_params_path'],'rb') as f:
                data = pickle.load(f)
                for k in data:
                    setattr(self,k,data[k])
        else:
            keys = ['K','D','Xi']
            for k in keys:
                if config[k] is not None:
                    setattr(self, k, np.array(config[k]))

    def detect_points(self,images):
        '''检测板子角点'''
        if self.config['tag_type'] == 'chess':
            checkerboard = self.config['checkerboard']
            objp = np.zeros((1, checkerboard[0] * checkerboard[1], 3), np.float32)
            objp[0, :, :2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2)
            objpoints = []
            imgpoints = []
            for img in images:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)
                ret, corners = cv2.findChessboardCorners(gray, checkerboard,
                                                         cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
                if ret == True:
                    objpoints.append(objp)
                    corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1),
                                                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.01))
                    imgpoints.append(corners2.reshape(1, -1, 2))
            return objpoints, imgpoints
        elif self.config['tag_type'] == 'aruco':
            raise NotImplementedError
            # TODO

    def compute_nc_d(self,objpoints,camerapoints):
        '''计算相机坐标下:板子的法线Nc,圆点到板子的距离'''
        Nc = []
        Ds = []
        for op,cp in zip(objpoints,camerapoints):
            success, rotation_vector, translation_vector= cv2.solvePnP(op, cp, np.eyes(3),
                         np.zeros(4), flags=cv2.SOLVEPNP_ITERATIVE)
            if success:
                Nc.append(rotation_vector[:,2])
                Ds.append((rotation_vector[:,2]*translation_vector).sum())
        return Nc,Ds

    def __call__(self,images):
        objpoints, imgpoints = self.detect_points(images)
        camerapoints = self.pixel2camera(imgpoints)
        return self.compute_nc_d(objpoints, camerapoints)

    def pixel2camera(self, imgpoints):
        raise NotImplementedError

class Omnidir(BaseCameraModel):
    '''
    FOV >150
    '''
    def __init__(self,config):
        super(Omnidir).__init__(config)

    def pixel2camera(self,imgpoints):
        camerapoints = cv2.omnidir.undistortPoints(imgpoints,self.K,self.D,self.Xi)
        camerapoints = self.compute_camera_coord(camerapoints)
        return camerapoints

    def compute_camera_coord(self,point):
        '''将坐标点恢复到相机坐标系'''
        x=point[:,:,0]
        y=point[:,:,1]
        r2 = x ** 2 + y ** 2
        a = r2 + 1
        b = 2 * self.xi * r2
        cc = r2 * self.xi ** 2 - 1
        z = np.squeeze((-b + np.sqrt(b * b - 4 * a * cc)) / (2 * a))
        scale = np.squeeze(self.xi + z)
        point = np.stack((x * scale / z, y * scale / z), axis=2)
        return point

class Fisheye(BaseCameraModel):
    '''
    120<FOV<150
    '''
    def __init__(self,config):
        super(Fisheye).__init__(config)

    def pixel2camera(self,imgpoints):
        camerapoints = cv2.fisheye.undistortPoints(imgpoints,self.K,self.D)
        return camerapoints

class PinHole(BaseCameraModel):
    '''
    FOV <120
    '''
    def __init__(self,config):
        super(PinHole).__init__(config)

    def pixel2camera(self,imgpoints):
        camerapoints = cv2.undistortPoints(imgpoints,self.K,self.D)
        return camerapoints


class CustomModel(BaseCameraModel):
    def __init__(self,config):
        super(CustomModel).__init__(config)

    def pixel2camera(self,imgpoints):
        camerapoints = []
        raise NotImplementedError
        return camerapoints

if __name__ == '__main__':
    a= get_camera_model('Omnidir')
    print(a)