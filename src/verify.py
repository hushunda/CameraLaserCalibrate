'''投影激光点到图像上'''
import os,sys,cv2
import pickle
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from src.camera_model import get_camera_model
from Configs import data_root,camera_config


def main():
    # load data

    with open(os.path.join(data_root, 'laser_image.pkl'), 'rb') as f:
        laser_img_data = pickle.load(f)
    if len(laser_img_data) == 0:
        print('No Find data')
        return
    images = []
    laseres = []
    for las, img in laser_img_data:
        if img is not None and las:
            images.append(img)
            laseres.append(las)

    # load camera model
    camera_config['inter_params_path']=camera_config['inter_params_path']
    camera_model = get_camera_model(camera_config['camera_model'])(camera_config)

    # load TCL RT
    with open(os.path.join(data_root,'Laser2Camera.pkl'),'rb') as f:
        TCL = pickle.load(f)
    R_TCL = TCL['R']
    T_TCL = TCL['T']

    theta_range = (-60/180*3.14,60/180*3.14)

    cv2.namedWindow('laser on image',cv2.WINDOW_GUI_NORMAL)
    for laser,image in zip(laseres,images):
        h,w,_ = image.shape
        theta = np.arange(len(laser.ranges)) * laser.angle_increment + laser.angle_min
        ranges = np.array(laser.ranges)
        x = ranges * np.cos(theta)
        y = ranges * np.sin(theta)

        laser_point = np.vstack([x,y,np.zeros(len(x))]).T
        laser_point = laser_point[np.logical_and(theta<theta_range[1],theta>theta_range[0])]
        cam_points = R_TCL.dot(laser_point.T).T+T_TCL
        pixel_points = camera_model.projectPoints(cam_points)
        pixel_points = pixel_points[np.logical_and(np.logical_and(pixel_points[:,0]>0,pixel_points[:,1]>0),
                                    np.logical_and(pixel_points[:,0]<w,pixel_points[:,1]<h))]
        for p in pixel_points:
            cv2.circle(image, (int(p[0]), int(p[1])), 5, (255, 255, 0), -1)
        cv2.imshow('laser on image',image)
        cv2.waitKey()

if __name__ == '__main__':
    main()
