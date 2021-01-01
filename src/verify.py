'''投影激光点到图像上'''
import os,sys,cv2
import pickle
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from src.camera_model import get_camera_model
from config import data_root,camera_config


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
    camera_model = get_camera_model(camera_config['camera_model'])(camera_config)

    # load TCL RT
    with open(os.path.join(data_root,'Laser2Camera.pkl'),'rb') as f:
        TCL = pickle.load(f)
    R_TCL = TCL['R']
    T_TCL = TCL['T']
    R_TCL = np.linalg.inv(R_TCL)
    T_TCL = - R_TCL.dot(T_TCL)
    # RT =np.array([[-0.08121124986486981, 0.07005866806829471, 0.9942316208630072, 0.1221971227223604],
    #      [-0.9966928448513781, -0.002858924935718341, -0.08121083406959438, 0.07310127015805462],
    #      [-0.002847089294866234, -0.9975387759765075, 0.07005915005078661, 1.149217402558326],
    #      [-0, -0, 0, 1]])
    # R_TCL = np.linalg.inv(RT[:3,:3])
    # T_TCL = -R_TCL.dot(RT[:3,3])

    cv2.namedWindow('laser on image',cv2.WINDOW_GUI_NORMAL)
    for laser,image in zip(laseres,images):
        h,w,_ = image.shape
        theta = np.arange(len(laser['ranges'])) * laser['angle_increment'] + laser['angle_min']
        ranges = np.array(laser['ranges'])
        x = ranges * np.cos(theta)
        y = ranges * np.sin(theta)

        laser_point = np.vstack([x,y,np.zeros(len(x))]).T
        cam_points = R_TCL.dot(laser_point.T).T+T_TCL
        pixel_points = camera_model.projectPoints(cam_points)
        pixel_points = pixel_points[np.logical_and(np.logical_and(pixel_points[:,0]>0,pixel_points[:,1]>0),
                                    np.logical_and(pixel_points[:,0]<w,pixel_points[:,1]<h))]
        for p in pixel_points:
            cv2.circle(image, (int(p[0]), int(p[1])), 1, (255, 0, 0), -1)
        cv2.imshow('laser on image',image)
        cv2.waitKey()

if __name__ == '__main__':
    main()
