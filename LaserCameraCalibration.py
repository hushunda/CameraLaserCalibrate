'''
标定的主函数
'''

import os
import pickle
from src.select_laser import SelectLaserData
from src.camera_model import get_camera_model
from src.calibrate_laser_camera import Optimize
from Configs import camera_config,calibrate_config,data_root

def main():
    '''标定分为是三步'''
    # load data
    with open(os.path.join(data_root,'laser_image_2.pkl'),'rb') as f:
        laser_img_data = pickle.load(f)
    with open(os.path.join(data_root,'laser_image_1.pkl'),'rb') as f:
        laser_img_data1 = pickle.load(f)
    if len(laser_img_data)==0:
        print('No Find data')
        return
    images = []
    laseres = []
    for las,img in laser_img_data:
        if img is not None and las:
            images.append(img)
            laseres.append(las)

    for las,img in laser_img_data1:
        if img is not None and las:
            images.append(img)
            laseres.append(las)


    # First compute camera pose
    camera_model = get_camera_model(camera_config['camera_model'])(camera_config)
    Nc,Ds = camera_model(images)

    # Second select laser data
    select_laser = SelectLaserData(calibrate_config)
    valid_laser = select_laser(laseres)

    # Third Calibrate
    calibrate = Optimize(calibrate_config)
    R,T = calibrate(Nc = Nc,Ds= Ds,laser_points = valid_laser)
    with open(os.path.join(data_root,'Laser2Camera.pkl'),'wb') as f:
        pickle.dump({'R':R,'T':T},f, protocol=2)
    print('R:  \n', R,'\nT: \n',T)

if __name__ == '__main__':
    main()
