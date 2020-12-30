'''
标定的主函数
'''

import os
import pickle
from src.select_laser import SelectLaserData
from src.camera_model import get_camera_model
from src.calibrate_laser_camera import Optimize
from config import camera_config,calibrate_config

def main():
    '''标定分为是三步'''
    # load data
    with open(os.path.join(calibrate_config['data_root'],'laser_image.pkl'),'rb') as f:
        laser_img_data = pickle.load(f)
    if len(laser_img_data):
        print('No Find data')
        return

    images = [x[1] for x in laser_img_data]
    laseres = [x[0] for x in laser_img_data]


    # First compute camera pose
    camera_model = get_camera_model(camera_config['camera_model'])(camera_config)
    Nc,Ds = camera_model(images)

    # Second select laser data
    select_laser = SelectLaserData(calibrate_config)
    valid_laser = select_laser(laseres)

    # Third Calibrate
    calibrate = Optimize(calibrate_config)
    R,T = calibrate(Nc = Nc,Ds= Ds,laser_points = valid_laser)
    print('R:  \n', R,'T: \n',T)

if __name__ == '__main__':
    main()
