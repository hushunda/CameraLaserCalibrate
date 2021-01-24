
'''采集图片和标定内参'''
import cv2
import pickle
import os,sys
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from Configs import camera_config,data_root,data_collection_config
from src.camera_model import get_camera_model

def record_img():
    img_save_path =os.path.join(data_root,'inter_img')
    os.makedirs(img_save_path,exist_ok=True)
    print('record image in  ',img_save_path)

    cap = cv2.VideoCapture(data_collection_config ['cam_id'])
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(3, data_collection_config ['img_wight'])
    cap.set(4, data_collection_config ['img_hight'])
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))
    ind = 0
    print(' start to record laser data and image data')
    print('        press "C" to record a frame data        ' )
    print('        press "K" to finish record data        ' )
    while True:
        ret, img = cap.read()
        if ret == True:
            cv2.imshow('show', img)
            k = cv2.waitKey(1)
            if k == ord('c'):
                cv2.imwrite(os.path.join(img_save_path,'%0.3d.jpg' % (ind)), img)
                print('INFO: had record %d image'%(ind+1))
                ind += 1
            if k == ord('k'):
                break
    print('+++++++++++++ finish record data +++++++++++++')

def calibrate():
    inter_params_path = os.path.join(data_root,'inter_params.pkl')
    os.makedirs(data_root, exist_ok=True)

    images = [cv2.imread(os.path.join(data_root,'inter_img',path))
              for path in os.listdir(os.path.join(data_root,'inter_img')) if path.endswith('jpg')]
    img_shape = images[0].shape[:2]
    camera_model = get_camera_model(camera_config['camera_model'])(camera_config)
    objpoints, imgpoints = camera_model.detect_points(images)
    if camera_model.config['camera_model'] == 'fisheye':
        K = np.eye(3)
        D = np.zeros([4,1])
        rms = cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            img_shape[::-1],
            K,
            D,
            flags= cv2.fisheye.CALIB_FIX_SKEW + cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-6))
        data = {'K': K, 'D': D}
    if camera_model.config['camera_model'] == 'omnidir':
        K = np.eye(3)
        D = np.zeros([1,4])
        Xi = np.ones((1,1))
        rms = cv2.omnidir.calibrate(
            objpoints,
            imgpoints,
            img_shape[::-1],
            K,
            Xi,
            D,
            flags=cv2.omnidir.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-6))
        print("xi=np.array(" + str(Xi.tolist()) + ")")
        data = {'K': K, 'D': D, 'Xi':Xi}
    if camera_model.config['camera_model'] == 'pinhole':
        K = np.zeros((3,3))
        D = np.zeros((5,1))
        rms = cv2.calibrateCamera(
            objpoints,
            imgpoints,
            img_shape[::-1],
            K,
            D,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-6))
        data = {'K': K, 'D': D}
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")
    with open(inter_params_path, 'wb') as f:
        pickle.dump(data,f)


if __name__ == '__main__':
    # record_img()
    calibrate()
