# coding:utf-8
'''
从记录的图片中计算图片的位姿。
保存的 txt 中格式是 时间 偏移矩阵 旋转矩阵的四元素 旋转矩阵的欧拉角（这些都是相机到世界坐标系的）


'''
import cv2
import numpy as np
import cv2.aruco as aruco
import math
import rosbag


from calibration import CameraModel
from scipy.spatial.transform import Rotation as R


cam_id = "fish_head_front"
data_root ='data/new_robot_with_arm/%s/laser_img/'%(cam_id)
inter_param_path = 'data/new_robot_with_arm/%s/inter_param.pkl'%(cam_id)

bag_path =data_root+'laser.bag'
new_bag_path = data_root+'new_laser.bag'
img_name_txt = data_root+\
               'time_img_name.txt'
pose_txt = data_root+'apriltag_pose.txt'
tag_size = 4.5  # 单位厘米
tag_type = 'chess'# 'chess' 'aruco'
img_save_root = data_root


def cal_cam_exter(img_bgr,tag_type,cam_model):

    if tag_type == 'chess':
        r_t = cam_model.calibrate_exter_from_img(img_bgr, (11, 8))
        return r_t
    elif tag_type=='aruco':
        # TODO
        '''
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              aruco_dict,
                                                              parameters=parameters)
        if ids is None:
            return []
        for i, point_pos in enumerate(corners):
            point = point_pos[0, 2]
            cv2.putText(img, str(ids[i][0]), (int(point[0]), int(point[1])), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(img, (int(point[0]), int(point[1])), 3, (0, 0, 255), -1)
            # aruco.drawDetectedMarkers(img, corners)
        print(corners)

        aruco_point = []
        new_corners = []
        for i, id in enumerate(np.squeeze(ids)):
            if id in config.aruco_point:
                aruco_point.append(config.aruco_point[id])
                new_corners.append(corners[i][0])
        if model.model == 'omnidir':
            undistorted = cv2.omnidir.undistortPoints(np.array(new_corners), model.K, model.D, model.xi, R=np.eye(3))
            undistorted = model.cal_omnidir_zs(undistorted)[:, None, :]
            undistorted = undistorted[:, 0]
        elif model.model =camera_model= 'normal':
            undistorted = cv2.undistortPoints(np.array(new_corners).reshape(-1, 2), model.K, model.D)
            undistorted = undistorted.reshape(-1, 4, 2)
        obj_point = []
        img_point = []
        for i in range(len(aruco_point)):
            x, y = aruco_point[i]
            for ind, corner in enumerate(undistorted[i]):
                obj_point.append([x + bais[ind][0], y + bais[ind][1], 0])
                img_point.append(corner)
        # 匹配点
        ret, rvec, tvec = cv2.solvePnP(np.array(obj_point), np.array(img_point), np.eye(3), np.zeros([4, 1]),
                                       flags=cv2.SOLVEPNP_ITERATIVE)
        model.rvec = rvec
        model.tvec = tvec
        # save
        f = open(param_save_path, 'wb')
        data = {'rvec': rvec, 'tvec': tvec}
        '''
    else:
        print('no the tag type ')
        return []

def main():
    cam_model = CameraModel(inter_param_path, model='omnidir')
    pose_f = open(pose_txt,'w')
    with open(img_name_txt, 'r') as f:
        for line in f.readlines():
            line = line.strip('\n')
            timestr, img_name = line.split(' ')
            img_rgb = cv2.imread(img_save_root+img_name)
            cam_pose = cal_cam_exter(img_rgb,tag_type,cam_model)
            if cam_pose:
                r_theta,t = cam_pose
                r_mat,_ = cv2.Rodrigues(r_theta)
                Rcw_inv = np.linalg.inv(r_mat)
                tcw_world = -Rcw_inv.dot(t)
                R_mat = R.from_matrix(Rcw_inv)
                R_quat = R_mat.as_quat()

                R_euler = R_mat.as_euler('xyz', degrees=False)
                write_info = [timestr]
                write_info.extend(np.squeeze(tcw_world*tag_size).tolist())
                print('camer pose T is ',np.squeeze(tcw_world*tag_size).tolist())
                write_info.extend(R_quat.tolist())
                write_info.extend(R_euler.tolist())
                pose_f.write(' '.join(list(map(str,write_info)))+'\n')
    pose_f.close()

    # delete laser

    txt_f = open(pose_txt,'r')
    time_str_list = ['%0.2f'%float(line.strip('\n').split(' ')[0]) for line in txt_f.readlines()]
    new_bag = rosbag.Bag(new_bag_path, 'w')
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            timestr = msg.header.stamp.to_sec()
            timestr ='%0.2f'%timestr
            print(timestr)
            if str(timestr) in time_str_list:
                print('save bag')
                new_bag.write(topic,msg,t)
    new_bag.close()
    txt_f.close()

if __name__ == '__main__':
    main()
