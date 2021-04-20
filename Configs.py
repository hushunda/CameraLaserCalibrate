import os
data_root = './data/calibration_data'# 绝对路径
data_collection_config = {'cam_id':0,
                          'img_wight':1280,'img_hight':720,
                          'scan_topic_name': 'scan', # 激光消息名字,一般为scan
                          'local_ros':True,
                          'ROS_HOSTNAME':"192.168.43.17",# if local_ros is False
                          'ROS_MASTER_UR':"http://192.168.43.56:11311/",# if local_ros is False
                          }

camera_config = {
        'camera_model': 'pinhole',# fisheye, pinhole or omnidir(Mei) model
        'tag_type': 'chess',  # 'chess' or 'aruco'
        'aruco_id': 0,  # 如果使用单个'aruco',需要id(使用aruco.DICT_4X4_250)
        'tag_size':  0.0104,#,  #单位米 0.012
        'checkerboard': (9,6),  # if choice chess ,need
        'inter_params_path': os.path.join(data_root,'inter_param.pkl'),# 内参路径
        'exter_params_path': os.path.join(data_root,'exter_param.pkl'),# 外参路径


        'inter_image_path': os.path.join(data_root,'inter_img'),# 内参图像路径
        'exter_image_path':os.path.join(data_root,'exter_param.jpg'),# 外参图像路径
        'K':None,'D':None,'Xi':None#或者手动输入内参,List
}


calibrate_config={
        'select_laser':'auto'
                       '',# auto or manual
        'one_by_one_show':True,# one by one frame to show laser data
        'optimize_method': 'svd',# 目前只有svd
        }
