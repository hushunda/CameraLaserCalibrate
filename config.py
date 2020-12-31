data_root = './data'# 绝对路径
data_collection_config = {'cam_id':0,
                          'img_wight':1920,'img_hight':1080,
                          'scan_topic_name': 'scan', # 激光消息名字,一般为scan
                          'local_ros':True,
                          'ROS_HOSTNAME':"192.168.43.17",# if local_ros is False
                          'ROS_MASTER_UR':"http://192.168.43.56:11311/",# if local_ros is False
                          }

camera_config = {
        'camera_model': 'omnidir',# fisheye, pinhole or omnidir(Mei) model
        'tag_type': 'chess',  # 'chess' # 暂时不支持'aruco'
        'tag_size': 0.045,  #单位米
        'checkerboard': (11,8),  # if choice chess ,need
        'inter_params_path': '/home/zjrobot/robot_perception/calibration/human-perception/data/robot_with_shell/fisheye_head_front/inter_param.pkl',# 内参路径
        'K':None,'D':None,'Xi':None#或者手动输入内参,List
}


calibrate_config={
        'select_laser':'auto',# auto or manunal
        'one_by_one_show':False,# one by one frame to show laser data
        'data_root': data_root,
        'optimize_method': 'svd',# 目前只有svd
        }