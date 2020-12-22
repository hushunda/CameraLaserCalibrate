# coding:utf-8
'''
记录用于标定激光和图像
注意：保证激光能打在标定板，标定板在激光前方120度，2米内。需要多个角度多个位置的数据

使用：
1.终端 ls /dev/video 按下tab键查看，摄像头的索引
2.修改参数cam_id
3.修改保存的图像的路径，修改bag路径
4.wifi链接底座的ros，配置ROS_HOSTNAME:本机ip和ROS_MASTER_URI:主机的ip。在本机的/etc/hosts文件，添加主机ip和机器名
5.使用中按下 c保存一次图像.。。按下k结束。
'''

import os
import rospy
from sensor_msgs.msg import LaserScan
import cv2
import rosbag
import time
import numpy as np
import multiprocessing as mp

## 配置
data_root = '../data/camera_front_fisheye/laser_img/'
cam_id = 0
img_path = data_root
time_img_name_txt = data_root+'time_img_name.txt'
bag_filename = data_root+'laser.bag'
os.environ["ROS_HOSTNAME"]="192.168.43.17"
os.environ["ROS_MASTER_URI"]="http://192.168.43.56:11311/"

if not os.path.exists(data_root):
    os.makedirs(data_root)

def callback(data,q):
    q.put(data)
    q.get() if q.qsize() > 1 else time.sleep(0.01)
def listenor(q):
    rospy.init_node('lasern_puber', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback, callback_args=q)
    rospy.spin()

def webcamImagePub(q):
    # make a video_object and init the video object
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30.0)
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))
    cv2.namedWindow('show',0)
    count = 0
    # loop until press 'esc' or 'q'
    f_txt = open(time_img_name_txt,'w')
    with rosbag.Bag(bag_filename, 'w') as bag:
        while 1:
            # header = Header(stamp=rospy.Time.now())
            ret, frame = cap.read()
            if ret:

                cv2.imshow('show',frame)
                k = cv2.waitKey(1)
                if k==ord('c'):
                    if q.qsize() >0:
                        laser_data = q.get()
                        bag.write('scan',laser_data,laser_data.header.stamp)
                        img_time = laser_data.header.stamp.to_sec()
                        cv2.imwrite(img_path+str(img_time)+'.jpg',frame)
                        write_info = [str(img_time),str(img_time)+'.jpg']
                        f_txt.write(' '.join(write_info)+'\n')
                        count+=1
                        print('have record %d data' % count)
                    else:
                        print('no laser data')

                elif k==ord('k'):
                    f_txt.close()
                    break

    print('ok')





if __name__ == '__main__':
    # mp.set_start_method(method='spawn')  # init
    queues = mp.Queue(maxsize=4)
    processes = [mp.Process(target=webcamImagePub, args=(queues,))]
    processes.append(mp.Process(target=listenor, args=(queues,)))

    for process in processes:
        process.daemon = True  # setattr(process, 'deamon', True)
        process.start()
    for process in processes:
        process.join()
