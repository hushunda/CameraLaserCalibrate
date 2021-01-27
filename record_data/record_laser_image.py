# coding:utf-8
'''
记录用于标定激光和图像
注意：保证激光能打在标定板，标定板在激光前方120度，2米内。需要多个角度多个位置的数据

使用：
1.终端 ls /dev/video 按下tab键查看，摄像头的索引
2.修改参数cam_id
3.修改保存的图像的路径，修改bag路径
4.远程ROS,需要配置ROS_HOSTNAME:本机ip和ROS_MASTER_URI:主机的ip。在本机的/etc/hosts文件，添加主机ip和机器名
5.使用中按下 c保存一次图像.。。按下k结束。
'''

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(os.path.dirname(__file__))
import os
import rospy
from sensor_msgs.msg import LaserScan
import pickle
import cv2
import time
import threading
import queue
from Configs import data_collection_config as config
from Configs import data_root
import ctypes



## 配置
img_path = os.path.join(data_root,'images')
os.makedirs(img_path,exist_ok=True)
save_pkl = os.path.join(data_root,'laser_image.pkl')

if not config['local_ros']:
    os.environ["ROS_HOSTNAME"]=config['ROS_HOSTNAME']
    os.environ["ROS_MASTER_URI"]=config['ROS_MASTER_URI']

if not os.path.exists(data_root):
    os.makedirs(data_root)

class RecordImageLaser():
    def __init__(self):
        self.config = config

        self.cap = cv2.VideoCapture(config['cam_id'])
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(3, config['img_wight'])
        self.cap.set(4, config['img_hight'])
        print('img height :', self.cap.get(3))
        print('img width:', self.cap.get(4))
        print('img fps:', self.cap.get(5))


        # loop until press 'esc' or 'q'
        print(' start to record laser data and image data')
        print('        press "C" to record a frame data        ')
        print('        press "K" to finish record data        ')

        self.laser_queue = queue.Queue(maxsize=4)
        self.laser_thread = threading.Thread(target=self.laser_listener, args=(self.laser_queue,))
        self.laser_thread.setDaemon(True)
        self.laser_thread.start()

    def run(self):
        all_data = []
        count = 0
        cv2.namedWindow('show', 0)
        while 1:
            ret, frame = self.cap.read()
            if ret:
                cv2.imshow('show', frame)
                k = cv2.waitKey(1)
                if k == ord('c'):
                    if self.laser_queue.qsize() > 0:
                        laser_data = self.laser_queue.get()
                        all_data.append([laser_data, frame])
                        cv2.imwrite(os.path.join(img_path, '%0.4d.jpg' % count), frame)
                        count += 1
                        print('had record %d data' % count)
                    else:
                        print('no laser data')

                elif k == ord('k'):
                    break
        with open(save_pkl, 'wb') as f:
            pickle.dump(all_data, f)

        print('+++++++++++++ finish record data +++++++++++++')
        # 结束激光callback线程

        ctypes.pythonapi.PyThreadState_SetAsyncExc(self.laser_thread.ident, ctypes.py_object(SystemExit))
        ctypes.pythonapi.PyThreadState_SetAsyncExc(self.laser_thread.ident, None)

    def laser_callback(self,data,q):
        q.put(data)
        q.get() if q.qsize() > 1 else time.sleep(0.01)

    def laser_listener(self,q):
        rospy.init_node('laser_listen', anonymous=True)
        rospy.Subscriber(self.config['scan_topic_name'], LaserScan, self.laser_callback, callback_args=q)
        rospy.spin()


if __name__ == '__main__':
    app = RecordImageLaser()
    app.run()

