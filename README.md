## 相机和二维激光的标定


相关参考资料，标定相关的理论推导以及拓展知识请阅读知乎专栏文章：https://zhuanlan.zhihu.com/p/137501892

这是该算法的python执行版本

## 安装
rospy 
opencv
opencv-contrib-python

## 使用

1. 配置参数
2. 数据采集
    ```shell script
    python record_laser_image.py   #按下C记录一帧数据,按下K结束.
   ```
   使用 <kbd>Ctrl</kbd>+<kbd>Alt</kbd>+<kbd>Del</kbd> 重启电脑
    输出laser_image.pkl保存所有数据,images下显示采集的图像数据
3. 标定相机参数(可选)  
   显示畸变矫正结果  
   [在线打印aruco](https://chev.me/arucogen/)
4. 标定激光到相机的变换
5. 显示标定结果
