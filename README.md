## 相机和二维激光的标定


相关参考资料，标定相关的理论推导以及拓展知识请阅读知乎专栏文章：https://zhuanlan.zhihu.com/p/137501892

这是该算法的python执行版本

## 安装
* rospy   
* opencv  
* opencv-contrib-python  

## 使用
说明： 最影响标定结果的，就是激光点的选择。

1. 配置参数
2. 数据采集
    ```shell script
    python record_laser_image.py   
   ```
   按下<kbd>C</kbd>采集一帧数据
   按下<kbd>K</kbd>结束采集数据
   输出laser_image.pkl保存所有数据,images下显示采集的图像数据
3. 标定相机参数(可选)  
   显示畸变矫正结果  
   [在线打印aruco](https://chev.me/arucogen/)
4. 标定激光到相机的变换
5. 显示标定结果
   ```shell script
    python verify.py
   ```
##　改进
1. 求解H矩阵时候，加入正则项
2. 取激光点，取最近的一线的点

## TODO
1. SGD的优化方法
2. 其他求解方法
3. 精度优化