## 相机和二维激光的标定

目前支持相机pinhole,fisheye,omndir

相关参考资料，标定相关的理论推导以及拓展知识请阅读知乎专栏文章：https://zhuanlan.zhihu.com/p/137501892

这是该算法的python执行版本

## 安装
* rospy   
* opencv  
* opencv-contrib-python  

## 使用
说明： 最影响标定结果的，就是激光点的选择。

1. 配置参数
	所有参数都在config.py中  
	data_collection_config: 相机数据采集的参数,图像的大小和ROS的配置
	camera_config: 相机的模型,如果已经标定了参数,可以直接将内参写入config.用List.也可以用```python record_data/calibrate_inter_params.py```进行数据的采集和标定.然后保存内参在```inter_params_path```路径下.
	calibrate_config: 相机和激光标定的参数
2. 数据采集
    ```shell script
    python record_laser_image.py   
   ```
   按下<kbd>C</kbd>采集一帧数据  
   按下<kbd>K</kbd>结束采集数据
   输出laser_image.pkl保存所有数据,images下显示采集的图像数据


3. 标定相机内参(可选)  
   ```python record_data/calibrate_inter_params.py```进行数据的采集和标定.然后保存内参在```inter_params_path```路径下.
   [在线打印aruco](https://chev.me/arucogen/)
4. 标定激光到相机的变换
	``` shell script
	python LaserCameraCalibration.py 
	```
   标定相机和激光.
5. 显示标定结果
	```shell script
	python src/verify.py
	```
   验证激光和相机的标定:将激光显示在图像上.

## TODO
1. SGD的优化方法
2. 其他求解方法
3. 精度优化
4. 支持aruco