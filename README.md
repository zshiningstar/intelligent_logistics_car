## 0 新电脑

### dependencies

```
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```

* 安装雷达相关驱动
```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
sudo apt install libapr1-dev
cd build && cmake ..
make
sudo make install
```
* 编译
cd logistics_ws/
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install libpcap-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
catkin_make
```
## 1、记录一段路经
* 启动记录路径节点:记录一段路经，保存为txt文件
```
roslaunch driver path_recorder.launch
```
## 2、启动循迹节点

* 按照刚才记录的路径进行循迹
```
roslaunch driver path_track.launch
```

## 3、启动RTK差分定位

```
./main /dev/rtk  需要具体看是哪个端口号
```



### 注意事项:
* 每次GPS上电之后,需要在上位机标定软件里面进行cool reset,保证其有正确的航向角度
* GPS主机固定后,其坐标系和车辆载体坐标系并不重合,通过启动以下节点进行标定,并将终端上打印的数值大户入GPS标定上位机软件之中(Head ..参数)
*  **标定过程中,除非将要发生安全问题,否则不要打方向,让其沿着直线往前行走**
```
roslaunch driver calibrate_gps.launch 
```







