
## 1、记录一段路经

* 启动记录路径节点:记录一段路经，保存为txt文件

```
roslaunch driver path_recorder.launch
```

## 2、启动上位机监听下位机传输数据节点

* 启动接收下位机传发来的信息节点
```
roslaunch driver talk_pc.launch
```

## 3、启动上位机向下位机发送数据节点
```
roslaunch driver talk_stm.launch
```
## 4、启动循迹节点

* 按照刚才记录的路径进行循迹
```
roslaunch driver path_track.launch
```

## 5、启动livox激光雷达雷达避障

**启动雷达**
```
roslaunch livox_ros_driver livox_lidar_rviz.launch 
```
**检测5米内障碍物**
* 具体数字可以在launch文件里面修改
```
roslaunch euclidean_cluster euclidean_cluster.launch
```


