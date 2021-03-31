#### 命令清单
* 1.记录路径
```
roslaunch driver path_recorder.launch
```

* 2.正向循迹(包含1和5)
```
roslaunch driver path_track.launch
```

* 3.倒车循迹(包含1和5)
```
roslaunch driver path_track_back.launch
```

* 4.监听下位机反馈信息,并发布车辆底盘里程计信息以及车辆速度转角等信息
```
roslaunch driver pub_car_state.launch
```

* 5.向下位机发布命令
```
roslaunch driver talk_stm.launch
```
