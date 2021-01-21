nav_msgs::Odometry 

pose.pose.position                              utm坐标
pose.pose.orientation                           gps姿态，ENU(东北天)坐标系
twist.twist.linear                              gps速度


四元数为gps姿态，航向为gps的x轴与大地东向夹角，或者说gps的y轴与大地北向的夹角，逆时针为正
pose.covariance[0] = deg2rad(inspvax.azimuth);  gps航向角(rad)(n系) 北东地坐标系
pose.covariance[1] = inspvax.longitude;         经度(deg)
pose.covariance[2] = inspvax.latitude;          纬度(deg)
pose.covariance[3] = yaw;                       航向角(rad) 转换后姿态航向
pose.covariance[4] = roll;                      横滚角
pose.covariance[5] = pitch;                     俯仰角
pose.covariance[6] = inspvax.north_velocity;    北向速度
pose.covariance[7] = inspvax.east_velocity;     东向速度
