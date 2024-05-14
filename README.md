# ros_car 

基于ROS的麦克纳姆轮小车上位机源码，其功能包的介绍如下所示：

1. car_base功能包中包含底盘节点和键盘控制节点。

2. robot_pose_ekf功能包用于将imu和odom数据融合，可以避免小车打滑时导致里程计不准。

3. robot_voice功能包中包含语音识别和语音合成节点，使用的科大讯飞的在线语音，可以用于对小车的语音控制。

4. car_description功能包是小车模型功能包，使用solidworks的urdf插件对小车三维模型导出得到。
