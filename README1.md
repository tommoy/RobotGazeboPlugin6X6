# RobotGazeboPlugin6X6
The Gazebo control Plugin

三个文件夹Plugin_6X6, Scene, lisenter分别包含机器人的控制插件程序，仿真场景，传感器数据读取。

Plugin_6X6编译：
$ mkdir  /Plugin_6X6/build
$ cd  /Plugin_6X6/build
$ cmake ..
$ make
在bulid下可查看到libPlugin_6X6.so动态连接

回到～下，打开.bashrc文件，写入
 export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Your dictionary/build
打开终端，souce ~/.bashrc
