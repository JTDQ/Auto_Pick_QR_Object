

# 京天例程：自动跟随二维码并抓取

# 1、配置安装准备

### A、配置Ubuntu，ros kinetic，turtlebot3

### B、配置openmanipulator 相关程序包

# 2、配置二维码移动抓取包 auto_pick_sc

### A、下载，编译

```shell
git clone https://github.com/JTDQ/Auto_Pick_QR_Object.git
cd catkin_ws/src
catkin_make
```

### B、配置相机

> 这个文件仅作为试用，你应该根据你的相机，进行标定后的文件做替换。

复制 文件夹内相机标定文件`head_camera.yaml`文件到`/home/sc/.ros/camera_info/head_camera.yaml`.

### C、运行

#### 方案一，如果你用的是树莓派相机，

先在树莓派上运行：

```
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch 

ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=om_with_tb3 set_lidar_frame_id:=om_with_tb3/base_scan 
```

然后，在PC电脑上运行：

```shell
# turtlebot3，机械臂moveit，ar_tracker
roslaunch auto_pick_sc raspicamera_ar_tracker.launch
# 任务控制节点
roslaunch auto_pick_sc sc_task_controller.launch
```

#### 方案二，如果你是用usb相机

然后在远程PC上运行

```shell
# turtlebot3，机械臂moveit，ar_tracker
roslaunch auto_pick_sc follow_ar.launch
# 任务控制节点
roslaunch auto_pick_sc sc_task_controller.launch
```



# 3、移动抓取包内文件解释

### 1.follow_ar.launch

包括`usb_cam_ar_tracker.launch`，`turtlebot3_robot.launch`: TB3启动文件，`manipulation.launch`：机械臂Moveit启动。

#### a.usb_cam_ar_tracker.launch

这个里面配置了USB摄像头节点`usb_cam`，二维码的节点`ar_track_alvar`

### 2.sc_task_controller.launch

主要是包括一个Python脚本`pick_and_place_state_machine_0310.py`，

> python 脚本加入到工作空间中，一定要给可执行权限。

### 3.pick_and_place_state_machine_0310.py

这个代码是`smach`任务节点管理的代码。

# 4、遥控器OpenCR sc_open_manipulator_chain

1. 修改了按键6为切换角度空间和操作空间。



# 5、tb3遥控操作机械臂

7多点巡航 <https://blog.csdn.net/baimei4833953/article/details/82928549>

# 6、相机没标定的按照下面流程标定

**准备标定板**

下载标定板：<http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=get&target=check-108.pdf>

操作步骤

```
# 地图构建
# 发布机器人bring，navigation， 
# 发布多点巡航  
# 修改launch文件中的三个点的坐标系。
$ roslaunch auto_pick_sc n_points.launch 
```



贴到一个

#### **标定相机**

首先插上USB相机，编辑相机发布配置文件：

$ roscd usb_cam/launch/

$ gedit calib.launch 

输入以下内容

```
<launch>

  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >

    <param name="video_device" value="/dev/video0" />

    <param name="image_width" value="640" />

    <param name="image_height" value="480" />

    <param name="pixel_format" value="yuyv" />

    <param name="io_method" value="mmap"/>

  </node>

</launch>
```

然后运行标定程序

$ roslaunch usb_cam calib.launch             

$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/usb_cam/image_raw camera:=/usb_cam

其中“--size 8x6”表示标定板每行8个内点，每列6个内点。“--square 0.03”表示标定板每个方格子边长为0.03米。

用标定板：

![img](file:////tmp/wps-sc/ksohtml/wpsSqYe7i.jpg) 

等待右边的 x y size skaw 都变绿，就可以点击 Calibration 运算，运算过程会非常的缓慢，电脑会卡住不动，稍安勿躁。

然后运算完成后，点“SAVE”，点“COMMIT”

然后我们在“~/.ros/camera_info/”得到了这个文件，它对我们后面的定位过程非常重要。

~/.ros/camera_info/head_camera.yaml

我们下一次运行 usb_cam 就会自动调用文件，并有如下的信息：

[ INFO] [1555832606.313276288]: camera calibration URL: [file:///home/sc/.ros/camera_info/head_camera.yaml
](file://home\sc\.ros\camera_info\head_camera.yaml)