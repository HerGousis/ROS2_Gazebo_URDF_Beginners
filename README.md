# ROS2 ���  ��������� 2
������ �� ����� ��� ������������ ����� ��� Gazebo , �� ��������� ��� ���� ��� robot car ��� �� �� ����� ��� RVIZ ��� ��� Gazebo

## Gazebo Create World
### Install Gazebo 
1. ��������� �� ��������� ��� �������� 
     ```shell
     sudo apt install ros-humble-gazebo-*
     ```
  ������ ��� ���� ������ �� ��������� 

### Gazibo world

2. ��� �� �� ������ ����� gazebo
��� Insert 
���� Add Path
��� http://models.gazebosim.org/
���� ����������� ������ ��� �� gazebo

3. ��� �� ����������� �������
=> edit => Building Editor 
����������� ���� ������� ��� ��� �� �� ������� save 
=>file =>Save As(Ctrl+Shift+S) => ������� �� ����� ��� ��� ������� �� �� ������������� 
��� Save =>Exit (�� ������� �������� )

4. ��� ��������� World 
File => Save World As => ���� �� �����(name.world) ��� ��� ���������  ��� => Save


��� ������ �� ����� ��� ����������� ������ �� ����� ���� ������ ��� ��� ���������� 
(hercules@hercules:~/gazebo_examples$)

��� �� ����� ��� terminal 
 ```shell
   gazebo (name.world)
```
 <div style="text-align:center;">
    <img src="image/4.png" alt="4" width="800">
</div>

## Create My Robot URDF
### Setup the URDF File
1. ��������� ���� ������ �� ����� my_robot

���� ���� ������ ����� :

 ```shell
   touch my_robot.urdf
   code my_robot.urdf
```

2. ������� ��� visual code  ��� ����� ��� ������ (������� ��� ������ my_robot.urdf ���� ������ my_robot)

��� ��������� ����

```shell
 ros2 launch urdf_tutorial display.launch.py model:=/home/hercules/my_robot/my_robot.urdf 
```


******������������ 

��� ��� ����� ��� ����� ��������������� ��� ���� type="continuous" 

��� ��� ���� ��� ��� ������� �� ����� ��������� ��� ����� �� ������������� ������� ��� ���� type="fixed"

 <div style="text-align:center;">
    <img src="image/3.png" alt="3" width="800">
</div>

##  URDF+Xacro creation ,Visualize in RVIZ2 and Gazebo +Teleoperation

1. ������ ���� ������ learn_ros2_ws ��� src  ���� ������ ��� src ��� �����
```shell
  ros2 pkg create --build-type ament_cmake --node-name my_node my_first_pkg (����� ������� �� my_first_pkg)
```

****�� ��� ERRORS****
```shell
  nano /home/hercules/learn_ros2_ws/src/my_first_pkg/package.xml
```

��� ���������� 
������ ��� �������:
```shell
<license>TODO: License declaration</license>
```
�������������� �� �� ��� ��� ��� ������������� ������ ������ ��� �������������� ��� �� ament:

��� ����������, �� �������������� Apache 2.0:
```shell
<license>Apache-2.0</license>
cd ..
```

��� ���������, ������������ ��� ������ LICENSE ��� ������ ��� ������� ���:

```shell
touch /home/hercules/learn_ros2_ws/src/my_first_pkg/LICENSE
```

���� 4: ������������ ��� ����������

����������� ��� ������� ��� ����������� ��� ���������:

```shell
colcon build 
```

��� ��� �� ����� �� ������  urdf ������ �� ����� ��� ��� ��� ������ ��� ����� ��� ������ :
```shell
ros2 launch urdf_tutorial display.launch.py model:=/home/hercules/learn_ros2_ws/src/my_first_pkg/urdf/my_robot1.urdf 
```
���� ������� ����� �������� ```include```  , ```launch``` , ```rviz```  ���� ��� ```my_first_pkg```
���� ���� ����� �� ������ ```.urdf``` ��� ���� save ��� ������ ```rviz```
���� ��� launch ��������� ��� ������ .py  (```display.launch.py``` && ```gazebo.launch.py``` )
��� ���� �� ������ �� ����� ��� �� ��� ��� ���� 

��� ������ ��� ��� ������ CMakeList.txt  ��� ���������� ���� 
```shell
#install launch files
install(DIRECTORY
 launch
 urdf
 rviz
 DESTINATION share/${PROJECT_NAME}/
)
```

���� �������� ��� ��� ���������  ���� ```cd``` 
����� �� ����� ����  ������� ```learn_ros2_ws``` ���� ```ls``` 
��� ���� ����  : 
```shell
 colcon build
 ```
���� 
```shell 
colcon build --symlink-install
source install/local_setup.bash
```

��� ���� �������� ���� �� ���� :
```shell
gedit ~/.bashrc  (������� �� ������ bashrc)
```

��� ���������� ���� : ```source ~/learn_ros2_ws/install/setup.bash```
��� ����� ��� terminal :
```shell
source .bashrc
```
��� ������ �� ������� ���� :
```shell
hercules@hercules:~$ source .bashrc
hercules@hercules:~$ 
```
����� ��� �� �� �����  

```shell
ros2 launch my_first_pkg display.launch.py
```
 <div style="text-align:center;">
    <img src="image/2.png" alt="2" width="800">
</div>

�  ��� �� ����� ��� gazebo ����� :

```shell
ros2 launch my_first_pkg gazebo.launch.py
```
 <div style="text-align:center;">
    <img src="image/1.png" alt="1" width="800">
</div>


## ��������� �� �� ������������ ��� PC
�� ���� ���������
```shell
 ros2 topic list 
 ```
 ��� �� ������ �� ��������� �� ���� :
```
/clock
/cmd_vel
/joint_states
/odom
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static
```
��� ���� 
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

��� �� ������ �� ��������� �� ���� :
```
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
```

### �� ������� ���� �������������

��� ��������� �����
```shell
ps aux | grep gazebo
```
����� ��� ���������� 
��� ������� ��� �����  �� ��� ������ ```kill -9 <pid>```
����  ��� ��������� �����
```shell
ps aux | grep gazebo
```
��� ����� �� ���� ��� ��� ��� ����� 
��� ���� ����� ��� ������ 

### ��� ������� ��� robot
http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

### ��� ��������� ��� robot
http://wiki.ros.org/urdf/XML/link

### ��� ����� ��������� ��� robot 
http://wiki.ros.org/urdf/XML/joint

## Create Robot Arm in RVIZ (file .Xacro)

 <div style="text-align:center;">
    <img src="image/5.png" alt="5" width="800">
</div>

1. ��������� ���� ������ braxionas ��� ���� �� ����� ���� ������ src
```shell
colcon build
```
1.  ���� ��� src ������ ��� ������
 ```shell
ros2 pkg create braxionas
```
�������� ���� �������� include , src ��� ������� launch , urdf ,meshes , rviz  
���������� �� ������ ```CMakeLists.txt``` ��������� ��� ���� 

1. ������ �� ������ bashrc
```shell
gedit ~/.bashrc
```
```shell
source ~/braxionas/install/setup.bash
```
��� ���� ���� ������ braxionas (log install src build)

```shell
colcon build
colcon build --symlink-install
```

����   ```source install/setup.bash```

```shell
ros2 launch braxionas display.launch.xml
```
�
```shell
ros2 launch braxionas display.launch.py
```

��� �� ���������� ����� :
 <div style="text-align:center;">
    <img src="image/6.png" alt="6" width="800">
</div>

### NOTES
�� ������ .stl �������� ��� ��� [Repositories](https://github.com/HerGousis/Robotic_ARM)

## Robot in the  my world (Gazebo) 

1. ��������� ���� ������ my_robot_description  ��� ���� �� ����� ���� ������ src
```shell
colcon build
```
  ���� ���������������� �� ������ ��� ����� ���������� ��� ������ ```my_robot_description/src ```
������ ����� ���� ������  my_robot_description (log install src build)

```shell
colcon build
colcon build --symlink-install
```

����   
```shell 
source install/setup.bash
```
��� ����� 
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

��� �� ���� ���������  
```shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5},angular:{z: 0}}" 
```
��� �� ����� �� �������� ���� ��� ����� � ���� ��� ����� ��� ������ ������������

� 
```shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5},angular:{z: 0.5}}" 
```
�� ����� ��� �������� ������� 

 <div style="text-align:center;">
    <img src="image/7.png" alt="7" width="800">
</div>

## Add Camera in Gazebo 

1. ���� ��� ������ urdf ```my_robot_description/src/my_robot_description/urdf```
 ����������� ��� ����� ������ ``` camera.xacro ``` ��� �� ������������� ��� ������ ``` my_robot.urdf.xacro```
������ ����� ���� ������  my_robot_description (log install src build)

```shell
colcon build
colcon build --symlink-install
```

����   
```shell 
source install/setup.bash
```
��� ����� 
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

 <div style="text-align:center;">
    <img src="image/8.png" alt="8" width="800">
</div>

* ��� rviz ��� �� ����������� ��� ������ ��������� ```Add``` ��� ������� �� ```Image``` ��� ���� �������� ��� ```Topic``` ������� ��� ������ ��� .

## Add lidar

������������ ��� ����������� ������ liadr.xacro ��� �� ����������� ��� ```my_robot.urdf.xacro```

```shell
colcon build
colcon build --symlink-install
```

����   
```shell 
source install/setup.bash
```
��� ����� 
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

 <div style="text-align:center;">
    <img src="image/9.png" alt="9" width="800">
</div>

* ��� �� ROS2 ��� lidar.xacro             
 ```shell
 <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
     <topic_name>/scan</topic_name>
     <frame_name>lidar_link</frame_name>
     <output_type>sensor_msgs/LaserScan</output_type>
 </plugin> 
```

 <div style="text-align:center;">
    <img src="image/10.png" alt="10" width="800">
</div>

 <div style="text-align:center;">
    <img src="image/11.png" alt="11" width="800">
</div>

* ��� rviz ��� �� ����������� ��� ������ ��������� ```Add``` ��� ������� �� ```By topic``` ��� ���� �������� ��� ```LaserScan``` ������� ��� ������ ��� .



## Install OpenCV

��� ��������� ����� :
```shell
pip install opencv-python
```
��� ���� 
```shell
sudo apt install ros-humble-cv-bridge
```
## Create Python Script 

 ����� ���� ������ ``` my_robot_description/src ```

 ```shell
 ros2 pkg create robot_controller --build-type ament_python --dependencies rclpy
```

��� ������������ � ������� ```robot_controller``` 

* ��� ������ ```package.xml```

���������� :

```shell
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>robot_state_publisher</depend>
  <depend>my_robot_description</depend>
```

* ��� ������ ```setup.py```
���������� :

```shell
     entry_points={
        'console_scripts': [
            "go=robot_controller.go:main",
            "go_with_laser=robot_controller.go_with_laser:main"
        ],
    },
```
* ��� ����� ��� ������ �� ������ .py ���� ��� ����� �� ```__init__.py``` 


����� ������ ���� ������ ``` my_robot_description/```
```shell
colcon build
colcon build --symlink-install
```
��� ������ ��� ��������� ���� ��� ������ ```my_robot_description``` 

* ��� ��� ����� 
```shell 
source install/setup.bash
```
��� ����  
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

* ��� ���� 

```shell 
source install/setup.bash
```
��� ����  
```shell
ros2 run robot_controller go_with_laser 
```
� 

```shell
ros2 run robot_controller go
```
� 

```shell
ros2 run robot_controller go_with_lidar 
```

�� ������ `go_with_lidar.py` ������ ���� ������ `.py` ��� ��� ������ `utils` ��� ���������� ����� �� ���� Python Script ��� ` go_with_laser`     

### NOTES
�� ������ ```go_with_laser.py``` � ```go_with_lidar.py```����� �� ������ �� ������� ���� ��� ��� ����������� ��������� ������� �������� 1.0 
������ ���������� ��� ������ ```laser_data``` ��� ����� ��� ���������� � lidar ���� ��� ����������� 
����� ���� ��� ```OpenCv``` ���������� ���� 10 ������������ ��� ���������� ���� ��� ������ ```image_data``` 
��� ���� ���� ������ �� ```go_with_laser.py``` � ```go_with_lidar.py``` ������ ��� ������ ����������� ��� ��������� ����������� ��� �� ������ ```laser_data.txt``` ������������

https://github.com/user-attachments/assets/7c2a3cc9-6bfc-4115-971e-3b9456bf6529

## SLAM spatial mapping

��������� ���� ���������
��� ����� ```hercules@hercules:~/my_robot_description$```
  ```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

��� ������� ```hercules@hercules:~/my_robot_description$```
  ```shell
ros2 launch slam_toolbox online_async_launch.py
```
��� ����� ```hercules@hercules:~/my_robot_description$```
  ```shell
ros2 run robot_controller go_with_lidar 
```

![Image](https://github.com/user-attachments/assets/e8c8144e-08a0-4d4b-b57c-d2e08d5c6eba)

## Add Qr and Object ��� Gazebo
### Qr
��� �� Github `` https://github.com/mikaelarguedas/gazebo_models``

��� ��������� ����
 ```shell
git clone https://github.com/mikaelarguedas/gazebo_models.git
cd gazebo_models/ar_tags/scripts/
python3 generate_markers_model.py -i /home/hercules/gazebo_models/ar_tags -s 1000 -w 500 -v
```

��� ������ �� Gazebo ��� ������� �� ����������� 

### Object 

�������� �� ����������� ��� Gazebo 
������ �� ������ ������ ``.obj`` ``.jpg`` ``.mtl``

���� ���������� �� ����� ��� ������ ���� ��������� ���� ������ 

 <div style="text-align:center;">
    <img src="image/14.png" alt="14" width="800">
</div>

## Add UR5 and robot in RVIZ and Gazebo (file .Xacro)
<div style="text-align:center;">
    <img src="image/15.png" alt="15" width="800">
</div>

���������� ���� ����������� ������� ``ur5.xacro ``
��� �������� ��� ������ `` my_robot.urdf.xacro``
��������� 
 ```shell
 <xacro:include filename="ur5.xacro" />
```
���� 
```shell
colcon build
colcon build --symlink-install
```

����   
```shell 
source install/setup.bash
```
��� ����� 
```shell
ros2 launch my_robot_description display.launch.xml 
```
�
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

 
 <div style="text-align:center;">
    <img src="image/21.png" alt="21" width="800">
</div>

## COLMAP

��� ��� ������� ��� ������������� ��� ������ ``image_data``
��� �� ����� ��� COLMAP
 <div style="text-align:center;">
    <img src="image/16.png" alt="16" width="800">
</div>
 <div style="text-align:center;">
    <img src="image/17.png" alt="17" width="800">
</div>

## RealityCapture
��� ��� ������� ��� ������������� ��� �� ����� ��� ```RealityCapture```
�������� �� ������� ������� ��� 3D ������������ 
 <div style="text-align:center;">
    <img src="image/22.jpg" alt="22" width="800">
</div>

��� ���� ��� ```MeshLab``` �������� �� ����� �� 3D ����������� ��� �������������� 

 <div style="text-align:center;">
    <img src="image/22.png" alt="22" width="800">
</div>

## Marker Detection ROS2

���������� ���� ����������� ������� ``qr.py ``
���� 
```shell
colcon build
colcon build --symlink-install
```

����   
```shell 
source install/setup.bash
```
��� ����� 
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
��� �� ���� ��������� 

```shell
ros2 run robot_controller qr
```

</div>
 <div style="text-align:center;">
    <img src="image/18.png" alt="18" width="800">
</div>

## Locating the point of the object with marker in rviz ROS2

</div>
 <div style="text-align:center;">
    <img src="image/20.png" alt="20" width="800">
</div>

���������� ���� ����������� ������� ``marker_kalo.py ``
���� 
```shell
colcon build
colcon build --symlink-install
```

����   
```shell 
source install/setup.bash
```
��� ����� 
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
��� �� ���� ��������� 

```shell
ros2 run robot_controller marker_kalo
```

</div>
 <div style="text-align:center;">
    <img src="image/19.png" alt="19" width="800">
</div>

