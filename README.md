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
    <img src="/4.png" alt="4" width="800">
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

������ ���� ������ learn_ros2_ws ��� src 

���� ������ ��� src ��� �����

******������������ 

��� ��� ����� ��� ����� ��������������� ��� ���� type="continuous" 

��� ��� ���� ��� ��� ������� �� ����� ��������� ��� ����� �� ������������� ������� ��� ���� type="fixed"

 <div style="text-align:center;">
    <img src="/3.png" alt="3" width="800">
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
    <img src="/2.png" alt="2" width="800">
</div>

�  ��� �� ����� ��� gazebo ����� :

```shell
ros2 launch my_first_pkg gazebo.launch.py
```
 <div style="text-align:center;">
    <img src="/1.png" alt="1" width="800">
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

