# iotbot
This package comprises a ROS interface for EduArt's Eduard-Plattform, aka IOTbot. It includes two kinematic concepts: Mecanum steering and skid steering.
Both can be used in dependency of the mounted wheels.

![IOTbot](/images/03_YellowSpotlight.jpg)

## Installation on IOT2050 Devices with Debian Buster from Scratch
```console
apt update
apt upgrade
reboot
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update
apt install ros-noetic-ros-base ros-noetic-joy git
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
source /opt/ros/noetic/setup.bash
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace 
git clone https://github.com/eduart-robotik/iotbot.git
cd ..
catkin_make
```
Additionally, the UART interface needs to be configured with
```console
iot2050setup
```
Configure I/O to use UART RX & TX pins. Reboot the device:
```console
reboot
```

## Launching the Robot
In order to run the robot, you need to launch the appropriate launch file. In the launch folder, there are four prepared templates.
Choose the right kinematic concept and the right motor variant (basic or performance).
```console
roslaunch iotbot skid_performance.launch
```
When everthing is initialized well, one should see the following output:
```console
[ WARN] [1628772862.880453339]: Lag detected ... deactivate motor control
```
This is not a bug, but a feature, since the robot will stop immediately, if no frequent communication with a controlling node is established.

Please notice also, that the ROS variables ROS_MASTER_URI, ROS_HOSTNAME and ROS_IP should be set properly. If you have a changing IP-address of the robot, you might consider to use the following bash code in your ~/.bashrc:

```console
MYIP=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://${MYIP}:11311
export ROS_IP=${MYIP}
export ROS_HOSTNAME=${MYIP}
``` 

### Steering with Joystick Extreme 3D Pro
Install the ROS joy interface on your host machine, if you have an Extreme 3D Pro.
```console
sudo apt install ros-noetic-joy
```
After installation, ensure to have configured the ROS communication well:
```console
export ROS_MASTER_URI=http://<IP_OF_IOTBOT>:11311
export ROS_HOSTNAME=<HOST_IP>
export ROS_IP=<HOST_IP>
```
Finally, launch the joy node as follows:
```console
rosrun joy joy_node _autorepeat_rate:=10
```

The output of warnings on the IOT2050 device should stop and the IOTbot can be steered. Please find the button and axis mapping of the joystick below:

| Button | Function         |
| ------ |:----------------:|
| F1     | Beam light       |
| F2     | Warning light    |
| F3     | Flash left       |
| F4     | Flash right      |
| F5     | Rotational light |
| F6     | Running light    |
| F11    | Enable robot     |

**Important:** The robot must be enabled, before it will move!

| Axis   | Function           |
| ------ |:------------------:|
| y (0)  | Move left/right    |
| x (1)  | Move for-/backward |
| z (2)  | Turn left/right    |
| 3      | Throttle           |

<p float="left">
    <img src="/images/01_MecanumHalfFront.jpg" alt="Frontlight" width="240"/>
    <img src="/images/06_MecanumAngleBack.jpg" alt="Taillight" width="240"/>
</p>

### Steering with Keyboard
For the IOTbot, a minimalistic GUI is provided, that enables the steering with the keyboard.
For installation and usage, please visit the following repository: https://github.com/eduart-robotik/iotbot_virtual_joy.git

# Quick Start for pre-installed Systems
1) Switch on robot and connect to the wireless network (pre-installed systems have the SSID "Eduard_{Color}").
2) The default IP adress of the robot is 192.168.0.100. Check the IP of your system with
```console
ip addr
```
and set the appropriate environment variables, e.g.:
```console
export ROS_IP=192.168.0.200; export ROS_HOSTNAME=192.168.0.200; export ROS_MASTER_URI=https://192.168.0.100:11311
```
3) Then connect to your robot with a second terminal:
```console
ssh root@192.168.0.100
```
The default password is "root". Please change the password at your first login. Consider also to add your own user as sudoer and disable the login for the root account.

4) The robot can be activated with a ROS launch script. It is recommended to start a rosmaster service beforehand:
```console
roscore &
```
Depending on the wheels mounted, start the corresponding script, e.g.:
```console
roslaunch iotbot skid_performance.launch
```
for robots having performance drives and off-road tires mounted. Other options are mecanum_performance.launch for mounted mecanum wheels or skid.launch and mecanum.launch for the low-cost drives.

5) Now go back to the terminal, where you have set the ROS environment variables and launch the virtual joystick node:
```console
python3 src/iotbot_virtual_joy/scripts/iotbot_virtual_joy_node.py _mecanum:=1
```
