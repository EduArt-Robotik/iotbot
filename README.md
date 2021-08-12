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
roslaunch src/iotbot/launch/skid_performance.launch
```
When everthing is initialized well, one should see the following output:
```console
[ WARN] [1628772862.880453339]: Lag detected ... deactivate motor control
```
This is not a bug, but a feature, since the robot will stop immediately, if no frequent communication with a controlling node is established.

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

![Frontlight](/images/01_MecanumHalfFront.jpg, "Frontlight")![Taillight](/images/06_MecanumAngleBack.jpg, "Taillight")

<img src="/images/01_MecanumHalfFront.jpg" alt="Frontlight" width="320"/>

### Steering with Keyboard
For the IOTbot, a minimalistic GUI is provided, that enables the steering with the keyboard.
For installation and usage, please visit the following repository: https://github.com/eduart-robotik/iotbot.git