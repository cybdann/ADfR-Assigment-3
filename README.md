# ROS2-Jiwy-Closed-Loop-Simulator
UTwente - Advanced Software Development for Robotics - Assigment 3

## Description of the package
Simulator for the Jiwy pan-tilt robot. The package will open two windows, one is the webcam imag, the second one is a moving image. The moving image will follow a bright source light.

## Building the package
Source your ROS2 SDK, then create a workspace, add this repository to its sources and build the packages.

```
$ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
$ mkdir -p ws/src
$ cd ws/src
$ git clone https://github.com/cybdann/ROS2-Webcam-Image-Processing
$ cd ..
$ colcon build
$ source install/local_setup.sh
```
## Running the executables
Every executable can be launched with the following command:
```
ros2 launch jiwy_simulator jiwy_simulator.launch.py
```
