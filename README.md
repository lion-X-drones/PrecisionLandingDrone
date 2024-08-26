# PrecisionLandingDrone

Precision landing using ArUco markers over GPS enhances the landing accuracy of drones by incorporating visual markers in addition to GPS data. While GPS provides a general location with an accuracy of 1-2 meters, it can be insufficient for precise applications, especially in environments where signal interference is common. By detecting ArUco markers placed at the landing site with an onboard camera, the drone can achieve much finer accuracy, often within a few centimeters.

ArUco markers are square fiducial markers with unique IDs that allow the drone's vision system to recognize and calculate the marker's position and orientation. This visual feedback enables the drone to adjust its landing trajectory in real-time, ensuring a more accurate descent. This method is particularly useful in scenarios requiring exact landings, such as on small charging pads or specific locations in complex environments.


## Prerequisites for simulation
You should have basic understanding of these
- **ROS Noetic (Ubuntu 20.04)**: Robot Operating System (ROS or ros) is an [open-source](https://en.wikipedia.org/wiki/Open-source_software) [robotics middleware](https://en.wikipedia.org/wiki/Robotics_middleware) suite. Although ROS is not an operating system (OS) but a set of software frameworks for robot software development. ROS Noetic Ninjemys is primarily targeted at the Ubuntu 20.04 (Focal) release, though other systems are supported to varying degrees. Note that you should install ROS Noetic Full Desktop version.

- **Ardupilot**- The ArduPilot software suite consists of navigation software (typically referred to as firmware when it is compiled to binary form for microcontroller hardware targets) running on the vehicle (either Copter, Plane, Rover, AntennaTracker, or Sub), along with ground station controlling software including Mission Planner, APM Planner, QGroundControl, MavProxy, Tower and others.

- You also need to install plugins which you can refer from [ardupilot-installation](https://github.com/Bhaveshmeghwal21/AMC_Summer_Camp-2024/blob/main/Intermediate/ROS/Ardupilot-installation.md)
- OpenCV Library


## Running the simulation
### Gazebo-ROS
Open one terminal and launch ROS integrated gazebo
```bash
#Make sure you have all the right environment, if you are not sure run the following first

source /opt/ros/noetic/setup.bash

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH 
export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:$GAZEBO_PLUGIN_PATH

#Launch ROS integrated Gazebo

source ~/ardupilot_gazebo_roscam/devel/setup.bash

roslaunch ardupilot_gazebo iris_with_roscam.launch
```
### Launch SITL Ardupilot
Open second terminal and launch SITL Ardupilot
```
cd ~/ardupilot/ArduCopter

sim_vehicle.py -f gazebo-iris --console --map
```

### Launch MAVROS
```bash
cd ~/ardupilot_ws/src/launch && roslaunch apm.launch
```
### Run the [script](https://github.com/lion-X-drones/PrecisionLandingDrone/blob/main/Script/aruco_landing.py)

```bash
python3 aruco_landing.py
```
