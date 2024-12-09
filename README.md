# Pharmacy_Robot
ENPM 662 Project 2 Pharmacy Robot

# Pharmacy Robot Instructions
Contains the robot description and controllers for simulation in ROS2 and Gazebo

### Build instructions

First, source your ROS 2 workspaces with all the required dependencies.
Then, you are ready to clone and build this repository.
You should only have to do this once per install.

```sh
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/khuzema-h/Pharmacy_Robot.git
cd ..
rosdep install --from-path src --ignore-src -yi
colcon build
```

### Initialization instructions

You will have to do this in every new session in which you wish to use these examples:

```sh
source ~/dev_ws/install/local_setup.sh
```

### Launch Robot in World

After Building and Sourcing the Workspace, Launch the Pharmacy Robot:

```sh
ros2 launch pharmacy_robot gazebo_launch.py
```
## Run Scanner Node

Open a Separate Terminal Window after Launching the robot in Gazebo to run the Scanner to read QR codes:

```sh
ros2 run pharmacy_robot scanner.py
```
![Screenshot from 2024-12-05 13-11-17](https://github.com/user-attachments/assets/76e933a2-a8f4-4c31-a25e-77d2407a179c)
## Run Teleop Controller:

Open a Separate Terminal Window after Launching the robot in Gazebo and run the following node for teleop: 

```sh
ros2 run pharmacy_robot teleop.py
```
![Screenshot from 2024-12-08 20-18-28](https://github.com/user-attachments/assets/29257812-2cc9-4bdd-ad53-7fa47363e69b)

### RVIZ Visualization

Open a Separate Terminal Window to visualize the camera feed in Rviz.


```sh
rviz2
```
Add -> By Topic -> image
![Screenshot from 2024-12-07 17-15-16](https://github.com/user-attachments/assets/1113ee50-6765-4d94-b571-1c5382e9414e)


## To visualize the joints in RVIZ: 

In another terminal run:

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
In Rviz:
Add -> Robot Model 

![Screenshot from 2024-12-07 17-09-45](https://github.com/user-attachments/assets/0bbf78a7-9f73-44f3-96ec-b585f3e7ee4a)







### Potential pitfalls

If you are unable to automatically install dependencies with rosdep (perhaps due to [this issue](https://github.com/ros-infrastructure/rosdep/issues/733)), please do be sure to manually install the dependencies for your particular example of interest, contained in its package.xml file.
