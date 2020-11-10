README.md

## Notes

ur_driver is not compatible with Polyscope v3.x and v5.x. Both ur_driver and ur_modern_driver are deprecated. Therefore, use ur_robot_driver instead.

## TLDR (How to use ROS Rviz MoveIt to control the realhardware)
Terminal 1: `roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.1.11`
Load and run external_control_test program on UR5 teaching pendant, now you should see `Robot ready to receive control commands.` on the first Terminal.
Terminal 2: `roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch` to allow motion planning.
Terminal 3: `roslaunch ur5_moveit_config moveit_rviz.launch config:=true` for starting up Rviz and MoveIt motion planning plugin. Now you should be able to plan and execute on Rviz MoveIt window. Please hold the emergency stop button while executing. 

## STEP 1: Building

**Note:** The driver consists of a [C++
library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) that abstracts the
robot's interfaces and a ROS driver on top of that. As the library can be built without ROS support,
it is not a catkin package and therefore requires a different treatment when being built inside the
workspace. See The alternative build method below if you'd like to build the library from source.

If you don't want to build the library from source, it is available as a binary package through the
ROS distribution of ROS kinetic, melodic and noetic. It will be installed automatically if you
follow the steps below. 

```bash
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream. This one is the same with ros-industial/universal_robot repo
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace. We need an isolated build because of the non-catkin library package.
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

## STEP 2: Setting up a UR robot for ur_robot_driver
### Prepare the robot
For using the *ur_robot_driver* with a real robot you need to install the
**externalcontrol-1.0.4.urcap** which can be found inside the **resources** folder of this driver.

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is
necessary.

For installing the necessary URCap and creating a program, please see the individual tutorials on
how to [setup a CB3 robot](ur_robot_driver/doc/install_urcap_cb3.md) or how to [setup an e-Series
robot](ur_robot_driver/doc/install_urcap_e_series.md).

To setup the tool communication on an e-Series robot, please consider the [tool communication setup
guide](ur_robot_driver/doc/setup_tool_communication.md).

### Prepare the ROS PC
For using the driver make sure it is installed (either by the debian package or built from source
inside a catkin workspace).

#### Extract calibration information
Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended
to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.


For this, there exists a helper script:

    $ roslaunch ur_calibration calibration_correction.launch \
      robot_ip:=192.168.1.11 target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As
`target_filename` provide an absolute path where the result will be saved to. This step may not be necessary as the generated calibration file is the same as the default file (ur5_default.yaml).

We recommend keeping calibrations for all robots in your organization in a common package. See the
[package's documentation](ur_calibration/README.md) for details.

#### Quick start
Once the driver is built and the **externalcontrol** URCap is installed on the
robot, you are good to go ahead starting the driver. (**Note**: We do
recommend, though, to [extract your robot's
calibration](#extract-calibration-information) first.)

To actually start the robot driver use one of the existing launch files

    $ roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.11


If you calibrated your robot before, pass that calibration to the launch file (this is only be used if you would like to pass your own calibration file, normally not needed):

    $ roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.11 \
      kinematics_config:=$(rospack find ur_calibration)/etc/ur5_example_calibration.yaml

If the parameters in that file don't match the ones reported from the robot, the driver will output
an error during startup, but will remain usable.

For more information on the launch file's parameters see its own documentation.

Once the robot driver is started, load the [previously generated program](#prepare-the-robot) on the
robot panel that will start the *External Control* program node and execute it. From that moment on
the robot is fully functional. You can make use of the *Pause* function or even *Stop* (:stop_button:) the
program.  Simply press the *Play* button (:arrow_forward:) again and the ROS driver will reconnect.

Inside the ROS terminal running the driver you should see the output `Robot ready to receive control commands.`


To control the robot using ROS, use the action server on

```bash
/scaled_pos_joint_traj_controller/follow_joint_trajectory
```

Use this with any client interface such as [MoveIt!](https://moveit.ros.org/) or simply the
`rqt_joint_trajectory_controller` gui:

```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

## STEP 3: Modification needed

This section will cover some modifications that I made in order to successfully control the UR5 robot with ROS Rviz MoveIt

### Unable to identify any set of controllers that can actuate the specified joints: [ elbow_joint shoulder_lift_joint shoulder_pan_joint wrist_1_joint wrist_2_joint wrist_3_joint ]
Open "/fmauch_universal_robot/ur5_moveit_config/config/controllers.yaml", cope this line to replace whatever after action_ns: scaled_pos_joint_traj_controller/follow_joint_trajectory


