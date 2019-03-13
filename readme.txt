Control of a 3-Finger Gripper using the Modbus TCP protocol 
http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%203-Finger%20Gripper%20using%20the%20Modbus%20TCP%20Protocol

Prerequisites
This tutorial assumes that you have a 3-Finger Gripper configured with the Modbus TCP protocol. 
The Gripper should be connected to a network, which has been properly configured 
(you can validate the communication with the Gripper using the Robotiq User Interface).

Finally, make sure that the external dependency for the package "robotiq_modbus_tcp" has been installed. 
The dependency is the python package pyModbus. From Ubuntu Precise, it is simply installed using "rosdep install robotiq_modbus_tcp". 
On other systems, it can be installed using "easy_install -U pymodbus".


1. Run the 3-Finger Gripper Driver Node
"rosrun robotiq_3f_gripper_control Robotiq3FGripperTcpNode.py 192.168.1.11".

The Gripper is driven by the node "Robotiq3FGripperTcpNode.py" contained in the package "robotiq_3f_gripper_control". 
The IP address of the Gripper has to be provided as an argument.

For example, the driver for controlling a 3-Finger Gripper with the IP address 192.168.1.11 is launched using the following command: 
"rosrun robotiq_3f_gripper_control Robotiq3FGripperTcpNode.py 192.168.1.11".

2. Run the 3-Finger Gripper Simple Controller Node
"rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py".

The driver listens for messages on "Robotiq3FGripperRobotOutput" using the "Robotiq3FGripper_robot_output" msg type. 
The messages are interpreted and commands are sent to the gripper accordingly. 
A simple controller node is provided which can be run (at another terminal) using 
"rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py".

The "Robotiq3FGripper_robot_output" msg type is simply composed of the robot output variables described in the Robotiq 3-Finger Gripper Instruction Manual. 
The simple controller node can therefore be modified to send custom commands to the Gripper.

3. Run the 3-Finger Gripper Status Listener Node
"rosrun robotiq_3f_gripper_control Robotiq3FGripperStatusListener.py".

In the package "robotiq_3f_gripper_control", there is also a node for listening to and interpreting the status of the Gripper. 
The driver publishes the status of the Gripper on "Robotiq3FGripperRobotInput" using the "Robotiq3FGripper_robot_input" msg type. 
The msg type is composed of the robot input variables described in the Robotiq 3-Finger Gripper Instruction Manual. The status listener node can be run (at another terminal) using the following command: 
"rosrun robotiq_3f_gripper_control Robotiq3FGripperStatusListener.py".

==================================================================================================================================================
Control of a 2-Finger Gripper using the Modbus RTU protocol 
1. Prerequisites
It is assumed you have the robotiq_2f_gripper_control package and the robotiq_modbus_rtu package installed for your ROS distribution 
(see the [[https://github.com/ros-industrial/robotiq || robotiq packages on ROS-I github).

2. Configuring the serial port
To control the gripper over a serial port, you may need to give proper privileges to the user:
sudo usermod -aG dialout $USER

To find out the port on which the controller is connected, use:
dmesg | grep tty


3. ROS Nodes to Control the Gripper
3.1 Run the 2-Finger Gripper Driver Node
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

The gripper is driven by the node "Robotiq2FGripperRtuNode.py" contained in the package "robotiq_2f_gripper_control". 
Note that "roscore" should be running prior to launching the driver node. 
The gripper device ID will also need to be provided as an argument. Thus, to know what is the device ID of your 2-finger gripper, 
one way is simply by typing "dmesg | grep ttyUSB" in a terminal right after the gripper was connected to the computer's USB port.

For example, the driver for controlling a 2-finger gripper having "ttyUSB0" as its device ID is launched using the following command:
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

3.2 Run the 2-Finger Gripper Simple Controller Node
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

The driver listens for messages on "Robotiq2FGripperRobotOutput" using the "Robotiq2FGripper_robot_output" msg type. 
The messages are interpreted and commands are sent to the gripper accordingly. 
A simple controller node is provided which can be run (in another terminal) using 
"rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py"

3.3 Run the 2-Finger Gripper Status Listener Node
In the package "robotiq_2f_gripper_control", there is also a node for listening to and interpreting the status of the Gripper. 
The driver publishes the status of the Gripper on "Robotiq2FGripperRobotInput" using the "Robotiq2FGripper_robot_input" msg type. 
The msg type is composed of the robot input variables described in the Robotiq 2-Finger Gripper Instruction Manual. 
The status listener node can be run (in another terminal) using the following command: 
"rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py".




----------------------------------------------------------------------------------------------------------------------------------------------

chunting@UR-Robot:~/catkin_ws$ rosdep install --from-paths src/ --ignore-src --rosdistro indigo
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
joint_trajectory_admittance_controller: Cannot locate rosdep definition for [kdl]
chunting@UR-Robot:~/catkin_ws$ rosdep install --from-paths src/ --ignore-src --rosdistro indigo
#All required rosdeps installed successfully
chunting@UR-Robot:~/catkin_ws$ rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml]:
	rosdep data from [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml] is not a YAML dictionary
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml]:
	rosdep data from [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml] is not a YAML dictionary
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml]:
	rosdep data from [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml] is not a YAML dictionary
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml]:
	rosdep data from [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml] is not a YAML dictionary
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml]:
	Failed to download target platform data for gbpdistro:
	targets data must be a dict
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml

ERROR: Rosdep experienced an error: string indices must be integers, not str
Please go to the rosdep page [1] and file a bug report with the stack trace below.
[1] : http://www.ros.org/wiki/rosdep

rosdep version: 0.15.1

Traceback (most recent call last):
  File "/usr/lib/python2.7/dist-packages/rosdep2/main.py", line 140, in rosdep_main
    exit_code = _rosdep_main(args)
  File "/usr/lib/python2.7/dist-packages/rosdep2/main.py", line 394, in _rosdep_main
    return _no_args_handler(command, parser, options, args)
  File "/usr/lib/python2.7/dist-packages/rosdep2/main.py", line 403, in _no_args_handler
    return command_handlers[command](options)
  File "/usr/lib/python2.7/dist-packages/rosdep2/main.py", line 601, in command_update
    skip_eol_distros=not options.include_eol_distros)
  File "/usr/lib/python2.7/dist-packages/rosdep2/sources_list.py", line 489, in update_sources_list
    for dist_name in sorted(get_index().distributions.keys()):
  File "/usr/lib/python2.7/dist-packages/rosdep2/rosdistrohelper.py", line 69, in get_index
    _RDCache.index = rosdistro.get_index(_RDCache.index_url)
  File "/usr/lib/python2.7/dist-packages/rosdistro/__init__.py", line 108, in get_index
    return Index(data, base_url, url_query=url_parts.query)
  File "/usr/lib/python2.7/dist-packages/rosdistro/index.py", line 46, in __init__
    assert data['type'] == Index._type, "Expected file type is '%s', not '%s'" % (Index._type, data['type'])
TypeError: string indices must be integers, not str


chunting@UR-Robot:~/catkin_ws$ sudo rosdep init 
[sudo] password for chunting: 
ERROR: default sources list file already exists:
	/etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize
chunting@UR-Robot:~/catkin_ws$ sudo rm /etc/ros/rosdep/sources.list.d/20-default.list 
chunting@UR-Robot:~/catkin_ws$ sudo rosdep init 
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

	rosdep update

chunting@UR-Robot:~/catkin_ws$ rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Add distro "bouncy"
Add distro "crystal"
Skip end-of-life distro "groovy"
Skip end-of-life distro "hydro"
Add distro "indigo"
Skip end-of-life distro "jade"
Add distro "kinetic"
Add distro "lunar"
Add distro "melodic"
updated cache in /home/chunting/.ros/rosdep/sources.cache
chunting@UR-Robot:~/catkin_ws$ rosdep update

