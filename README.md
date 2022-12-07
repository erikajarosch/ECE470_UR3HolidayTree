# Christmas-tree decorating robotic arm ##

## Description
- The code is to be executed in a pre-built virtual environment <a href="http://coecsl.ece.illinois.edu/ece470/fall21/ECE470VM.zip" target="_blank">ECE470</a> in <a href="https://www.vmware.com/content/vmware/vmware-published-sites/us/products/workstation-player/workstation-player-evaluation.html.html" target="_blank">VMware Workstation 16 Player</a>
- This is the final course project for **ECE470 - Introduction to Robotics** course at the University of Illinois at Urbana-Champaign

<!-- - In a new *Ubuntu* terminal ,run **rosrun lab2_pkg.py lab2_spawn.py** to input the blocks in the simulation -->
<!-- - Enter **3** and **n** to input 3 blocks -->
<!-- - Run **rosrun lab2pkg_py lab2_exec.py --simulator True** -->
<!-- - Input **{1,2,3}** for start and end towers  -->


## Dependencies
- We use Ubuntu 16.04, Python 2.7.12, ROS kinetic (1.12.14), Gazebo 7.16.1 within the virtual environment

## How to use
- In a *Ubuntu* terminal run **source devel/setup.bash** to properly set up the catkin directory; alternatively, add **source devel/setup.bash** to **~/.bashrc** the **.bash** file is sourced automatically each time a new terminal is opened 
- In the same terminal run **roslaunch ur3_driver ur3_gazebo_git.launch** to initiate the UR3 robot in the Gazebo simulation engine
- By default, when the above command is run the UR3 robot and an **empty.world** Gazebo environment shows up;
  to insert external models, include the model parameters in **test.world** located at this path: */src/lab2andDriver/drivers/ur3_driver/worlds*
- **test.world** is our customized configuration file added to **ur3_gazebo.launch** located at this path */src/lab2andDriver/drivers/ur3_driver/launch*
```
  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find ur3_driver)/worlds/test.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

```


<!-- ## Code Walk Through -->
<!-- *Initialize Q matrix*
- Initialize **Q** which is a 2D array that consists of the robot's orientation at each position(E.g **Q11** is the orientation of the robot when it is at the first tower position with $3$ blocks stacked, **Q21** is the orientation of the robot when it is at the second tower position with $3$ blocks stacked). -->

<!-- 
*gripper_callback(msg)*
- Takes in the info published from ur3/gripper_input
- gets the state of suction cup based on the input -->

<!-- *position_callback(msg)*
- Callback function thaat takes in the new angle position data published -->


<!-- *gripper(pub_cmd, loop_rate, io_0)*
- A function that controls the gripper based on the current position of the gripper and desired position of the block.


*move_arm(pub_cmd, loop_rate, dest, vel, accel)*
- A function that moves the robotic arm using the input destination, velocity and acceleration.

*move_block(pub_cmd, loop_rate, start_loc, start_height, end_loc, end_height)*
- A function that moves one block from start position to end position.

*main()*
- Main loop that determines how the UR3 robot moves according to user input           
          -->
          
## References
http://sdformat.org/spec?ver=1.9&elem=sdf
