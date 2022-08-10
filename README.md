# ur10e_moveit_sfm

Download and install moveit tutorials:

```
mkdir ~/ws_moveit/src 
cd ~/ws_moveit/src
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git 
```

download and install ur10e moveit config:
```
git: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur10_e_moveit_config
```

Copy my python script to this path:
```
~/ws_moveit/src/moveit_tutorials/doc/move_group_python_interface/scripts/SfM_trajectory.py
```

Use catkin to build workspace then source devel/setup.bash

Make my script excutible using:
```
chmod +x src/moveit_tutorials/doc/move_group_python_interface/scripts/SfM_trajectory.py
```

Run rviz moveit with ur10e config: 
```
roslaunch ur10_e_moveit_config demo.launch
```

Then start another shell, source devel/setup.bash and run my script using:
```
rosrun moveit_tutorials SfM_trajectory.py
```
(press enter when prompted to continue)

Within the script, the function called my_go_to_pose_goal() is where all the important parameters can be found.
