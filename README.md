# RP_robot_manipulator
Repository for my project for the course Robot Programming.
Academic year: 2021/2022

To build the project you need to follow these commands:
* ```mkdir -p project_folder/src```
* ```cd project_folder/src```
* ```git clone https://github.com/Coerulatus/RP_robot_manipulator.git```
* ```cd ..```
* ```catkin build project_rp```
* ```source devel/setup.bash```

To launch gazebo:
* ```roslaunch project_rp gazebo.launch```

To create manipulator's files from DH_params:
* ```rosrun project_rp make_manipulator```

To controll the joints:
* ```rusrun project_rp set_joints```

The program loops over the joints asking the value to set the current joint to. More than one value can be specified at a time. Use 's' to keep a joint still. Use 'q' to exit the program.

ATTENTION: DH_params.txt file needs to be either in project_folder or in project_folder/src/src_cpp/
