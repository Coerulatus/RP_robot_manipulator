# RP_robot_manipulator
Repository for my project for the course Robot Programming.
Academic year: 2021/2022

### Instructions to use the program
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

To control robot position:
* ```rosrun project_rp inverse_kinematics```

The program asks for the x,y,z coordinates that the robot end effector should reach. A gradient method is used so the robot might stop close to singularities or might try an unreachable configuration. Joint limits are not checked. The initial configuration is the current robot pose.

### DH_params.txt explanation
DH_params.txt file needs to be either in project_folder or in project_folder/src/RP_robot_manipulator/src_cpp/

The robot's links have default radius of 0.1, which means they are suitable for links of length between 0.5 and 5. The scale parameter changes the default value of the joint radius, by multiplying 0.1 by scale.

The p_limit parameter sets the limit for the prismatic joints of the model.

The third line is there to separate the file but is not used by the program; it needs to be kept for the program to work properly.

For the *Denavit–Hartenberg* parameters the conventions used are:
* the first joint is positioned on [0,0,0] and has the z-axis pointing upwards
* if the last joint is revolute then the last reference frame needs to have the x-axis pointing in the direction of the link
* if the last joint is prismatic then the last reference frame needs to have the z-axis pointing in the direction of the link

The projects comes with an example file already present in the src_cpp folder.
