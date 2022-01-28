# RP_robot_manipulator
Repository for my project for the course Robot Programming.
Academic year: 2021/2022

To build the project you need to follow these commands:
* ```mkdir -p project_folder/src```
* ```cd project_folder```
* ```catkin build project_rp```
* ```source devel/setup.bash```

To launch gazebo:
* ```roslaunch project_rp gazebo.launch```

To launch executable:
* ```rosrun project_rp make_manipulator```

ATTENTION: DH_params.txt file needs to be either in project_folder or in project_folder/src/src_cpp/
