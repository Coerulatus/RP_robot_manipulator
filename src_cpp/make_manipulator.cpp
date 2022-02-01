#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include "utils_iofile.h"

using namespace std;

int main(int argc, char** argv) {
  // read DH_params.txt
  robot_params dh_params(read_dh_params());
  
  // add links to main.xacro
	make_main_xacro(dh_params.is_joint_revolute, dh_params.alphas, dh_params.as, dh_params.ds, dh_params.thetas, dh_params.scale, dh_params.p_limit);
  
  // add controllers to gazebo.launch file
  add_gazebo_controllers(dh_params.fixed_joints);
  	
  // add joints to joints.yaml file
  add_yaml_joints(dh_params.fixed_joints);
  
  return 0;
}

