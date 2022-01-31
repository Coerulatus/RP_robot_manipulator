#include <Eigen/Core>
#include <Eigen/StdVector>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "utils_control.h"
#include "utils_kinematics.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
	ros::init(argc, argv, "inverse_kinematics_node");
	ros::NodeHandle n;
	
	vector<int> moving_joint_idxs;
  if(get_joints_idxs(moving_joint_idxs)==-1)
  	return -1;
  int n_moving_joints = moving_joint_idxs.size();
  
  vector<ros::Publisher> joints_pubs;
  string topic;
  for(int i=0;i<n_moving_joints;++i){
		topic = "l"+to_string(moving_joint_idxs[i])+"_controller/command";
		joints_pubs.push_back(n.advertise<std_msgs::Float64>(topic, 100));
  }
  
  ifstream input;
  input.open("DH_params.txt");
  if(!input){
  	input.open("./src/RP_robot_manipulator/src_cpp/DH_params.txt");
  	if(!input){
  		cerr<<"Error: could not open DH_params.txt file. \n The file needs to be in the project folder or in the src/src_cpp folder."<<endl;
  		return -1;
  	}
  }
  
  string s1, s2, s3, s4;
  vector<float> alphas,as,ds,thetas;
  vector<bool> is_joint_revolute;
  float scale;
  float p_limit;
  
  input>>s1>>s2;
  scale = stof(s2);
  input>>s1>>s2;
  p_limit = stof(s2);
  //ignore line
  input>>s1;
  input>>s1>>s2>>s3>>s4;
  while(!input.eof()){
  	// check alpha_i for +-pi/2
  	if(s1=="pi/2")
  		alphas.push_back(M_PI/2);
  	else if(s1=="-pi/2")
  		alphas.push_back(-M_PI/2);
  	else if(s1=="-pi")
  		alphas.push_back(-M_PI);
  	else if(s1=="pi")
  		alphas.push_back(M_PI);
  	else
  		alphas.push_back(stof(s1));

  	as.push_back(stof(s2));

  	// check d_i for qi  	
  	if(s3[0]=='q'){
  		// initialize q values as 0 for prismatic joint
  		ds.push_back(0); 
  		is_joint_revolute.push_back(0);
  	}else{
  		ds.push_back(stof(s3));
  	}

  	//check theta_i for qi
  	if(s4[0]=='q'){
  		// initialize q values as 0 for revolute joints
  		thetas.push_back(0);
  		is_joint_revolute.push_back(1);
  	}else{
  		thetas.push_back(stof(s4));
  	}
  	
  	input>>s1>>s2>>s3>>s4;
  }
  	
  input.close();
  
  MatrixXf J;
  Vector3f current_p;
  Vector3f desired_p;
  float error;
  float error_prev;
  float conv_rate = 1e-3;
  VectorXf delta_q;
  cout << "Waiting for desired coordinates.\nType q to exit."<<endl;
  int n_iter;
  
  std_msgs::Float64 msg;
  while(ros::ok()){
  	cin >> s1;
  	if(s1=="q")
  	  break;
  	cin >> s2;
  	if(s2=="q")
  	  break;
  	cin >> s3;
  	if(s3=="q")
  	  break;
  	try{
			desired_p(0) = stof(s1);
			desired_p(1) = stof(s2);
			desired_p(2) = stof(s3);
			J = jacobian(is_joint_revolute,alphas,as,ds,thetas);
			current_p = direct_kinematics(alphas,as,ds,thetas);
			error = euclidean_distance(current_p,desired_p);
			error_prev = error;
			n_iter = 0;
			while(error>1e-4){
				delta_q = conv_rate*J.transpose()*(desired_p-current_p);
				for(int i=0;i<delta_q.size();++i){
					if(is_joint_revolute[i])
						thetas[i] += delta_q(i);
					else
						ds[i] += delta_q(i);
				}
				J = jacobian(is_joint_revolute,alphas,as,ds,thetas);
				current_p = direct_kinematics(alphas,as,ds,thetas);
				error = euclidean_distance(current_p,desired_p);
				if(error_prev<error)
					conv_rate /= 2;
				else if(abs(error_prev-error)<1e-5)
					conv_rate *= 2;
				error_prev = error;
				n_iter++;
				if(n_iter > 10000){
					cout<<"Max number of iterations reached, couldn't converge"<<endl;
					break;
				}
			}
			for(int i=0;i<n_moving_joints;++i){
				if(is_joint_revolute[i])
					msg.data = thetas[i];
				else
					msg.data = ds[i];
				joints_pubs[i].publish(msg);
			}
		}catch( ... ){
  		cout << "Invalid input" << endl;
  	}
  }
	return 0;
}
