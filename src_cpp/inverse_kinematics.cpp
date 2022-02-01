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
#include "utils_iofile.h"

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
  
  // read DH_params.txt
  robot_params dh_params(read_dh_params());
  
  MatrixXf J;
  Vector3f current_p;
  Vector3f desired_p;
  float error;
  float error_prev;
  float conv_rate = 1e-3;
  VectorXf delta_q;
  cout << "Waiting for desired coordinates.\nType q to exit."<<endl;
  int n_iter;
  string s1,s2,s3;
  
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
			J = jacobian(dh_params.is_joint_revolute,dh_params.alphas,dh_params.as,dh_params.ds,dh_params.thetas);
			current_p = direct_kinematics(dh_params.alphas,dh_params.as,dh_params.ds,dh_params.thetas);
			error = euclidean_distance(current_p,desired_p);
			error_prev = error;
			n_iter = 0;
			while(error>1e-4){
				delta_q = conv_rate*J.transpose()*(desired_p-current_p);
				for(int i=0;i<delta_q.size();++i){
					if(dh_params.is_joint_revolute[i])
						dh_params.thetas[i] += delta_q(i);
					else
						dh_params.ds[i] += delta_q(i);
				}
				J = jacobian(dh_params.is_joint_revolute,dh_params.alphas,dh_params.as,dh_params.ds,dh_params.thetas);
				current_p = direct_kinematics(dh_params.alphas,dh_params.as,dh_params.ds,dh_params.thetas);
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
				if(dh_params.is_joint_revolute[i])
					msg.data = dh_params.thetas[i];
				else
					msg.data = dh_params.ds[i];
				joints_pubs[i].publish(msg);
			}
		}catch( ... ){
  		cout << "Invalid input" << endl;
  	}
  }
	return 0;
}
