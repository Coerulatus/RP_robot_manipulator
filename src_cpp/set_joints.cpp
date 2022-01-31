#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "utils_control.h"

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "set_joints_node");
	
	ros::NodeHandle n;
	
  vector<int> moving_joint_idxs;
  if(get_joints_idxs(moving_joint_idxs)==-1)
  	return -1;
  int n_moving_joints = moving_joint_idxs.size();
  
  vector<ros::Publisher> joints_pubs;
  vector<string> j_values;
  string topic;
  for(int i=0;i<n_moving_joints;++i){
		topic = "l"+to_string(moving_joint_idxs[i])+"_controller/command";
		joints_pubs.push_back(n.advertise<std_msgs::Float64>(topic, 100));
		j_values.push_back("0");
  }
  /*for(int i=0;i<fixed_joint.size();i++)
  	cout<<i<<" "<<fixed_joint[i]<<endl;*/
  cout << "Waiting for the values for the joints. \nUse s to keep the joint still.\nType q to exit." << endl; 
  int j_idx=0;
  string j_v;
	std_msgs::Float64 msg;
  while(ros::ok()){
		cin >> j_v;
		if(j_v=="q")
			break;
		else if(j_v=="s"){
			cout << "Keeping joint "<<(moving_joint_idxs[j_idx%n_moving_joints])<<" still"<<endl;
			++j_idx;
		}else{
			try{
				msg.data = stof(j_v);
				joints_pubs[j_idx%n_moving_joints].publish(msg);
				cout << "Setting position of joint " << (moving_joint_idxs[j_idx%n_moving_joints]) << " to: "<<j_v<<endl;
				++j_idx;
			}catch( ... ){
				cout << "Invalid argument" <<endl;
			}
		}
  }
	return 0;
}
