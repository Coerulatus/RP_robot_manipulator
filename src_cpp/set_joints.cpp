#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "set_joints_node");
	
	ros::NodeHandle n;

	int n_joints = 0;
	// get number of joints directly from DH_params
	ifstream input;
  input.open("DH_params.txt");
  if(!input){
  	input.open("./src/RP_robot_manipulator/src_cpp/DH_params.txt");
  	if(!input){
  		cerr<<"Error: could not open DH_params.txt file. \n The file needs to be in the project folder or in the src/src_cpp folder."<<endl;
  		return -1;
  	}
  }
  
  char l[256];
  //ignore first line
  input.getline(l,256);
  input.getline(l,256);
  while(!input.eof()){
  	++n_joints;
  	input.getline(l,256);
  }
  
  vector<ros::Publisher> joints_pubs;
  vector<string> j_values;
  string topic;
  for(int i=0;i<n_joints;++i){
  	topic = "l"+to_string(i+1)+"_controller/command";
  	joints_pubs.push_back(n.advertise<std_msgs::Float64>(topic, 100));
  	j_values.push_back("0");
  }
   
  while(ros::ok()){
		cout << "Waiting for the values for the joints. \n Use s to keep the joint still.\n Type q to exit." << endl;
		string j_v;
		std_msgs::Float64 msg;
		int j_idx=0;
		while(cin >> j_v){
			if(j_v=="q")
				break;
			else if(j_v=="s"){
				cout << "Keeping joint "<<(j_idx%n_joints)+1<<" still"<<endl;
				++j_idx;
			}else{
				try{
					msg.data = stof(j_v);
					joints_pubs[j_idx%n_joints].publish(msg);
					cout << "Setting position of joint " << (j_idx%n_joints)+1 << " to: "<<j_v<<endl;
					++j_idx;
				}catch( ... ){
					cout << "Invalid argument" <<endl;
				}
			}
		}
		break;
  }
	return 0;
}
