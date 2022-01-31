#pragma once
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int get_joints_idxs(vector<int>& moving_joint_idxs){
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
  
  int n_joints = 0;
	string s1, s2, s3, s4;
  
  input>>s1>>s2; //scale
  input>>s1>>s2; //p_limit
  
  input>>s1; //ignore line
  
  input>>s1>>s2>>s3>>s4;
  while(!input.eof()){
  	moving_joint_idxs.push_back(n_joints+1);
  	if(!stof(s2)==0)
  		++n_joints;
  	input>>s1>>s2>>s3>>s4;
  	++n_joints;
  }
  return n_joints;
}
