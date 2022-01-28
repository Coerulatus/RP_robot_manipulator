#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

using namespace std;

int main(int argc, char** argv) {
  ifstream input;
  input.open("DH_params.txt");
  if(!input){
  	input.open("./src/src_cpp/DH_params.txt");
  	if(!input){
  		cerr<<"Error: could not open DH_params.txt file. \n The file needs to be in the project folder or in the src/src_cpp folder."<<endl;
  		return -1;
  	}
  }
  
  char l[256];
  string s1, s2, s3, s4;
  vector<float> alphas,as,ds,thetas;
  vector<bool> is_joint_revolute;
  
  //ignore first line
  input.getline(l,256);
  
  input>>s1>>s2>>s3>>s4;
  while(!input.eof()){
  	// check alpha_i for +-pi/2
  	if(s1=="pi/2")
  		alphas.push_back(M_PI/2);
  	else if(s1=="-pi/2")
  		alphas.push_back(-M_PI/2);
  	else
  		alphas.push_back(stof(s1));

  	as.push_back(stof(s2));

  	// check d_i for qi  	
  	if(s3[0]=='q'){
  		// initialize q values as zero
  		ds.push_back(0);
  		is_joint_revolute.push_back(0);
  	}else{
  		ds.push_back(stof(s3));
  	}

  	//check theta_i for qi
  	if(s4[0]=='q'){
  		// initialize q values as zero
  		thetas.push_back(0);
  		is_joint_revolute.push_back(1);
  	}else{
  		thetas.push_back(stof(s4));
  	}
  	
  	
  	cout<<alphas.back()<<' '<<as.back()<<' '<<ds.back()<<' '<<thetas.back()<<endl;
  	
  	input>>s1>>s2>>s3>>s4;
  }
  return 0;
}

