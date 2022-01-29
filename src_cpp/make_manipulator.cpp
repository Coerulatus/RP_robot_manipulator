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
  	input.open("./src/RP_robot_manipulator/src_cpp/DH_params.txt");
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
  	
  	//cout<<alphas.back()<<' '<<as.back()<<' '<<ds.back()<<' '<<thetas.back()<<endl;
  	
  	input>>s1>>s2>>s3>>s4;
  }
  input.close();
  is_joint_revolute.push_back(1);
  
  //make xacro file
  ofstream xacro_file;
  xacro_file.open("./src/urdf/main.xacro");
  xacro_file<<"<?xml version=\"1.0\"?>"<<endl;
  xacro_file<<"<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"my_robot\">"<<endl;
  xacro_file<<"    <xacro:include filename=\"$(find project_rp)/urdf/r_link.xacro\" />"<<endl;
  xacro_file<<"    <xacro:include filename=\"$(find project_rp)/urdf/base.xacro\" />"<<endl;
  xacro_file<<"    <xacro:base name=\"l0\" />"<<endl;
  for(int i=0;i<alphas.size()+1;++i){
  	if(is_joint_revolute[i]){
  		string s_joint = "    <xacro:r_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_rpy=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\" type=\"\"/>";
  		
  		string j_type;
  		string xyz_link;
  		string link_rpy;
  		string xyz_joint;
  		string joint_rpy;
  		string length;
  		if(i==0){
  			j_type = "continuous"; 
  		 	xyz_link = "0 0 "+to_string(ds[i]/2);
  		 	link_rpy = "0 0 0";
  		 	xyz_joint = "0 0 0";
  		 	joint_rpy = "0 0 0";
  		 	if(ds[i]==0)
					length = "0.2";
				else
					length = to_string(ds[i]);
  		}else if(i==alphas.size()){
  			j_type = "revolute"; //with no specified limits it's fixed
  			xyz_link = to_string(as[i-1]/2)+" 0 0";
  			link_rpy = "0 "+to_string(M_PI/2)+" 0";
  			xyz_joint = "0 0 0";
  			joint_rpy = to_string(alphas[i-1])+" 0 0";
  			length = to_string(as[i-1]);
  		}else{
  			j_type = "continuous";
  			xyz_link = "0 0 "+to_string(ds[i]/2);
  			link_rpy = "0 0 0";
  			xyz_joint = "0 0 "+to_string(ds[i-1]);
  			joint_rpy = to_string(alphas[i-1])+" 0 0";
  			if(ds[i]==0)
					length = "0.2";
				else
					length = to_string(ds[i]);
  		}
  		
			//type
			s_joint.insert(116,j_type);
			//link_xyz
			s_joint.insert(108,xyz_link);
			//link_rpy
			s_joint.insert(96,link_rpy);
			//joint_xyz
			s_joint.insert(84,xyz_joint);
			//rpy
			s_joint.insert(71,joint_rpy);
  		//radius
  		s_joint.insert(58,"0.1");
  		//length
  		s_joint.insert(48,length);
  		//parent name
  		s_joint.insert(38,to_string(i));
  		//link name
  		s_joint.insert(27,to_string(i+1));
  		
  		xacro_file<<s_joint<<endl;
  	}else{
  		cout<<"No prismatic joint implementation"<<endl;
  		return -1;
  	}
  }
  xacro_file<<"    <gazebo>"<<endl;
  xacro_file<<"        <plugin name=\"gazebo_ros_control\" filename=\"libgazebo_ros_control.so\">"<<endl;
  xacro_file<<"            <robotNamespace>/</robotNamespace>"<<endl;
  xacro_file<<"        </plugin>"<<endl;
  xacro_file<<"    </gazebo>"<<endl;
  xacro_file<<"</robot>"<<endl;
  
  xacro_file.close();
  
  //gazebo file
  ifstream g_template;
  g_template.open("./src/RP_robot_manipulator/launch/gazebo_template.txt");
  ofstream gazebo;
  gazebo.open("./src/RP_robot_manipulator/launch/gazebo.launch");
  int line = 1;
  while(!g_template.eof()){
  	if(line == 43){
  		for(int i=0;i<alphas.size();++i)
  			gazebo << "        l" << i+1 << "_controller" <<endl;
  	}
  	g_template.getline(l,256);
  	gazebo << l <<endl;
  	++line;
  }
  g_template.close();
  gazebo.close();
  	
  //joints file
  ofstream joints_file;
	joints_file.open("./src/RP_robot_manipulator/config/joints.yaml");
	joints_file << "joint_state_controller:" << endl;
	joints_file << "  type: \"joint_state_controller/JointStateController\"" << endl;  
	joints_file << "  publish_rate: 50" << endl;
	for(int i=0;i<alphas.size();++i){
		joints_file << "l" << i+1 << "_controller:" << endl;
		joints_file << "  type: \"position_controllers/JointPositionController\"" << endl;
		joints_file << "  joint: l" << i << "_to_l" << i+1 <<endl;
	}

	joints_file.close();

  
  
  return 0;
}

