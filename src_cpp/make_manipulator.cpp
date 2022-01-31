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
  
  string s1, s2, s3, s4;
  vector<float> alphas,as,ds,thetas;
  vector<bool> is_joint_revolute;
  char l[256];
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
  
  //make xacro file
  ofstream xacro_file;
  xacro_file.open("./src/RP_robot_manipulator/urdf/main.xacro");
  xacro_file<<"<?xml version=\"1.0\"?>"<<endl;
  xacro_file<<"<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"my_robot\">"<<endl;
  xacro_file<<"    <xacro:include filename=\"$(find project_rp)/urdf/r_link.xacro\" />"<<endl;
  xacro_file<<"    <xacro:include filename=\"$(find project_rp)/urdf/p_link.xacro\" />"<<endl;
  xacro_file<<"    <xacro:include filename=\"$(find project_rp)/urdf/f_link.xacro\" />"<<endl;
  xacro_file<<"    <xacro:include filename=\"$(find project_rp)/urdf/base.xacro\" />"<<endl;
  xacro_file<<"    <xacro:base name=\"l0\" />"<<endl;
  
	string xyz_link;
	string link_rpy;
	string xyz_joint;
	string joint_rpy;
	string length;
	int j_idx = 0;
	vector<bool> fixed_joint;
  for(int i=0;i<alphas.size();++i){
  	if(is_joint_revolute[i]){
  		string s_joint = "    <xacro:r_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_rpy=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\"/>";

  		if(i==0){
  		 	xyz_link = "0 0 "+to_string(ds[i]/2);
  		 	link_rpy = "0 0 0";
  		 	xyz_joint = "0 0 0";
  		 	joint_rpy = "0 0 0";
  		 	if(ds[i]==0)
					length = "0.2";
				else
					length = to_string(ds[i]);
  		}else{
  			xyz_link = "0 0 "+to_string(ds[i]/2);
  			link_rpy = "0 0 0";
  			xyz_joint = to_string(as[i-1])+" 0 "+to_string(ds[i-1]);
  			joint_rpy = to_string(alphas[i-1])+" 0 0";
  			if(ds[i]==0)
					length = "0.2";
				else
					length = to_string(ds[i]);
  		}
  		
			//link_xyz
			s_joint.insert(108,xyz_link);
			//link_rpy
			s_joint.insert(96,link_rpy);
			//joint_xyz
			s_joint.insert(84,xyz_joint);
			//rpy
			s_joint.insert(71,joint_rpy);
  		//radius
  		s_joint.insert(58,to_string(0.1*scale));
  		//length
  		s_joint.insert(48,length);
  		//parent name
  		s_joint.insert(38,to_string(j_idx));
  		//link name
  		s_joint.insert(27,to_string(j_idx+1));
  		
  		xacro_file<<s_joint<<endl;
  		++j_idx;
  		fixed_joint.push_back(0);
  	}else{
  		string s_joint = "    <xacro:p_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_rpy=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\"/>";
			if(i==0){
  		 	xyz_link = "0 0 "+to_string(ds[i]/2-1);
  		 	link_rpy = "0 0 0";
  		 	xyz_joint = "0 0 0";
  		 	joint_rpy = "0 0 0";
  		}else{
  			xyz_link = "0 0 "+to_string(ds[i]-p_limit/2);
  			link_rpy = "0 0 0";
  			xyz_joint = to_string(as[i-1])+" 0 "+to_string(ds[i-1]);
  			joint_rpy = to_string(alphas[i-1])+" 0 0";
  		}
  		
  		//link_xyz
			s_joint.insert(108,xyz_link);
			//link_rpy
			s_joint.insert(96,link_rpy);
			//joint_xyz
			s_joint.insert(84,xyz_joint);
			//rpy
			s_joint.insert(71,joint_rpy);
  		//radius
  		s_joint.insert(58,to_string(0.08*scale));
  		//length - prismatic joint has limit of p_limit, fix length to that
  		s_joint.insert(48,to_string(p_limit));
  		//parent name
  		s_joint.insert(38,to_string(j_idx));
  		//link name
  		s_joint.insert(27,to_string(j_idx+1));
  		
  		xacro_file<<s_joint<<endl;
  		++j_idx;
  		fixed_joint.push_back(0);
  	}
  	// adding fixed links for shoulders
  	if(!as[i]==0){
  		string s_joint = "    <xacro:f_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" link_rpy=\"\" link_xyz=\"\"/>";
			xyz_link = to_string(as[i]/2)+" 0 "+to_string(ds[i]);
			link_rpy = "0 "+to_string(M_PI/2)+" 0";
			length = to_string(as[i]);
			
			//link_xyz
			s_joint.insert(82,xyz_link);
			//link_rpy
			s_joint.insert(70,link_rpy);
			//radius
			s_joint.insert(58,"0.1");
			//length
			s_joint.insert(48,length);
			//parent name
			s_joint.insert(38,to_string(j_idx));
			//link name
			s_joint.insert(27,to_string(j_idx+1));
			
			xacro_file<<s_joint<<endl;
			++j_idx;
			fixed_joint.push_back(1);
  	}
  }
  
  // final link, it's a fixed link
  if(is_joint_revolute.back()){
		string s_joint = "    <xacro:f_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" link_rpy=\"\" link_xyz=\"\"/>";
		xyz_link = to_string(as.back()/2)+" 0 "+to_string(ds.back());
		link_rpy = "0 "+to_string(M_PI/2)+" 0";
		length = to_string(as.back());
		
		//link_xyz
		s_joint.insert(82,xyz_link);
		//link_rpy
		s_joint.insert(70,link_rpy);
		//radius
		s_joint.insert(58,"0.1");
		//length
		s_joint.insert(48,length);
		//parent name
		s_joint.insert(38,to_string(j_idx));
		//link name
		s_joint.insert(27,to_string(j_idx+1));
		
		xacro_file<<s_joint<<endl;
		++j_idx;
		fixed_joint.push_back(1);
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
  		for(int i=0;i<j_idx;++i){
  			if(!fixed_joint[i])
  				gazebo << "        l" << i+1 << "_controller" <<endl;
  		}
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
	for(int i=0;i<j_idx;++i){
		if(!fixed_joint[i]){
			joints_file << "l" << i+1 << "_controller:" << endl;
			joints_file << "  type: \"position_controllers/JointPositionController\"" << endl;
			joints_file << "  joint: l" << i << "_to_l" << i+1 <<endl;
		}
	}

	joints_file.close();

  
  
  return 0;
}

