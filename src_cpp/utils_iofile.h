#pragma once#include <fstream>

#include <iostream>

#include <vector>

#include <string>

using namespace std;

struct robot_params {
  vector < float > alphas;
  vector < float > as;
  vector < float > ds;
  vector < float > thetas;
  float scale;
  float p_limit;
  vector < bool > is_joint_revolute;
  vector < bool > fixed_joints;
};

robot_params read_dh_params(void) {
  robot_params dh_params;
  string s1, s2, s3, s4;

  ifstream input;
  input.open("DH_params.txt");
  if (!input) {
    input.open("./src/RP_robot_manipulator/src_cpp/DH_params.txt");
    if (!input) {
      cerr << "Error: could not open DH_params.txt file. \n The file needs to be in the project folder or in the src/src_cpp folder." << endl;
      return dh_params;
    }
  }

  input >> s1 >> s2;
  dh_params.scale = stof(s2);
  input >> s1 >> s2;
  dh_params.p_limit = stof(s2);
  //ignore line
  input >> s1;
  char l[256];
  input.getline(l, 256);

  input >> s1 >> s2 >> s3 >> s4;
  while (!input.eof()) {
    // check alpha_i for +-pi/2
    if (s1 == "pi/2")
      dh_params.alphas.push_back(M_PI / 2);
    else if (s1 == "-pi/2")
      dh_params.alphas.push_back(-M_PI / 2);
    else if (s1 == "-pi")
      dh_params.alphas.push_back(-M_PI);
    else if (s1 == "pi")
      dh_params.alphas.push_back(M_PI);
    else
      dh_params.alphas.push_back(stof(s1));

    dh_params.fixed_joints.push_back(0);
    dh_params.as.push_back(stof(s2));
    if (!dh_params.as.back() == 0)
      dh_params.fixed_joints.push_back(1);
    // check d_i for qi  	
    if (s3[0] == 'q') {
      // initialize q values as 0 for prismatic joint
      dh_params.ds.push_back(0);
      dh_params.is_joint_revolute.push_back(0);
    } else {
      dh_params.ds.push_back(stof(s3));
    }

    //check theta_i for qi
    if (s4[0] == 'q') {
      // initialize q values as 0 for revolute joints
      dh_params.thetas.push_back(0);
      dh_params.is_joint_revolute.push_back(1);
    } else {
      dh_params.thetas.push_back(stof(s4));
    }

    input >> s1 >> s2 >> s3 >> s4;
  }
  if (dh_params.is_joint_revolute.back())
    dh_params.fixed_joints.push_back(1);

  input.close();
  return dh_params;
}

void make_main_xacro(vector < bool > & is_joint_revolute, vector < float > & alphas, vector < float > & as, vector < float > & ds, vector < float > & thetas, float scale, float p_limit) {
  ofstream xacro_file;
  xacro_file.open("./src/RP_robot_manipulator/urdf/main.xacro");

  // head of main.xacro
  xacro_file << "<?xml version=\"1.0\"?>" << endl;
  xacro_file << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"my_robot\">" << endl;
  xacro_file << "    <xacro:include filename=\"$(find project_rp)/urdf/r_link.xacro\" />" << endl;
  xacro_file << "    <xacro:include filename=\"$(find project_rp)/urdf/p_link.xacro\" />" << endl;
  xacro_file << "    <xacro:include filename=\"$(find project_rp)/urdf/f_link.xacro\" />" << endl;
  xacro_file << "    <xacro:include filename=\"$(find project_rp)/urdf/base.xacro\" />" << endl;
  xacro_file << "    <xacro:include filename=\"$(find project_rp)/urdf/claw.xacro\" />" << endl;
  xacro_file << "    <xacro:base name=\"l0\" />" << endl;

  //adding links
  string xyz_link;
  string link_rpy;
  string xyz_joint;
  string joint_rpy;
  string length;
  string s_joint;
  string s_scale;
  int j_idx = 0;
  for (int i = 0; i < alphas.size(); ++i) {
    if (is_joint_revolute[i]) {
      //revolute joints use r_link macro
      s_joint = "    <xacro:r_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_rpy=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\"/>";

      if (i == 0) {
        xyz_link = "0 0 " + to_string(ds[i] / 2);
        link_rpy = "0 0 0";
        xyz_joint = "0 0 0";
        joint_rpy = "0 0 0";
        if (ds[i] == 0)
          length = "0.2";
        else
          length = to_string(ds[i]);
      } else {
        xyz_link = "0 0 " + to_string(ds[i] / 2);
        link_rpy = "0 0 0";
        xyz_joint = to_string(as[i - 1]) + " 0 " + to_string(ds[i - 1]);
        joint_rpy = to_string(alphas[i - 1]) + " 0 0";
        if (ds[i] == 0)
          length = "0.2";
        else
          length = to_string(ds[i]);
      }

      //link_xyz
      s_joint.insert(108, xyz_link);
      //link_rpy
      s_joint.insert(96, link_rpy);
      //joint_xyz
      s_joint.insert(84, xyz_joint);
      //rpy
      s_joint.insert(71, joint_rpy);
      //radius
      s_joint.insert(58, to_string(0.1 * scale));
      //length
      s_joint.insert(48, length);
      //parent name
      s_joint.insert(38, to_string(j_idx));
      //link name
      s_joint.insert(27, to_string(j_idx + 1));

      // write link to file
      xacro_file << s_joint << endl;
      ++j_idx;
    } else {
      //prismatic joints use p_link macro
      s_joint = "    <xacro:p_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_rpy=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\"/>";
      if (i == 0) {
        xyz_link = "0 0 " + to_string(ds[i] / 2 - 1);
        link_rpy = "0 0 0";
        xyz_joint = "0 0 0";
        joint_rpy = "0 0 0";
      } else {
        xyz_link = "0 0 " + to_string(ds[i] - p_limit / 2);
        link_rpy = "0 0 0";
        xyz_joint = to_string(as[i - 1]) + " 0 " + to_string(ds[i - 1]);
        joint_rpy = to_string(alphas[i - 1]) + " 0 0";
      }

      //link_xyz
      s_joint.insert(108, xyz_link);
      //link_rpy
      s_joint.insert(96, link_rpy);
      //joint_xyz
      s_joint.insert(84, xyz_joint);
      //rpy
      s_joint.insert(71, joint_rpy);
      //radius
      s_joint.insert(58, to_string(0.08 * scale));
      //length - prismatic joint has limit of p_limit, fix length to that
      s_joint.insert(48, to_string(p_limit));
      //parent name
      s_joint.insert(38, to_string(j_idx));
      //link name
      s_joint.insert(27, to_string(j_idx + 1));

      // write link to file
      xacro_file << s_joint << endl;
      ++j_idx;
    }
    // adding fixed links for shoulders, otherwise when you have a shoulder the link floats. Just visual
    if (!as[i] == 0) {
      // fixed links use f_link macro
      s_joint = "    <xacro:f_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\"/>";
      xyz_link = to_string(as[i] / 2) + " 0 " + to_string(ds[i]);
      xyz_joint = "0 0 0";
      link_rpy = "0 " + to_string(M_PI / 2) + " 0";
      length = to_string(as[i]);

      //link_xyz
      s_joint.insert(95, xyz_link);
      //link_rpy
      s_joint.insert(83, link_rpy);
      //joint_xyz
      s_joint.insert(71, xyz_joint);
      //radius
      s_joint.insert(58, to_string(0.1 * scale));
      //length
      s_joint.insert(48, length);
      //parent name
      s_joint.insert(38, to_string(j_idx));
      //link name
      s_joint.insert(27, to_string(j_idx + 1));

      // write link to file
      xacro_file << s_joint << endl;
      ++j_idx;
    }
  }

  // final link is a fixed link, it's needed if joint is revolute, if it's prismatic the end effector is on the link's end
  if (is_joint_revolute.back() && as.back() == 0) {
    s_joint = "    <xacro:f_link prefix=\"l\" parent=\"l\" length=\"\" radius=\"\" joint_xyz=\"\" link_rpy=\"\" link_xyz=\"\"/>";
    xyz_joint = to_string(as.back()) + " 0 " + to_string(ds.back());
    xyz_link = to_string(-as.back() / 2) + " 0 0";
    if (as.back() == 0)
      link_rpy = "0 " + to_string(M_PI / 2) + " 0";
    else
      link_rpy = "0 0 0";
    length = to_string(as.back());

    //link_xyz
    s_joint.insert(95, xyz_link);
    //link_rpy
    s_joint.insert(83, link_rpy);
    //joint_xyz
    s_joint.insert(71, xyz_joint);
    //radius
    s_joint.insert(58, to_string(0.1 * scale));
    //length
    s_joint.insert(48, length);
    //parent name
    s_joint.insert(38, to_string(j_idx));
    //link name
    s_joint.insert(27, to_string(j_idx + 1));

    // write link to file
    xacro_file << s_joint << endl;
    ++j_idx;
  }
  // add claw
  s_joint = "    <xacro:claw parent=\"l\" rpy=\"\" xyz=\"\" scale=\"\"/>";
  float x_offset = 0.642; //model is not centered in 0,0,0
  float scale_default = 0.015; //when manipulator scale is 1 this is the right scale for the claw
  s_scale = to_string(scale_default * scale) + " " + to_string(scale_default * scale) + " " + to_string(scale_default * scale);
  if (as.back() == 0 || !is_joint_revolute.back()) {
    link_rpy = "0 0 0";
    xyz_link = to_string(x_offset * scale) + " 0 0";
  } else {
    link_rpy = "0 " + to_string(M_PI / 2) + " 0";
    xyz_link = to_string(as.back()) + " 0 " + to_string(-x_offset * scale);

    //link_rpy = "0 "+to_string(M_PI/2)+" 0";
    //xyz_link = "0 0 0";//to_string(x_offset*scale)+" 0 "+to_string(-as.back());
  }
  //scale
  s_joint.insert(48, s_scale);
  //link_xyz
  s_joint.insert(39, xyz_link);
  //link_rpy
  s_joint.insert(32, link_rpy);
  //parent
  s_joint.insert(25, to_string(j_idx));

  // write claw to file
  xacro_file << s_joint << endl;
  ++j_idx;

  // tail of main.xacro
  xacro_file << "    <gazebo>" << endl;
  xacro_file << "        <plugin name=\"gazebo_ros_control\" filename=\"libgazebo_ros_control.so\">" << endl;
  xacro_file << "            <robotNamespace>/</robotNamespace>" << endl;
  xacro_file << "        </plugin>" << endl;
  xacro_file << "    </gazebo>" << endl;
  xacro_file << "</robot>" << endl;

  xacro_file.close();
}

void add_gazebo_controllers(vector < bool > & fixed_joints) {
  char l[256];
  int n_joints = fixed_joints.size();
  //template gazebo file. Used since the file is long
  ifstream g_template;
  g_template.open("./src/RP_robot_manipulator/launch/gazebo_template.txt");
  ofstream gazebo;
  gazebo.open("./src/RP_robot_manipulator/launch/gazebo.launch");
  int line = 1;
  while (!g_template.eof()) {
    if (line == 43) {
      for (int i = 0; i < n_joints; ++i) {
        if (!fixed_joints[i])
          gazebo << "        l" << i + 1 << "_controller" << endl;
      }
    }
    g_template.getline(l, 256);
    gazebo << l << endl;
    ++line;
  }
  g_template.close();
  gazebo.close();
}

void add_yaml_joints(vector < bool > & fixed_joints) {
  int n_joints = fixed_joints.size();
  ofstream joints_file;
  joints_file.open("./src/RP_robot_manipulator/config/joints.yaml");
  joints_file << "joint_state_controller:" << endl;
  joints_file << "  type: \"joint_state_controller/JointStateController\"" << endl;
  joints_file << "  publish_rate: 50" << endl;
  for (int i = 0; i < n_joints; ++i) {
    if (!fixed_joints[i]) {
      joints_file << "l" << i + 1 << "_controller:" << endl;
      joints_file << "  type: \"position_controllers/JointPositionController\"" << endl;
      joints_file << "  joint: l" << i << "_to_l" << i + 1 << endl;
    }
  }

  joints_file.close();
}
