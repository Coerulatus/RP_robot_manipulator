#include <iostream>

#include <fstream>

#include <vector>

#include <string>

#include "ros/ros.h"

#include "std_msgs/Float64.h"

#include "utils_iofile.h"

using namespace std;

int main(int argc, char ** argv) {

  ros::init(argc, argv, "set_joints_node");

  ros::NodeHandle n;

  // read DH_params.txt
  robot_params dh_params(read_dh_params());

  vector < int > moving_joint_idxs;
  for (int i = 0; i < dh_params.fixed_joints.size(); ++i) {
    if (!dh_params.fixed_joints[i])
      moving_joint_idxs.push_back(i + 1);
  }
  int n_moving_joints = moving_joint_idxs.size();

  vector < ros::Publisher > joints_pubs;
  vector < string > j_values;
  string topic;
  for (int i = 0; i < n_moving_joints; ++i) {
    topic = "l" + to_string(moving_joint_idxs[i]) + "_controller/command";
    joints_pubs.push_back(n.advertise < std_msgs::Float64 > (topic, 100));
    j_values.push_back("0");
  }
  cout << "Waiting for the values for the joints. \nUse s to keep the joint still.\nType q to exit." << endl;
  int j_idx = 0;
  string j_v;
  std_msgs::Float64 msg;
  while (ros::ok()) {
    cin >> j_v;
    if (j_v == "q")
      break;
    else if (j_v == "s") {
      cout << "Keeping joint " << (moving_joint_idxs[j_idx % n_moving_joints]) << " still" << endl;
      ++j_idx;
    } else {
      try {
        msg.data = stof(j_v);
        joints_pubs[j_idx % n_moving_joints].publish(msg);
        cout << "Setting position of joint " << (moving_joint_idxs[j_idx % n_moving_joints]) << " to: " << j_v << endl;
        ++j_idx;
      } catch (...) {
        cout << "Invalid argument" << endl;
      }
    }
  }
  return 0;
}
