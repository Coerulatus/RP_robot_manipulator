#pragma once#include <Eigen/Core>

#include <Eigen/StdVector>

#include <vector>

#include <cmath>

using namespace std;
using namespace Eigen;

Matrix4f get_R(float alpha, float a, float d, float theta) {
  Matrix4f R;
  R(0, 0) = cos(theta);
  R(0, 1) = -cos(alpha) * sin(theta);
  R(0, 2) = sin(alpha) * sin(theta);
  R(0, 3) = a * cos(theta);
  R(1, 0) = sin(theta);
  R(1, 1) = cos(alpha) * cos(theta);
  R(1, 2) = -sin(alpha) * cos(theta);
  R(1, 3) = a * sin(theta);
  R(2, 0) = 0;
  R(2, 1) = sin(alpha);
  R(2, 2) = cos(alpha);
  R(2, 3) = d;
  R(3, 0) = 0;
  R(3, 1) = 0;
  R(3, 2) = 0;
  R(3, 3) = 1;
  return R;
}

Matrix4f get_dR_dtheta(float alpha, float a, float d, float theta) {
  Matrix4f R;
  R(0, 0) = -sin(theta);
  R(0, 1) = -cos(alpha) * cos(theta);
  R(0, 2) = sin(alpha) * cos(theta);
  R(0, 3) = -a * sin(theta);
  R(1, 0) = cos(theta);
  R(1, 1) = -cos(alpha) * sin(theta);
  R(1, 2) = sin(alpha) * sin(theta);
  R(1, 3) = a * cos(theta);
  R(2, 0) = 0;
  R(2, 1) = 0;
  R(2, 2) = 0;
  R(2, 3) = 0;
  R(3, 0) = 0;
  R(3, 1) = 0;
  R(3, 2) = 0;
  R(3, 3) = 0;
  return R;
}

Matrix4f get_dR_dd(float alpha, float a, float d, float theta) {
  Matrix4f R;
  R(0, 0) = 0;
  R(0, 1) = 0;
  R(0, 2) = 0;
  R(0, 3) = 0;
  R(1, 0) = 0;
  R(1, 1) = 0;
  R(1, 2) = 0;
  R(1, 3) = 0;
  R(2, 0) = 0;
  R(2, 1) = 0;
  R(2, 2) = 0;
  R(2, 3) = 1;
  R(3, 0) = 0;
  R(3, 1) = 0;
  R(3, 2) = 0;
  R(3, 3) = 0;
  return R;
}

MatrixXf jacobian(vector < bool > & is_joint_revolute, vector < float > & alphas, vector < float > & as, vector < float > & ds, vector < float > & thetas) {
  int n_joints = is_joint_revolute.size();
  MatrixXf J(3, n_joints);
  vector < Matrix4f, aligned_allocator < Matrix4f > > Rs;

  for (int i = 0; i < n_joints; i++) {
    Rs.push_back(get_R(alphas[i], as[i], ds[i], thetas[i]));
  }

  Matrix4f T;
  T.setIdentity();
  for (int i = 0; i < n_joints; ++i) {
    for (int j = 0; j < n_joints; ++j) {
      if (i == j) {
        if (is_joint_revolute[i])
          T = T * get_dR_dtheta(alphas[i], as[i], ds[i], thetas[i]);
        else
          T = T * get_dR_dd(alphas[i], as[i], ds[i], thetas[i]);
      } else
        T = T * Rs[j];
    }
    J.block(0, i, 3, 1) = T.block(0, 3, 3, 1);
    T.setIdentity();
  }
  return J;
}

Vector3f direct_kinematics(vector < float > & alphas, vector < float > & as, vector < float > & ds, vector < float > & thetas) {
  Matrix4f T;
  T.setIdentity();
  int n_joints = alphas.size();
  for (int i = 0; i < n_joints; i++) {
    T *= get_R(alphas[i], as[i], ds[i], thetas[i]);
  }
  return T.block(0, 3, 3, 1);
}

float euclidean_distance(Vector3f & x1, Vector3f & x2) {
  return sqrt((x1 - x2).transpose() * (x1 - x2));
}
