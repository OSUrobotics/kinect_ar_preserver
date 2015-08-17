#ifndef POSE_FILE_IO_H
#define POSE_FILE_IO_H

#include "ros/ros.h"
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;

void print_vec(tf::Vector3 vec, string prec);
void print_quat(tf::Quaternion quat, string prec);

int load_transform(tf::Transform& out_transform);
tf::Transform convert_floats_to_pose (float* pose);
void save_transform(int kinect_pose_config_fd, tf::StampedTransform& out_transform);

#endif
