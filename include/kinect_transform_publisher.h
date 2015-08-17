#ifndef KINECT_TRANSFORM_PUBLISHER_H
#define KINECT_TRANSFORM_PUBLISHER_H
#include "ros/ros.h"
#include <ros/package.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/registration/icp.h>
//#include <plane_reps_and_3dmath.h>
//#include <cluster_segmentation.h>
//#include <pose_transform.h>

#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>

#include <cmath>
#include <pcl_ros/point_cloud.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;
using std::fstream;
using std::ofstream;
using std::ifstream;

//extern const string kinect_frame;
extern const string world_frame;
extern tf::TransformBroadcaster* kinect_broadcaster;

void print_vec(tf::Vector3 vec, string prec = string(""));
void print_quat(tf::Quaternion quat, string prec = string(""));
void print_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr color_pass_through_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud, int rmin, int rmax, int gmin, int gmax, int bmin, int bmax);

int return_idx_largest_cluster(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector);
void remove_NANs(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector);

class Kinect_calibrator {
public:
	Kinect_calibrator();
	~Kinect_calibrator();
	
	void externally_calibrate_kinect();
	pcl::PointCloud<pcl::PointXYZ>::Ptr mk_reference_points();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud);
	void filter_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& out_clusters);
	tf::Transform calculate_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud);
	void kinect_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	tf::Transform get_transform();
	bool get_has_new_transform();

	void calibration_request_cb(const std_msgs::String::ConstPtr msg);
private:
	void save_transform();
	void load_transform();
	void convert_floats_to_pose (float* pose);

	ros::NodeHandle n;
	ros::Subscriber kinect_cloud_subscriber;
	ros::Subscriber calibration_request_sub;
	tf::Transform current_transform;
	sensor_msgs::PointCloud2::ConstPtr current_kinect_cloud;
	bool has_new_transform;
	//ofstream* kinect_config_file_out;
	//ifstream* kinect_config_file_in;
	int kinect_pose_config_fd;
	string kinect_config_location;
};

//void publish_kinect_transform(Kinect_calibrator* cally);

//Blame Forrest for mistakes found here
//int return_idx_largest_cluster(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector);
//void remove_NANs(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector);

#endif
