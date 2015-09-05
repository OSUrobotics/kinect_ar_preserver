#include <pose_file_io.h>

void print_vec(tf::Vector3 vec, string prec)
{
	cout << prec << "x: " << vec[0] << " y: " << vec[1] << " z: " << vec[2] << endl;
}

void print_quat(tf::Quaternion quat, string prec)
{
	cout << prec;
	print_vec(quat.getAxis(), "\tAxis - ");

	cout << "\tRotation Angle: " << quat.getAngle() << endl;
}

//Returns the file descriptor of the kinect calibration file
int load_transform(tf::Transform& out_transform)
{
	string kinect_pkg_location = ros::package::getPath("kinect_ar_preserver");
	kinect_pkg_location += "/kinect_config/";
	
	string kinect_config_location = kinect_pkg_location + "kinect_ar_pose.dat";
	cout << "Kinect config location: " << kinect_config_location << endl;

	//Since I can't get c++ file io to do my bidding, I'll use system calls
	int kinect_pose_config_fd = open(kinect_config_location.c_str(), O_CREAT | O_RDWR, S_IRWXU | S_IRWXG);
	if (kinect_pose_config_fd == -1){
		ROS_ERROR_STREAM("Could not open config file in kinect_transform_publisher(). Please copy one over from kinect_ar_preserver/kinect_config.");
		exit(1);

	} else {
		ROS_INFO("Kinect config found");
		float pose[7];
		short pose_byte_cnt = sizeof(float) * 7;
		short num_bytes_read = read(kinect_pose_config_fd, (void*) pose, pose_byte_cnt);
		if (num_bytes_read == pose_byte_cnt){
			out_transform = convert_floats_to_pose(pose);
			ROS_INFO_STREAM("Kinect config loaded. Read " << num_bytes_read << " bytes.");

		} else {
			perror("Could not load the kinect config file entirely.");
			return kinect_pose_config_fd;
		}
	}

	return kinect_pose_config_fd;
}

tf::Transform convert_floats_to_pose (float* pose)
{
	tf::Vector3 axis (pose[4], pose[5], pose[6]);
	tf::Vector3 pos(pose[0], pose[1], pose[2]);
	tf::Quaternion tf_quat(axis, pose[3]);

	tf::Transform current_transform = tf::Transform(tf_quat, pos);
	print_quat(tf_quat, "Quaternion on load:\n");
	return current_transform;
}

void save_transform(int kinect_pose_config_fd, tf::StampedTransform& out_transform)
{
	//kinect_config_file_out->seekp(0, std::ios::beg);
	lseek(kinect_pose_config_fd, 0, SEEK_SET);
	float pose[7]; //x, y, z; angle, x, y, z
	tf::Vector3 tf_pos = out_transform.getOrigin();
	tf::Vector3 tf_axis = out_transform.getRotation().getAxis();
	pose[0] = tf_pos[0];
	pose[1] = tf_pos[1];
	pose[2] = tf_pos[2];
	pose[3] = out_transform.getRotation().getAngle();
	pose[4] = tf_axis[0];
	pose[5] = tf_axis[1];
	pose[6] = tf_axis[2];

	//(*kinect_config_file_out).write((char*) pose, 7 * sizeof(float));
	int num_bytes_written = write(kinect_pose_config_fd, (char*) pose, 7 * sizeof(float));
	if (num_bytes_written != 28){
		ROS_ERROR("Write to kinect pose config file unsuccessful.");
	}
}
