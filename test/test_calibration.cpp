#include "gtest/gtest.h"
#include "kinect_transform_publisher.h"

bool vecs_are_equal(tf::Vector3 v1, tf::Vector3 v2, double tolerance=0.01);

TEST(alignment, basic_translation){
	Kinect_calibrator cally;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud = cally.mk_reference_points();
	pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud = cally.mk_reference_points();

	for (unsigned int i = 0; i < translated_cloud->size(); ++i){
		(*translated_cloud)[i].x += 0.05;
	}

	cout << "Ref cloud: " << endl; print_cloud(ref_cloud);
	cout << "Translated cloud: " << endl; print_cloud(translated_cloud);
	tf::Transform final_trans = cally.calculate_transform(translated_cloud);

	EXPECT_EQ(true, vecs_are_equal(final_trans.getOrigin(), tf::Vector3(-0.05, 0, 0)));

	EXPECT_GT(0.001, fabs(final_trans.getRotation().getAngle()));
	//cout << "Final rotation: " << final_trans.getRotation() << endl;
	print_quat(final_trans.getRotation(), "Quaternion Test Result\n");
}

TEST(alignment, out_of_order){
	Kinect_calibrator cally;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud = cally.mk_reference_points();
	pcl::PointCloud<pcl::PointXYZ>::Ptr rearranged_cloud = cally.mk_reference_points();

	pcl::PointXYZ temp = (*rearranged_cloud)[0];
	(*rearranged_cloud)[0] = (*rearranged_cloud)[2];
	(*rearranged_cloud)[2] = temp;

	tf::Transform final_trans = cally.calculate_transform(rearranged_cloud);
	//EXPECT_EQ(true, vecs_are_equal(final_trans.getOrigin(), tf::Vector3(0,0,0)));
	//vecs_are_equal does not handle the zero vector... (11/9/2014)

	EXPECT_GT(0.001, fabs(final_trans.getRotation().getAngle()));
	print_quat(final_trans.getRotation(), "Quaternion Test Result\n");

}

TEST(alignment, simple_y_flip){
	Kinect_calibrator cally;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud = cally.mk_reference_points();
	pcl::PointCloud<pcl::PointXYZ>::Ptr flipped_cloud = cally.mk_reference_points();
	
	for (unsigned int i = 0; i < flipped_cloud->size(); ++i){
		(*flipped_cloud)[i].y = -(*flipped_cloud)[i].y;
		(*flipped_cloud)[i].x = -(*flipped_cloud)[i].x;
	}

	tf::Transform final_trans = cally.calculate_transform(flipped_cloud);

	EXPECT_LT(0.001, fabs(final_trans.getRotation().getAngle() - M_PI));

	print_quat(final_trans.getRotation(), "Quaternion Test Result\n");
}

TEST(home_grown_alignment, basic_translation){
	pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud ( new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ pt = init_pt(0.05, 0, 0);
	trans_cloud->push_back(pt);
	pt = init_pt(0.05, 0.1, 0);
	trans_cloud->push_back(pt);
	pt = init_pt(0.1, 0, 0);
	trans_cloud->push_back(pt);

	tf::Transform final_trans = calculate_transform_jc(trans_cloud);

	EXPECT_EQ(true, vecs_are_equal(final_trans.getOrigin(), tf::Vector3(0.05, 0, 0)));
	print_quat(final_trans.getRotation(), "Quaternion Test Result\n");
}

TEST(home_grown_alignment, simple_y_flip){
	pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud ( new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ pt = init_pt(0, 0, 0);
	trans_cloud->push_back(pt);
	pt = init_pt(0, 0.1, 0);
	trans_cloud->push_back(pt);
	pt = init_pt(-0.1, 0, 0);
	trans_cloud->push_back(pt);

	tf::Transform final_trans = calculate_transform_jc(trans_cloud);

	EXPECT_GT(0.001, fabs(final_trans.getRotation().getAngle() - M_PI));
	print_quat(final_trans.getRotation(), "Quaternion Test Result\n");
}

bool vecs_are_equal(tf::Vector3 v1, tf::Vector3 v2, double tolerance)
{
	v1.normalize();
	v2.normalize();

	if (fabs(v1[0] - v2[0]) < tolerance &&
		fabs(v1[1] - v2[1]) < tolerance &&
		fabs(v1[2] - v2[2]) < tolerance){
			return true;
		}

	return false;
}

bool quats_are_equal(tf::Quaternion q1, tf::Quaternion q2, double tolerance)
{
	q1.normalize();
	q2.normalize();

}

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_calibration_tests");
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
