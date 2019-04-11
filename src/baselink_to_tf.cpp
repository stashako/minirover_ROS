#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"


tf::TransformBroadcaster* tfB_;
tf::StampedTransform transforms_;
tf::TransformListener* tfL_;
tf::Quaternion tmp_;

//call back from camera pose
void camPoseCB(const geometry_msgs::PoseStamped::ConstPtr &cam_pose)
{
	//std::cout <<"received pos" << std::endl;
	double px = cam_pose->pose.position.x;
	double py = cam_pose->pose.position.y;
	double pz = cam_pose->pose.position.z;
	double qx = cam_pose->pose.orientation.x;
	double qy = cam_pose->pose.orientation.y;
	double qz = cam_pose->pose.orientation.z;
	double qw = cam_pose->pose.orientation.w;

	
	//prepare transform
	tf::Quaternion rotation (qx,qy,qz,qw);
	tf::Vector3 origin (px,py,pz);
	tf::Transform t (rotation,origin);

    //from zed example "Position Tracking" main.cpp
    // To get the position of the center of the camera, we transform the pose data into a new frame located at the center of the camera.
    // The generic formula used here is: Pose(new reference frame) = M.inverse() * Pose (camera frame) * M, where M is the transform between two frames.
	//TODO: find out why ???!!!! 
    tf::Transform bl_t = transforms_.inverse() * t * transforms_;
    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;
    geometry_msgs::Pose pose3;
    tf::poseTFToMsg (t, pose1);
    tf::poseTFToMsg (transforms_, pose2);
    tf::poseTFToMsg (bl_t, pose3);
	std::cout <<"pose1=" << pose1 << std::endl;
    std::cout <<"pose2=" << pose2 << std::endl;
    std::cout <<"pose3=" << pose3 << std::endl;
    tf::StampedTransform odomToBase (bl_t,cam_pose->header.stamp, "odom", "base_link");
	tfB_->sendTransform(odomToBase);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "baselink_to_tf");
    ros::NodeHandle n;
	//ros::Rate loop_rate(30);
    ros::Subscriber cam_sub = n.subscribe("camera_pose",1,camPoseCB);
    tfB_ = new tf::TransformBroadcaster();
    //wait for the transform to be published
    tfL_ = new tf::TransformListener();
    try{
    	tfL_->waitForTransform("camera_optical","base_link",ros::Time::now(),ros::Duration(3.0));
    	tfL_->lookupTransform("camera_optical","base_link",ros::Time(0),transforms_);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg (transforms_, pose);

        double roll, pitch, yaw;
        tf::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::cout << "rpy = "<< roll <<","<<pitch<<","<<yaw<< std::endl;
    	std::cout << "tf = "<< pose<< std::endl;
    } catch(tf::TransformException ex)
    {
    	std::cout << "tf err" << ex.what() << std::endl;
    	ROS_ERROR("%s",ex.what());
    }

    /*while(ros::ok())
    {

    	//std::cout << "here" << std::endl;
    	loop_rate.sleep();
    }*/
    ros::spin();
    return 0;
}
