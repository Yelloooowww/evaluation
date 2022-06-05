#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace ros;
using namespace std;
using namespace pcl;
using namespace message_filters;

sensor_msgs::PointCloud2 lidar_filter_points;
Publisher pub_lidar_crop;
double x_max,x_min,y_max,y_min,z_max,z_min;
Publisher pub_lidar_filter_points;


void callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  PointCloud<PointXYZ>::Ptr pc (new PointCloud<PointXYZ>);
  tf::TransformListener listener;
  tf::StampedTransform tf_pose1, tf_pose2;
  Eigen::Matrix4f pose_matrix1,pose_matrix2;


  //tf
  fromROSMsg(*pc_msg, *pc);
  try{
      ros::Duration five_seconds(5.0);
      listener.waitForTransform("stick_move_link", "front_laser", ros::Time(0), five_seconds);
      listener.lookupTransform("stick_move_link", "front_laser", ros::Time(0), tf_pose1);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
  }
  pcl_ros::transformAsMatrix(tf_pose1, pose_matrix1);
  pcl::transformPointCloud(*pc, *pc, pose_matrix1);


  PointCloud<PointXYZ>::Ptr lidar_filter (new PointCloud<PointXYZ>);
  for (size_t i = 0; i < pc->points.size(); i++){
    if ((pc->points[i].x>x_min) && (pc->points[i].x<x_max) && (pc->points[i].y>y_min) && (pc->points[i].y<y_max) && (pc->points[i].z>z_min) && (pc->points[i].z<z_max));
    else lidar_filter->points.push_back(pc->points[i]);
  }

  //tf
  try{
      ros::Duration five_seconds(5.0);
      listener.waitForTransform("front_laser","stick_move_link", ros::Time(0), five_seconds);
      listener.lookupTransform("front_laser","stick_move_link", ros::Time(0), tf_pose2);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
  }
  pcl_ros::transformAsMatrix(tf_pose2, pose_matrix2);
  pcl::transformPointCloud(*lidar_filter, *lidar_filter, pose_matrix2);
  toROSMsg(*lidar_filter, lidar_filter_points);
  lidar_filter_points.header = pc_msg->header;
  pub_lidar_filter_points.publish(lidar_filter_points);
}

int main(int argc, char **argv)
{
    init(argc, argv, "lidar_crop_v2");


    NodeHandle nh("");
    param::get("~x_max", x_max);
    param::get("~y_max", y_max);
    param::get("~z_max", z_max);
    param::get("~x_min", x_min);
    param::get("~y_min", y_min);
    param::get("~z_min", z_min);

    ros::Subscriber sub = nh.subscribe("/robot/points", 10, callback);
    pub_lidar_filter_points = nh.advertise<sensor_msgs::PointCloud2>("lidar_crop", 10);

    ROS_INFO("lidar_crop_v2");
    spin();
    return 0;
}
