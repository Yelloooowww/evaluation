#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/io/ply_io.h>

using namespace ros;
using namespace std;
using namespace pcl;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PLYReader Reader;
// Reader.read("Path of the PLY file", *cloud);


class Visualize_map
{
private:
    sensor_msgs::PointCloud2 ros_map;
    PointCloud<PointXYZ>::Ptr cloud;
    PLYReader Reader;
    Publisher pub_map;
    Timer timer;

public:
    Visualize_map(NodeHandle &nh);
    void callback(const ros::TimerEvent& event);
    ~Visualize_map();
};

Visualize_map::Visualize_map(NodeHandle &nh)
{
    ros_map.header.frame_id = "map";
    cloud.reset(new PointCloud<PointXYZ>);
    pub_map = nh.advertise<sensor_msgs::PointCloud2>("visualize_map", 1);
    Reader.read("/home/argsubt/pokingbot-rl/autonomous_exploration_development_environment/map.ply", *cloud);
    toROSMsg(*cloud, ros_map);

    timer = nh.createTimer(ros::Duration(0.1), &Visualize_map::callback, this);
    ROS_INFO("Visualize_map initialized");
}

Visualize_map::~Visualize_map()
{
}

void Visualize_map::callback(const ros::TimerEvent& event){
  // cout<<"TIMER CB"<<endl;
  ros_map.header.frame_id = "map";
  pub_map.publish(ros_map);
}
int main(int argc, char **argv)
{
    init(argc, argv, "Visualize_map");
    NodeHandle nh;
    Visualize_map vm(nh);
    spin();
    return 0;
}
