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

class MapFilter
{
private:
    PassThrough<PointXYZ> passX;
    PassThrough<PointXYZ> passY;
    PassThrough<PointXYZ> passZ;
    VoxelGrid<PointXYZ> voxel;
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    tf::TransformListener listener;
    tf::StampedTransform tf_pose;
    Eigen::Matrix4f pose_matrix;
    Eigen::Matrix4f camera_init_to_map;
    float x,y;
    float x_range = 10;
    float y_range = 10;
    float z_range = 20;
    float leaf_size = 0.4;
    bool flag = 0;
    sensor_msgs::PointCloud2 ros_map, ros_point, ros_map_all_part;
    PointCloud<PointXYZ>::Ptr pc, map, map_all_part;

    Publisher pub_map, pub_point, pub_map_all_part;
    Subscriber sub_map, joy_sub_, sub_pose;

public:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void pc_cb(const sensor_msgs::PointCloud2 msg);
    void pose_cb(const geometry_msgs::PoseStamped pose);
    MapFilter(NodeHandle &nh);
    ~MapFilter();
};

MapFilter::MapFilter(NodeHandle &nh)
{
    param::get("~x_range", x_range);
    param::get("~y_range", y_range);
    param::get("~z_range", z_range);
    param::get("~voxel_size", leaf_size);

    ros_map.header.frame_id = "map";
    ros_point.header.frame_id = "map";
    ros_map_all_part.header.frame_id = "map";
    pc.reset(new PointCloud<PointXYZ>);
    map.reset(new PointCloud<PointXYZ>);
    passX.setFilterFieldName("x");
    passY.setFilterFieldName("y");
    passZ.setFilterFieldName("z");

    passZ.setFilterLimits(0.3,0.6);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    icp.setMaxCorrespondenceDistance(1);
    icp.setTransformationEpsilon(1e-12);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setMaximumIterations(100);


    pub_map = nh.advertise<sensor_msgs::PointCloud2>("map_all", 1);
    pub_point = nh.advertise<sensor_msgs::PointCloud2>("map_part", 1);
    pub_map_all_part = nh.advertise<sensor_msgs::PointCloud2>("map_all_part", 1);
    sub_map = nh.subscribe("points", 1, &MapFilter::pc_cb, this);
    joy_sub_ = nh.subscribe("joy_teleop/joy", 10, &MapFilter::joyCallback, this);
    // sub_pose = nh.subscribe("/truth_map_posestamped", 10, &MapFilter::pose_cb, this);
    ROS_INFO("map filter initialized");
}

MapFilter::~MapFilter()
{
}


void MapFilter::pose_cb(const geometry_msgs::PoseStamped pose)
{

  x = pose.pose.position.x;
  y = pose.pose.position.y;
  // cout<<"cb pose"<<x<<","<<y<<endl;
}

void MapFilter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->buttons[0]==1){
    cout<<" start add map"<< endl;
    flag = 1;
  }
  if (joy->buttons[1]==1){
    cout<<" don't add map"<< endl;
    flag = 0 ;
  }
}

void MapFilter::pc_cb(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, *pc);

    try
    {
        ros::Duration five_seconds(5);
        listener.waitForTransform("map", "front_laser", ros::Time(0), five_seconds);
        listener.lookupTransform("map", "front_laser", ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pcl_ros::transformAsMatrix(tf_pose, pose_matrix);
    pcl::transformPointCloud(*pc, *pc, pose_matrix);
    // cout << pc->size() << endl;


    // if(map->size()>0){
    //     icp.setInputSource(pc);
    //     icp.setInputTarget(map);
    //     icp.align(*pc);
    //
    //     cout << "ICP:------------" << endl;
    //     cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    // }
    //
    voxel.setInputCloud(pc);
    // voxel.filter(*pc);
    passZ.setInputCloud(pc);
    passZ.filter(*pc);
    if(flag) *map += *pc;
    pcl::io::savePLYFileBinary("/home/argsubt/pokingbot-rl/forest_env.ply", *map);

    ROS_INFO("filtered cloud numbers %d", map->size());

    toROSMsg(*map, ros_map);
    ros_map.header.frame_id = "map";
    ros_map.header.stamp = msg.header.stamp;
    pub_map.publish(ros_map);
    toROSMsg(*pc, ros_point);
    ros_point.header.frame_id = "map";
    ros_point.header.stamp = msg.header.stamp;
    pub_point.publish(ros_point);
}

int main(int argc, char **argv)
{
    init(argc, argv, "mapfilter");
    NodeHandle nh;

    MapFilter mapfilter(nh);

    spin();

    return 0;
}
