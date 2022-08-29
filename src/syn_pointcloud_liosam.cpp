#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/point_types.h>
#include <string> 
#include <iostream> 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h> 
using namespace std;

ros::Publisher pointcloud_pub;
sensor_msgs:: Imu imu_data;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_ptr(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*laserCloudMsg, *cloud_ptr);
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_ptr, cloud_msg);
    cloud_msg.header.frame_id = "velodyne";
    cloud_msg.header.stamp = imu_data.header.stamp;
    pointcloud_pub.publish(cloud_msg);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{ 
    imu_data.header.stamp = msg->header.stamp;
}  

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "segment_pcd");
    cout << "syn_start" << endl;

    ros::NodeHandle nh_;
    pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("points_raw_syn", 10);  
    ros::Subscriber pointcloud_sub = nh_.subscribe("/forsyn_points_raw", 1, pointcloud_callback); 
    ros::Subscriber imu_sub = nh_.subscribe("/imu/filtered", 1, imu_callback); 

    ros::spin(); 
    return 0; 
}
