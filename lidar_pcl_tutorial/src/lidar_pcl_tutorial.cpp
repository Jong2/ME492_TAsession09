#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <algorithm>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

// Global variable
sensor_msgs::PointCloud2 msg_cloud;

void lidar_cb(sensor_msgs::LaserScan msg){

    // angle in radian
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;

    // size of range vector
    int len = range.size();
    float angle_temp;

    /// 1. LaserScan msg to PCL::PointXYZ

    // initializae pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new::pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 (new::pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new::pcl::PointCloud<pcl::PointXYZ>);

    // Create the extract object for removal of infinite ditance points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());

    cloud->is_dense = false;
    cloud->width = len;
    cloud->height = 1;
    cloud->points.resize(len);

    // fill the pointcloud
    for(int i = 0; i < len; i++){
        angle_temp = angle_min + i*angle_increment;
        if (std::isinf(range[i])==false){

            cloud->points[i].x = range[i]*cos(angle_temp);
            cloud->points[i].y = range[i]*sin(angle_temp);
            cloud->points[i].z = 0;
        }
        else{
            // indices of infinite distance points
            inf_points->indices.push_back(i);
        }
    }

    // Remove infinite distance points from cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inf_points);
    extract.setNegative(true);
    extract.filter(*cloud);

    /// 2. Apply passthrough filter to current pointcloud data
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud (cloud);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (0.0, 1.5);
    pass1.filter(*cloud_filtered_1);

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud (cloud_filtered_1);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-1.5, 1.5);
    pass2.filter(*cloud_filtered_2);

    // Coonvert PCL type to sensor_msgs/PointCloud2 type
    pcl::toROSMsg(*cloud_filtered_2, msg_cloud);

    // Free memory
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered_1.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered_2.reset(new pcl::PointCloud<pcl::PointXYZ>);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_pcl_tutorial");
    ros::NodeHandle nh;

    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        ros::spinOnce();

        msg_cloud.header.frame_id = "base_scan"; 
        msg_cloud.header.stamp = ros::Time::now();
        pub_cloud.publish(msg_cloud);

        loop_rate.sleep();
    }

    return 0;
}
