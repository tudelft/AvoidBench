#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
 
 
int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_example");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/hyyu/AvoidBench/src/avoidbench/avoidmetrics/point_clouds_data/pointclouds0.ply", cloud) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
    sensor_msgs::PointCloud2 output;
    
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";
 
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    return 0;
}
