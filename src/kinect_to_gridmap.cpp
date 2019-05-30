#include <cstring>
#include <hypergrid/gridmap.hpp>
#include <hypergrid/conversions/lidar_converter.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>


// Map params
double map_height;
double map_width;
double cell_size;
bool DEBUG;


// Maximum height of obstacles. Everything higher will out of the map
double max_height;

// Laser frame to base_footprint tf transform
std::string map_frame_id;
std::string cloud_topic;
std::string output_topic;
ros::Publisher cloud_map_pub;

// Transform Listener 
tf::TransformListener* tf_listener;



void kinect_callback(sensor_msgs::PointCloud2Ptr cloud_msg)
{
    ros::Time t0 = ros::Time::now();
    ros::Time t_start = t0;
   
    tf::StampedTransform kinect_footprint_transform;
    try
    {
        tf_listener->lookupTransform(map_frame_id, cloud_msg->header.frame_id , ros::Time(0), kinect_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("Warning: %s", ex.what());
    }
    
}



int main(int argc, char  **argv)
{
    af::info();

    ros::init(argc, argv, "pcl_to_gridmap");
    ros::NodeHandle nh, priv_nh("~");

    
    priv_nh.param<std::string>("map_frame_id", map_frame_id, "base_footprint");
    priv_nh.param("height", map_height, 50.0);
    priv_nh.param("width", map_width, 50.0);
    priv_nh.param("cell_size", cell_size, 0.2);
    priv_nh.param("max_height", max_height, 2.5);
    priv_nh.param("DEBUG", DEBUG, true);

    cloud_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/kinect_to_gridmap", 2);
    ros::Subscriber cloud_sub = nh.subscribe("velodyne_points", 5, kinect_callback);

   

    tf_listener = new tf::TransformListener();
   
    ros::Rate r(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
