#include <hypergrid/gridmap.hpp>
#include <hypergrid/conversions/laserscan_converter.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>



double map_height;
double map_width;
double cell_size;
bool DEBUG;

ros::Publisher laser_gridmap_pub;

// Laser frame to base_footprint tf transform
std::string map_frame_id;
tf::TransformListener* tf_listener;

void laser_callback(const sensor_msgs::LaserScanConstPtr scan)
{
    ros::Time t0 = ros::Time::now();
    // Update the laser transform
    tf::StampedTransform laser_footprint_transform;
    try
    {
        tf_listener->lookupTransform(map_frame_id, scan->header.frame_id, ros::Time(0), laser_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    geometry_msgs::Pose origin;
    origin.position.x = 0;
    origin.position.y = - (map_height / 2);
    origin.orientation.w = 1;   
    hypergrid::LaserScanConverter laser_converter(map_width, map_height, cell_size, origin, map_frame_id); 
   
    hypergrid::GridMap gridmap = laser_converter.convert(scan, laser_footprint_transform);
    // Publish OccupancyGrid map
    laser_gridmap_pub.publish(gridmap.toMapMsg());
    if (DEBUG) std::cout << "Publish time: " << ros::Time::now() - t0 << std::endl;
}


int main(int argc, char **argv)
{
    af::info();

    ros::init(argc, argv, "laser_to_gridmap");
    ros::NodeHandle public_nh, private_nh("~");

    private_nh.param<std::string>("map_frame_id", map_frame_id, "base_footprint");
    private_nh.param("height", map_height, 50.0);
    private_nh.param("width", map_width, 50.0);
    private_nh.param("cell_size", cell_size, 0.2);
    private_nh.param("DEBUG", DEBUG, false);

    laser_gridmap_pub = public_nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/laser_to_gridmap", 2);
    ros::Subscriber laser_sub = public_nh.subscribe("scan", 5, laser_callback);

    tf_listener = new tf::TransformListener;

    ros::spin();

    return 0;
}
