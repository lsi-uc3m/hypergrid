#include <hypergrid/gridmap.hpp>

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
    if (DEBUG) std::cout << "Transform time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();
    geometry_msgs::Pose origin;
    origin.position.x = 0.0;
    origin.position.y = - (map_height /2);

    hypergrid::GridMap gridmap(map_width, map_height, cell_size, origin, map_frame_id);

    if (DEBUG) std::cout << "\n------------------------\nNew laser" << std::endl;

    af::array ranges(scan->ranges.size(), 1, scan->ranges.data());
    af::array scan_angle(scan->ranges.size(), 1);

    af::array ranges_seq = af::seq(0, scan->ranges.size() - 1);
    scan_angle = scan->angle_min + ranges_seq * scan->angle_increment;

    // Remove NaNs and Inf values
    af::array ranges_nan_inf = af::isNaN(ranges) || af::isInf(ranges);
    af::replace(ranges, !ranges_nan_inf, scan->range_max);

    if (DEBUG) std::cout << "Init time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    af::array obstacles = af::constant(1.0, scan->ranges.size(), 3, f64);

    obstacles(af::span, 0) = ranges * af::cos(scan_angle);
    obstacles(af::span, 1) = ranges * af::sin(scan_angle);

    // Get Rotation and translation from TF transform
    tf::Vector3 trans = laser_footprint_transform.getOrigin();
    double theta = tf::getYaw(laser_footprint_transform.getRotation());
    double t_r_array[] = {std::cos(theta),  std::sin(theta), 0,
                          -std::sin(theta), std::cos(theta), 0,
                          trans.getX(),     trans.getY(),    1};
    af::array transformation(3, 3, t_r_array);
    // Transform the obstacles to the map frame
    obstacles = af::matmul(transformation, obstacles.T()).T();

    if (DEBUG) std::cout << "Obstacles transform time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();


    // Add the free lines to all obstacles
    // Cannot use gfor because of branching inside addFreeLine method and isCellInside check
    // TODO: Find a way to parallelize this
    // gfor(af::seq i, scan->ranges.size())
    for (int i = 0; i < scan->ranges.size(); ++i)
    {
        hypergrid::Cell obs_coords = gridmap.cellCoordsFromLocal(obstacles(i, 0).scalar<double>(),
                                                                 obstacles(i, 1).scalar<double>());
        // TODO: Draw the free line from the sensor point, and not from the vehicle center
        // Check if the obstacle coords lay inside the map before modifying the cell
        if (gridmap.isCellInside(obs_coords)) gridmap.addFreeLine(obstacles(i, 0).scalar<double>(),
                                                                  obstacles(i, 1).scalar<double>());
    }

    if (DEBUG) std::cout << "Set lines time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    // Set the obstacles in the map
    // Cannot use gfor because of isCellInside check
    // TODO: Find a way to parallelize this
    // gfor(af::seq i, scan->ranges.size())
    for (int i = 0; i < scan->ranges.size(); ++i)
    {
        hypergrid::Cell obs_coords = gridmap.cellCoordsFromLocal(obstacles(i, 0).scalar<double>(),
                                                                 obstacles(i, 1).scalar<double>());
        if (gridmap.isCellInside(obs_coords))
        {
            gridmap[obs_coords] = hypergrid::GridMap::OBSTACLE;
        }
    }

    if (DEBUG) std::cout << "Set obstacles time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();
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
