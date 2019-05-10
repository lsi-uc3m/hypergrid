#include <hypergrid/gridmap.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>


// Map params
double map_height;
double map_width;
double cell_size;
bool DEBUG;

// Parameters for floor removal algorithm
bool floor_filter;

// Vehicle box to ignore points inside it
double vehicle_box_size;

// Maximum height of obstacles. Everything higher will out of the map
double max_height;
double heightmap_threshold;
double heightmap_cell_size;

ros::Publisher cloud_map_pub;

// PCLConverter instance to convert pointclouds to grid maps
hypergrid::PCLConverter* pcl_converter;


/* Returns true if the height difference in a given cell is more than the heightmap threshold and the point height is less than max_height */
bool is_obstacle(af::array p, const af::array min, const af::array max)
{
   /* int height_cells = map_height / heightmap_cell_size;
    int width_cells = map_width / heightmap_cell_size;

    int x = (height_cells / 2) + (p.x / heightmap_cell_size);
    int y = (width_cells / 2) + (p.y / heightmap_cell_size);

    return ((x >= 0) && (x < width_cells) && (y >= 0) && (y < height_cells)) && // Point is inside the grid
           ((p.z < max_height))                                              && // Point is inside height limit
           ((abs(p.x) > vehicle_box_size) || (abs(p.y) > vehicle_box_size))  && // Point is not inside the vehicle box
           ((max[x][y] - min[x][y]) > heightmap_threshold);                     // Point is inside a cell considered obstacle by the heightmap
           */
}


/* Removes all the points in the floor with a heightmap algorithm */
void remove_floor(const af::array cloud)
{

}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr cloud_msg)
{
    printf("\nPoint step : %d \n", cloud_msg->point_step);

    //Convert PointCloud2 to ArrayFire array

    af::array pcl_points(5, (cloud_msg->width)/5 , cloud_msg->data.data());

   // af_print(pcl_points);
    
    pcl_points = pcl_points(af::seq(3), af::span);
    
    af_print(pcl_points);
    
    
    
    
    
    
   /* if (floor_filter)
    {
        // Remove the floor points
      //  remove_floor(cloud_msg);
    }
    hypergrid::GridMap gridmap = pcl_converter->convertPointcloud(cloud_msg);
    // Check if map is empty due to missing tf
    if (!gridmap) return;

    // Publish OccupancyGrid map
    cloud_map_pub.publish(gridmap.toMapMsg());

    */

}

int main(int argc, char  **argv)
{
    af::info();

    /* code */
    ros::init(argc, argv, "pcl_to_gridmap");
    ros::NodeHandle nh, priv_nh("~");

    std::string cloud_topic;
    std::string output_topic;
    std::string map_frame_id;
    std::string frame_vehicle;
    priv_nh.param<std::string>("map_frame_id", map_frame_id, "/base_footprint");
    priv_nh.param("height", map_height, 50.0);
    priv_nh.param("width", map_width, 50.0);
    priv_nh.param("cell_size", cell_size, 0.2);
    priv_nh.param("floor_filter", floor_filter, true);
    priv_nh.param("heightmap_threshold", heightmap_threshold, 0.15);
    priv_nh.param("heightmap_cell_size", heightmap_cell_size, 0.25);
    priv_nh.param("max_height", max_height, 2.5);
    priv_nh.param("vehicle_box_size", vehicle_box_size, 1.0);
    priv_nh.param("DEBUG", DEBUG, true);

    cloud_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/pcl_to_gridmap", 2);
    ros::Subscriber cloud_sub = nh.subscribe("velodyne_points", 5, cloud_callback);

    geometry_msgs::Pose origin;
    origin.position.x = - (map_width / 2);
    origin.position.y = - (map_height / 2);
    origin.orientation.w = 1;

    pcl_converter = new hypergrid::PCLConverter(map_width, map_height, cell_size, max_height, origin, frame_vehicle + "/" + map_frame_id);


    ros::Rate r(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
