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

// Parameters for floor removal algorithm
bool floor_filter;

// Vehicle box to ignore points inside it
double vehicle_box_size;

// Maximum height of obstacles. Everything higher will out of the map
double max_height;
double heightmap_threshold;
double heightmap_cell_size;

// Laser frame to base_footprint tf transform
std::string map_frame_id;
std::string cloud_topic;
std::string output_topic;
ros::Publisher cloud_map_pub;

// Transform Listener 
tf::TransformListener* tf_listener;


/* Removes all the points in the floor with a heightmap algorithm */
void remove_floor(af::array& cloud)
{
    float* x = cloud(0, af::span).host<float>();
    float* y = cloud(1, af::span).host<float>();
    float* z = cloud(2, af::span).host<float>();

    int height_cells = map_height / heightmap_cell_size;
    int width_cells = map_width / heightmap_cell_size;

    af::array x_arr = (width_cells / 2) + (cloud(0, af::span) / heightmap_cell_size);
    af::array y_arr = (height_cells / 2) + (cloud(1, af::span) / heightmap_cell_size);
    af::array z_arr = cloud(2, af::span);

    float* min = new float[width_cells * height_cells];
    float* max = new float[width_cells * height_cells];

    // Resize the cloud to make it non-organized and work faster
    int width = cloud.dims(0) * cloud.dims(1);
    int height = 1;

    // Init maps
    for (int i = 0; i < width_cells; ++i)
    {
        for (int j = 0; j < height_cells; ++j)
        {
            int index = i * height_cells + j;
            min[index] = 9999.9;
            max[index] = -9999.9;
        }
    }

    // Build height map
    for (int i = 0; i < cloud.dims(1); ++i)
    {
        int x_idx = (width_cells / 2) + (x[i] / heightmap_cell_size);
        int y_idx = (height_cells / 2) + (y[i] / heightmap_cell_size);

        if (x_idx >= 0 && x_idx < width_cells && y_idx >= 0 && y_idx < height_cells)
        {
            int index = x_idx * height_cells + y_idx;
            min[index] = std::min<float>(min[index], z[i]);
            max[index] = std::max<float>(max[index], z[i]);
        }
    }

    std::vector<int> indices;
    for (int i = 0; i < width; ++i)
    {
        int x_idx = (width_cells / 2) + (x[i] / heightmap_cell_size);
        int y_idx = (height_cells / 2) + (y[i] / heightmap_cell_size);
        int index = x_idx * height_cells + y_idx;

        bool is_obstacle = ((x_idx >= 0) && (x_idx < width_cells) && (y_idx >= 0) && (y_idx < height_cells)) && // Point is inside the grid
                           ((z[i] < max_height))                                                             && // Point is inside height limit
                           ((abs(x[i]) > vehicle_box_size) || (abs(y[i]) > vehicle_box_size))                && // Point is not inside the vehicle box
                           ((max[index] - min[index]) > heightmap_threshold);                                   // Point is inside a cell considered obstacle by the heightmap
        if (is_obstacle) indices.push_back(i);
    }

    af::array indices_arr(indices.size(), indices.data());
    delete[] x;
    delete[] y;
    delete[] z;
    delete[] min;
    delete[] max;
    cloud = af::lookup(cloud, indices_arr, 1);
}

void cloud_callback(sensor_msgs::PointCloud2Ptr cloud_msg)
{
    ros::Time t0 = ros::Time::now();
    ros::Time t_start = t0;

    tf::StampedTransform pcl_footprint_transform;
    try
    {
        tf_listener->lookupTransform(map_frame_id, cloud_msg->header.frame_id , ros::Time(0), pcl_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("Warning: %s", ex.what());
    }

    geometry_msgs::Pose origin;
    origin.position.x = - (map_width / 2) + cell_size;
    origin.position.y = - (map_height / 2) + cell_size;
    origin.orientation.w = 1;

    hypergrid::LIDARConverter lidar_converter(map_width, map_height, cell_size,
                                              origin, map_frame_id,
                                              heightmap_threshold,
                                              heightmap_cell_size,
                                              max_height, vehicle_box_size,
                                              DEBUG);

    hypergrid::GridMap gridmap = lidar_converter.convert(cloud_msg, pcl_footprint_transform);

    // Publish OccupancyGrid map
    cloud_map_pub.publish(gridmap.toMapMsg());
    if (DEBUG) std::cout << "Publish time: " << ros::Time::now() - t0 << std::endl;
    if (DEBUG) std::cout << "Callback time: " << ros::Time::now() - t_start << std::endl;
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
    priv_nh.param("floor_filter", floor_filter, false);
    priv_nh.param("heightmap_threshold", heightmap_threshold, 0.15);
    priv_nh.param("heightmap_cell_size", heightmap_cell_size, 0.25);
    priv_nh.param("max_height", max_height, 2.5);
    priv_nh.param("vehicle_box_size", vehicle_box_size, 2.0);
    priv_nh.param("DEBUG", DEBUG, true);

    cloud_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/pcl_to_gridmap", 2);
    ros::Subscriber cloud_sub = nh.subscribe("velodyne_points", 5, cloud_callback);

    tf_listener = new tf::TransformListener();
   
    ros::Rate r(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
