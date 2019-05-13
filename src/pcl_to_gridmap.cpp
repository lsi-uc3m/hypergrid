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
    int height_cells = map_height / heightmap_cell_size;
    int width_cells = map_width / heightmap_cell_size;

    af::array min = af::constant(9999.9, width_cells, height_cells);
    af::array max = af::constant(-9999.9, width_cells, height_cells);

    af::array x_arr = (width_cells / 2) + (cloud(af::seq(0,4),af::span) / heightmap_cell_size);
    af::array y_arr = (height_cells / 2) + (cloud(af::seq(4,8),af::span) / heightmap_cell_size);

    gfor(af::seq i, cloud.dims(1))
    {
        int x = x_arr(i).as(s32).scalar<int>(); 
        int y = y_arr(i).as(s32).scalar<int>();
        af::array cond = (x >= 0 && x < width_cells && y >= 0 && y < height_cells);
        min(i) = (!cond).as(f32) * min(i) + cond.as(f32) * af::min(min(i), cloud(af::seq(8,12),i));
        max(i) = (!cond).as(f32) * max(i) + cond.as(f32) * af::max(max(i), cloud(af::seq(8,12),i));
    }


}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr cloud_msg)
{
    ros::Time t0 = ros::Time::now();
    
    //Convert PointCloud2 to ArrayFire array
    af::array pcl_points(32, cloud_msg->width, cloud_msg->data.data());
    
    //remove Intenisty and Ring Bytes
    pcl_points = pcl_points(af::seq(12), af::span);

    //Putting the X,Y,Z bytes into bytes array 
    uint8_t* x_y_z_bytes = af::flat(pcl_points).host<uint8_t>();
    
    //convertting from bytes array to float array 
    float f[3*cloud_msg->width] ;
    std::memcpy(f, x_y_z_bytes, 12*(cloud_msg->width));
   
    //putting it to ArrayFire
    af::array pcl_points2 (3, cloud_msg->width, f); 
    
    if (DEBUG) std::cout << "Convert PointCloud2 to ArrayFire array time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();
    // af_print(pcl_points2.T());
    
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
