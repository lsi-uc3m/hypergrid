#include <cstring>
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

// Laser frame to base_footprint tf transform
std::string map_frame_id;
std::string cloud_topic;
std::string output_topic;
std::string frame_vehicle;
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

void cloud_callback(const sensor_msgs::PointCloud2Ptr cloud_msg)
{
    ros::Time t0 = ros::Time::now();
    ros::Time t_start = t0;
    if (DEBUG) std::cout << "\n------------------------\nNew pcl" << std::endl;
    
    //Convert PointCloud2 to ArrayFire array
    // af::array pcl_points(32, cloud_msg->width, cloud_msg->data.data());
    float* data_f = reinterpret_cast<float *>(cloud_msg->data.data());
    af::array pcl_points(8, cloud_msg->width, data_f);
   
    //remove Intenisty and Ring Bytes
    pcl_points = pcl_points(af::seq(3), af::span);
   
 
    if (DEBUG) std::cout << "Convert PointCloud2 to ArrayFire array time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();
    
    tf::StampedTransform pcl_footprint_transform;
    try
    {
        tf_listener->lookupTransform(frame_vehicle + "/" + map_frame_id, cloud_msg->header.frame_id , ros::Time(0), pcl_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("Warning: %s", ex.what());
    }

    geometry_msgs::Pose origin;
    origin.position.x = - (map_width / 2);
    origin.position.y = - (map_height / 2);
    origin.orientation.w = 1;   

    hypergrid::GridMap gridmap(map_width, map_height, cell_size, origin, frame_vehicle + "/" + map_frame_id);


    Eigen::Affine3d e;
    tf::transformTFToEigen(pcl_footprint_transform, e);

    double t_r_array[] = {  e(0,0), e(1,0), e(2,0), e(3,0),
                            e(0,1), e(1,1), e(2,1), e(3,1),
                            e(0,2), e(1,2), e(2,2), e(3,2),
                            e(0,3), e(1,3), e(2,3), e(3,3) };

    af::array transformation(4, 4, t_r_array);
    af::array ones = af::constant(1, 1, pcl_points.dims(1));
    pcl_points = af::join(0,pcl_points, ones);
        
        
    // Transform the obstacles to the map frame
    pcl_points = (af::matmul(transformation, pcl_points.as(f64))).as(f32);
    pcl_points = pcl_points(af::seq(3), af::span);
    std::cout << "Obstacles transform time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    if (floor_filter)
    {
        // Remove the floor points
        std::cout << "points before: " << pcl_points.dims(1) << std::endl;
        remove_floor(pcl_points);
        std::cout << "points after: " << pcl_points.dims(1) << std::endl;
    
        if (DEBUG) std::cout << "remove floor time: " << ros::Time::now() - t0 << std::endl;
        t0 = ros::Time::now();
    }
    
    af::array obs = (af::join(0,pcl_points(af::seq(2), af::span), ones(af::span, af::seq(pcl_points.dims(1) )))).as(f64) ;
    
    pcl_points = pcl_points.T();
    af::array obstacle_coords =  gridmap.cellCoordsFromLocal(obs.T());
    
    
    af::array conds = ( (!(  (af::isInf(pcl_points(af::span, 0))) || 
                            (af::isInf(pcl_points(af::span, 1))) ||
                            (af::isInf(pcl_points(af::span, 2))) ) )
                             && pcl_points(af::span, 2) < max_height 
                             && gridmap.isCellInside(obstacle_coords) ) ;
    
    
    //using the sort function only to get the indices array
    af::array out;
    af::array indices_inside;
    af::sort(out, indices_inside, conds, 0, false);    
    
    // getting only the indices of the obstacles
    indices_inside = indices_inside(af::seq(af::sum(conds).scalar<unsigned>()));
    
    af::array inside_obstacles_coords = af::lookup(obstacle_coords(af::span, af::seq(2)), indices_inside);
    std::cout << "Remove outside obstacles time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();
    //af_print(inside_obstacles_coords);    
    tf::Vector3 trans = pcl_footprint_transform.getOrigin();   
    
    // Add a free line to each obstacle inside the grid
    // Set the origin of the lines at the sensor point
    hypergrid::Cell start = gridmap.cellCoordsFromLocal(trans.getX(), trans.getY());
    gridmap.addFreeLines(start, inside_obstacles_coords);   
    
    std::cout << "Set free lines time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();
    
    // Set the obstacles in the map
    af::array indices = inside_obstacles_coords(af::span, 1).as(s32) * gridmap.grid.dims(0) + inside_obstacles_coords(af::span, 0).as(s32);
    gridmap.grid(indices) = hypergrid::GridMap::OBSTACLE;    
    
    std::cout << "Set obstacles time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    
    // Publish OccupancyGrid map
    cloud_map_pub.publish(gridmap.toMapMsg());
    if (DEBUG) std::cout << "Publish time: " << ros::Time::now() - t0 << std::endl;
    if (DEBUG) std::cout << "Callback time: " << ros::Time::now() - t_start << std::endl;

    

}

int main(int argc, char  **argv)
{
    af::info();

    /* code */
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
