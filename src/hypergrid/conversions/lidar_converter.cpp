#include <hypergrid/conversions/lidar_converter.hpp>

namespace hypergrid
{

/* Constructor */
LIDARConverter::LIDARConverter(double width, double height, double cell_size,
                               geometry_msgs::Pose origin, std::string map_frame_id,
                               double heightmap_threshold, double heightmap_cell_size,
                               double max_height, double vehicle_box_size, bool DEBUG) :
    origin_(origin)
{
    width_ = width;
    height_ = height;
    cell_size_ = cell_size;
    map_frame_id_ = map_frame_id;
    heightmap_threshold_ = heightmap_threshold;
    heightmap_cell_size_ = heightmap_cell_size;
    max_height_ = max_height;
    vehicle_box_size_ = vehicle_box_size;
    DEBUG_ = DEBUG;

}

/* Convert a sensor_msgs::PointCloud2 to a hypergrid::GridMap */
GridMap LIDARConverter::convert(sensor_msgs::PointCloud2& cloud_msg, const tf::StampedTransform transform) const
{
    auto start = std::chrono::high_resolution_clock::now(); 
    
  
    // unsync the I/O of C and C++. 
    std::ios_base::sync_with_stdio(false); 
  
  

    

    // ros::Time t0 = ros::Time::now();
    // ros::Time t_start = t0;
    if (DEBUG_) std::cout << "\n------------------------\nNew pcl" << std::endl;
    
    //Convert PointCloud2 to ArrayFire array
    float* data_f = reinterpret_cast<float *>(cloud_msg.data.data());
    af::array pcl_points(8, cloud_msg.width*cloud_msg.height, data_f);
   
    //remove Intenisty and Ring Bytes
    pcl_points = pcl_points(af::seq(3), af::span);

    // if (DEBUG_) std::cout << "Convert PointCloud2 to ArrayFire array time: " << ros::Time::now() - t0 << std::endl;
    // t0 = ros::Time::now();

    if (DEBUG_) {std::cout << "Convert PointCloud2 to ArrayFire array time: " << std::fixed  
         << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
          << std::setprecision(9); 
        std::cout << " sec" << std::endl; 
        start = std::chrono::high_resolution_clock::now(); 
    } 

    hypergrid::GridMap gridmap(width_, height_, cell_size_, origin_, map_frame_id_);


    Eigen::Affine3d e;
    tf::transformTFToEigen(transform, e);

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
    // std::cout << "Obstacles transform time: " << ros::Time::now() - t0 << std::endl;
    // t0 = ros::Time::now();
    if (DEBUG_) {std::cout << "Obstacles transform time: " << std::fixed  
         << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
          << std::setprecision(9); 
        std::cout << " sec" << std::endl; 
        start = std::chrono::high_resolution_clock::now(); 
    } 
    
    

    if (DEBUG_)std::cout << "points before: " << pcl_points.dims(1) << std::endl;
    //af::sync();
    // Remove the floor points
    remove_floor(pcl_points);


    if (DEBUG_)std::cout << "points after: " << pcl_points.dims(1) << std::endl;

    // if (DEBUG_) std::cout << "remove floor time: " << ros::Time::now() - t0 << std::endl;
    // t0 = ros::Time::now();

    if (DEBUG_) {std::cout << "remove floor time: " << std::fixed  
         << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
          << std::setprecision(9); 
        std::cout << " sec" << std::endl; 
        start = std::chrono::high_resolution_clock::now(); 
    } 
    
    pcl_points.eval();
    af::array obs = (af::join(0,pcl_points(af::seq(2), af::span), ones(af::span, af::seq(pcl_points.dims(1) )))).as(f64) ;

    pcl_points = pcl_points.T();
    af::array obstacle_coords =  gridmap.cellCoordsFromLocal(obs.T());
    
    
    af::array conds = ( (!(  (af::isInf(pcl_points(af::span, 0))) || 
                            (af::isInf(pcl_points(af::span, 1))) ||
                            (af::isInf(pcl_points(af::span, 2))) ) )
                             && pcl_points(af::span, 2) < max_height_ 
                             && gridmap.isCellInside(obstacle_coords) ) ;
    
    
    //using the sort function only to get the indices array
    af::array out;
    af::array indices_inside;
    af::sort(out, indices_inside, conds, 0, false);    
    
    // getting only the indices of the obstacles
    indices_inside = indices_inside(af::seq(af::sum(conds).scalar<unsigned>()));
    
    af::array inside_obstacles_coords = af::lookup(obstacle_coords(af::span, af::seq(2)), indices_inside);
    // std::cout << "Remove outside obstacles time: " << ros::Time::now() - t0 << std::endl;
    // t0 = ros::Time::now();
    //af_print(inside_obstacles_coords);    
    if (DEBUG_) {std::cout << "Remove outside obstacles time: " << std::fixed  
         << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
          << std::setprecision(9); 
        std::cout << " sec" << std::endl; 
        start = std::chrono::high_resolution_clock::now(); 
    } 
    
    tf::Vector3 trans = transform.getOrigin();   
    
    // Add a free line to each obstacle inside the grid
    // Set the origin of the lines at the sensor point
    hypergrid::Cell start_cell = gridmap.cellCoordsFromLocal(trans.getX(), trans.getY());
    gridmap.addFreeLines(start_cell, inside_obstacles_coords);   
    
    // std::cout << "Set free lines time: " << ros::Time::now() - t0 << std::endl;
    // t0 = ros::Time::now();

    if (DEBUG_) {std::cout << "Set free lines time: " << std::fixed  
         << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
          << std::setprecision(9); 
        std::cout << " sec" << std::endl; 
        start = std::chrono::high_resolution_clock::now(); 
    } 
    
    // Set the obstacles in the map
    af::array indices = inside_obstacles_coords(af::span, 1).as(s32) * gridmap.grid.dims(0) + inside_obstacles_coords(af::span, 0).as(s32);
    gridmap.grid(indices) = hypergrid::GridMap::OBSTACLE;    
    
    // std::cout << "Set obstacles time: " << ros::Time::now() - t0 << std::endl;
    // t0 = ros::Time::now();

    if (DEBUG_) {std::cout << "Set obstacles time: " << std::fixed  
         << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
          << std::setprecision(9); 
        std::cout << " sec" << std::endl; 
        start = std::chrono::high_resolution_clock::now(); 
    } 

    

    return gridmap;

}

GridMap LIDARConverter::convert( sensor_msgs::PointCloud2Ptr& cloud_msg, const tf::StampedTransform transform) const
{
    return convert(*cloud_msg, transform);
}

/* Removes all the points in the floor with a heightmap algorithm */
void LIDARConverter::remove_floor(af::array& cloud) const
{
    if (DEBUG_) std::cout << "points inside revmove floor: " << cloud.dims(1) << std::endl;
    float* x = cloud(0, af::span).host<float>();
    float* y = cloud(1, af::span).host<float>();
    float* z = cloud(2, af::span).host<float>();

    int height_cells = height_ / heightmap_cell_size_;
    int width_cells = width_ / heightmap_cell_size_;

    af::array x_arr = (width_cells / 2) + (cloud(0, af::span) / heightmap_cell_size_);
    af::array y_arr = (height_cells / 2) + (cloud(1, af::span) / heightmap_cell_size_);
    af::array z_arr = cloud(2, af::span);
    if (DEBUG_) std::cout << "points inside revmove floor: " << cloud.dims(1) << std::endl;
    float* min = new float[width_cells * height_cells];
    float* max = new float[width_cells * height_cells];

    // Resize the cloud to make it non-organized and work faster
    int width = cloud.dims(1);
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
    for (int i = 0; i < width; ++i)
    {
        int x_idx = (width_cells / 2) + (x[i] / heightmap_cell_size_);
        int y_idx = (height_cells / 2) + (y[i] / heightmap_cell_size_);

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
        int x_idx = (width_cells / 2) + (x[i] / heightmap_cell_size_);
        int y_idx = (height_cells / 2) + (y[i] / heightmap_cell_size_);
        int index = x_idx * height_cells + y_idx;

        bool is_obstacle = ((x_idx >= 0) && (x_idx < width_cells) && (y_idx >= 0) && (y_idx < height_cells)) && // Point is inside the grid
                           ((z[i] < max_height_))                                                             && // Point is inside height limit
                           ((abs(x[i]) > vehicle_box_size_) || (abs(y[i]) > vehicle_box_size_))                && // Point is not inside the vehicle box
                           ((max[index] - min[index]) > heightmap_threshold_);                                   // Point is inside a cell considered obstacle by the heightmap
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

} // hypergrid namespace
