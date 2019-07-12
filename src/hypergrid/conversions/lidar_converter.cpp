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

    // ros::Time t0 = ros::Time::now();
    // ros::Time t_start = t0;
    if (DEBUG_) std::cout << "\n------------------------\nNew pcl" << std::endl;
    
    // Convert PointCloud2 to ArrayFire array
    float* data_f = reinterpret_cast<float *>(cloud_msg.data.data());
    af::array pcl_points(8, cloud_msg.width*cloud_msg.height, data_f);
   
    // Remove Intenisty and Ring Bytes
    pcl_points = pcl_points(af::seq(3), af::span);

    if (DEBUG_)
    {
        std::cout << "Convert PointCloud2 to ArrayFire array time: " << std::fixed
        << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
        << std::setprecision(9) << " sec" << std::endl;
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

    if (DEBUG_)
    {
        std::cout << "Obstacles transform time: " << std::fixed
        << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
        << std::setprecision(9) << " sec" << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    if (DEBUG_) std::cout << "points before: " << pcl_points.dims(1) << std::endl;

    // Remove the floor points
    remove_floor(pcl_points);

    if (DEBUG_) std::cout << "points after: " << pcl_points.dims(1) << std::endl;

    if (DEBUG_)
    {
        std::cout << "remove floor time: " << std::fixed
        << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
        << std::setprecision(9) << " sec" << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    pcl_points.eval();
    af::array obs = (af::join(0, pcl_points(af::seq(2), af::span), ones(af::span, af::seq(pcl_points.dims(1) )))).as(f64);

    pcl_points = pcl_points.T();
    af::array obstacle_coords = gridmap.cellCoordsFromLocal(obs.T())(af::span, af::seq(2));

    if (DEBUG_)
    {
        std::cout << "Remove outside obstacles time: " << std::fixed
        << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
        << std::setprecision(9) << " sec" << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    // Add a free line to each obstacle inside the grid
    // Set the origin of the lines at the sensor point
    hypergrid::Cell start_cell = gridmap.cellCoordsFromLocal(transform.getOrigin().getX(), transform.getOrigin().getY());
    gridmap.addFreeLines(start_cell, obstacle_coords);

    if (DEBUG_)
    {
        std::cout << "Set free lines time: " << std::fixed
        << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
        << std::setprecision(9) << " sec" << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    // Set the obstacles in the map
    af::array indices = obstacle_coords(af::span, 1).as(s32) * gridmap.grid.dims(0) + obstacle_coords(af::span, 0).as(s32);
    gridmap.grid(indices) = hypergrid::GridMap::OBSTACLE;

    if (DEBUG_)
    {
        std::cout << "Set obstacles time: " << std::fixed
        << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
        << std::setprecision(9) << " sec" << std::endl;
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
    float* x = cloud(0, af::span).host<float>();
    float* y = cloud(1, af::span).host<float>();
    float* z = cloud(2, af::span).host<float>();

    int height_cells = height_ / heightmap_cell_size_;
    int width_cells = width_ / heightmap_cell_size_;

    af::array x_arr = (width_cells / 2) + (cloud(0, af::span) / heightmap_cell_size_);
    af::array y_arr = (height_cells / 2) + (cloud(1, af::span) / heightmap_cell_size_);
    af::array z_arr = cloud(2, af::span);

    float* min = new float[width_cells * height_cells];
    float* max = new float[width_cells * height_cells];

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

    std::vector<int> indices;
    // Build height map and remove points outside, too high or inside the vehicle box
    for (int i = 0; i < cloud.dims(1); ++i)
    {
        int x_idx = (width_cells / 2) + (x[i] / heightmap_cell_size_);
        int y_idx = (height_cells / 2) + (y[i] / heightmap_cell_size_);
        bool valid_point = ((x_idx >= 0) && (x_idx < width_cells) && (y_idx >= 0) && (y_idx < height_cells)) && // Point is inside the grid
                           ((z[i] < max_height_))                                                            && // Point is inside height limit
                           (std::isfinite(x[i]) && std::isfinite(y[i]) && std::isfinite(z[i]))               && // Point is not NaN or Inf
                           ((abs(x[i]) > vehicle_box_size_) || (abs(y[i]) > vehicle_box_size_));                // Point is not inside the vehicle box
        if (valid_point)
        {
            // Mark the point for removal
            indices.push_back(i);

            // Add the point to the heightmap
            if (x_idx >= 0 && x_idx < width_cells && y_idx >= 0 && y_idx < height_cells)
            {
                int index = x_idx * height_cells + y_idx;
                min[index] = std::min<float>(min[index], z[i]);
                max[index] = std::max<float>(max[index], z[i]);
            }
        }
    }

    af::array indices_arr(indices.size(), indices.data());
    delete[] x;
    delete[] y;
    delete[] z;
    cloud = af::lookup(cloud, indices_arr, 1);
    cloud.eval();
    indices.clear();

    x = cloud(0, af::span).host<float>();
    y = cloud(1, af::span).host<float>();

    // Remove the floor points
    for (int i = 0; i < cloud.dims(1); ++i)
    {
        int x_idx = (width_cells / 2) + (x[i] / heightmap_cell_size_);
        int y_idx = (height_cells / 2) + (y[i] / heightmap_cell_size_);

        if (x_idx >= 0 && x_idx < width_cells && y_idx >= 0 && y_idx < height_cells)
        {
            int index = x_idx * height_cells + y_idx;
            if ((max[index] - min[index]) > heightmap_threshold_)
            {
                indices.push_back(i);
            }
        }
    }

    indices_arr = af::array(indices.size(), indices.data());
    delete[] x;
    delete[] y;
    delete[] min;
    delete[] max;

    cloud = af::lookup(cloud, indices_arr, 1);
}

} // hypergrid namespace
