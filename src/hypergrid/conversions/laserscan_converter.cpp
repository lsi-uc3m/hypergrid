#include <hypergrid/conversions/laserscan_converter.hpp>


namespace hypergrid
{

/* Constructor */
LaserScanConverter::LaserScanConverter(double width, double height, double cell_size,
                                       geometry_msgs::Pose origin, std::string map_frame_id, bool DEBUG) :
    origin_(origin)
{
    width_ = width;
    height_ = height;
    cell_size_ = cell_size;
    map_frame_id_ = map_frame_id;
    DEBUG_ = DEBUG;
}

/* Convert a sensor_msgs::LaserScan to a hypergrid::GridMap */
GridMap LaserScanConverter::convert(const sensor_msgs::LaserScan& scan_msg, const tf::StampedTransform transform) const
{
    ros::Time t0 = ros::Time::now();
    if (DEBUG_) std::cout << "\n------------------------\nNew laser" << std::endl;

    hypergrid::GridMap gridmap(width_, height_, cell_size_, origin_, map_frame_id_);

    af::array ranges(scan_msg.ranges.size(), 1, scan_msg.ranges.data());
    af::array scan_angle(scan_msg.ranges.size(), 1);

    af::array ranges_seq = af::seq(0, scan_msg.ranges.size() - 1);
    scan_angle = scan_msg.angle_min + ranges_seq * scan_msg.angle_increment;

    // Remove NaNs and Inf values
    af::array ranges_nan_inf = af::isNaN(ranges) || af::isInf(ranges);
    af::replace(ranges, !ranges_nan_inf, scan_msg.range_max);

    if (DEBUG_) std::cout << "Init time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    af::array obstacles = af::constant(1.0, scan_msg.ranges.size(), 3, f64);

    obstacles(af::span, 0) = ranges * af::cos(scan_angle);
    obstacles(af::span, 1) = ranges * af::sin(scan_angle);

    // Get Rotation and translation from TF transform
    tf::Vector3 trans = transform.getOrigin();
    double theta = tf::getYaw(transform.getRotation());
    double t_r_array[] = {std::cos(theta),  std::sin(theta), 0,
                          -std::sin(theta), std::cos(theta), 0,
                          trans.getX(),     trans.getY(),    1};
    af::array transformation(3, 3, t_r_array);
    // Transform the obstacles to the map frame
    obstacles = af::matmul(transformation, obstacles.T()).T();

    if (DEBUG_) std::cout << "Obstacles transform time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    // Get obstacle cell coordinates
    af::array obstacle_coords = gridmap.cellCoordsFromLocal(obstacles);

    // Remove outside obstacles
    // Use the sort function only to get the indices array
    af::array cond;
    af::array indices_inside;
    af::sort(cond, indices_inside, gridmap.isCellInside(obstacle_coords), 0, false);

    // Get ony the indices of the obstacles
    indices_inside = indices_inside(af::seq(af::sum(cond).scalar<unsigned>()));

    // inside_obstacles_coords = inside_obstacles_coords(af::seq(num_obs_inside), af::span);
    
    af::array inside_obstacles_coords = af::lookup(obstacle_coords(af::span, af::seq(2)), indices_inside);
   
    if (DEBUG_) std::cout << "Remove outside obstacles time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    // Add a free line to each obstacle inside the grid
    // Set the origin of the lines at the sensor point
    hypergrid::Cell start = gridmap.cellCoordsFromLocal(trans.getX(), trans.getY());
    gridmap.addFreeLines(start, inside_obstacles_coords);

    if (DEBUG_) std::cout << "Set free lines time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    // Set the obstacles in the map
    af::array indices = inside_obstacles_coords(af::span, 1).as(s32) * gridmap.grid.dims(0) + inside_obstacles_coords(af::span, 0).as(s32);
    gridmap.grid(indices) = hypergrid::GridMap::OBSTACLE;

    if (DEBUG_) std::cout << "Set obstacles time: " << ros::Time::now() - t0 << std::endl;
    t0 = ros::Time::now();

    return gridmap;
}

GridMap LaserScanConverter::convert(const sensor_msgs::LaserScanPtr& scan_msg, const tf::StampedTransform transform) const
{
    return convert(*scan_msg, transform);
}

GridMap LaserScanConverter::convert(const sensor_msgs::LaserScanConstPtr& scan_msg, const tf::StampedTransform transform) const
{
    return convert(*scan_msg, transform);
}


} // hypergrid namespace
