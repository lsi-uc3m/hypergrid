#include <hypergrid/conversions/lidar_converter.hpp>


namespace hypergrid
{

/* Constructor */
LIDARConverter::LIDARConverter(double width, double height, double cell_size,
                               geometry_msgs::Pose origin, std::string map_frame_id,
                               double heightmap_threshold, double heightmap_cell_size,
                               double max_height, double vehicle_box_size) :
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
}

/* Convert a sensor_msgs::PointCloud2 to a hypergrid::GridMap */
GridMap LIDARConverter::convert(const sensor_msgs::PointCloud2& cloud_msg, const tf::StampedTransform transform) const
{

}

GridMap LIDARConverter::convert(const sensor_msgs::PointCloud2Ptr& cloud_msg, const tf::StampedTransform transform) const
{
    return convert(*cloud_msg, transform);
}

GridMap LIDARConverter::convert(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const tf::StampedTransform transform) const
{
    return convert(*cloud_msg, transform);
}


} // hypergrid namespace
