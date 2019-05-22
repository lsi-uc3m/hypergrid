#include <hypergrid/conversions/laserscan_converter.hpp>


namespace hypergrid
{

/* Constructor */
LaserScanConverter::LaserScanConverter(double width, double height, double cell_size,
                                       geometry_msgs::Pose origin, std::string map_frame_id) :
    origin_(origin)
{
    width_ = width;
    height_ = height;
    cell_size_ = cell_size;
    map_frame_id_ = map_frame_id;
}

/* Convert a sensor_msgs::LaserScan to a hypergrid::GridMap */
GridMap LaserScanConverter::convert(const sensor_msgs::LaserScan& scan_msg, const tf::StampedTransform transform) const
{
    // TODO
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
