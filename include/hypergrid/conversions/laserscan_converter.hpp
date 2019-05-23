#ifndef HYPERGRID_LASERSCAN_CONVERTER_HPP
#define HYPERGRID_LASERSCAN_CONVERTER_HPP

#include <hypergrid/gridmap.hpp>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>


namespace hypergrid
{

class LaserScanConverter
{
public:

    /* Smart Pointers typedefs */
    typedef std::shared_ptr<LaserScanConverter> Ptr;
    typedef std::shared_ptr<const LaserScanConverter> ConstPtr;

    /* Constructor */
    LaserScanConverter(double width, double height, double cell_size,
                       geometry_msgs::Pose origin = geometry_msgs::Pose(),
                       std::string map_frame_id = "base_footprint", bool DEBUG_ = true);

    /* Convert a sensor_msgs::LaserScan to a hypergrid::GridMap */
    GridMap convert(const sensor_msgs::LaserScan& scan_msg, const tf::StampedTransform transform) const;
    GridMap convert(const sensor_msgs::LaserScanPtr& scan_msg, const tf::StampedTransform transform) const;
    GridMap convert(const sensor_msgs::LaserScanConstPtr& scan_msg, const tf::StampedTransform transform) const;

protected:
    double width_;
    double height_;
    double cell_size_;
    geometry_msgs::Pose origin_;
    std::string map_frame_id_;
    bool DEBUG_;
};


} // hypergrid namespace

#endif
