#ifndef HYPERGRID_LIDAR_CONVERTER_HPP
#define HYPERGRID_LIDAR_CONVERTER_HPP

#include <hypergrid/gridmap.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


namespace hypergrid
{

class LIDARConverter
{
public:

    /* Smart Pointers typedefs */
    typedef std::shared_ptr<LIDARConverter> Ptr;
    typedef std::shared_ptr<const LIDARConverter> ConstPtr;

    /* Constructor */
    LIDARConverter(double width, double height, double cell_size,
                   geometry_msgs::Pose origin = geometry_msgs::Pose(),
                   std::string map_frame_id = "base_footprint",
                   double heightmap_threshold = 0.15,
                   double heightmap_cell_size = 0.25,
                   double max_height = 2.5,
                   double vehicle_box_size = 2.0);

    /* Convert a sensor_msgs::PointCloud2 to a hypergrid::GridMap */
    GridMap convert(const sensor_msgs::PointCloud2& cloud_msg, const tf::StampedTransform transform) const;
    GridMap convert(const sensor_msgs::PointCloud2Ptr& cloud_msg, const tf::StampedTransform transform) const;
    GridMap convert(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const tf::StampedTransform transform) const;

protected:
    double width_;
    double height_;
    double cell_size_;
    geometry_msgs::Pose origin_;
    std::string map_frame_id_;

    double heightmap_threshold_;
    double heightmap_cell_size_;
    double max_height_;
    double vehicle_box_size_;

};


} // hypergrid namespace

#endif
