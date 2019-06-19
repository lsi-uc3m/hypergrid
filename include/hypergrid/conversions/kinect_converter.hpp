#ifndef HYPERGRID_KINECT_CONVERTER_HPP
#define HYPERGRID_KINECT_CONVERTER_HPP

#include <cstring>
#include <hypergrid/gridmap.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>



namespace hypergrid
{

class KINECTConverter
{
public:

    /* Smart Pointers typedefs */
    typedef std::shared_ptr<KINECTConverter> Ptr;
    typedef std::shared_ptr<const KINECTConverter> ConstPtr;

    /* Constructor */
    KINECTConverter(double width, double height, double cell_size,
                   geometry_msgs::Pose origin = geometry_msgs::Pose(),
                   std::string map_frame_id = "base_footprint",
                   double max_height = 2.5,
                   bool DEBUG = true,
                   double dis_VW = 6.0
                   );

    /* Convert a sensor_msgs::PointCloud2 to a hypergrid::GridMap */
    GridMap convert(sensor_msgs::PointCloud2& cloud_msg, const tf::StampedTransform transform) ;
    GridMap convert(sensor_msgs::PointCloud2Ptr& cloud_msg, const tf::StampedTransform transform) ;
    
    double conver_rad(double grados);

protected:
    double width_;
    double height_;
    double cell_size_;
    geometry_msgs::Pose origin_;
    std::string map_frame_id_;
    double dis_VW_;
    pcl::SACSegmentation<pcl::PointXYZ> seg_;

    double max_height_;
    bool DEBUG_;
};


} // hypergrid namespace

#endif
