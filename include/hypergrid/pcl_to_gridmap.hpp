

#ifndef PCL_TO_MAP_HPP
#define PCL_TO_MAP_HPP

#include <hypergrid/gridmap.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>


namespace hypergrid
{

class PCLConverter
{

private:
    int width_;
    int height_;
    double cell_size_;
    double max_height_;
    geometry_msgs::Pose origin_;
    std::string vehicle_frame_id_;

    tf::TransformListener tf_;

public:
    PCLConverter(){}

    PCLConverter(int width, int height, double cell_size, double max_height, std::string vehicle_frame="base_footprint") :
        width_(width),
        height_(height),
        cell_size_(cell_size),
        max_height_(max_height),
        vehicle_frame_id_(vehicle_frame)
    {
        origin_ = geometry_msgs::Pose();
        origin_.position.x = - (width_ / 2);
        origin_.position.y = - (height_ / 2);
        origin_.orientation.w = 1;
    }

    PCLConverter(int width, int height, double cell_size, double max_height, geometry_msgs::Pose origin, std::string vehicle_frame="base_footprint") :
        width_(width),
        height_(height),
        cell_size_(cell_size),
        max_height_(max_height),
        origin_(origin),
        vehicle_frame_id_(vehicle_frame)
    {}
    
    ~PCLConverter() {}

    GridMap convertPointcloud(const sensor_msgs::PointCloud2ConstPtr cloud_msg)
    {
        //Update the sensor tf
        tf::StampedTransform sensor_footprint_transform;
        try
        {
            tf_.lookupTransform(vehicle_frame_id_, cloud_msg->header.frame_id, ros::Time(0), sensor_footprint_transform);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
          //  return nullptr;
        }
        GridMap gridmap(width_, height_, cell_size_, origin_, vehicle_frame_id_);

        return gridmap;

        
        
    }
};

}

#endif