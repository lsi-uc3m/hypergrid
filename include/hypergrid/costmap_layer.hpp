#ifndef HYPERGRID_COSTMAP_LAYER_H_
#define HYPERGRID_COSTMAP_LAYER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <hypergrid/gridmap.hpp>
#include <hypergrid/conversions/laserscan_converter.hpp>
#include <hypergrid/conversions/lidar_converter.hpp>

namespace hypergrid
{

class HypergridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    HypergridLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                                                 int max_i, int max_j);
    bool isDiscretized()
    {
        return true;
    }

    virtual void matchSize();

    void laser_callback(const sensor_msgs::LaserScanPtr scan_msg);
    void lidar_callback(sensor_msgs::PointCloud2Ptr cloud_msg);

    

private:
    std::vector<GridMap> new_maps_;
    std::vector<ros::Subscriber> laserscan_subs_;
    std::vector<ros::Subscriber> lidar_subs_;
    ros::Publisher merged_map_pub;
    std::vector<std::string> laser_topics;
    std::vector<std::string> lidar_topics;
    tf::TransformListener* tf_listener_;
    hypergrid::LaserScanConverter* laser_converter;
    hypergrid::LIDARConverter* lidar_converter;

    double width;
    double height;
    double cell_size; 
    double heightmap_threshold;
    double heightmap_cell_size;
    double max_height;
    double vehicle_box_size;
    std::string map_frame_id;
    bool DEBUG;


    bool rolling_window_;
    
    // TODO
};
}
#endif
