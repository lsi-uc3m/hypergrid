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

private:
    bool rolling_window_;
    void laser_map_callback(const sensor_msgs::LaserScan scan_msg);
    void lidar_map_callback(sensor_msgs::PointCloud2Ptr cloud_msg);
    // TODO
};
}
#endif
