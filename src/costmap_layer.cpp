#include <hypergrid/costmap_layer.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hypergrid::HypergridLayer, costmap_2d::Layer)

namespace hypergrid
{

HypergridLayer::HypergridLayer() {}

void HypergridLayer::onInitialize()
{
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle public_nh;

    current_ = true;
    rolling_window_ = layered_costmap_->isRolling();
    default_value_ = costmap_2d::NO_INFORMATION;
    matchSize();

    // TODO
}

void HypergridLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
              master->getResolution(), master->getOriginX(), master->getOriginY());
}

void HypergridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (rolling_window_)
    {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }

    *min_x = std::min(*min_x, getOriginX());
    *min_y = std::min(*min_y, getOriginY());
    *max_x = std::max(*max_x, getSizeInMetersX() + getOriginX());
    *max_y = std::max(*max_y, getSizeInMetersY() + getOriginY());
}

void HypergridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                                                   int max_i, int max_j)
{
    // TODO
}

} // end namespace
