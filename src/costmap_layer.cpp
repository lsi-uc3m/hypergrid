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

    tf_listener_ = new tf::TransformListener;


    private_nh.param<std::string>("map_frame_id", map_frame_id, "base_footprint");
    private_nh.param("height", height, 50.0);
    private_nh.param("width", width, 50.0);
    private_nh.param("cell_size", cell_size, 0.2);
    private_nh.param("DEBUG", DEBUG, false);
    private_nh.param("heightmap_threshold", heightmap_threshold, 0.15);
    private_nh.param("heightmap_cell_size", heightmap_cell_size, 0.25);
    private_nh.param("max_height", max_height, 2.5);
    private_nh.param("vehicle_box_size", vehicle_box_size, 2.0);
    private_nh.getParam("laser_topics", laser_topics);
    std::cout << "laser topics size: " <<  laser_topics.size() << std::endl;
    private_nh.getParam("lidar_topics", lidar_topics);
    std::cout << "lidar topics size: " <<  lidar_topics.size() << std::endl;


    geometry_msgs::Pose origin;
    origin.position.x = - (width / 2) + cell_size;
    origin.position.y = - (height / 2) + cell_size;
    origin.orientation.w = 1;   

    // width = this->getSizeInMetersX();
    // height = this->getSizeInMetersY();
    // cell_size = this->getResolution();

    // std::cout << "width : " << width << std::endl;
    // std::cout << "height : " << height << std::endl;
    // std::cout << "cell_size : " << cell_size << std::endl;
    

    laser_converter = new hypergrid::LaserScanConverter(width, height, cell_size, origin, map_frame_id);

    lidar_converter = new hypergrid::LIDARConverter(width, height, cell_size,
                                                    origin,
                                                    map_frame_id,
                                                    heightmap_threshold,
                                                    heightmap_cell_size,
                                                    max_height ,
                                                    vehicle_box_size);

    laserscan_subs_.resize(laser_topics.size());
    
    for (int i = 0; i < laser_topics.size(); ++i)
    {
        laserscan_subs_[i] = public_nh.subscribe(laser_topics[i], 5, &HypergridLayer::laser_callback, this);
        if (DEBUG) std::cout << "Subscribed to " << laserscan_subs_[i].getTopic() << std::endl;
    }

    lidar_subs_.resize(lidar_topics.size());
    for (int i = 0; i < lidar_topics.size(); ++i)
    {
        lidar_subs_[i] = public_nh.subscribe(lidar_topics[i], 5, &HypergridLayer::lidar_callback, this);
        if (DEBUG) std::cout << "Subscribed to " << lidar_subs_[i].getTopic() << std::endl;
    }

    merged_map_pub = public_nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/laser_to_mergedgridmap", 2);

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

    //if (DEBUG) std::cout << "Master grid origin x    : " <<  master_grid.getOriginX()   << "Master grid origin y    : "  << master_grid.getOriginY()  << std::endl; 
    
    if(new_maps_.size()==0)
    {
        ROS_WARN("Now new map received yet: costmap will NOT be updated");
        return;
    }
    else if (DEBUG) std::cout << "size   : " << new_maps_.size() << std::endl;
    
  
    ros::Time t0 = ros::Time::now();
    hypergrid::GridMap Merged_gridmap =  new_maps_[0];
    for (int i = 1; i < new_maps_.size(); i++)
    {
        Merged_gridmap.grid = af::max(Merged_gridmap.grid, new_maps_[i].grid);
    }
    
    // Merged_gridmap.setOrigin(master_grid.getOriginX(), master_grid.getOriginY());
    /*
    int numOfObstcales = 0 ;
    for (int j = min_j; j < max_j; j++)
    {
        for (int i = min_i; i < max_i; i++)
        {
            // Get the cell coordinates in meters
            double wx;
            double wy;
            mapToWorld(i, j, wx, wy);

            // Get the local_map cell value corresponding to this costmap cell
            hypergrid::Cell coords = Merged_gridmap.cellCoordsFromLocal(wx, wy);

            // Check if the local coords lay inside the map
            if (!Merged_gridmap.isCellInside(coords))
            {
                master_grid.setCost(i, j, costmap_2d::NO_INFORMATION);
                continue;
            }

            // Get the cell value from local_map
            int cell_value = Merged_gridmap.grid(coords.x, coords.y).scalar<int>();
            // std::cout << " Cell value    : " <<  cell_value << std::endl;
            switch (cell_value)
            {
                case hypergrid::GridMap::OBSTACLE:
                    master_grid.setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
                    numOfObstcales++;
                    break;
                case hypergrid::GridMap::FREE:
                    master_grid.setCost(i, j, costmap_2d::FREE_SPACE);
                    break;
                default:
                    master_grid.setCost(i, j, costmap_2d::NO_INFORMATION);
                    break;
            }
        }
    }
    */
    hypergrid::Cell coords = Merged_gridmap.cellCoordsFromLocal(0, 0);
    Merged_gridmap.grid.eval();
    merged_map_pub.publish(Merged_gridmap.toMapMsg());

    // Custom debug modifications
    int width_cells = this->width / this->cell_size;
    int height_cells = this->height / this->cell_size;

    // Lookup table
    unsigned char gridmap_costmap_lut[256] = {(unsigned char)costmap_2d::NO_INFORMATION};
    gridmap_costmap_lut[hypergrid::GridMap::OBSTACLE] = (unsigned char)costmap_2d::LETHAL_OBSTACLE;
    gridmap_costmap_lut[hypergrid::GridMap::FREE] = (unsigned char)costmap_2d::FREE_SPACE;

    unsigned char* master_grid_ptr = master_grid.getCharMap();
    int* merged_grid_ptr = Merged_gridmap.grid.host<int32_t>();

    for (int i = 0; i < width_cells * height_cells; ++i)
    {
        master_grid_ptr[i] = gridmap_costmap_lut[merged_grid_ptr[i]];
    }

    new_maps_.clear();
    if (DEBUG) std::cout << "updateBounds time: " << ros::Time::now() - t0 << std::endl;
}

void HypergridLayer::laser_callback(const sensor_msgs::LaserScanPtr scan_msg)
{
    ROS_WARN("laser call back ");
    tf::StampedTransform laser_footprint_transform;
    try
    {
        tf_listener_->lookupTransform(map_frame_id, scan_msg->header.frame_id, ros::Time(0), laser_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    hypergrid::GridMap gridmap =  laser_converter->convert(scan_msg, laser_footprint_transform);
    new_maps_.push_back(gridmap);
    ROS_WARN("laser call back FINISHED");
}

void HypergridLayer::lidar_callback(sensor_msgs::PointCloud2Ptr cloud_msg)
{

    tf::StampedTransform lidar_footprint_transform;
    try
    {
        tf_listener_->lookupTransform(map_frame_id, cloud_msg->header.frame_id, ros::Time(0), lidar_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    new_maps_.push_back(lidar_converter->convert(cloud_msg, lidar_footprint_transform));
}

} // end namespace
