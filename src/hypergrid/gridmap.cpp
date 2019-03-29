#include <hypergrid/gridmap.hpp>

namespace hypergrid
{

/* Constructors */
GridMap::GridMap() :
    origin_()
{
    width_ = 0.0;
    height_ = 0.0;
    cell_size_ = 1.0;
    map_frame_id_ = "";
}

GridMap::GridMap(double width, double height, double cell_size,
                 geometry_msgs::Pose origin, std::string map_frame_id) :
    origin_(origin)
{
    width_ = width;
    height_ = height;
    cell_size_ = cell_size;
    map_frame_id_ = map_frame_id;
    // Check for empty origin pose and fix the quaternion
    if (origin_.orientation.x == 0.0 &&
        origin_.orientation.y == 0.0 &&
        origin_.orientation.z == 0.0 &&
        origin_.orientation.w == 0.0)
    {
        origin_.orientation.w = 1.0;
    }
    int width_cells = width_ / cell_size_;
    int height_cells = height_ / cell_size_;
    grid = af::constant((int)Label::UNKNOWN, width_cells, height_cells, s32);
}

GridMap::GridMap(const nav_msgs::OccupancyGrid map_msg)
{
    map_frame_id_ = map_msg.header.frame_id;
    origin_ = map_msg.info.origin;
    cell_size_ = map_msg.info.resolution;
    width_ = map_msg.info.width * cell_size_;
    height_ = map_msg.info.height * cell_size_;

    // Convert the 8 bit data from the ROS msg to 32 bits
    int32_t* array_data = new int32_t[map_msg.data.size()];
    for (int i = 0; i < map_msg.data.size(); ++i)
    {
        array_data[i] = map_msg.data[i];
    }

    grid = af::array(map_msg.info.width, map_msg.info.height, array_data, afHost);
}

GridMap::GridMap(const nav_msgs::OccupancyGridConstPtr map_msg) : GridMap(*map_msg){}

/* Copy constructor */
GridMap::GridMap(const GridMap& other_map)
{
    grid = other_map.grid;
    origin_ = other_map.getOrigin();
    width_ = other_map.getWidth();
    height_ = other_map.getHeight();
    cell_size_ = other_map.getCellSize();
    map_frame_id_ = other_map.getMapFrameId();
}

/* Move constructor */
GridMap::GridMap(GridMap&& other_map) noexcept :
    grid(std::move(other_map.grid)),
    map_frame_id_(std::move(other_map.getMapFrameId()))

{
    origin_ = other_map.getOrigin();
    width_ = other_map.getWidth();
    height_ = other_map.getHeight();
    cell_size_ = other_map.getCellSize();

}

/* Assignement operator */
GridMap& GridMap::operator = (const GridMap& other_map)
{
    // check for self-assignment
    if(&other_map == this) return *this;

    grid = other_map.grid;
    origin_ = other_map.getOrigin();
    this->width_ = other_map.getWidth();
    this->height_ = other_map.getHeight();
    this->cell_size_ = other_map.getCellSize();
    this->map_frame_id_ = other_map.getMapFrameId();

    return *this;
}


/* Convert to ROS nav_msgs::OccupancyGrid msg */
void GridMap::toMapMsg(nav_msgs::OccupancyGrid& map_msg)
{
    map_msg = toMapMsg();
}

/* Convert to ROS nav_msgs::OccupancyGrid msg */
nav_msgs::OccupancyGrid GridMap::toMapMsg()
{
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_id_;

    nav_msgs::MapMetaData map_info;
    map_info.map_load_time = map_msg.header.stamp;
    map_info.resolution = cell_size_;
    map_info.width = grid.dims(0);
    map_info.height = grid.dims(1);

    // Set the map origin pose
    map_info.origin = origin_;
    map_msg.info = map_info;

    // Get the data from the device to a host pointer
    int32_t* array_data = grid.host<int32_t>();

    size_t total_size = grid.dims(0) * grid.dims(1);
    map_msg.data.resize(total_size);

    // Perform a casting from 32 bit to 8
    for (int i = 0; i < total_size; ++i)
    {
        map_msg.data[i] = array_data[i] & 0xFF;
    }

    delete[] array_data;
    return map_msg;
}

/* Resize the map to a different resolution (size in meters remains, number of cells changes) */
void GridMap::resize(double output_cell_size)
{
    // TODO
}

/* Apply a rotation to the map */
void GridMap::rotate(double angle)
{
    // TODO
}

/* Clear the map and set all cells to UNKNOWN */
void GridMap::clear()
{
    // TODO
}


/* Cell coordinates from local / global coordinates */
template<typename T>
Cell GridMap::cellCoordsFromLocal(T x, T y)
{
    // TODO
}

/* Local / global coordinates from cell coords */
template<typename T>
Point<T> GridMap::localCoordsFromCell(size_t x, size_t y)
{
    // TODO
}

/* Cell access from local / global coordinates */
template<typename T>
uint8_t cellFromLocal(T x, T y)
{
    // TODO
}


/* Add a free line from the vehicle to the given point */
template<typename T>
void GridMap::addFreeLine(Point<T> end)
{
    // TODO
}

/* Apply the inverse map origin transformation to get the cell (in meters) from a local point */
template<typename T>
Point<T> GridMap::originFromLocal_(Point<T> src) const
{
    // TODO
}

/* Apply the map origin transformation to get the local point from a cell point (in meters) */
template<typename T>
Point<T> GridMap::localFromOrigin_(Point<T> src) const
{
    // TODO
}


} // hypergrid namespace
