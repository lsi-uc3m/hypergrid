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

GridMap::GridMap(const nav_msgs::OccupancyGridConstPtr map_msg)
{
    // TODO
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
    grid_ = af::constant(Label::UNKNOWN, width_cells, height_cells, s32);
}

/* Copy constructor */
GridMap::GridMap(const GridMap& other_map)
{
    grid_ = other_map.getGrid();
    origin_ = other_map.getOrigin();
    width_ = other_map.getWidth();
    height_ = other_map.getHeight();
    cell_size_ = other_map.getCellSize();
    map_frame_id_ = other_map.getMapFrameId();
}

/* Move constructor */
GridMap::GridMap(GridMap&& other_map) noexcept :
    grid_(std::move(other_map.getGrid())),
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

    grid_ = other_map.getGrid();
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
    // TODO
}

/* Convert to ROS nav_msgs::OccupancyGrid msg */
nav_msgs::OccupancyGrid toMapMsg()
{
    // TODO
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
