#include <hypergrid/gridmap.hpp>

namespace hypergrid
{

/* Constructors */
GridMap::GridMap()
{
    // TODO
}

GridMap::GridMap(const nav_msgs::OccupancyGridConstPtr map_msg, std::string global_frame_id)
{
    // TODO
}

GridMap::GridMap(double width, double height, double cell_size,
                 geometry_msgs::Pose origin, std::string local_frame_id,
                 std::string global_frame_id)
{
    // TODO
}

/* Copy constructor */
GridMap::GridMap(const GridMap& other_map)
{
    // TODO
}

/* Move constructor */
GridMap::GridMap(GridMap&& other_map) noexcept
{
    // TODO
}

/* Assignement operator */
GridMap& GridMap::operator = (const GridMap& other_point)
{
    // TODO
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


/* Coordinates convention: x -> col, y -> row.
   Conversion between global <--> local coordinates.
   All functions using global coordinates can throw a TF exception.
*/
template<typename T>
Point<T> GridMap::localFromGlobal(T x, T y)
{
    // TODO
}

template<typename T>
Point<T> GridMap::globalFromLocal(T x, T y)
{
    // TODO
}

/* Cell coordinates from local / global coordinates */
template<typename T>
Cell GridMap::cellCoordsFromLocal(T x, T y)
{
    // TODO
}

template<typename T>
Cell GridMap::cellCoordsFromGlobal(T x, T y)
{
    // TODO
}

/* Local / global coordinates from cell coords */
template<typename T>
Point<T> GridMap::localCoordsFromCell(size_t x, size_t y)
{
    // TODO
}

template<typename T>
Point<T> GridMap::globalCoordsFromCell(size_t x, size_t y)
{
    // TODO
}

/* Cell access from local / global coordinates */
template<typename T>
uint8_t cellFromLocal(T x, T y)
{
    // TODO
}

template<typename T>
uint8_t cellFromGlobal(T x, T y)
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

/* Return the tf::Transform for the [map -> origin] transformation */
tf::Transform GridMap::getOriginTransform_() const
{
    // TODO
}


} // hypergrid namespace
