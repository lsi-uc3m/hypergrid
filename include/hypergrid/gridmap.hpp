#ifndef HYPERGRID_GRIDMAP_HPP
#define HYPERGRID_GRIDMAP_HPP

#include <iostream>
#include <memory>
#include <arrayfire.h>
#include <hypergrid/point.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>


namespace hypergrid
{

class GridMap
{
public:
    /* Map labels.
       Please note that cells can have values in the range [0-100] to indicate occupation probability.
    */
    enum Label
    {
        UNKNOWN = -1,
        FREE = 0,
        OBSTACLE = 100
    };

    /* Smart Pointers typedefs */
    typedef std::shared_ptr<GridMap> Ptr;
    typedef std::shared_ptr<const GridMap> ConstPtr;

    /* Constructors */
    GridMap();
    GridMap(double width, double height, double cell_size,
             geometry_msgs::Pose origin = geometry_msgs::Pose(),
             std::string map_frame_id = "base_footprint");
    GridMap(const nav_msgs::OccupancyGrid map_msg);
    GridMap(const nav_msgs::OccupancyGridConstPtr map_msg);
    /* Copy constructor */
    GridMap(const GridMap& other_map);
    /* Move constructor */
    GridMap(GridMap&& other_map) noexcept;
    /* Assignement operator */
    GridMap& operator = (const GridMap& other_point);

    /* Bool conversion operator to check if the map has been initialized */
    inline explicit operator bool() const {return !grid.isempty();}

    /* Convert to ROS nav_msgs::OccupancyGrid msg */
    void toMapMsg(nav_msgs::OccupancyGrid& map_msg);
    nav_msgs::OccupancyGrid toMapMsg();

    /* Resize the map to a different resolution (size in meters remains, number of cells changes) */
    void resize(double output_cell_size);
    /* Apply a rotation to the map */
    void rotate(double angle);

    /* Clear the map and set all cells to UNKNOWN */
    void clear();

    /* Return a copy of the grid matrix */
    inline double getWidth() const {return width_;}
    inline double getHeight() const {return height_;}
    inline double getCellSize() const {return cell_size_;}
    inline std::string getMapFrameId() const {return map_frame_id_;}
    inline geometry_msgs::Pose getOrigin() const {return origin_;}

    /* Check if the cell / local coordinates lay on the map */
    template<typename T>
    inline bool isCellInside(T x, T y) {return (x >= 0 && y >= 0 && x < grid.dims(0) && y < grid.dims(1));}
    inline bool isCellInside(Cell c) {return isCellInside(c.x, c.y);}

    template<typename T>
    inline bool isLocalInside(T x, T y) {return isCellInside(cellCoordsFromLocal<T>(x, y));}
    template<typename T>
    inline bool isLocalInside(Point<T> p) {return isLocalInside<T>(p.x, p.y);}

    /* Direct cell access */
    // TODO: Check Arrayfire coords order. It's probably wrong...
    inline int32_t operator[](Cell c) {return grid(c.x, c.y).scalar<int32_t>();}
    inline int32_t cell(size_t x, size_t y) {return grid(x, y).scalar<int32_t>();}
    inline int32_t cell(Cell c) {return grid(c.x, c.y).scalar<int32_t>();}

    /* Cell coordinates from local coordinates */
    template<typename T>
    Cell cellCoordsFromLocal(T x, T y);
    template<typename T>
    inline Cell cellCoordsFromLocal(Point<T> p) {return cellCoordsFromLocal(p.x, p.y);}

    /* Local coordinates from cell coords */
    template<typename T>
    Point<T> localCoordsFromCell(size_t x, size_t y);
    template<typename T>
    inline Point<T> localCoordsFromCell(Cell c) {return localCoordsFromCell<T>(c.x, c.y);}

    /* Cell access from local coordinates */
    template<typename T>
    int32_t cellFromLocal(T x, T y);
    template<typename T>
    inline int32_t cellFromLocal(Point<T> p) {return cellFromLocal(p.x, p.y);}

    /* Add a free line from the vehicle to the given point */
    template<typename T>
    void addFreeLine(Point<T> end);

    /* 2D matrix containing the map. Data type is 32 bit signed integer to avoid conversion in the GPU. */
    af::array grid;

protected:

    double width_;      // meters
    double height_;     // meters
    double cell_size_;  // meters / cell

    std::string map_frame_id_;
    geometry_msgs::Pose origin_;

    /* Apply the inverse map origin transformation to get the cell (in meters) from a local point */
    template<typename T>
    Point<T> originFromLocal_(Point<T> src) const;
    /* Apply the map origin transformation to get the local point from a cell point (in meters) */
    template<typename T>
    Point<T> localFromOrigin_(Point<T> src) const;
    /*Getting the Origin Transform matrix*/
    af::array getOriginTransform_() const;
};


} // hypergrid namespace

#endif
