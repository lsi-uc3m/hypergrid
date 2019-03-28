#ifndef HYPERGRID_GRIDMAP_HPP
#define HYPERGRID_GRIDMAP_HPP

#include <iostream>
#include <memory>
#include <arrayfire.h>
#include <hypergrid/point.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>


namespace hypergrid
{

class GridMap
{
public:
    /* Smart Pointers typedefs */
    typedef std::shared_ptr<GridMap> Ptr;
    typedef std::shared_ptr<const GridMap> ConstPtr;

    /* Constructors */
    GridMap();
    GridMap(const nav_msgs::OccupancyGridConstPtr map_msg, std::string global_frame_id="odom");
    GridMap(double width, double height, double cell_size,
             geometry_msgs::Pose origin=geometry_msgs::Pose(),
             std::string local_frame_id="base_footprint",
             std::string global_frame_id="map");
    /* Copy constructor */
    GridMap(const GridMap& other_map);
    /* Move constructor */
    GridMap(GridMap&& other_map) noexcept;
    /* Assignement operator */
    GridMap& operator = (const GridMap& other_point);

    ~GridMap();

    /* Bool conversion operator to check if the map has been initialized */
    // TODO
    // inline explicit operator bool() const {return ...;}

    /* Convert to ROS nav_msgs::OccupancyGrid msg */
    void toMapMsg(nav_msgs::OccupancyGrid& map_msg);
    nav_msgs::OccupancyGrid toMapMsg();

    /* Resize the map to a different resolution (size in meters remains, number of cells changes) */
    void resize(double output_cell_size);
    /* Apply a rotation to the map */
    void rotate(double angle);

    /* Clear the map and set all cells to UNKNOWN */
    void clear();

    inline double width() const {return width_;}
    inline double height() const {return height_;}
    inline double cell_size() const {return cell_size_;}
    inline geometry_msgs::Pose origin() const {return origin_;}

    /* Coordinates convention: x -> col, y -> row.
       Conversion between global <--> local coordinates.
       All functions using global coordinates can throw a TF exception.
    */
    template<typename T>
    Point<T> localFromGlobal(T x, T y);
    template<typename T>
    inline Point<T> localFromGlobal(Point<T> p) {return localFromGlobal(p.x,p.y);}
    template<typename T>
    Point<T> globalFromLocal(T x, T y);
    template<typename T>
    inline Point<T> globalFromLocal(Point<T> p) {return globalFromLocal(p.x,p.y);}

    /* Check if the cell / local / global coordinates lay on the map */
    template<typename T>
    inline bool isCellInside(T x, T y) {return (x >= 0 && y >= 0 && x < grid.dims(0) && y < grid.dims(1));}
    inline bool isCellInside(Cell c) {return isCellInside(c.x, c.y);}
    
    template<typename T>
    inline bool isLocalInside(T x, T y) {return isCellInside(cellCoordsFromLocal<T>(x, y));}
    template<typename T>
    inline bool isLocalInside(Point<T> p) {return isLocalInside<T>(p.x, p.y);}
    template<typename T>
    inline bool isGlobalInside(T x, T y) {return isLocalInside(localFromGlobal<T>(x, y));}
    template<typename T>
    inline bool isGlobalInside(Point<T> p) {return isGlobalInside(p.x, p.y);}
    
    /* Direct cell access */
    // TODO: Check Arrayfire coords order. It's probably wrong...
    inline uint8_t operator[](Cell c) {return grid(c.x, c.y).scalar<uint8_t>();}
    inline uint8_t cell(size_t x, size_t y) {return grid(x, y).scalar<uint8_t>();}
    inline uint8_t cell(Cell c) {return grid(c.x, c.y).scalar<uint8_t>();}
    
    /* Cell coordinates from local / global coordinates */
    template<typename T>
    Cell cellCoordsFromLocal(T x, T y);
    template<typename T>
    inline Cell cellCoordsFromLocal(Point<T> p) {return cellCoordsFromLocal(p.x, p.y);}
    template<typename T>
    Cell cellCoordsFromGlobal(T x, T y);
    template<typename T>
    inline Cell cellCoordsFromGlobal(Point<T> p) {return cellCoordsFromGlobal(p.x, p.y);}

    /* Local / global coordinates from cell coords */
    template<typename T>
    Point<T> localCoordsFromCell(size_t x, size_t y);
    template<typename T>
    inline Point<T> localCoordsFromCell(Cell c) {return localCoordsFromCell<T>(c.x, c.y);}
    template<typename T>
    Point<T> globalCoordsFromCell(size_t x, size_t y);
    template<typename T>
    inline Point<T> globalCoordsFromCell(Cell c) {return globalCoordsFromCell<T>(c.x, c.y);}

    /* Cell access from local / global coordinates */
    template<typename T>
    uint8_t cellFromLocal(T x, T y);
    template<typename T>
    inline uint8_t cellFromLocal(Point<T> p) {return cellFromLocal(p.x, p.y);}
    template<typename T>
    uint8_t cellFromGlobal(T x, T y);
    template<typename T>
    inline uint8_t cellFromGlobal(Point<T> p) {return cellFromGlobal(p.x, p.y);}

    /* Add a free line from the vehicle to the given point */
    template<typename T>
    void addFreeLine(Point<T> end);

protected:
    af::array grid;

    double width_;      // meters
    double height_;     // meters
    double cell_size_;  // meters / cell

    geometry_msgs::Pose origin_;

    tf::TransformListener tf_;

    /* Apply the inverse map origin transformation to get the cell (in meters) from a local point */
    template<typename T>
    Point<T> originFromLocal_(Point<T> src) const;
    /* Apply the map origin transformation to get the local point from a cell point (in meters) */
    template<typename T>
    Point<T> localFromOrigin_(Point<T> src) const;
    /* Return the tf::Transform for the [map -> origin] transformation */
    tf::Transform getOriginTransform_() const;
};


} // hypergrid namespace

#endif
