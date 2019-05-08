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

/* Resize the map to a different resolution (size in meters remains, number of cells changes).
   Works only if cell_size > output_cell_size
*/
void GridMap::resize(double output_cell_size)
{
    grid = af::resize(cell_size_ / output_cell_size, grid, AF_INTERP_LOWER);
    cell_size_ = output_cell_size;
}

/* Apply a rotation to the map */
void GridMap::rotate(double angle)
{
    // Original  Dimension
    long dim0 = grid.dims(0);
    long dim1 = grid.dims(1);
    // Adding grid.dim(0) rows to each side then adding grid.dim(1) columns to each side to modfiy rotate function
    af::array bigger0 = af::constant((int)Label::UNKNOWN, grid.dims(0), grid.dims(1), s32);
    af::array bigger1 = af::constant((int)Label::UNKNOWN, grid.dims(0) * 3, grid.dims(1), s32);

    // Rows
    grid = af::join(0, grid, bigger0);
    grid = af::join(0, bigger0, grid);
    // Columns
    grid = af::join(1, grid, bigger1);
    grid = af::join(1, bigger1, grid);
    // Rotate
    grid = af::rotate(grid, angle, true);
    // Return to original size
    grid = grid(af::seq(dim0, grid.dims(0) - dim0 - 1), af::seq(dim1, grid.dims(1) - dim1 - 1));
}

/* Clear the map and set all cells to UNKNOWN */
void GridMap::clear()
{
    grid = UNKNOWN;
}

/* Cell coordinates from local coordinates */
template<typename T>
Cell GridMap::cellCoordsFromLocal(T x, T y)
{
    Pointd t_point = originFromLocal_(Pointd(x, y));
    return Cell(std::floor(t_point.x / cell_size_), std::floor(t_point.y / cell_size_));
}

/* Local coordinates from cell coords */
template<typename T>
Point<T> GridMap::localCoordsFromCell(size_t x, size_t y)
{
    // Get the local point in the center of the cell
    Pointd t_point = localFromOrigin_(Pointd(x * cell_size_ + cell_size_/2, y * cell_size_ + cell_size_/2));
    return Point<T>(t_point.x, t_point.y);
}

/* Cell access from local coordinates */
template<typename T>
af::array::array_proxy GridMap::cellFromLocal(T x, T y)
{
    Cell p = cellCoordsFromLocal(x, y);
    if (!isCellInside(p)) throw std::out_of_range("Index out of map bounds");
    return cell(p);
}

/* Add a free line from the vehicle to the given cell */
void GridMap::addFreeLine(Cell end)
{
    Cell start = cellCoordsFromLocal(0, 0);
    dda_(start.x, start.y, end.x, end.y);
}

/* Add a free line from the start cell to the end cell */
void GridMap::addFreeLine(Cell start, Cell end)
{
    dda_(start.x, start.y, end.x, end.y);
}

/* Add multiple free lines from a given point */
void GridMap::addFreeLines(Cell start, af::array endpoints)
{
    add_lines(grid, FREE, start.x, start.y, endpoints);
}


/* Apply the inverse map origin transformation to get the cell (in meters) from a local point */
Pointd GridMap::originFromLocal_(Pointd src) const
{
    double src_arr[] = {src.x, src.y, 1};
    af::array af_src(3, 1, src_arr);
    af::array t_src = af::matmul(af::inverse(getOriginTransform_()), af_src);
    return Pointd(t_src(0).scalar<double>(), t_src(1).scalar<double>());
}

/* Apply the map origin transformation to get the local point from a cell point (in meters) */
Pointd GridMap::localFromOrigin_(Pointd src) const
{
    double src_arr[] = {src.x, src.y, 1};
    af::array af_src(3, 1, src_arr);
    af::array t_src = af::matmul(getOriginTransform_(), af_src);
    return Pointd(t_src(0).scalar<double>(), t_src(1).scalar<double>());
}

/* Get the Origin Transform matrix */
af::array GridMap::getOriginTransform_() const
{
    // Get Theta
    double theta = tf::getYaw(origin_.orientation);
    
    // Create the 2D translation and rotation matrix
    double t_r_array[] = {std::cos(theta),    std::sin(theta),    0,
                          -std::sin(theta),   std::cos(theta),    0,
                          origin_.position.x, origin_.position.y, 1};
    af::array t_r_matrix(3, 3, t_r_array);
    return t_r_matrix;
}

/*
    DDA algorithm to draw line from one point to another in the occupancy grid.
*/
void GridMap::dda_(float x1, float y1, float const x2, float const y2)
{
    float x, y, dx, dy, step;

    dx = abs(x2 - x1);
    dy = abs(y2 - y1);

    step = dx >= dy ? dx : dy;

    dx = (x2 - x1) / step;
    dy = (y2 - y1) / step;

    af::array x_arr = af::constant(x1, step);
    af::array y_arr = af::constant(y1, step);
    af::array count = af::seq(0, step - 1);

    x_arr = x_arr + dx * count;
    y_arr = y_arr + dy * count;

    af::array indices(x_arr.dims(0));
    indices = y_arr.as(s32) * grid.dims(0) + x_arr.as(s32);

    grid(indices) = FREE;
}

/* Bresenham algorithm to draw line from one point to another in the occupancy grid.
   Copied from: http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
*/
void GridMap::bresenham_(int x1, int y1, int const x2, int const y2)
{
    int delta_x(x2 - x1);
    // If x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;

    int delta_y(y2 - y1);
    // If y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;

    if (grid(x1, y1).scalar<int>() == OBSTACLE) return;
    grid(x1, y1) = FREE;

    if (delta_x >= delta_y)
    {
        // Error may go below zero
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2)
        {
            // Reduce error, while taking into account the corner case of error == 0
            if ((error > 0) || (!error && (ix > 0)))
            {
                error -= delta_x;
                y1 += iy;
            }
            // Else do nothing

            error += delta_y;
            x1 += ix;

            if (grid(x1, y1).scalar<int>() == OBSTACLE) return;
            grid(x1, y1) = FREE;
        }
    }
    else
    {
        // Error may go below zero
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2)
        {
            // Reduce error, while taking into account the corner case of error == 0
            if ((error > 0) || (!error && (iy > 0)))
            {
                error -= delta_y;
                x1 += ix;
            }
            // Else do nothing

            error += delta_x;
            y1 += iy;

            if (grid(x1, y1).scalar<int>() == OBSTACLE) return;
            grid(x1, y1) = FREE;
        }
    }
}


/* Macro to instantiate all template functions for a given type */
#define INSTANTIATE_TEMPLATES(TYPE)                                         \
    template Cell GridMap::cellCoordsFromLocal<TYPE>(TYPE x, TYPE y);       \
    template Point<TYPE> GridMap::localCoordsFromCell(size_t x, size_t y);  \
    template af::array::array_proxy GridMap::cellFromLocal(TYPE x, TYPE y);


INSTANTIATE_TEMPLATES(char)
INSTANTIATE_TEMPLATES(unsigned char)
INSTANTIATE_TEMPLATES(short)
INSTANTIATE_TEMPLATES(unsigned short)
INSTANTIATE_TEMPLATES(int)
INSTANTIATE_TEMPLATES(unsigned int)
INSTANTIATE_TEMPLATES(long)
INSTANTIATE_TEMPLATES(unsigned long)
INSTANTIATE_TEMPLATES(long long)
INSTANTIATE_TEMPLATES(unsigned long long)
INSTANTIATE_TEMPLATES(float)
INSTANTIATE_TEMPLATES(double)
INSTANTIATE_TEMPLATES(long double)


} // hypergrid namespace
