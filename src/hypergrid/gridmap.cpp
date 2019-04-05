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
/*Works only if cellsize  > output_cell_size*/

void GridMap::resize(double output_cell_size)
{
    grid = af::resize(cell_size_/output_cell_size, grid,  AF_INTERP_LOWER);
    cell_size_ = output_cell_size;
}

/* Apply a rotation to the map */
/*not finished*/
void GridMap::rotate(double angle)
{
    /*Original  Dimension*/
    long dim0 = grid.dims(0) ;
    long dim1 = grid.dims(1) ;
    /*adding grid.dim(0) rows to each side then adding grid.dim(1) columns to each side to modfiy rotate function*/
    af::array bigger0 = af::constant((int)Label::UNKNOWN, grid.dims(0), grid.dims(1), s32);
    af::array bigger1 = af::constant((int)Label::UNKNOWN, grid.dims(0)*3, grid.dims(1), s32);
    //rows
    grid = af::join(0,grid,bigger0);
    grid = af::join(0,bigger0,grid);
    //columns
    grid = af::join(1,grid,bigger1);
    grid = af::join(1,bigger1,grid);
    af_print(grid);
    af_print(grid(6,5));
    af::array temp(grid.dims(0), dim1, s32);

  /*  af::array B(3,3);
    gfor(af::seq ii ,0,2) {
         af::array A = af::randu(3,1);
            B(af::span,ii) = A;
            }
    af_print(B);*/

    //af_print(af::seq (dim0, grid.dims(0)-dim0 ));
    for(int i = 4; i <= 7; i++)
    {
        af_print(grid(af::span, i));
        temp(af::span,i-dim1) = grid(af::span, i) * 1;

    }

    af::array temp2(dim0, dim1, s32);

    for(int i = 6; i <= 11; i++)
    {
        af_print(grid(af::span, i));
        temp2(i-6,af::span) = temp(i,af::span) * 1;

    }
    
    af_print(temp2) ;
    
    
    //af_print(temp);
    //rotate
    //grid = af::rotate(grid, angle,true);
   // af::array temp
   // grid = grid()
    // TODO
}

/* Clear the map and set all cells to UNKNOWN */
/*DONE*/
void GridMap::clear()
{
    grid = UNKNOWN ;
}


/* Cell coordinates from local coordinates */
/*TODO : Only works with double, needs more modification*/ 
template<typename T>
Cell GridMap::cellCoordsFromLocal(T x, T y)
{
    Point<T> t_point = originFromLocal_(Point<T> (x,y));
    Cell A = Point<size_t> (floor(t_point.x/cell_size_),floor(t_point.y/cell_size_));
    return A ;
    
}

/* Local / global coordinates from cell coords */
template<typename T>
Point<T> GridMap::localCoordsFromCell(size_t x, size_t y)
{
    // TODO
}

/* Cell access from local / global coordinates */
template<typename T>
int32_t GridMap::cellFromLocal(T x, T y)
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
    T src_arr[] = {src.x, src.y, 1};
    af::array af_src(3, 1,src_arr);
    af::array t_src = af::matmul(af::inverse(getOriginTransform_()),  af_src);
    af_print(t_src);
    return Point<T> (t_src(0).scalar<T>(), t_src(1).scalar<T>());
    // TODO
}

/* Apply the map origin transformation to get the local point from a cell point (in meters) */
template<typename T>
Point<T> GridMap::localFromOrigin_(Point<T> src) const
{
    // TODO
}

/*Getting the Origin Transform matrix*/
af::array GridMap::getOriginTransform_() const
{
    // getting Theta
    double x_diff = origin_.orientation.x - 0 ;
    double y_diff = origin_.orientation.y - 0 ;
    double theta = 0 ;
    if(x_diff!=0) theta = atan(y_diff/x_diff);
    
    //creating the translation and rotation matrix 2D
    double t_r_array[] = {cos(theta), sin(theta), 0, 
                        -sin(theta), cos(theta), 0,
                        origin_.position.x, origin_.position.y, 1};
    af::array t_r_matrix(3,3,t_r_array);
    return t_r_matrix;
        
}

//template Cell GridMap::cellCoordsFromLocal<int>(int x, int y);
template Cell GridMap::cellCoordsFromLocal<double>(double x, double y);
//template Cell GridMap::cellCoordsFromLocal<size_t>(size_t x,size_t y);

} // hypergrid namespace

