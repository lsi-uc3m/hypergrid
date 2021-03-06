#include <hypergrid/gridmap.hpp>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    af::info();

    ros::init(argc, argv, "gridmap_test");
    ros::NodeHandle public_nh, private_nh("~");

    ros::Publisher map_pub = public_nh.advertise<nav_msgs::OccupancyGrid>("gridmap_test", 5);
    ros::Publisher map_pub_2 = public_nh.advertise<nav_msgs::OccupancyGrid>("gridmap_test_2", 5);

    af::array test_rand_array = af::randu(10, 10, f32);
    // af_print(test_rand_array);

    double width = 28.0;
    double height = 22.0;
    double cell_size = 2.0;
    geometry_msgs::Pose origin;
    origin.position.x = - (width /2);
    origin.position.y = - (height /2);

    hypergrid::GridMap gridmap(width, height, cell_size, origin);
    hypergrid::GridMap gridmap2(width, height, cell_size, origin);

    // Set some cells
    /*
    gridmap.grid(0, 0) = hypergrid::GridMap::FREE;
    gridmap.grid(4, 1) = hypergrid::GridMap::FREE;
    gridmap.grid(0, 1) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(1, 1) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(2, 1) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(5, 3) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(0, 3) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(0, 2) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(1, 3) = hypergrid::GridMap::OBSTACLE;
    gridmap.grid(1, 2) = hypergrid::GridMap::OBSTACLE;
    */
    // hypergrid::Cell c = gridmap.cellCoordsFromLocal(1, -1);


    //obstacles = af::join(0,obesracle_af,obstacles);

    // gridmap.grid(c.x,c.y) = hypergrid::GridMap::OBSTACLE;
    // gridmap.grid(6, 7) = hypergrid::GridMap::OBSTACLE;

    // af_print(gridmap.grid);


    float a[] = {1,2,3,4};
    af::array a_arr(4,a);

    float b[] = {0,5,6,7};
    af::array b_arr(4,b);

    af_print(af::min(a_arr,b_arr));


    gridmap.addFreeLine(gridmap.cellCoordsFromLocal(hypergrid::Point<double>(10, 5)));
    gridmap.addFreeLine(gridmap.cellCoordsFromLocal(hypergrid::Point<double>(10, -10)));
    gridmap.addFreeLine(gridmap.cellCoordsFromLocal(hypergrid::Point<double>(-5, 10)));
    gridmap.addFreeLine(gridmap.cellCoordsFromLocal(hypergrid::Point<double>(-10, -10)));
    gridmap.addFreeLine(gridmap.cellCoordsFromLocal(hypergrid::Point<double>(-11, -9)));

    gridmap.cellFromLocal(hypergrid::Point<double>(10, 5)) = hypergrid::GridMap::OBSTACLE;
    gridmap.cellFromLocal(hypergrid::Point<double>(10, -10)) = hypergrid::GridMap::OBSTACLE;
    gridmap.cellFromLocal(hypergrid::Point<double>(-5, 10)) = hypergrid::GridMap::OBSTACLE;
    gridmap.cellFromLocal(hypergrid::Point<double>(-10, -10)) = hypergrid::GridMap::OBSTACLE;
    gridmap.cellFromLocal(hypergrid::Point<double>(-11, -9)) = hypergrid::GridMap::OBSTACLE;


    // hypergrid::Cell start = gridmap2.cellCoordsFromLocal(0, 0);
    // hypergrid::Cell end = gridmap2.cellCoordsFromLocal(10, 5);

    // hypergrid::LineIterator it(gridmap2.grid, start, end, 4);

    // for (int i = 0; i < it.size(); i++, it++)
    // {
    //     // std::cout << "Iterator pos: " << it().str() << std::endl;
    //     gridmap2[it()] = hypergrid::GridMap::FREE;
    // }
    // draw_line_local(gridmap2, hypergrid::Point<double>(0, 0), hypergrid::Point<double>(10, 5));
    // draw_line_local(gridmap2, hypergrid::Point<double>(0, 0), hypergrid::Point<double>(10, -10));
    // draw_line_local(gridmap2, hypergrid::Point<double>(0, 0), hypergrid::Point<double>(-5, 10));
    // draw_line_local(gridmap2, hypergrid::Point<double>(0, 0), hypergrid::Point<double>(-10, -10));
    // draw_line_local(gridmap2, hypergrid::Point<double>(0, 0), hypergrid::Point<double>(-11, -9));

    hypergrid::Cell start_c = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(0, 0));
    af::array endpoints(5, 2, s32);
    endpoints(0, 0) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(10, 5)).x;
    endpoints(0, 1) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(10, 5)).y;
    endpoints(1, 0) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(10, -10)).x;
    endpoints(1, 1) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(10, -10)).y;
    endpoints(2, 0) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(-5, 10)).x;
    endpoints(2, 1) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(-5, 10)).y;
    endpoints(3, 0) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(-10, -10)).x;
    endpoints(3, 1) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(-10, -10)).y;
    endpoints(4, 0) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(-11, -9)).x;
    endpoints(4, 1) = gridmap2.cellCoordsFromLocal(hypergrid::Point<double>(-11, -9)).y;
    af_print(endpoints);

    hypergrid::add_lines(gridmap2.grid, hypergrid::GridMap::FREE, start_c.x, start_c.y, endpoints);

    gridmap2.cellFromLocal(hypergrid::Point<double>(10, 5)) = hypergrid::GridMap::OBSTACLE;
    gridmap2.cellFromLocal(hypergrid::Point<double>(10, -10)) = hypergrid::GridMap::OBSTACLE;
    gridmap2.cellFromLocal(hypergrid::Point<double>(-5, 10)) = hypergrid::GridMap::OBSTACLE;
    gridmap2.cellFromLocal(hypergrid::Point<double>(-10, -10)) = hypergrid::GridMap::OBSTACLE;
    gridmap2.cellFromLocal(hypergrid::Point<double>(-11, -9)) = hypergrid::GridMap::OBSTACLE;

    af_print(gridmap2.grid);
    // int example1[] = {1,2,3,4,5,6,7,8,9};
    // int example2[] = {11,22,33,44,55,66,77,88,99};
    // int example3[] = {111,222,333,444,555,666,777,888,999};

    // af::array join1(3,3,example1);
    // af::array join2(3,3,example2);
    // af::array join3(3,3,example3);

    // af_print(af::join(0,join1,join2));
    // af_print(af::join(1,join1,join2));
    // af_print(af::join(2,join1,join2));

    /*
    gfor(af::seq i ,2,2 ,join1.dims(0))
    {
        af_print(join1(i));
    }
    */

    // gridmap.clear();

    // std::cout << "Local [1, 1] -> Cell coords " << gridmap.cellCoordsFromLocal(1.0, 1.0).str() << std::endl;
    // std::cout << "Local [1, 1] cell: " << gridmap.cellFromLocal(1.0, 1.0).scalar<int32_t>() << std::endl;

    // size_t a = 4;
    // size_t b = 3;
    // c = hypergrid::Cell(a, b);
    // hypergrid::Point<double> p = gridmap.localCoordsFromCell<double>(a, b);
    // std::cout << "Cell coords [4, 3] -> Local " << gridmap.localCoordsFromCell<double>(a, b).str() << std::endl;

    // Test direct cell access
    // std::cout << "Cell [4, 3]: " << gridmap[c].scalar<int32_t>() << std::endl;
    // std::cout << "Cell [4, 3]: " << gridmap.cell(c).scalar<int32_t>() << std::endl;
    // std::cout << "Cell [4, 3]: " << gridmap.cell(a, b).scalar<int32_t>() << std::endl;
    // std::cout << "Cell from local [1.5, 1.5]: " << gridmap.cellFromLocal(1.5, 1.5).scalar<int32_t>() << std::endl;

    // Direct cell modification
    // gridmap[c] = hypergrid::GridMap::FREE;
    // gridmap.cellFromLocal(1.5, 1.5) = hypergrid::GridMap::FREE;
    // std::cout << "Cell [4, 3]: " << gridmap[c].scalar<int32_t>() << std::endl;
    // std::cout << "Cell from local [1.5, 1.5]: " << gridmap.cellFromLocal(1.5, 1.5).scalar<int32_t>() << std::endl;

    // gridmap.rotate(af::Pi/4); 
    // gridmap.resize(1);
   
    // hypergrid::Cell z(3,2);
    // gridmap.localCoordsFromCell(a,b);

    nav_msgs::OccupancyGrid map_msg = gridmap.toMapMsg();
    nav_msgs::OccupancyGrid map_msg2 = gridmap2.toMapMsg();
   
    // hypergrid::GridMap converted_grid(map_msg);
    // af_print(converted_grid.grid);

    ros::Rate rate(1);
    while(ros::ok())
    {
        map_pub.publish(map_msg);
        map_pub_2.publish(map_msg2);
        rate.sleep();
    }

    return 0;
}