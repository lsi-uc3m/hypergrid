#include <hypergrid/gridmap.hpp>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    af::info();

    ros::init(argc, argv, "coloc");
    ros::NodeHandle public_nh, private_nh("~");

    ros::Publisher map_pub = public_nh.advertise<nav_msgs::OccupancyGrid>("gridmap_test", 5);

    af::array test_rand_array = af::randu(10, 10, f32);
    af_print(test_rand_array);

    double width = 10.0;
    double height = 8.0;
    double cell_size = 2.0;
    geometry_msgs::Pose origin;
    origin.position.x = - (width / cell_size);
    origin.position.y = - (height / cell_size);

    hypergrid::GridMap gridmap(width, height, cell_size, origin);


    // Set some cells
    gridmap.grid(0, 0) = hypergrid::GridMap::FREE;
    gridmap.grid(2, 1) = hypergrid::GridMap::OBSTACLE;
    af_print(gridmap.grid);

    nav_msgs::OccupancyGrid map_msg = gridmap.toMapMsg();

    hypergrid::GridMap converted_grid(map_msg);
    af_print(converted_grid.grid);

    ros::Rate rate(1);
    while(ros::ok())
    {
        map_pub.publish(map_msg);
        rate.sleep();
    }

    return 0;
}