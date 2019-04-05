#include <hypergrid/gridmap.hpp>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    af::info();

    ros::init(argc, argv, "gridmap_test");
    ros::NodeHandle public_nh, private_nh("~");

    ros::Publisher map_pub = public_nh.advertise<nav_msgs::OccupancyGrid>("gridmap_test", 5);
    ros::Publisher map_pub_resize = public_nh.advertise<nav_msgs::OccupancyGrid>("gridmap_test_resized", 5);

    af::array test_rand_array = af::randu(10, 10, f32);
    af_print(test_rand_array);

    double width = 12.0;
    double height = 8.0;
    double cell_size = 2.0;
    geometry_msgs::Pose origin;
    origin.position.x = - (width /2);
    origin.position.y = - (height /2);

    hypergrid::GridMap gridmap(width, height, cell_size, origin);

    // Set some cells
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
    af_print(gridmap.grid);
    int example1[] = {1,2,3,4,5,6,7,8,9};
    int example2[] = {11,22,33,44,55,66,77,88,99};
    int example3[] = {111,222,333,444,555,666,777,888,999};

    af::array join1(3,3,example1);
    af::array join2(3,3,example2);
    af::array join3(3,3,example3);

    //af_print(af::join(0,join1,join2));
    //af_print(af::join(1,join1,join2));
    //af_print(af::join(2,join1,join2));

   /* gfor(af::seq i ,2,2 ,join1.dims(0)){

    
        af_print(join1(i));
    }*/
    
    
    
    
    // gridmap.clear();

    /*
    double x =1.0  ;
    double y =2.0  ;
    printf("x:                  %lu\n y:             %lu\n",gridmap.cellCoordsFromLocal(x,y).x, gridmap.cellCoordsFromLocal(x,y).y );
    */
    nav_msgs::OccupancyGrid map_msg = gridmap.toMapMsg();

    gridmap.rotate(af::Pi/4); 
    //gridmap.resize(1);

    nav_msgs::OccupancyGrid map_msg2 = gridmap.toMapMsg(); //if you want to publish the original and resized map
   // std::cout << "Local [1, 1] -> Cell coords " << gridmap.cellCoordsFromLocal(1.0, 1.0).str() << std::endl;


   
    hypergrid::GridMap converted_grid(map_msg);
    af_print(converted_grid.grid);

    ros::Rate rate(1);
    while(ros::ok())
    {
        map_pub.publish(map_msg);
        map_pub_resize.publish(map_msg2);
        rate.sleep();
    }

    return 0;
}