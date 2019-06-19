#include <hypergrid/conversions/kinect_converter.hpp>


// Map params
double map_height;
double map_width;
double cell_size;
bool DEBUG;


// Maximum height of obstacles. Everything higher will out of the map
double max_height;

// Distance of virtual wall
double dis_VW;

// Laser frame to base_footprint tf transform
std::string map_frame_id;
std::string cloud_topic;
std::string output_topic;
ros::Publisher cloud_map_pub;
ros::Publisher cloud_map_pub_pcl;

// Transform Listener 
tf::TransformListener* tf_listener;


/*
* Function CONVER_RAD - Convert grades in radians
*/
double conver_rad(double grados)
{
    return grados * (M_PI / 180);
}




void kinect_callback(sensor_msgs::PointCloud2Ptr cloud_msg)
{
   
    // Convert to PCL
    if (DEBUG) std::cout << "\n------------------------\nNew Kinect" << std::endl;

    auto start = std::chrono::high_resolution_clock::now(); 
  
    // unsync the I/O of C and C++. 
    std::ios_base::sync_with_stdio(false); 

    
    tf::StampedTransform kinect_footprint_transform;
    try
    {
        tf_listener->lookupTransform(map_frame_id, cloud_msg->header.frame_id , ros::Time(0), kinect_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("Warning: %s", ex.what());
    }

    geometry_msgs::Pose origin;
    origin.position.x = - map_width;
    origin.position.y = - (map_height / 2) ;
    origin.orientation.w = 1;

     hypergrid::KINECTConverter kinect_converter(map_width, map_height, cell_size,
                                              origin,
                                              map_frame_id,
                                              max_height,
                                              DEBUG,
                                              dis_VW);

    hypergrid::GridMap gridmap = kinect_converter.convert(cloud_msg, kinect_footprint_transform);

    
    // Publish OccupancyGrid map
    cloud_map_pub.publish(gridmap.toMapMsg());


    if (DEBUG) {std::cout << "Kinect callback: " << std::fixed  
            << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
            << std::setprecision(9); 
            std::cout << " sec" << std::endl; 
            start = std::chrono::high_resolution_clock::now(); 
        } 

   
    
}



int main(int argc, char  **argv)
{
    af::info();

    ros::init(argc, argv, "kinect_to_gridmap");
    ros::NodeHandle nh, priv_nh("~");

    
    priv_nh.param<std::string>("map_frame_id", map_frame_id, "base_footprint");
    priv_nh.param("height", map_height, 50.0);
    priv_nh.param("width", map_width, 50.0);
    priv_nh.param("cell_size", cell_size, 0.2);
    priv_nh.param("max_height", max_height, 2.5);
    priv_nh.param("DEBUG", DEBUG, true);
    priv_nh.param("dis_VW", dis_VW, 6.0);

    cloud_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/kinect_to_gridmap", 2);
    cloud_map_pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("hypergrid/kinect_to_gridmap_DEBUG", 2);
    ros::Subscriber cloud_sub = nh.subscribe("kinect2/sd/points", 5, kinect_callback);

   

    tf_listener = new tf::TransformListener();

    ros::Rate r(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
