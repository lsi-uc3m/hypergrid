#include <hypergrid/gridmap.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

//#include <image_transport/image_transport.h>


using namespace std;

double map_height;
double map_width;
double cell_size;
bool DEBUG;

ros::Publisher laser_gridmap_pub;

// Laser frame to base_footprint tf transform
string map_frame_id, frame_vehicle;
tf::TransformListener* tf_listener;

void laser_callback(const sensor_msgs::LaserScanConstPtr scan){

    ROS_INFO("Callback");
    // Update the laser transform
    tf::StampedTransform laser_footprint_transform;
    try
    {
        tf_listener->lookupTransform(frame_vehicle + "/" + map_frame_id, scan->header.frame_id, ros::Time(0), laser_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    geometry_msgs::Pose origin;
    origin.position.x = - (map_width /2);
    origin.position.y = - (map_height /2);

    hypergrid::GridMap gridmap(map_width, map_height, cell_size, origin);

    if (DEBUG) cout << "\n------------------------\nNew laser\n";
    
    af::array ranges(scan->ranges.size(), 1, scan->ranges.data());
    af::array scan_angle(scan->ranges.size(), 1);

    float angle_min = scan->angle_min;
    float angle_inc = scan->angle_increment;
    float range_max = scan->angle_max;
    
    af::seq a(0,scan->ranges.size());
    af::array i = a ;
    
    scan_angle = angle_min + i * angle_inc ;
    
    
    af::array obstacle_x (scan->ranges.size(), 1);
    af::array obstacle_y (scan->ranges.size(), 1);

    af::array ranges_nan_inf = af::isNaN(ranges) + af::isInf(ranges);    

    gfor(af::seq i, scan_angle.dims(0))
    {
        af::array condition = (ranges_nan_inf(i).scalar<int>()==1); // good
        ranges(i) = (!condition).as(f32) * ranges(i)  
                    + condition.as(f32) * range_max;
    }

    obstacle_x =  ranges * af::cos(scan_angle);
    obstacle_y =  ranges * af::sin(scan_angle);
    
    //update obstacle_x and obstacle_y with tf 
    tf::Vector3 translate = laser_footprint_transform.getOrigin();
    tf::Quaternion rotate = laser_footprint_transform.getRotation();
    // Get Theta
    double theta = tf::getYaw(rotate);
    af::array temp = obstacle_x * cos(theta) - obstacle_y * sin(theta) + translate.getX();
    obstacle_y = obstacle_x * sin(theta) + obstacle_y * cos(theta) + translate.getY();
    obstacle_x = temp ;

    // Set the obstacle in the map
    gfor(af::seq i, scan->ranges.size()){
        hypergrid::Cell obs_coords = gridmap.cellCoordsFromLocal(obstacle_x(i).scalar<double>(), obstacle_y(i).scalar<double>());
        // Check if the obstacle coords lay inside the map before modifying the cell
        if (gridmap.isCellInside(obs_coords)) gridmap.grid(obs_coords.x, obs_coords.y) = hypergrid::GridMap::UNKNOWN;
    }

    // Add the free lines to all obstacles
    gfor(af::seq i, scan->ranges.size())
    {
        gridmap.addFreeLine(obstacle_x(i).scalar<double>(),obstacle_y(i).scalar<double>());
    }

    // Publish OccupancyGrid map
    laser_gridmap_pub.publish(gridmap.toMapMsg());
    
}

int main(int argc, char **argv)
{
    af::info();

    ros::init(argc, argv, "laser_to_gridmap");
    ros::NodeHandle public_nh, private_nh("~");
    ROS_INFO("Starting laser to gridmap");
    string laser_topic;
    string output_topic;
    private_nh.param<string>("laser_topic", laser_topic, "/icab1/scan");
    private_nh.param<string>("output_topic", output_topic, "laser_to_gridmap");
    private_nh.param<string>("map_frame_id", map_frame_id, "/icab1/base_footprint");
    private_nh.param("height", map_height, 150.0);
    private_nh.param("width", map_width, 100.0);
    private_nh.param("cell_size", cell_size, 0.1);
    private_nh.param("DEBUG", DEBUG, true);
    public_nh.param<std::string>("vehicle", frame_vehicle, "");

    // ROS_INFO("Listening to %s", laser_topic);   

    laser_gridmap_pub = public_nh.advertise<nav_msgs::OccupancyGrid>("hypergrid/" + output_topic, 2);
    ros::Subscriber laser_sub = public_nh.subscribe(laser_topic, 5, laser_callback);

    tf_listener = new tf::TransformListener;
    ROS_INFO("Ready");

    ros::spin();
   
   
   
    ros::Rate r(2.0);
   /* while(ros::ok()){
        ROS_INFO("Loop");
        r.sleep();
        ros::spinOnce();
        //if(DEBUG)
        
    }*/
    

    return 0;
}
