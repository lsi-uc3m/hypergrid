#include <hypergrid/conversions/kinect_converter.hpp>

#define PI 3.14159265

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

// Segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

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

    pcl::PCLPointCloud2 new_pcl;
    pcl_conversions::toPCL(*cloud_msg, new_pcl);

    // Convert to XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud in XYZ
    pcl::fromPCLPointCloud2(new_pcl, *cloud_xyz);

    // Filter to eliminate noise
    std::vector<int> removed_indexes;
    pcl::removeNaNFromPointCloud(*cloud_xyz, *cloud_xyz, removed_indexes);//element NaN values

    ros::Time t0 = ros::Time::now();
    ros::Time t_start = t0;
   
    tf::StampedTransform kinect_footprint_transform;
    try
    {
        tf_listener->lookupTransform(map_frame_id, cloud_xyz->header.frame_id , ros::Time(0), kinect_footprint_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("Warning: %s", ex.what());
    }

    // Transform the cloud coordinates to the base_footprint given the tf
    pcl_ros::transformPointCloud(*cloud_xyz, *cloud_xyz, kinect_footprint_transform);
    cloud_xyz->header.frame_id = map_frame_id;

    // Remove the first 90 cm
    float min_x_distance = - 0.9 + kinect_footprint_transform.getOrigin().getX();
    // Remove points outside the map
    float max_x_distance = - map_width;
    cloud_xyz->points.erase(std::remove_if(cloud_xyz->points.begin(),
                                           cloud_xyz->points.end(),
                                           [&](const pcl::PointXYZ& p)
                                           {
                                               return (p.x > min_x_distance) ||
                                                      (p.x < max_x_distance);
                                           }),
                                           cloud_xyz->points.end());

                                    
    // Ground plane coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Segment floor plane
    seg.setInputCloud(cloud_xyz);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    // Filter cloud to eliminate detected ground points
    pcl::ExtractIndices<pcl::PointXYZ> filtered_clouds;
    filtered_clouds.setInputCloud(cloud_xyz); // pass points to be filtered
    filtered_clouds.setIndices(inliers); // pass indices to be filtered
    filtered_clouds.setNegative(true);
    filtered_clouds.filter(*cloud_xyz);

    cloud_map_pub_pcl.publish(*cloud_xyz);
    
    //std::cout << "size : " << size << std::endl;    
    // Filter point cloud using RadiusOutlier filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // Build the filter
    outrem.setInputCloud(cloud_xyz);
    outrem.setRadiusSearch(0.03);
    outrem.setMinNeighborsInRadius(5);
    // Apply filter
    outrem.filter(*cloud_xyz);

    cloud_map_pub_pcl.publish(*cloud_xyz);
    


    geometry_msgs::Pose origin;
    origin.position.x = - map_width;
    origin.position.y = - (map_height / 2) ;
    origin.orientation.w = 1;


    hypergrid::GridMap gridmap(map_width, map_height, cell_size, origin, map_frame_id);

    tf::Vector3 trans = kinect_footprint_transform.getOrigin();   

    
    // Set the origin of the lines at the sensor point
    hypergrid::Cell start_cell = gridmap.cellCoordsFromLocal(trans.getX(), trans.getY());

    if(cloud_xyz->points.size() != 0)
    {

        //Convert PointCloud2 to ArrayFire array
        float* data_f = reinterpret_cast<float *>(cloud_xyz->points.data());

        af::array pcl_points(4, cloud_xyz->points.size(), data_f);
        pcl_points = pcl_points(af::seq(3), af::span); 
        
        
        af::array ones = af::constant(1, 1, pcl_points.dims(1));
        af::array obs = (af::join(0,pcl_points(af::seq(2), af::span), ones(af::span, af::seq(pcl_points.dims(1) )))).as(f64) ;
           
        
        pcl_points = pcl_points.T();
        af::array obstacle_coords =  gridmap.cellCoordsFromLocal(obs.T());
        
        af::array conds = ( (!(  (af::isInf(pcl_points(af::span, 0))) || 
                                (af::isInf(pcl_points(af::span, 1))) ||
                                (af::isInf(pcl_points(af::span, 2))) ) )
                                && pcl_points(af::span, 2) < max_height 
                                && gridmap.isCellInside(obstacle_coords) ) ;
        
        //using the sort function only to get the indices array
        af::array out;
        af::array indices_inside;
        af::sort(out, indices_inside, conds, 0, false);    

        // getting only the indices of the obstacles
        indices_inside = indices_inside(af::seq(af::sum(conds).scalar<unsigned>()));

        af::array inside_obstacles_coords;
        if(indices_inside.dims(0) !=obstacle_coords.dims(0))
        {
            inside_obstacles_coords = af::lookup(obstacle_coords(af::span, af::seq(2)), indices_inside);
        }
        else
        {
            inside_obstacles_coords = obstacle_coords(af::span, af::seq(2));
        }
        //std::cout << "type: " << inside_obstacles_coords.type() << std::endl;
        // Add a free line to each obstacle inside the grid
        gridmap.addFreeLines(start_cell, inside_obstacles_coords);
        
        
        // Set the obstacles in the map
        af::array indices = inside_obstacles_coords(af::span, 1).as(s32) * gridmap.grid.dims(0) + inside_obstacles_coords(af::span, 0).as(s32);
        gridmap.grid(indices) = hypergrid::GridMap::OBSTACLE;    
    } 




    // creating the VW to add free space at the back of the car 
    //getting the length of the VW
    int length = round ( 2 * ( dis_VW * tan( PI * 30 /180) ) );
    //getting the poistion of VW in x axis in cell cords
    int vw_x = (map_width/cell_size) - dis_VW/cell_size;


    af::array vw = af::constant(vw_x, length, 2, s32);
    int start_vw = round( std::abs( ( (map_height/cell_size) / 2) - length/2 )  ) ;
    int end_vw = start_vw + length ;
    vw(af::span, 1) = af::seq(start_vw, end_vw -1);
   
    gridmap.addFreeLines(start_cell, vw);


    // gfor(af::seq i, start_vw, start_vw + length-1 )
    // {
    //     gridmap.grid(vw_x, i) = 100;
    // }

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

    // Segmentation parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE); //Plane parallel to the plane of the camera
    seg.setEpsAngle(conver_rad(20)); // It is estimated that the ground will form 20gra with kinect
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1); // TODO: Change this to be more parameterizable
   
    ros::Rate r(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
