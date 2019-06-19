#include <hypergrid/conversions/kinect_converter.hpp>

#define PI 3.14159265


namespace hypergrid
{

/* Constructor */
KINECTConverter::KINECTConverter(double width, double height, double cell_size,
                                geometry_msgs::Pose origin, std::string map_frame_id,
                                double max_height, bool DEBUG,double dis_VW) :
    origin_(origin)
{
    width_ = width;
    height_ = height;
    cell_size_ = cell_size;
    map_frame_id_ = map_frame_id;
    max_height_ = max_height;
    DEBUG_ = DEBUG;
    dis_VW_ = dis_VW;

    // Segmentation parameters
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PARALLEL_PLANE); //Plane parallel to the plane of the camera
    seg_.setEpsAngle(conver_rad(20)); // It is estimated that the ground will form 20gra with kinect
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setDistanceThreshold(0.1); // TODO: Change this to be more parameterizable
   


}

/* Convert a sensor_msgs::PointCloud2 to a hypergrid::GridMap */
GridMap KINECTConverter::convert(sensor_msgs::PointCloud2& cloud_msg, const tf::StampedTransform transform) 
{
    // Convert to PCL
    if (DEBUG_) std::cout << "\n------------------------\nNew Kinect" << std::endl;

    auto start = std::chrono::high_resolution_clock::now(); 
  
    // unsync the I/O of C and C++. 
    std::ios_base::sync_with_stdio(false); 

    pcl::PCLPointCloud2 new_pcl;
    pcl_conversions::toPCL(cloud_msg, new_pcl);

    // Convert to XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud in XYZ
    pcl::fromPCLPointCloud2(new_pcl, *cloud_xyz);

    // Filter to eliminate noise
    std::vector<int> removed_indexes;
    pcl::removeNaNFromPointCloud(*cloud_xyz, *cloud_xyz, removed_indexes);//element NaN values
   
    // Transform the cloud coordinates to the base_footprint given the tf
    pcl_ros::transformPointCloud(*cloud_xyz, *cloud_xyz, transform);
    cloud_xyz->header.frame_id = map_frame_id_;

    // Remove the first 90 cm
    float min_x_distance = - 0.9 + transform.getOrigin().getX();
    // Remove points outside the map
    float max_x_distance = - width_;
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
    seg_.setInputCloud(cloud_xyz);
    seg_.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        //return null;
    }

    // Filter cloud to eliminate detected ground points
    pcl::ExtractIndices<pcl::PointXYZ> filtered_clouds;
    filtered_clouds.setInputCloud(cloud_xyz); // pass points to be filtered
    filtered_clouds.setIndices(inliers); // pass indices to be filtered
    filtered_clouds.setNegative(true);
    filtered_clouds.filter(*cloud_xyz);

   
    // Filter point cloud using RadiusOutlier filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // Build the filter
    outrem.setInputCloud(cloud_xyz);
    outrem.setRadiusSearch(0.03);
    outrem.setMinNeighborsInRadius(5);
    // Apply filter
    outrem.filter(*cloud_xyz);

    //cloud_map_pub_pcl.publish(*cloud_xyz);
    



    hypergrid::GridMap gridmap(width_, height_, cell_size_, origin_, map_frame_id_);

    tf::Vector3 trans = transform.getOrigin();   

    
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
                                && pcl_points(af::span, 2) < max_height_ 
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
    int length = round ( 2 * ( dis_VW_ * tan( PI * 30 /180) ) );
    //getting the poistion of VW in x axis in cell cords
    int vw_x = (width_/cell_size_) - dis_VW_/cell_size_;


    af::array vw = af::constant(vw_x, length, 2, s32);
    int start_vw = round( std::abs( ( (height_/cell_size_) / 2) - length/2 )  ) ;
    int end_vw = start_vw + length ;
    vw(af::span, 1) = af::seq(start_vw, end_vw -1);
   
    //adding free lines to the VW
    gridmap.addFreeLines(start_cell, vw);


    // gfor(af::seq i, start_vw, start_vw + length-1 )
    // {
    //     gridmap.grid(vw_x, i) = 100;
    // }

    // Publish OccupancyGrid map


    if (DEBUG_) {std::cout << "Kinect converter: " << std::fixed  
            << (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count()) * 1e-9
            << std::setprecision(9); 
            std::cout << " sec" << std::endl; 
            start = std::chrono::high_resolution_clock::now(); 
        } 

   

    return gridmap;

}

GridMap KINECTConverter::convert( sensor_msgs::PointCloud2Ptr& cloud_msg, const tf::StampedTransform transform) 
{
    return convert(*cloud_msg, transform);
}

/*
* Function CONVER_RAD - Convert grades in radians
*/
double KINECTConverter::conver_rad(double grados)
{
    return grados * (M_PI / 180);
}


}