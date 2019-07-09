# Hypergrid

Hypergrid is a ROS package for building local maps from multiple sensors in the blink of an eye. Hypergrid relies on [Arrayfire](https://github.com/arrayfire/arrayfire), in order to parallelize the computation.

## Dependencies

* [Arrayfire](https://github.com/arrayfire/arrayfire)


## What's inside

### Library
Hypergrid is packaged with catkin and provides several libraries to work with grid maps and to convert the raw sensor data to this format.

### Costmap Plugin
The main contribution of this package is a [costmap_2d](http://wiki.ros.org/costmap_2d/) plugin, meant to replace the default *obstacle_layer* inside the *local_costmap* of *move_base*. The typical *local_costmap_params.yaml* configuration file can be configured as follows:

```yaml
local_costmap:
  
  # ...
  
  plugins:
   - {name: hypergrid_layer,          type: "hypergrid::HypergridLayer"}
  
  hypergrid_layer:
    # LaserScan input topics
    laser_topics: ["icab1/scan"]
    # LiDAR PointCloud2 input topics
    lidar_topics: ["icab1/velodyne_points"]
    # Frame of the output local map
    map_frame_id: icab1/base_footprint
    # Map size (in meters)
    width: 100
    height: 100
    # Map resolution (in meters / cell)
    cell_size: 0.1
    # Remove floor points in pointcloud with heightmap algorithm
    floor_filter: true
    # Heightmap algorithm parameters
    heightmap_threshold: 0.15
    heightmap_cell_size: 0.25
    # Remove all points higher than max_height
    max_height: 2.5
    # Remove all points inside the vehicle box
    # All points with |x| < vehicle_box_size or |y| < vehicle_box_size will be filtered.
    vehicle_box_size: 2.0
    # Enable debug console output
    DEBUG: true
```


### Nodes
In addition to the library and the costmap_2d plugin, Hypergrid also includes stand-alone conversion nodes, which subscribe to the sensor messages and publish the output instantaneous local map.

* laser_to_gridmap: Conversion node from [LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html) msgs to [OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) maps.

* pcl_to_gridmap: Conversion node from LiDAR [PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html) msgs to [OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) maps.

The launch and configuration files are inside the [launch](launch) and [config](config) directories, respectively.

