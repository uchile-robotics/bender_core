
# uchile_laser_pipeline

It provides a preprocessing pipeline for bender laser scans sensors, by means of the ROS laser_pipeline architecture. It aims to work on the robot lasers ROS messages (sensor_msgs/LaserScan), in a filtering chain for: outlier removal, range, intensity and shadows filtering, interpolation of removed points and finally self-collision filtering.

The main contribution relates to the self-collision filter, using tf information for removal of the robot arms, which can be useful for navigation purposes.

The remaining filters are taken from the laser_filters package, and the pipeline is built on top of it.


## Laser Filters Plugins

The following plugins are compliant with the [laser_filter](http://wiki.ros.org/laser_filters) plugin architecture.

### LaserScanSelfFilter

This is a filter which removes points inside a polygon, which is obtained as a geometry_msgs::PolygonStamped from a ROS topic. It was designed to filter points inside the robot silhouette, as it a dynamic polygon which is modified when the robot moves its arms or carries something.

The filter is provided as a `laser_filter` plugin. The type is: `LaserScanSelfFilter`.

#### Parameters

Sample configuration:

```yaml
TODO
```


## Sample Usage

TODO

```bash
# filtering pipeline
$ roslaunch uchile_laser_pipeline laser_pipeline.launch
```


## Testing

Testing files are provided on the `test/` folder. The `test.launch` launchfile runs a rosbag with laser and joint state data and a rviz instance.

The rosbag file requires the following topics:

- /bender/joint_states (sensor_msgs/JointState)
- /bender/sensors/laser_front/parameter_descriptions (dynamic_reconfigure/ConfigDescription)
- /bender/sensors/laser_front/parameter_updates (dynamic_reconfigure/Config)
- /bender/sensors/laser_front/scan (sensor_msgs/LaserScan)

Then you should run:


```bash
# rosbag information
$ roslaunch uchile_laser_pipeline test.launch rosbag_file:=<rosbag_path>

# filtering pipeline
$ roslaunch uchile_laser_pipeline laser_pipeline.launch --screen
```