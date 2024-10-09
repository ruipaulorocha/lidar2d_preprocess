# lidar2d_preprocess
ROS2 package to preprocess readings from a 360 deg. FoV 2D LiDAR occluded by poles attached to the robot chassis.

It was originally developed by Rui P. Rocha for ROS1 Noetic distribution and has been migrated to ROS2 Jazzy distribution through this version.

#### Description
The package `lidar2d_preprocess` contains a node with the same name that discards from a laser rangefinder (LRF) scan readings related with robot's physical artifacts located near the sensor, such as poles attached to the robot chassis. Readings to be discarded are specified through a int array parameter; each pair of integer numbers specify an interval of reading indexes to be discarded.

There is an auxiliary node (`distance_histogram`) that helps identifying the indexes of the LRF scan corresponding to readings that should be discarded (small distances related to the aforementioned robot's artifacts).

When a reading is discarded, its distance is set to the constant `Inf` (a very large distance).

If intensity values are available, the intensity associated with a discarded measured distance is set to zero.

#### Nodes

##### `lidar2d_preprocess`

Pre-process the LRF scan by discarding undesirable readings.

###### Subscribed Topics
- `<in_topic>` (`sensor_msgs::msg::LaserScan` message type)
    - Input raw data from LRF to be pre-processed.

###### Published Topics
- `<out_topic>` (`sensor_msgs::msg::LaserScan` message type)
    - Output data after pre-processing.


##### Parameters
- `in_topic` (string, default: `scan_in`)
    - Topic with input raw data from LRF to be subscribed.
- `outp_topic` (string, default: `scan_out`)
    - Topic where LRF pre-processed data is published.    
- `indexes` (integer array, default: `[820, 825, 1142, 1151]`)
    - Indexes array that defines which LRF readings are discarded. The default value means 2 ranges: readings between indexes 820 and 825 and readings between indexes 1142 and 1151. If the size of the array is odd, the last index of the array is ignored. If the array is empty, the output scan becomes equal to the input scan (no pre-processing is done).



##### `distance_histogram`

The node prints in the console, for every scan, the number of readings less or equal to the 1st threshold (0.35 for the default value of `distance_th` parameter), correspondig to the right-most histogram bin and potentially to readings that should be discarded (the threshold may be adjusted through this parameter). The node also displays the remainder histogram values for every scan.

###### Subscribed Topics
- `<topic>` (`sensor_msgs::msg::LaserScan` message type)
    - Input raw data from LRF that needs to be pre-processed through the main node (`lidar2d_preprocess`).


##### Parameters
- `topic` (string, default: `scan_in`)
    - Topic with input raw data from LRF to be subscribed.  
- `distance_th` (double array, default: `[0.35, 1.0, 4.0, 25.0]`)
    - Distance thresholds that are used to configure a histogram of distances measured by the LRF. The default value represents a histogram with 5 bins: [0, 0.35[, [0.35, 1.0[, [1.0, 4.0[, [4.0, 25.0[, and [25.0, Inf[.