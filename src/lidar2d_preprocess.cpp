#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <lidar2d_preprocess/lidar2d_preprocess_Config.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <limits>
using namespace std;

const char node_name[] = "lidar2d_preprocess";

// default values of ROS node parameters
const string default_input_topic  = "scan_in";
const string default_output_topic = "scan_out";
const vector<int> default_indexes{820, 825, 1142, 1151};

class CLiDARpreprocess{
	// ROS node parameters
	string input_topic;
	string output_topic;
	vector<int> indexes;

	// other class attributes
	ros::NodeHandle private_node_handle;
	ros::Rate rate;
	ros::Subscriber laser_sub;
	ros::Publisher  laser_pub;
	dynamic_reconfigure::Server<lidar2d_preprocess::lidar2d_preprocess_Config> server;
	dynamic_reconfigure::Server<lidar2d_preprocess::lidar2d_preprocess_Config>::CallbackType f;
	int nranges;

	// callbacks
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const;
	void dynreconfCallback(lidar2d_preprocess::lidar2d_preprocess_Config &config, uint32_t level);
public:
	CLiDARpreprocess(ros::NodeHandle&);
	void run();
	void configIndexes();
};

CLiDARpreprocess::CLiDARpreprocess(ros::NodeHandle &node_handle) : private_node_handle("~"),
															   rate(10000) // rate in Hz
{
    // default values of ROS node parameters
    private_node_handle.param<string>("in_topic", input_topic, default_input_topic);
    private_node_handle.param<string>("out_topic", output_topic, default_output_topic);
	private_node_handle.param("distance_th", indexes, default_indexes);

	//read ROS node parameters
	string paramName;
	if (private_node_handle.searchParam("in_topic", paramName) )
		private_node_handle.getParam(paramName, input_topic);
	else ROS_WARN("Parameter 'in_topic' undefined");
	if (private_node_handle.searchParam("out_topic", paramName) )
		private_node_handle.getParam(paramName, output_topic);
	else ROS_WARN("Parameter 'out_topic' undefined");
	if (private_node_handle.searchParam("indexes", paramName) )
		private_node_handle.getParam(paramName, indexes);
	else ROS_WARN("Parameter 'indexes' undefined");

	this->configIndexes();

	// ROS publishers and subscribers
	ROS_INFO("Subscribe to '%s' topic", input_topic.c_str());
	laser_sub  = node_handle.subscribe<sensor_msgs::LaserScan>(input_topic.c_str(),
		10, &CLiDARpreprocess::scanCallback, this);
	ROS_INFO("Publish to '%s' topic", output_topic.c_str());
	laser_pub = node_handle.advertise<sensor_msgs::LaserScan>(output_topic, 10);

	f = boost::bind(&CLiDARpreprocess::dynreconfCallback, this, _1, _2);
	server.setCallback(f);
}

void CLiDARpreprocess::scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const{
//	ROS_INFO("Input scan callback");

	//float min_dist = +numeric_limits<float>::infinity();

	if (nranges >= 1) {
		sensor_msgs::LaserScan preproc_scan;

		preproc_scan.header.stamp = pt->header.stamp;//ros::Time::now();
		preproc_scan.header.frame_id = pt->header.frame_id;
		preproc_scan.angle_min = pt->angle_min;
		preproc_scan.angle_max = pt->angle_max;
		preproc_scan.angle_increment = pt->angle_increment;
		preproc_scan.scan_time = pt->scan_time;
		preproc_scan.time_increment = pt->time_increment;
		preproc_scan.range_min = pt->range_min;
		preproc_scan.range_max = pt->range_max;
		preproc_scan.ranges.resize(pt->ranges.size());
		preproc_scan.intensities.resize(pt->intensities.size());

   		//string s = "";
   		//ROS_INFO("Size of ranges array: %ld; size of intensities array: %ld",
   		//	pt->ranges.size(), pt->intensities.size());

		for (int i = 0; i < pt->ranges.size(); i++)
		{
			bool to_be_discarded = false;
			for (int range = 0; range < nranges && !to_be_discarded; ++range) {
				if (i >= indexes[2 * range] && i <= indexes[2 * range + 1])
					to_be_discarded = true;
			}
			if (to_be_discarded) {
				preproc_scan.ranges[i] = numeric_limits<float>::infinity();
				//s += ("\n" + to_string(i) + " : " + to_string(pt->ranges[i])
				//		+ " --> " + to_string(preproc_scan.ranges[i]) );
				if (pt->intensities.size() > 0) preproc_scan.intensities[i] = 0;
			}
			else {
				preproc_scan.ranges[i] = pt->ranges[i];
				// just republish intensity, if available
				if (pt->intensities.size() > 0) preproc_scan.intensities[i] = pt->intensities[i];
			}

		}
		//ROS_INFO("Discarded readings:%s", s.c_str());

		laser_pub.publish(preproc_scan);
	}
	else // nothing to preprocess; just republish the same scan in output topic
		laser_pub.publish(*pt);

	//ros::Duration(0.1).sleep();
}

void CLiDARpreprocess::run(){
	bool stop = false;
	do{
		// do some processing here...
		// ...
		ros::spinOnce();	// trigger callbacks once
        rate.sleep();		// sleep some time to attain desired processing rate (see constructor)
	} while (!stop);
}

void CLiDARpreprocess::dynreconfCallback(lidar2d_preprocess::lidar2d_preprocess_Config &config, uint32_t level){
	ROS_INFO("***Reconfigure request: %s", config.indexes.c_str() );

	stringstream stream_param;
	stream_param << config.indexes;
	string str_anumber;
	indexes.clear();
	do{
		stream_param >> str_anumber;
		if (!stream_param.fail() ) indexes.push_back( stof(str_anumber) );
	} while (stream_param.good() );

	this->configIndexes();
}

void CLiDARpreprocess::configIndexes(){
	nranges = indexes.size() / 2;
	string s = "Parameters (topics / indexes / # ranges):\n "
					+ input_topic + ", " + output_topic + " / [ ";
	for (int i = 0; i < indexes.size(); i++) s += (to_string(indexes[i]) + " ");
	s += ("] / " + to_string(nranges) + " ranges");		
	ROS_INFO("%s", s.c_str());
	if (nranges < 1) ROS_WARN("No valid index ranges makes pre-processing unviable.");
}

int main(int argc, char** argv){
	// setup ROS node
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	CLiDARpreprocess preprocess(nh);
	ros::spin(); 			// to trigger callbacks without run loop
	//preprocess.run(); 	// run loop with processing rate control
	return(0);
}
