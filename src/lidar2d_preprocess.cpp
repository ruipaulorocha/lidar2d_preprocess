/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 * 
 *  Copyright (c) Rui P. Rocha, 2024
 »
 *  All rights reserved.
 * 
 *  Version 2.0.0, Oct. 9, 2024
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Rui P. Rocha */

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <limits>

using namespace std;

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <dynamic_reconfigure/server.h>
// #include <lidar2d_preprocess/lidar2d_preprocess_Config.h>


const char node_name[] = "lidar2d_preprocess";

// default values of ROS node parameters
const string default_input_topic  = "scan_in";
const string default_output_topic = "scan_out";
//const vector<long int> default_indexes{820, 825, 1142, 1151};
const vector<long int> default_indexes{};

//class CLiDARpreprocess{
class CLiDARpreprocess : public rclcpp::Node {
	// ROS node parameters
	string input_topic;
	string output_topic;
	vector<long int> indexes;

	// other class attributes
	// ros::NodeHandle private_node_handle;
	// ros::Rate rate;
	// ros::Subscriber laser_sub;
	// ros::Publisher  laser_pub;
	// dynamic_reconfigure::Server<lidar2d_preprocess::lidar2d_preprocess_Config> server;
	// dynamic_reconfigure::Server<lidar2d_preprocess::lidar2d_preprocess_Config>::CallbackType f;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;

	std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
  	std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_param_upd; 

	int nranges;

	// callback
	//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const;
	void scanCallback(const sensor_msgs::msg::LaserScan &ptref) const;
	//void dynreconfCallback(lidar2d_preprocess::lidar2d_preprocess_Config &config, uint32_t level);

	// private methods
	void configIndexes();
public:
	//CLiDARpreprocess(ros::NodeHandle&);
	CLiDARpreprocess();
};

// CLiDARpreprocess::CLiDARpreprocess(ros::NodeHandle &node_handle) : private_node_handle("~"),
// 															   rate(10000) // rate in Hz
CLiDARpreprocess::CLiDARpreprocess() : Node(node_name)
{
    // default values of ROS node parameters
    // private_node_handle.param<string>("in_topic", input_topic, default_input_topic);
    // private_node_handle.param<string>("out_topic", output_topic, default_output_topic);
	// private_node_handle.param("indexes", indexes, default_indexes);
	this->declare_parameter<string>("in_topic",  default_input_topic);
	this->declare_parameter<string>("out_topic", default_output_topic);
	this->declare_parameter< vector<long int> >("indexes", default_indexes);

	//read ROS node parameters
	// string paramName;
	// if (private_node_handle.searchParam("in_topic", paramName) )
	// 	private_node_handle.getParam(paramName, input_topic);
	// else ROS_WARN("Parameter 'in_topic' undefined");
	// if (private_node_handle.searchParam("out_topic", paramName) )
	// 	private_node_handle.getParam(paramName, output_topic);
	// else ROS_WARN("Parameter 'out_topic' undefined");
	// if (private_node_handle.searchParam("indexes", paramName) )
	// 	private_node_handle.getParam(paramName, indexes);
	// else ROS_WARN("Parameter 'indexes' undefined");

	this->get_parameter("in_topic", input_topic);
	RCLCPP_INFO_STREAM(this->get_logger(), "Parameter 'in_topic' set succesfully to '" << input_topic << "'");
	this->get_parameter("out_topic", output_topic);
	RCLCPP_INFO_STREAM(this->get_logger(), "Parameter 'out_topic' set succesfully to '" << output_topic << "'");
	this->get_parameter("indexes", indexes);
	RCLCPP_INFO_STREAM(this->get_logger(), "indexes parameter set succesfully");

	this->configIndexes();

	// ROS publishers and subscribers
	//ROS_INFO("Subscribe to '%s' topic", input_topic.c_str());
	RCLCPP_INFO_STREAM(this->get_logger(), "Subscribe to topic '" << input_topic << '\'');
	// laser_sub  = node_handle.subscribe<sensor_msgs::LaserScan>(input_topic.c_str(),
	// 	10, &CLiDARpreprocess::scanCallback, this);
	laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(input_topic.c_str(), 10,
				std::bind(&CLiDARpreprocess::scanCallback, this, std::placeholders::_1));
	//ROS_INFO("Publish to '%s' topic", output_topic.c_str());
	RCLCPP_INFO_STREAM(this->get_logger(), "Configure publisher to topic '" << output_topic << '\'');
	//laser_pub = node_handle.advertise<sensor_msgs::LaserScan>(output_topic, 10);
	laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic.c_str(), 1);

	// f = boost::bind(&CLiDARpreprocess::dynreconfCallback, this, _1, _2);
	// server.setCallback(f);

	param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
	// Set a callback for this node's parameter, "indexes" (a lambda function)
    auto callback_indexes_th = [this](const rclcpp::Parameter &p) {
      RCLCPP_INFO_STREAM(this->get_logger(),
		"callback_indexes_th: received an update to parameter \"" << p.get_name() 
			<< "\" " << "of type \"" << p.get_type_name() << '"');
      /*vector<long int> aux_v = p.as_integer_array();
      this->indexes.clear();
      for (unsigned long int i = 0; i < aux_v.size(); i++)
      	this->indexes.push_back( (int) aux_v[i]);*/
      this->indexes = p.as_integer_array();
      this->configIndexes();
    };

    cb_handle_param_upd = param_subscriber->add_parameter_callback("indexes", callback_indexes_th);
}

//void CLiDARpreprocess::scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const{
void CLiDARpreprocess::scanCallback(const sensor_msgs::msg::LaserScan &ptref) const{
	
	const sensor_msgs::msg::LaserScan *pt = &ptref; // just to adapt to paramenter that now is a reference (not a pointer)

	if (nranges >= 1) {
		//sensor_msgs::LaserScan preproc_scan;
		sensor_msgs::msg::LaserScan preproc_scan;

		preproc_scan.header.stamp = pt->header.stamp;
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
   		//RCLCPP_INFO(this->get_logger(), "Size of ranges array: %ld; size of intensities array: %ld",
   		//	pt->ranges.size(), pt->intensities.size());

		for (long int i = 0; i < ( (long int) pt->ranges.size() ); i++)
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
		//RCLCPP_INFO(this->get_logger(), "Discarded readings:%s", s.c_str());

		//laser_pub.publish(preproc_scan);
		laser_pub->publish(preproc_scan);
	}
	else // nothing to preprocess; just republish the same scan in output topic
		//laser_pub.publish(*pt);
		laser_pub->publish(*pt);
}

// void CLiDARpreprocess::run(){
// 	bool stop = false;
// 	do{
// 		// do some processing here...
// 		// ...
// 		ros::spinOnce();	// trigger callbacks once
//         rate.sleep();		// sleep some time to attain desired processing rate (see constructor)
// 	} while (!stop);
// }

// void CLiDARpreprocess::dynreconfCallback(lidar2d_preprocess::lidar2d_preprocess_Config &config, uint32_t level){
// 	ROS_INFO("***Reconfigure request: %s", config.indexes.c_str() );

// 	stringstream stream_param;
// 	stream_param << config.indexes;
// 	string str_anumber;
// 	indexes.clear();
// 	do{
// 		stream_param >> str_anumber;
// 		if (!stream_param.fail() ) indexes.push_back( stof(str_anumber) );
// 	} while (stream_param.good() );

// 	this->configIndexes();
// }

void CLiDARpreprocess::configIndexes(){
	nranges = indexes.size() / 2;
	string s = "Parameters (topics / indexes / # ranges):\n "
					+ input_topic + ", " + output_topic + " / [ ";
	for (unsigned long int i = 0; i < indexes.size(); i++) s += (to_string(indexes[i]) + " ");
	s += ("] / " + to_string(nranges) + " ranges");		
	//ROS_INFO("%s", s.c_str());
	RCLCPP_INFO_STREAM(this->get_logger(), s);
	if (nranges < 1) //ROS_WARN("No valid index ranges makes pre-processing unviable.");
		RCLCPP_WARN_STREAM(this->get_logger(), "No valid index ranges makes pre-processing unviable");
}

int main(int argc, char** argv){
	// setup ROS node
	//ros::init(argc, argv, node_name);
	//ros::NodeHandle nh;
	rclcpp::init(argc, argv);

	// CLiDARpreprocess preprocess(nh);
	// ros::spin(); 			// to trigger callbacks without run loop
	// //preprocess.run(); 	// run loop with processing rate control

	rclcpp::Rate rate(100.0); // loop rate = 100 Hz
	std::shared_ptr<rclcpp::Node> node = std::make_shared<CLiDARpreprocess>();

	// spin without controlling explicitly in main() the execution loop
	//rclcpp::spin(node->get_node_base_interface()); //trigger callbacks and prevents exiting
	while (rclcpp::ok()){
		// do some job on each iteration...

		// spin
		rclcpp::spin_some(node);
		// sleep to achieve desired rate
		rate.sleep(); 
	}

	rclcpp::shutdown();
	return(0);
}
