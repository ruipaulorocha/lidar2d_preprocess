/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 * 
 *  Copyright (c) Rui P. Rocha, 2024
 »
 *  All rights reserved.
 * 
 *  Version 2.0.0, Oct. 8, 2024
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
//#include <dynamic_reconfigure/server.h>
//#include <lidar2d_preprocess/distance_histogram_Config.h>


const char node_name[] = "distance_histogram";

// default values of ROS node parameters
const string default_topic = "scan_in";
const vector<double> default_distance_th{0.35, 1.0, 4.0, 25.0};

//class CDistHistogram{
class CDistHistogram : public rclcpp::Node {
private:
	// ROS node parameters
	string topic;
	vector<double> distance_th;

	// other class attributes
	//ros::NodeHandle private_node_handle;
	//ros::Rate rate;
	//ros::Subscriber laser_sub;
	//dynamic_reconfigure::Server<lidar2d_preprocess::distance_histogram_Config> server;
	//dynamic_reconfigure::Server<lidar2d_preprocess::distance_histogram_Config>::CallbackType f;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

	std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
  	std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_param_upd; 

	int nbins;
	int *histogram;

	// callback
	//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const;
	void scanCallback(const sensor_msgs::msg::LaserScan &ptref) const;
	//void dynreconfCallback(lidar2d_preprocess::distance_histogram_Config &config, uint32_t level);

	// private methods
	void configHistogram();
public:
	//CDistHistogram(ros::NodeHandle&);
	CDistHistogram();
	~CDistHistogram();
};

// CDistHistogram::CDistHistogram(ros::NodeHandle &node_handle) : private_node_handle("~"),
//															   rate(10000) // rate in Hz
CDistHistogram::CDistHistogram() : Node(node_name)
{
    // default values of ROS node parameters
    // private_node_handle.param<string>("topic", topic, default_topic);
	// private_node_handle.param("distance_th", distance_th, default_distance_th);
	this->declare_parameter<string>("topic", default_topic);
	this->declare_parameter< vector<double> >("distance_th", default_distance_th);
	//this->declare_parameter("distance_th", rclcpp::PARAMETER_DOUBLE_ARRA);
	//this->set_parameter(rclcpp::Parameter("distance_th", default_distance_th));
	//distance_th = default_distance_th;

	//read ROS node parameters
	// string paramName;
	// if (private_node_handle.searchParam("topic", paramName) )
	// 	private_node_handle.getParam(paramName, topic);
	// else ROS_WARN("Parameter 'topic' undefined");
	// if (private_node_handle.searchParam("distance_th", paramName) )
	// 	private_node_handle.getParam(paramName, distance_th);
	// else ROS_WARN("Parameter 'distance_th' undefined");
	this->get_parameter("topic", topic);
	RCLCPP_INFO_STREAM(this->get_logger(), "topic parameter set succesfully to '" << topic << "'");
	this->get_parameter("distance_th", distance_th);
	RCLCPP_INFO_STREAM(this->get_logger(), "distance_th parameter set succesfully");

	this->configHistogram();

	// ROS subscriber
	//ROS_INFO("Subscribe to '%s' topic", topic.c_str());
	RCLCPP_INFO_STREAM(this->get_logger(), "Subscribe to topic '" << topic << '\'');
	//laser_sub  = node_handle.subscribe<sensor_msgs::LaserScan>(topic.c_str(),
	//	10, &CDistHistogram::scanCallback, this);
	//f = boost::bind(&CDistHistogram::dynreconfCallback, this, _1, _2);
	//server.setCallback(f);
	laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(topic.c_str(), 10,
				std::bind(&CDistHistogram::scanCallback, this, std::placeholders::_1));

	param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
	// Set a callback for this node's parameter, "distance_th" (a lambda function)
    auto callback_distance_th = [this](const rclcpp::Parameter &p) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                  "callback_distance_th: received an update to parameter \"" << p.get_name() 
                  << "\" " << "of type \"" << p.get_type_name() << '"');
      this->distance_th = p.as_double_array();
      this->configHistogram();
    };

    cb_handle_param_upd = param_subscriber->add_parameter_callback("distance_th", callback_distance_th);
}

CDistHistogram::~CDistHistogram(){ delete []histogram; }

//void CDistHistogram::scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const{
void CDistHistogram::scanCallback(const sensor_msgs::msg::LaserScan &ptref) const{
	const sensor_msgs::msg::LaserScan *pt = &ptref; // just to adapt to paramenter that now is a reference (not a pointer)

	const int size = pt->ranges.size(); // number of points in the scan
	if (nbins < 2) return; // invalid histogram (just one bin)

	for (int bin = 0; bin < nbins; bin++) histogram[bin] = 0; // initialize histogram
	float angle = pt->angle_min;
	float min_dist = +numeric_limits<float>::infinity();
	float max_dist = -numeric_limits<float>::infinity();

	for (int i = 0; i < size; i++)
	{
		// update bins
		if (pt->ranges[i] <= distance_th[0]){
			++histogram[0];
			//ROS_INFO("(%d, %.3f):%f", i, angle, pt->ranges[i]);
			RCLCPP_INFO_STREAM(this->get_logger(), "(" << i << ", "
				<< fixed << setprecision(3) << angle << "):" << pt->ranges[i]);
		}
	
		for (int bin = 1; bin < nbins - 1; bin++)
			if (pt->ranges[i] > distance_th[bin - 1] && pt->ranges[i] <= distance_th[bin])
				++histogram[bin];
	
		if (pt->ranges[i] > distance_th[nbins - 2]) ++histogram[nbins - 1];
		// nbins - 2 == distance_th.size() - 1 <=> nbins = distance_th.size() + 1

		if (pt->ranges[i] < min_dist) min_dist = pt->ranges[i];
		if (pt->ranges[i] > max_dist) max_dist = pt->ranges[i];

		angle += pt->angle_increment;
	}
	//ROS_INFO(" ");
	stringstream s;
	// cout << right << setw(6) << size << setw(8) << fixed << setprecision(4) << pt->angle_min
	// 	 << ' ' << pt->angle_max << setw(11) << setprecision(8) << pt->angle_increment
	// 	 << setw(7) << setprecision(4)<< setw(7) << min_dist << ' ' << max_dist
	// 	 << endl << "[0.00," << setw(5) << setprecision(2) << distance_th[0] << "]:"
	// 	 << left << setw(5) << histogram[0];
	// for (int bin = 1; bin < nbins - 1; bin++) {
	// 	cout << "]" << right << setw(5) << setprecision(2) << distance_th[bin - 1] << ", "
	// 		 << setw(5) << setprecision(2) << distance_th[bin]
	// 		 << "]:" << left << setw(5) << histogram[bin];
	// 	if (bin % 3 == 0) cout << endl;
	// }

	// cout << "]" << right << setw(5) << setprecision(2) << distance_th[nbins - 2] << ",+inf[:"
	// 	 << left << setw(5) << histogram[nbins - 1] << endl;

	s << endl << right << setw(6) << size << setw(8) << fixed << setprecision(4) << pt->angle_min
		 << ' ' << pt->angle_max << setw(11) << setprecision(8) << pt->angle_increment
		 << setw(7) << setprecision(4)<< setw(7) << min_dist << ' ' << max_dist
		 << endl << "[0.00," << setw(5) << setprecision(2) << distance_th[0] << "]:"
		 << left << setw(5) << histogram[0];
	for (int bin = 1; bin < nbins - 1; bin++) {
		s << "]" << right << setw(5) << setprecision(2) << distance_th[bin - 1] << ", "
			 << setw(5) << setprecision(2) << distance_th[bin]
			 << "]:" << left << setw(5) << histogram[bin];
		if (bin % 3 == 0) s << endl;
	}

	s << "]" << right << setw(5) << setprecision(2) << distance_th[nbins - 2] << ",+inf[:"
		 << left << setw(5) << histogram[nbins - 1] << endl; 

	RCLCPP_INFO_STREAM(this->get_logger(), s.str());
}

// void CDistHistogram::dynreconfCallback(lidar2d_preprocess::distance_histogram_Config &config, uint32_t level){
// 	ROS_INFO("***Reconfigure request: %s", config.distance_th.c_str() );

// 	stringstream stream_param;
// 	stream_param << config.distance_th;
// 	string str_anumber;
// 	distance_th.clear();
// 	do{
// 		stream_param >> str_anumber;
// 		if (!stream_param.fail() ) distance_th.push_back( stof(str_anumber) );
// 	} while (stream_param.good() );

// 	delete []histogram;
// 	this->configHistogram();
// }

void CDistHistogram::configHistogram(){
	nbins = distance_th.size() + 1;
	histogram = new int[nbins];
	string s = "Parameters (topic / distance thresholds / # bins):\n '" + topic + "' / [ ";
	for (long unsigned int i = 0; i < distance_th.size(); i++) s += (to_string(distance_th[i]) + " ");
	s += ("] / " + to_string(nbins) + " bins");
	//ROS_INFO("%s", s.c_str());
	RCLCPP_INFO_STREAM(this->get_logger(), s);
	if (nbins < 2) //ROS_WARN("No distance thresholds makes histogram unviable.");
		RCLCPP_WARN_STREAM(this->get_logger(), "No distance thresholds makes histogram unviable");	
}

int main(int argc, char** argv){
	// setup ROS node
	//ros::init(argc, argv, node_name);
	//ros::NodeHandle nh;
	rclcpp::init(argc, argv);

	//CDistHistogram distHistogram(nh);
	//ros::spin(); 			// to trigger callbacks without run loop

	rclcpp::Rate rate(10000.0); // loop rate = 10 kHz
	std::shared_ptr<rclcpp::Node> node = std::make_shared<CDistHistogram>();
	
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
