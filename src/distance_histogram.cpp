#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <lidar2d_preprocess/distance_histogram_Config.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <limits>
using namespace std;

const char node_name[] = "distance_histogram";

// default values of ROS node parameters
const string default_topic = "scan_raw";
const vector<double> default_distance_th{0.35, 1.0, 4.0, 25.0};

class CDistHistogram{
	// ROS node parameters
	string topic;
	vector<double> distance_th;

	// other class attributes
	ros::NodeHandle private_node_handle;
	ros::Rate rate;
	ros::Subscriber laser_sub;
	dynamic_reconfigure::Server<lidar2d_preprocess::distance_histogram_Config> server;
	dynamic_reconfigure::Server<lidar2d_preprocess::distance_histogram_Config>::CallbackType f;
	int nbins;
	int *histogram;

	// callbacks
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const;
	void dynreconfCallback(lidar2d_preprocess::distance_histogram_Config &config, uint32_t level);
public:
	CDistHistogram(ros::NodeHandle&);
	~CDistHistogram();
	void run();
	void configHistogram();
};

CDistHistogram::CDistHistogram(ros::NodeHandle &node_handle) : private_node_handle("~"),
															   rate(10000) // rate in Hz
{
    // default values of ROS node parameters
    private_node_handle.param<string>("topic", topic, default_topic);
	private_node_handle.param("distance_th", distance_th, default_distance_th);

	//read ROS node parameters
	string paramName;
	if (private_node_handle.searchParam("topic", paramName) )
		private_node_handle.getParam(paramName, topic);
	else ROS_WARN("Parameter 'topic' undefined");
	if (private_node_handle.searchParam("distance_th", paramName) )
		private_node_handle.getParam(paramName, distance_th);
	else ROS_WARN("Parameter 'distance_th' undefined");

	this->configHistogram();

	// ROS publishers and subscribers
	ROS_INFO("Subscribe to '%s' topic", topic.c_str());
	laser_sub  = node_handle.subscribe<sensor_msgs::LaserScan>(topic.c_str(),
		10, &CDistHistogram::scanCallback, this);

	f = boost::bind(&CDistHistogram::dynreconfCallback, this, _1, _2);
	server.setCallback(f);
}

CDistHistogram::~CDistHistogram(){ delete []histogram; }

void CDistHistogram::scanCallback(const sensor_msgs::LaserScan::ConstPtr& pt) const{
	const int size = pt->ranges.size(); // number of points in the scan
	if (nbins < 2) return; // invalid histogram (just one bin)

	for (int bin = 0; bin < nbins; bin++) histogram[bin] = 0; // initialize histogram
	float angle = pt->angle_min;
	float min_dist = +numeric_limits<float>::infinity();
	float max_dist = -numeric_limits<float>::infinity();

	// 1st version with a fixed number of bins equal to 5
	/*for (int i = 0; i < size; i++)
	{
		if (pt->ranges[i] <= distance_th[0]){
			++histogram[0];
			ROS_INFO("(%d, %.3f):%f", i, angle, pt->ranges[i]);
		}
		else if (pt->ranges[i] <= distance_th[1]) ++histogram[1];
		else if (pt->ranges[i] <= distance_th[2]) ++histogram[2];
		else if (pt->ranges[i] <= distance_th[3]) ++histogram[3];
		else ++histogram[4];

		if (pt->ranges[i] < min_dist) min_dist = pt->ranges[i];
		if (pt->ranges[i] > max_dist) max_dist = pt->ranges[i];

		angle += pt->angle_increment;
	}

	ROS_INFO("%d %f %f %f %f %f\n[0,%.2f]:%d ]%.2f,%.2f]:%d ]%.2f,%.2f]:%d ]%.2f,%.2f]:%d ]%.2f,+inf[:%d\n",
			size, pt->angle_min, pt->angle_max, pt->angle_increment,
			min_dist, max_dist,
			distance_th[0], histogram[0], distance_th[0], distance_th[1], histogram[1],
			distance_th[1], distance_th[2], histogram[2],
			distance_th[2], distance_th[3], histogram[3],
			distance_th[3], histogram[4]);
	*/

	for (int i = 0; i < size; i++)
	{
		// update bins
		if (pt->ranges[i] <= distance_th[0]){
			++histogram[0];
			ROS_INFO("(%d, %.3f):%f", i, angle, pt->ranges[i]);
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
	ROS_INFO(" ");
	cout << right << setw(6) << size << setw(8) << fixed << setprecision(4) << pt->angle_min
		 << ' ' << pt->angle_max << setw(11) << setprecision(8) << pt->angle_increment
		 << setw(7) << setprecision(4)<< setw(7) << min_dist << ' ' << max_dist
		 << endl << "[0.00," << setw(5) << setprecision(2) << distance_th[0] << "]:"
		 << left << setw(5) << histogram[0];
	for (int bin = 1; bin < nbins - 1; bin++) {
		cout << "]" << right << setw(5) << setprecision(2) << distance_th[bin - 1] << ", "
			 << setw(5) << setprecision(2) << distance_th[bin]
			 << "]:" << left << setw(5) << histogram[bin];
		if (bin % 3 == 0) cout << endl;
	}

	cout << "]" << right << setw(5) << setprecision(2) << distance_th[nbins - 2] << ",+inf[:"
		 << left << setw(5) << histogram[nbins - 1] << endl; 

	//ros::Duration(0.1).sleep();
}

void CDistHistogram::run(){
	bool stop = false;
	do{
		// do some processing here...
		// ...
		ros::spinOnce();	// trigger callbacks once
        rate.sleep();		// sleep some time to attain desired processing rate (see constructor)
	} while (!stop);
}

void CDistHistogram::dynreconfCallback(lidar2d_preprocess::distance_histogram_Config &config, uint32_t level){
	ROS_INFO("***Reconfigure request: %s", config.distance_th.c_str() );

	stringstream stream_param;
	stream_param << config.distance_th;
	string str_anumber;
	distance_th.clear();
	do{
		stream_param >> str_anumber;
		if (!stream_param.fail() ) distance_th.push_back( stof(str_anumber) );
	} while (stream_param.good() );

	delete []histogram;
	this->configHistogram();
}

void CDistHistogram::configHistogram(){
	nbins = distance_th.size() + 1;
	histogram = new int[nbins];
	string s = "Parameters (topic / distance thresholds / # bins):\n " + topic + " / [ ";
	for (int i = 0; i < distance_th.size(); i++) s += (to_string(distance_th[i]) + " ");
	s += ("] / " + to_string(nbins) + " bins");		
	ROS_INFO("%s", s.c_str());
	if (nbins < 2) ROS_WARN("No distance thresholds makes histogram unviable.");
}

int main(int argc, char** argv){
	// setup ROS node
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	CDistHistogram distHistogram(nh);
	ros::spin(); 			// to trigger callbacks without run loop
	//distHistogram.run(); 	// run loop with processing rate control
	return(0);
}


//-------------------------- First implementation which was not Object Oriented
/*#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <limits>
using namespace std;

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

//parameters
string TOPIC;
vector<double> DISTANCE_TH;

//topic to publish velocity commands
//ros::Publisher cmd_vel_pub;

float Euclidist(float x, float y)
{ return sqrt(x*x+y*y); }

#define PI2	1.5707963268

void scanReceived(const sensor_msgs::LaserScan::ConstPtr& pt){
	int size = pt->ranges.size(); // number of points in the scan
	int histogram[5];
	for (int k = 0; k < 5; k++) histogram[k] = 0;
	float angle = pt->angle_min;
	float min_dist = numeric_limits<float>::infinity(), max_dist = -numeric_limits<float>::infinity();

	for (int i = 0; i < size; i++)
	{
		if (pt->ranges[i] <= DISTANCE_TH[0]){
			++histogram[0];
			ROS_INFO("(%d, %.3f):%f", i, angle, pt->ranges[i]);
		}
		else if (pt->ranges[i] <= DISTANCE_TH[1]) ++histogram[1];
		else if (pt->ranges[i] <= DISTANCE_TH[2]) ++histogram[2];
		else if (pt->ranges[i] <= DISTANCE_TH[3]) ++histogram[3];
		else ++histogram[4];

		if (pt->ranges[i] < min_dist) min_dist = pt->ranges[i];
		if (pt->ranges[i] > max_dist) max_dist = pt->ranges[i];

		angle += pt->angle_increment;
	}

	ROS_INFO("%d %f %f %f %f %f\n[0,%.2f]:%d ]%.2f,%.2f]:%d ]%.2f,%.2f]:%d ]%.2f,%.2f]:%d ]%.2f,+inf]:%d\n",
			size, pt->angle_min, pt->angle_max, pt->angle_increment,
			min_dist, max_dist,
			DISTANCE_TH[0], histogram[0], DISTANCE_TH[0], DISTANCE_TH[1], histogram[1],
			DISTANCE_TH[1], DISTANCE_TH[2], histogram[2],
			DISTANCE_TH[2], DISTANCE_TH[3], histogram[3],
			DISTANCE_TH[3], histogram[4]);

//	cmd_vel_pub.publish(cmd_vel);
	ros::Duration(0.1).sleep(); // desired control frequency
}

int main(int argc, char** argv){
  
	ros::init(argc, argv, "distance_histogram");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	ros::Rate loop_rate(1); //1 Hz
  
	//default parameter values
	pn.param("topic", TOPIC, string("scan_raw"));
	vector<double> aux{0.25, 0.35, 0.5, 25.0};
	pn.param("distance_th", DISTANCE_TH, aux);

	//read parameters
	string paramName;
	if (pn.searchParam("topic", paramName) )
		pn.getParam(paramName, TOPIC);
	else ROS_WARN("Parameter 'topic' undefined");
	if (pn.searchParam("distance_th", paramName) )
		pn.getParam(paramName, DISTANCE_TH);
	else ROS_WARN("Parameter 'distance_th' undefined");

	string s = "Parameters (topic / distance thresholds):\n " + TOPIC + " / [ ";
	for (int i = 0; i < DISTANCE_TH.size(); i++) s += (to_string(DISTANCE_TH[i]) + " ");
	s+="]";		
	ROS_INFO("%s", s.c_str());

	// ROS publishers and subscribers
	ros::Subscriber laser_sub;
	laser_sub  = n.subscribe<sensor_msgs::LaserScan>(TOPIC.c_str(), 1, scanReceived);
	//cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ROS_INFO("distance_histogram has been started");
	ros::spin(); //trigger callbacks and prevents exiting
	return(0);
}
*/