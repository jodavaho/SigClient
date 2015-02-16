#include <dcc/RosDccData.h>
#include <dcc/RosDccPulseData.h>
#include <dcc/FreqChange.h>
#include <dcc/Aerial.h>
#include <dcc/Manual.h>
#include <paramon/paramon.h>
#include <serialcoms/PortBuffer.h>
#include <serialcoms/PortMessage.h>
#include <serialcoms/PortUtils.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <dcc/dccmsg.h>
#include <ros/ros.h>
#include <string>
#include <algorithm>

int baud = 57600;
bool answerCalls;
bool useListFilter;

bool resetInput(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
//if (!p_in->isOK()){*p_in = ScopedPortLock(screenPortString.c_str(),9600,'r');}
//pbIn = PortReader(p_in->getFD());
	return true;
}

int main(int a, char** c){

	//ros stuff
	answerCalls = true;
	ros::init(a,c,"dcc");
	ros::NodeHandle nh("/");
	ros::Publisher dccOut = nh.advertise<dcc::RosDccData>("/dcc/signal",10);
	ros::Publisher dccPuseOut = nh.advertise<dcc::RosDccPulseData>("/dcc/ps",10);
	
	std::string screenPortString;
	nh.getParam("/dcc/port",screenPortString);
	ROS_INFO("Opening: %s",screenPortString.c_str());

	int fd = open_port_read(screenPortString.c_str());
	if (fd<=0){return EXIT_FAILURE;}
	configure_port(fd,9600);
	if (fd<=0){
		ROS_ERROR("Cannot open port: %s",screenPortString.c_str());
		return EXIT_FAILURE;
	}
	ros::Duration(1).sleep();
	PortReader pbIn(fd);
	
	ROS_INFO("Starting loop");
	ros::Duration(1).sleep();
	while( ros::ok() ){
		ros::Duration(1).sleep();
		ros::spinOnce();
		ROS_INFO("%d available",pbIn.available());
		if (pbIn.available()>0){
			rawmsg dmsg;
			pbIn>>dmsg;
			if (dmsg.isOK()){
				ROS_INFO("Reading screen message ... (%d available)",pbIn.available());
				dcc::RosDccData dd;
				dcc::RosDccPulseData pdd;
				dd.strength = dmsg.getSS();
				dd.frequency= dmsg.getFrequency();
				pdd.strength = dmsg.getSS();
				pdd.frequency= dmsg.getFrequency();
				pdd.pulse = dmsg.getPulse();
				dccOut.publish(dd);
				dccPuseOut.publish(pdd);
			} else {
				ROS_INFO("Bad message. Dropped");
			}
		}
	}
	close(fd);
	ROS_INFO("Closing dcc node");
	return EXIT_SUCCESS;
}
