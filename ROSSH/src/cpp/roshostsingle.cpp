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
PortBuffer* pbuf;
std::vector<int> current_frequencies;
bool resetPorts(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
 return false;
}
bool aerialHandler(dcc::AerialRequest &req, dcc::AerialResponse &res){
	dccmsg rezzet = dccmsg::LogAerial((unsigned char)req.table);
	return pbuf->send(rezzet);
}
bool manualHandler(dcc::ManualRequest &req, dcc::ManualResponse &res){
	dccmsg rezzet = dccmsg::LogManual(req.freq);
	return pbuf->send(rezzet);
}
bool resetHandler(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res){
	dccmsg rezzet = dccmsg::Reset();
	return pbuf->send(rezzet);
}
bool freqExpected(int f){
	if (!useListFilter) return true;
	std::vector<int>::iterator it = find(current_frequencies.begin(),current_frequencies.end(),f);
	return (it!=current_frequencies.end());
}
bool checkList(std::vector<int> one, std::vector<int> two){
	if (one.size() != two.size()){return true;}
	std::sort(one.begin(),one.end()); std::sort(two.begin(),two.end());
	for (unsigned int i=0;i<one.size();i++)
	{
		if (one[i] != two[i]){return true;}
	}
	return false;
}
bool freqChangeHandler(dcc::FreqChange::Request &req, dcc::FreqChange::Response &res){
	bool success = false;
	if (answerCalls){
		answerCalls = false;
		success = false;
		ROS_INFO("Found request for %Zu frequencies. Verifying...",req.desired_list.size());
		bool reprogram = checkList(current_frequencies,req.desired_list);
		if (reprogram){
			ROS_INFO("Different list, reprogramming");
			dccmsg t = dccmsg::SetFreq(req.desired_list);
			success = pbuf->send(t);
			if (success){
				current_frequencies = std::vector<int>(req.desired_list);
			}
		} else {
			ROS_INFO("Same list, not reprogramming");
			success = true;
		}
		answerCalls = true;
	} else {
		ROS_WARN("Does not currently support concurrent requests.");
		success = false;
		
	}
	return success;
}
void pcbh(const std_msgs::String::ConstPtr &msg){
	std::string port;
	//nh.getParam("/dcc/port",port);
	//int fdr =	open_port(port.c_str());
	//file_in = configure_port(fdr);
	//ROS_INFO("%d",fdr);
}
void openPortFromRos(ros::NodeHandle nh,int &file){
	std::string port;
	nh.getParam("/dcc/port",port);
	ROS_INFO("Using port: %s",port.c_str());
	int fd =	open_port(port.c_str());
	if (fd<=0){
		ROS_WARN("FAILED open");
		return;
	}
	file = configure_port(fd);
}
int main(int a, char** c){

	//ros stuff
	answerCalls = true;
	ros::init(a,c,"dcc");
	ros::NodeHandle nh("/");
	ros::Publisher dccOut = nh.advertise<dcc::RosDccData>("/dcc/signal",10);
	ros::Publisher dccPuseOut = nh.advertise<dcc::RosDccPulseData>("/dcc/ps",10);
	ros::ServiceServer freqChange = nh.advertiseService("/dcc/freq_change",&freqChangeHandler);
	ros::ServiceServer portchange = nh.advertiseService("/dcc/reload_ports",&resetPorts);
	ros::ServiceServer reset = nh.advertiseService("/dcc/reset",&resetHandler);
	ros::ServiceServer aerial = nh.advertiseService("/dcc/aerial",&aerialHandler);
	ros::ServiceServer manual = nh.advertiseService("/dcc/manual",&manualHandler);
	ros::Subscriber paramonIn = nh.subscribe(paramon::getCallbackTopic(), 10,&pcbh);
	useListFilter=false;
	nh.getParam("use_list_filter",useListFilter);
	paramon::addWatch("/dcc/use_list_filter");
	paramon::addWatch("/dcc/port");
	//paramon::addWatch("/dcc/in_port");

	//load up ports
	//std::string screenPortString;
	std::string roboPortString;

	//bool useTwo=nh.hasParam("/dcc/in_port");
	//nh.getParam("/dcc/in_port",screenPortString); 
	nh.getParam("/dcc/port",roboPortString);

	ScopedPortLock roboPort(roboPortString.c_str(),57600,'x');
	//ScopedPortLock screenPort(screenPortString,9600,'r');

	if (roboPort.getFD()<=0){
		ROS_ERROR("Cannot open port: %s",roboPortString.c_str());
		return -1;
	}

	PortBuffer pb(roboPort.getFD());
	pbuf = &pb;

	//begin!
	dccmsg dmsg;
	int lastfreq=-1;
	while(ros::ok() ){
		ros::Duration(.1).sleep();
		ros::spinOnce();
		if (pb.available()>0){
			ROS_INFO("Reading");
			pb.receive(dmsg);
			if (dmsg.isOK()){
				if (dmsg.isSignal()){
					dcc::RosDccData dd;
					dcc::RosDccPulseData pdd;
					dd.strength = dmsg.getSS();
					dd.frequency= dmsg.getFrequency();
					pdd.strength = dmsg.getSS();
					pdd.frequency= dmsg.getFrequency();
					pdd.pulse = dmsg.getPulse();
					if (lastfreq!=dd.frequency){
						lastfreq = dd.frequency;
						ros::Time timeToStop = ros::Time::now()+ros::Duration(2);
						ROS_INFO("New frequency. Waiting...");
						while(ros::Time::now()<timeToStop ){
							ros::Duration(.1).sleep();
							ros::spinOnce();
							if (pb.available()>0){
								pb.receive(dmsg);
								dd.strength = dmsg.getSS();
								dd.frequency= dmsg.getFrequency();
								pdd.strength = dmsg.getSS();
								pdd.frequency= dmsg.getFrequency();
								pdd.pulse = dmsg.getPulse();
								if (lastfreq!=dd.frequency){
									timeToStop = ros::Time::now();
								}
							}
						}
					}
					if (answerCalls) {
						if (!freqExpected(dd.frequency)){
							ROS_WARN("Unexpected frequency! %d",dd.frequency);
						} else {
							dccOut.publish(dd);
							dccPuseOut.publish(pdd);
						}
					}
				}
			} else if (dmsg.isAck()){
				ROS_INFO("Rec'd ACK from robostix");
			}
		}
	}
	ROS_INFO("Closing dcc node");
	return EXIT_SUCCESS;
}
