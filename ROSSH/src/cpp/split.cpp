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

PortBuffer pbOut;
PortReader pbIn;
ScopedPortLock inport;
ScopedPortLock outport;
std::string screenPortString;
std::string roboPortString;

std::vector<int> current_frequencies;

bool resetInput(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	if (!inport.isOK()){inport = ScopedPortLock(screenPortString.c_str(),9600,'r');}
	pbIn = PortReader(inport.getFD());
	return true;
}
bool resetOutput(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	if (!outport.isOK()){outport = ScopedPortLock(roboPortString.c_str(),57600,'x');}
	pbOut = PortBuffer(outport.getFD());
	return true;
}
bool resetPorts(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	return resetInput(req,res) && resetOutput(req,res);
}
bool aerialHandler(dcc::AerialRequest &req, dcc::AerialResponse &res){
	dccmsg rezzet = dccmsg::LogAerial((unsigned char)req.table);
	return pbOut.send(rezzet);
}
bool manualHandler(dcc::ManualRequest &req, dcc::ManualResponse &res){
	dccmsg rezzet = dccmsg::LogManual(req.freq);
	return pbOut.send(rezzet);
}
bool resetHandler(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res){
	dccmsg rezzet = dccmsg::Reset();
	return pbOut.send(rezzet);
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
		bool repbInogram = checkList(current_frequencies,req.desired_list);
		if (repbInogram){
			ROS_INFO("Different list, repbInogramming");
			dccmsg t = dccmsg::SetFreq(req.desired_list);
			success = pbOut.send(t);
			if (success){
				current_frequencies = std::vector<int>(req.desired_list);
			}
		} else {
			ROS_INFO("Same list, not repbInogramming");
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
	ros::NodeHandle nh("/");
	useListFilter=false;
	nh.getParam("use_list_filter",useListFilter);
	nh.getParam("/dcc/in_port",screenPortString); 
	nh.getParam("/dcc/port",roboPortString);
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
	paramon::addWatch("/dcc/in_port");

	nh.getParam("/dcc/in_port",screenPortString); 
	nh.getParam("/dcc/port",roboPortString);

	outport = ScopedPortLock(roboPortString.c_str(),57600,'x');
	inport = ScopedPortLock(screenPortString.c_str(),9600,'r');

	ROS_INFO("OK!");
////////if (outport.getFD()<=0){
////////	ROS_ERROR("Cannot open port: %s",roboPortString.c_str());
////////	return -1;
////////}
////////if (inport.getFD()<=0){
////////	ROS_ERROR("Cannot open port: %s",screenPortString.c_str());
////////	return -1;
////////}

	pbOut = PortBuffer(outport.getFD());
	pbIn = PortReader(inport.getFD());
	
	//begin!
	ROS_INFO("BEGIN!");
	int lastfreq=-1;
	while(ros::ok() ){
		ros::Duration(.01).sleep();
		ros::spinOnce();
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
				if (lastfreq!=dd.frequency){
					lastfreq = dd.frequency;
					ros::Time timeToStop = ros::Time::now()+ros::Duration(2);
					ROS_INFO("New frequency. Waiting...");
					while(ros::Time::now()<timeToStop ){
						ros::Duration(.1).sleep();
						ros::spinOnce();
						if (pbIn.available()>0){
							rawmsg dmsg;
							pbIn>>dmsg;
							if (dmsg.isOK()){
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
		}
		if (pbOut.available()>0){
			dccmsg robomsg;
			ROS_INFO("Reading robostix msg ... ");
			pbOut>>robomsg;
			if (robomsg.isAck()){
				ROS_INFO("Ack received");
			} else {
				ROS_INFO("Msg type: %c received",robomsg.type);
			}
		}
	}
	ROS_INFO("Closing dcc node");
	return EXIT_SUCCESS;
}
