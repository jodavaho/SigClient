#include <dcc/RosDccData.h>
#include <dcc/RosDccPulseData.h>
#include <dcc/FreqChange.h>
#include <paramon/paramon.h>
#include <serialcoms/PortBuffer.h>
#include <serialcoms/PortMessage.h>
#include <serialcoms/PortUtils.h>
#include <vector>
#include <dcc/dccmsg.h>
#include <ros/ros.h>
#include <string>
#include <algorithm>

int baud = 57600;
bool answerCalls;
PortWriter* pout;
PortReader* pin;
pthread_mutex_t outlock;
std::vector<int> current_frequencies;
bool checkList(std::vector<int> one, std::vector<int> two){
	if (one.size() != two.size()){return true;}
	std::sort(one.begin(),one.end()); std::sort(two.begin(),two.end());
	//one.sort(); two.sort();
	for (unsigned int i=0;i<one.size();i++)
	{
		if (one[i] != two[i]){return true;}
	}
	return false;
}
bool freqChangeHandler(dcc::FreqChange::Request &req, dcc::FreqChange::Response &res){
	if (answerCalls){
		answerCalls = false;
		bool success = false;
		ROS_INFO("Found request for %Zu frequencies. Verifying...",req.desired_list.size());
		bool reprogram = checkList(current_frequencies,req.desired_list);
		if (reprogram){
			dccmsg t = dccmsg::SetFreq(req.desired_list);
			success = pout->send(t);
		} else {
			success = true;
		}
		answerCalls = true;
		return success;
	} else {
		ROS_WARN("Does not currently support concurrent requests.");
		return false;
	}
}
void openPortFromRos(ros::NodeHandle nh,int &file_in, int &file_out){
	std::string port;
	nh.getParam("/dcc/port",port);
	ROS_INFO("Using port: %s",port.c_str());
	int fdr =	open_port_read(port.c_str());
	int fdw =	open_port_write(port.c_str());
	file_in = configure_port(fdr);
	file_out = configure_port(fdw);
}
int main(int a, char** c){

	//ros stuff
	answerCalls = true;
	ros::init(a,c,"dcc");
	ros::NodeHandle nh("/");
	ros::Publisher dccOut = nh.advertise<dcc::RosDccData>("/dcc/signal",10);
	ros::Publisher dccPuseOut = nh.advertise<dcc::RosDccPulseData>("/dcc/ps",10);
	ros::ServiceServer freqChange = nh.advertiseService("/dcc/freq_change",&freqChangeHandler);
	//ros::Subscriber paramonIn = nh.subscribe(paramon::getCallbackTopic(), 10,&pcbh);
	paramon::addWatch("/dcc/port");

	//load up ports
	int fdr,fdw;
	openPortFromRos(nh,fdr,fdw);
	if (fdw<=0 || fdr<=0){
		ROS_ERROR("Cannot open port %d",fdw);
		return -1;
	}
	PortWriter pw(fdw);
	pout = &pw;
	PortReader pr(fdr);
	pin = &pr;

	//begin!
	dccmsg dmsg;
	int lastfreq=-1;
	while(ros::ok() ){
		ros::Duration(.1).sleep();
		ros::spinOnce();
		if (pr.available()>0){
			pr>>dmsg;
			if (dmsg.isOK()){
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
						if (pr.available()>0){
							pr>>dmsg;
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
				if (answerCalls) {dccOut.publish(dd);}
				if (answerCalls) {dccPuseOut.publish(pdd);}
			}
		}
	}
	close(fdw);
	close(fdr);
	return EXIT_SUCCESS;
}
