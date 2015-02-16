#include <dcc/RosDccData.h>
#include <dcc/History.h>
#include <dcc/ClearFreq.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <pthread.h>
#include <map>
#include <vector>
#include <list>

std::map<int, int> counts;
std::map<int, int> nonzero;
ros::Subscriber	dccSub;
ros::Publisher outgoing;
ros::ServiceServer clearall;
ros::ServiceServer clearind;
pthread_mutex_t dcclock;
std::list<int> my_freqs;

void loadParams(){

}
bool contains(std::list<int> l, int test){
	for (std::list<int>::iterator it = my_freqs.begin();it!=my_freqs.end();it++){
		if (*it==test){
			return true;
		}
	}
	return false;
}
bool clearIndService(dcc::ClearFreq::Request &req, dcc::ClearFreq::Response &res){
	pthread_mutex_lock(&dcclock);
	counts[req.freq]=0;
	nonzero[req.freq]=0;
	pthread_mutex_unlock(&dcclock);
	return true;
}
bool clearService(std_srvs::Empty::Request &req ,std_srvs::Empty::Response &res){
	pthread_mutex_lock(&dcclock);
	my_freqs.clear();
	counts.clear();
	nonzero.clear();
	pthread_mutex_unlock(&dcclock);
	return true;
}
void dccIn(const dcc::RosDccData::ConstPtr &msg){
	int str = msg->strength;
	int f = msg->frequency;

	pthread_mutex_lock(&dcclock);

	if (!contains(my_freqs,f)){
			my_freqs.push_back(f);
			my_freqs.sort();
	}

	counts[f]++;
	if (str>0){
		nonzero[f]++;
	}

	dcc::History omsg;
	std::list<int>::iterator it;
	for (it = my_freqs.begin();it!=my_freqs.end();it++){
		omsg.frequencies.push_back(*it);
		omsg.counts.push_back(counts[*it]);
		omsg.nonzeros.push_back(nonzero[*it]);
	}
	outgoing.publish(omsg);

	pthread_mutex_unlock(&dcclock);
	ROS_INFO("Have: %Zu freqs",my_freqs.size());
}
void launch(){
	ros::NodeHandle nh("dcc");
	dccSub = nh.subscribe<dcc::RosDccData>("signal",10,&dccIn);
	outgoing = nh.advertise<dcc::History>("history",10);
	clearall = nh.advertiseService("clear_counts",&clearService);
	clearind = nh.advertiseService("clear_single",&clearIndService);
}

int main(int argc,char** argv){
	if (!ros::isInitialized()){
		ros::init(argc,argv,"dcchistory");
	}
	ROS_INFO("Starting history...");
	pthread_mutex_init(&dcclock,NULL);
	loadParams();
	launch();
	ROS_INFO("Publishing history of /dcc/signal to /dcc/history");
	ros::spin();
	pthread_mutex_destroy(&dcclock);
	return EXIT_SUCCESS;
}
