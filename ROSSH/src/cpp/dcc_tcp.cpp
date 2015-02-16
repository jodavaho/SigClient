#include <networking/SocketUtils.h>
#include <networking/protocol.h>
#include <networking/DCCTCP.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include <time.h>
#include <map>
#include <netdb.h>
#include <fcntl.h>
#include <string>
#include <sstream>
#include <arpa/inet.h>
#include <pthread.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stropts.h>
#include <sys/ioctl.h>
#include <linux/netdevice.h>
#include <netinet/in.h>
#include <signal.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <dcc/RosDccPulseData.h>

#include <errno.h>

using DCCTCP::dcc_port;
using SimpleProtocol::MessageToMap;
using SimpleProtocol::MapToMessage;

namespace DCCTCP{
	static const int BUF_SIZE=8192;
	static const int MAX_THREAD=32;
	bool dcc_on;
	void sig_handler(int signum);
	class WorkerArgs;
	char MY_HOST[HOST_NAME_MAX];
	char MY_IP[DCCTCP::BUF_SIZE];
	void *get_in_addr(struct sockaddr *sa);
	int *sigset_; //oh god
	bool dcc_ON(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool dcc_OFF(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
}
//////////////////////////////////////////////

// get sockaddr, IPv4 or IPv6:
void *DCCTCP::get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
class DCCTCP::WorkerArgs{
	public:
		int fd;
		struct sockaddr_in *addr;
		socklen_t addr_len;
		bool ok;
		pthread_mutex_t*block;
		void* other;
};


bool DCCTCP::dcc_ON(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	printf("Powering up Signal Hound\n");
	DCCTCP::dcc_on = true;
	return true;
}

bool DCCTCP::dcc_OFF(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	printf("Powering down Signal Hound\n");
	DCCTCP::dcc_on = false;
	return true;
}

void DCCTCP::sig_handler(int signum){
	fprintf(stdout,"Caught your sig\n");
	switch(signum){
		case SIGINT:{/*fall through!*/}
		case SIGKILL:{/*fall through!*/}
		case SIGTERM:{*DCCTCP::sigset_=0;break;}
		default:{printf("Unhandled sig: %d\n",signum);break;}
	}
}

int main(int argc, char** argv){

	DCCTCP::dcc_on = false;
	//int threadptr =0;
	//int numactive =0;
	//pthread_t[DCCTCP::MAX_THREAD];
	printf("Repeater Server starting...\n");
	printf("Takes TCP->ROS.  Useful for devices that can't / don't run ROS\n");
	printf("Invoke as: DCCTCP_server I X to change the port to X on interface I\n");

	struct addrinfo addr_hints,*addr_res;
	//struct addrinfo *servinfo;
	//struct sockaddr_un address;

	int res=0,ter=0,sock_fd=0;
	int opt=1;
	int sig_OK = 1;
	DCCTCP::sigset_ = &sig_OK;

	char ports[DCCTCP::BUF_SIZE];
	DCCTCP::WorkerArgs* tt = new DCCTCP::WorkerArgs();


	signal(SIGINT, DCCTCP::sig_handler);
	signal(SIGKILL, DCCTCP::sig_handler);
	signal(SIGTERM, DCCTCP::sig_handler);

	memset(DCCTCP::MY_HOST,0,HOST_NAME_MAX);
	gethostname(DCCTCP::MY_HOST,HOST_NAME_MAX);
	printf("\t*I am %s\n",DCCTCP::MY_HOST);
	//allocate

	//parse args
	if (argc < 2){
		printf("\t*No interface given as argument\n");
	} 

	if (argc>2) 
		strncpy(ports,argv[3],DCCTCP::BUF_SIZE);
	else
		snprintf(ports,DCCTCP::BUF_SIZE,"%d",dcc_port);

	fprintf(stdout,"Port: %s\n",ports);
	addr_hints.ai_family=AF_INET;
	addr_hints.ai_socktype = SOCK_STREAM;
	addr_hints.ai_flags = AI_PASSIVE;

	getaddrinfo(NULL,ports,&addr_hints,&addr_res);

	for (addrinfo* p=addr_res;p!=NULL;p=p->ai_next){

		sock_fd = socket(p->ai_family,p->ai_socktype,p->ai_protocol);
		ter = errno;
		if (sock_fd<0 ){
			fprintf(stderr,"invalid fd! (%s)\n",strerror(ter));
			continue;
		}


		if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt,
					sizeof(int)) == -1) {
			ter = errno;
			fprintf(stderr,"sock opt failed! (%s)\n",strerror(ter));
			continue;
		}

		res = bind(sock_fd,p->ai_addr,p->ai_addrlen);
		if (res<0){
			close(sock_fd);
			ter = errno;
			char buf[DCCTCP::BUF_SIZE];
			inet_ntop(AF_INET,&((struct sockaddr_in*)p)->sin_addr,buf,DCCTCP::BUF_SIZE);
			fprintf(stderr,"DCCTCP bind failed on %s! (%s)\n",
					buf,
					strerror(ter));
			continue;
		} else {
			char buf[DCCTCP::BUF_SIZE];
			inet_ntop(AF_INET,&((struct sockaddr_in*)p)->sin_addr,buf,DCCTCP::BUF_SIZE);
			printf("\t*DCCTCP bind ok on %s:%s\n",
					buf,
					ports	
					);
		}

	}
	fcntl(sock_fd, F_SETFL, O_NONBLOCK);  // set to non-blocking
	freeaddrinfo(addr_res);
	struct sockaddr_storage them;
	socklen_t tsz = sizeof them;
	listen(sock_fd,DCCTCP::MAX_THREAD);

	printf("\t*Checking ROS\n");

	bool ros_up = false;
	bool printedRosError = false;
	ros::Publisher dcc_out;
	ros::Publisher dcc_out2;
	ros::Publisher dcc_temp;
	ros::Publisher dcc_error;
	ros::ServiceServer dcc_powerup;
	ros::ServiceServer dcc_powerdown;
	if (!ros::isInitialized()){
		ros::init(argc, argv, "DCCTCP");
	}


	while(sig_OK){

		if (!ros::master::check()){
			if (ros_up){//not anymore
				dcc_out.shutdown();
				ros_up = false;
				printedRosError=false;
			}
			if (!printedRosError){
				printf("\t*ROS looks down. proceeding without it\n");
				printedRosError = true;
			}
		} else if (!ros_up){ 
			//Master is up, but we haven't initialized our ROS stuff
			if (!ros::isInitialized()){
				ros::init(argc, argv, "DCCTCP");
			}
			ros::NodeHandle nh("/DCCTCP");
			if (!ros::ok()){ 
				
				//master up but there's issues that prevent us from running
				fprintf(stderr,"Everything was fine, then lost ROS\n");
				
			} else {
				ros_up = true;
				ROS_INFO("Seting up ROS node");
				dcc_out = nh.advertise<dcc::RosDccPulseData>("/dcc/ps",10);
				dcc_out2 = nh.advertise<dcc::RosDccPulseData>("/sensors/antenna",10);
				dcc_temp = nh.advertise<std_msgs::Float32>("/sensors/dcc_temp",10);
				dcc_error = nh.advertise<std_msgs::Int8>("/dcc/status",10);
				dcc_powerup = nh.advertiseService("/dcc/on",DCCTCP::dcc_ON);
				dcc_powerdown= nh.advertiseService("/dcc/off",DCCTCP::dcc_OFF);
				printf("\t*Stealing sig Handler back from ROS \n");
				signal(SIGINT, DCCTCP::sig_handler);
				signal(SIGKILL, DCCTCP::sig_handler);
				signal(SIGTERM, DCCTCP::sig_handler);
			}
		}
		if (ros_up){
			ros::spinOnce();
		}


		int new_fd = accept4(sock_fd, (struct sockaddr *)&them, &tsz,0);
		if (new_fd == -1) {
			//resource temporarily unavailable, no doubt
		} else {
			char buf[DCCTCP::BUF_SIZE];
			memset(buf,0,DCCTCP::BUF_SIZE);
			inet_ntop(them.ss_family,
					DCCTCP::get_in_addr((struct sockaddr *)&them),
					buf, sizeof buf);
			printf("server: got connection from %s\n", buf);
		
			int recd=0;
			int totrec = recd;
			//while (sig_OK && recd>0){
				recd = recv(new_fd,&buf[recd],DCCTCP::BUF_SIZE-1-recd,0);
				//totrec += recd>0?recd:0;
				fprintf(stdout,"rec:%d -> %d\n",recd,totrec);
			//}
			std::map<std::string,std::string> data;
			MessageToMap(std::string(buf),data,false); //no meta data please

			printf("Got: %Zu items\n",data.size());

			std::map<std::string,std::string> out_data;
			out_data["dcc_on"]=DCCTCP::dcc_on?"true":"false";
			std::string out_msg = MapToMessage(DCCTCP::MY_HOST,out_data);
			int sent = 0;
			if ( (sent = send(new_fd,out_msg.c_str(),out_msg.size()+1,0)) == -1){
				perror("sending");
			}	else {
				printf("Sent ok!\n");
			}

			for (std::map<std::string,std::string>::iterator it=data.begin();
					it!=data.end();it++){
				printf("%s=%s\n",it->first.c_str(),it->second.c_str());
				if (ros_up){
					dcc::RosDccPulseData msg;
					//msg.header.time_stamp= ros::Time::now();
					if ( strcmp(it->first.c_str(),"Temp") == 0){
						std_msgs::Float32 data;
						data.data = atof(it->second.c_str());
						dcc_temp.publish(data);
					}else if ( strcmp(it->first.c_str(),"Error") == 0){
						std_msgs::Int8 data;
						data.data = atoi(it->second.c_str());
						dcc_error.publish(data);
					} else {
						msg.pulse =1.1; //lie to me
						msg.strength= (int)(atof(it->second.c_str())*100.0);
						msg.frequency= (int)(atof(it->first.c_str())*1000.0);
						dcc_out.publish(msg);
						dcc_out2.publish(msg);
					}
					//publish
				}
			}
			fflush(stdout);
			close(new_fd);
		}
		usleep(1e4); //100hz inner loop. Yeah, polling blah blah
	}
	close(sock_fd);
	delete tt;
	printf("Done!\n");
	return EXIT_SUCCESS;
}
