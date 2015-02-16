
#include <serialcoms/PortBuffer.h>
#include <serialcoms/PortUtils.h>
#include <serialcoms/PortMessage.h>
#include <ros/ros.h>
#include <dcc/dccmsg.h>

//C++
#include <stdio.h> 
#include <string.h> 
#include <time.h>  
#include <vector>
#include <fstream>
#include <iostream>

//Unix
#include <pthread.h>
#include <termios.h> 
#include <fcntl.h> 
#include <errno.h> 
#include <unistd.h> 
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#define DEFAULT_GAIN 50
int main(int argc, char** argv){
	int fd;
	unsigned int gain;
	PortWriter* pout = NULL;
	if (argc<3){
		fprintf(stderr,"Need a port and gain!...Printing to %s with gain = %d\n","stdout",DEFAULT_GAIN);
		gain = DEFAULT_GAIN;
		fd = 0;
		pout = new PortWriter(fd);
	}else if (argc<2){
		fprintf(stderr,"Need a gain as argv[2] defaulting to %d\n",DEFAULT_GAIN);
		fd = open_port(argv[1]);
		fd = configure_port(fd);
		pout = new PortWriter(fd);
		gain = DEFAULT_GAIN;
	} else {
		fd = open_port(argv[1]);
		fd = configure_port(fd);
		gain = atoi(argv[2]);
		pout = new PortWriter(fd);
	}

	if (pout==NULL){
		fprintf(stderr,"pout==NULL\n");
		return EXIT_FAILURE;
	}

	dccmsg sf = dccmsg::SetGain(gain);
	pout->send(sf);
	while(pout->busy()){
		usleep(1000);
	}
	printf("Sent...\n");
	delete pout;
	close(fd);
	return EXIT_SUCCESS;
}
