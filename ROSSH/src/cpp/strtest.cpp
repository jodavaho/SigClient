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
	PortWriter* pout = NULL;
	if (argc<2){
		fprintf(stderr,"defaulting to stdout\n");
		fd = 0;
		pout = new PortWriter(fd);
	} else {
		fd = open_port(argv[1]);
		fd = configure_port(fd);
		pout = new PortWriter(fd);
	}

	if (pout==NULL){
		fprintf(stderr,"pout==NULL\n");
		return EXIT_FAILURE;
	}

	dccmsg sg = dccmsg::SetGain(50);
	dccmsg sf = dccmsg::SetFreq(100);
	pout->send(sf);
	pout->send(sg);
	pout->send(sf);
	pout->send(sg);
	pout->send(sf);
	pout->send(sg);
	pout->send(sf);
	pout->send(sg);
	pout->send(sf);
	pout->send(sg);
	while(pout->busy()){
		usleep(1000);
	}
	printf("Sent...\n");
	delete pout;
	close(fd);
	return EXIT_SUCCESS;
}
