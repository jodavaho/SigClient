
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

int main(int argc, char** argv){
	int fd;
	if (argc<2){
		fprintf(stderr,"Need a port!...Printing to stdout\n");
		fd = 0;
	} else {
		fd = open_port_write(argv[1]);
		fd = configure_port(fd);
	}
	PortWriter* pr = new PortWriter(fd);
	AckMsg a;
	(*pr) << a;
	while(pr->busy()){
		usleep(1000);
	}
	delete pr;
	close(fd);
	return 0;
}
