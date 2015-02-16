
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

#define FREQ_DEFAULT 50000
int main(int argc, char** argv){
	int fd;
	int* freq=NULL;
	int fc;
	fc = argc-2;
	PortWriter* pout = NULL;
	if (argc==1){
		fprintf(stderr,"Need a port and freq!...Printing to %s with freq = %d\n","stdout",FREQ_DEFAULT);
		fc = 1;
		fd = 0;
		freq =new int[1];
		freq[0]=50000;
		pout = new PortWriter(fd);
	}else if (argc==2){
		fprintf(stderr,"Need a freq as argv[2] defaulting to %d\n",FREQ_DEFAULT);
		fc = 1;
		freq =new int[1];
		freq[1]=50000;
		fd = open_port(argv[1]);
		fd = configure_port(fd);
		pout = new PortWriter(fd);
	} else {
		fd = open_port(argv[1]);
		fd = configure_port(fd);
		pout = new PortWriter(fd);
		freq = new int[fc];
		for (int i=0;i<fc;i++){
			freq[i]=atoi(argv[2+i]);
		}
	}

	if (pout==NULL){
		fprintf(stderr,"pout==NULL\n");
		return EXIT_FAILURE;
	}
	dccmsg sf = dccmsg::SetFreq(fc,freq);
	pout->send(sf);
	while(pout->busy()){
		usleep(1000);
	}
	printf("Sent...\n");
	delete pout;
	delete[]freq;
	close(fd);
	return EXIT_SUCCESS;
}
