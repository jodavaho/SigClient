#include <serialcoms/PortUtils.h>
#include <serialcoms/PortBuffer.h>
#include <dcc/dccmsg.h>

int main(int argc, char** argv){
	if (argc<2){fprintf(stderr,"Need a port!");}
	ScopedPortLock spl(argv[1],9600,'x');
	PortBuffer pr(spl.getFD());
	while(1){
		if(pr.available()>0){
			rawmsg rm;
			pr>>rm;
		}
		sleep(.1);
	}
}
