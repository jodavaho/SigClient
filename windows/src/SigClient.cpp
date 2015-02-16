#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <cstring>
#include <vector>
#include <sstream>
#include <dcc/DCCTCP.h>

//#include <Windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>

#pragma comment(lib, "Ws2_32.lib")

//Adjust project settings to  include 
//$(HOMEPATH)/Desktop/carpMonitoring/heartbeat/include
#include "networking/protocol.h"
/**Local API **/
#include "BB60APIW32.h"
/** Expand the number of timers in helpers.h**/
#define _MAX_TIMERS 128
#include "helpers/jv_help.h"
#define er( a1, a2 ) sig_dog::_lerchk(a1,a2,__FILE__,__LINE__)
/**************************************************************************/
/*Socket context as global variables? Sure! */


//Should be a class, bro
namespace sig_dog{
	int cur_error;
	float temp,v1,v2,usbVoltage,usbCurrent;
	const int BUF_SIZE = 512;
	const int  T_t=(_MAX_TIMERS-1);
	const int  T_c=(_MAX_TIMERS-2);
	const int  T_s=(_MAX_TIMERS-3);

	bool DCC_ON;
	SOCKET serverSocket;//broadcast socket FD
	sockaddr_in recver;//listener: Only use for error checking if at all
	sockaddr_in bdcast_out;//outgoing address: Broadcast
	char hostname[BUF_SIZE];//hostname of this device, used for message headers


	//Bucket struct for keeping track of signal variances and means
	class SigBucket{
	public:
		SigBucket(){}
		void initialize_bucket(double f_min,double f_max);
		double avg;
		double max_trace;
		double mhd;
		double freq_min;
		double freq_max;
		double freq_var;
		double period;
		int idx;
		static const int kHistMax = 4095;
		double hist[kHistMax];
	};		

	bbStatus status;		//Signal Hound device status struct, used to check for errors
	int devID;		//Device ID (used if multiple devices connected)
	unsigned int traceSize;		//Size of the trace: Number of buckets in a sweep. A few hundred, hopefully
	double binSize;		//bin size (frequency width of a bucket)
	double startFreq;		//Lowest frequency (the min freq of the first bucket)
	double*mins;		//Returned minimum signal value
	double*maxs;		//returned maximum detected signal value: maxs[traceSize]
	double*avg;		//average signal value: mins[traceSize]
	sig_dog::SigBucket* buckets;		//array struct to keep track of signal properties: size=traceSize


	/**************************************************************************/

	int openSH();
	int openComs(int port=DCCTCP::dcc_port);
	int sampleBKG( double bkgSampleTime =30.0);
	int goSigDog(int numFreq,double* freqList,double vals[], double sampleTime=2.2, int notag=0, double notagList[]=NULL);
	int sendVals(int N, double freqList[], double vals[]);
	void closeSH();
	int closeComs();
	double SS(SigBucket *b);
	int detection(SigBucket*b,int samples = 1);
	bool _lerchk(bbStatus &status,int devID,char* msg,int num);

	int bucket_SS_desc(const void* b1, const void* b2);
	int bucket_max_desc(const void* b1, const void* b2);
	int bucket_norm_desc(const void* b1, const void* b2);
	int bucket_mhd_desc(const void* b1, const void* b2);
}

/**************************************************************************/

//error checking the bbstatus.  use the *er* macro instead
bool  sig_dog::_lerchk(bbStatus &status,int devID,char* msg,int num){
	if (status!=bbNoError){
		_fmecho("Signal Hound Error", bbGetErrorString(status),msg,num, 0,"on DevID: %d\n",devID );
		return false;
	} else {
		//_fmecho("OK", bbGetErrorString(status),msg,num, 0,"on DevID: %d\n",devID );
		return true;
	}
}

/**initialization helper. Essentially zeros it out.*/
void sig_dog::SigBucket::initialize_bucket(double f_min,double f_max){
	freq_min = f_min;
	freq_max = f_max;
	avg =0.0;
	max_trace = -9999999.0;
	freq_var = 0.0;
	period=0.0;
}

//for sorting buckets with qsort(). Sorts by Mahalanobis distance of max: (max-avg)/sqrt(var) descending.
int sig_dog::bucket_mhd_desc(const void* b1, const void* b2){
	SigBucket* bp1 = (SigBucket*) b1;
	SigBucket* bp2 = (SigBucket*) b2;
	if (bp1->mhd/bp1->avg > bp2->mhd/bp2->avg)
		return -1;
	if (bp1->mhd/bp1->avg < bp2->mhd/bp2->avg)
		return 1;
	return 0;
}
//for sorting buckets with qsort(). 
int sig_dog::bucket_norm_desc(const void* b1, const void* b2){
	SigBucket* bp1 = (SigBucket*) b1;
	SigBucket* bp2 = (SigBucket*) b2;
	if (bp1->max_trace/bp1->avg > bp2->max_trace/bp2->avg)
		return -1;
	if (bp1->max_trace/bp1->avg < bp2->max_trace/bp2->avg)
		return 1;
	return 0;
}

//for sorting buckets with qsort(). Sorts by max amplitude descending
int sig_dog::bucket_max_desc(const void* b1, const void* b2){
	SigBucket* bp1 = (SigBucket*) b1;
	SigBucket* bp2 = (SigBucket*) b2;
	if (bp1->max_trace > bp2->max_trace)
		return -1;
	if (bp1->max_trace < bp2->max_trace)
		return 1;
	return 0;
}

//returns the preceived signal strength of the single bucket pointed to by SigBucket*b
//Should be something like (max-avg)/sqrt(var)
double sig_dog::SS(sig_dog::SigBucket *b){
	return (b->max_trace-b->avg)/sqrt(b->freq_var);
}
/*for sorting buckets with qsort(). Sorts by SS descending*/
int sig_dog::bucket_SS_desc(const void* b1, const void* b2){
	sig_dog::SigBucket* bp1 = (sig_dog::SigBucket*) b1;
	sig_dog::SigBucket* bp2 = (sig_dog::SigBucket*) b2;
	double ss1 = SS(bp1);
	double ss2 = SS(bp2);
	if (ss1 > ss2)
		return -1; //descending
	if (ss1 < ss2)
		return 1;
	return 0;
}

//returns 1 if the SS(b)>T for some threashold T. Not so useful, do this yourself
int sig_dog::detection(sig_dog::SigBucket*b,int samples){
	return sig_dog::SS(b)>1.5;
}


/*****************************COMS*****************************************/


//close the socket, deallocate working memory, and shut down WinSock
int sig_dog::closeComs(){
	shutdown(sig_dog::serverSocket,SD_BOTH);
	closesocket(serverSocket);
	WSACleanup ();
	return 0;
}

//opens and prepares sockets for communication. non-zero means error.
int sig_dog::openComs(int port){

	int iResult;
	WSADATA wsaData;
	char pbuf[256];
	struct addrinfo hits;
	struct addrinfo server_addr,*res; //we're hardcoding the server addr like chumps

	//damn legacy interface:
	int YES = 1;
	int NO = 0;

	//for checking ifaces:
	struct addrinfo *p;

	sprintf_s(pbuf,256,"%d",port);

	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		FINFO("WSAStartup failed: %d\n", iResult);
		return 1;
	}
	FINFO("Port %d\n",port);

	gethostname(hostname,BUF_SIZE);
	memset(&server_addr,0,sizeof server_addr);

	server_addr.ai_family = AF_INET;
	server_addr.ai_socktype = SOCK_STREAM;
	server_addr.ai_flags = AI_PASSIVE;

	//int gar = getaddrinfo("10.42.43.21",pbuf,&server_addr,&res);
	//int gar = getaddrinfo("10.42.43.40",pbuf,&server_addr,&res); //Conan
	int gar = getaddrinfo("10.42.43.99",pbuf,&server_addr,&res); //Conan
	//int gar = getaddrinfo("10.42.43.39",pbuf,&server_addr,&res); //Beast
	if (gar!=0){FINFO("weird return!\n")};

	//iterate through returned interfaces
	p=res;
	while(p!=NULL){
		serverSocket = socket(p->ai_family , p->ai_socktype,p->ai_protocol );
		if (serverSocket == INVALID_SOCKET){
			closesocket(serverSocket);
			continue;
		} else {
			FINFO("Socket is ok\n");
		}
		//set re-use for the heck of it
		if (setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR ,"true",strlen("true"))==-1){
			perror("set opt");
			continue;
		}else {
			FINFO("Set opt ok\n");
		}
		if (gar=connect(serverSocket,p->ai_addr,p->ai_addrlen)==-1){
			int wsaerr = WSAGetLastError ();
			FINFO("Error in connect():%d\n",wsaerr);
			if (wsaerr == 10060){
				FINFO("Connection Timeout\n");
			} else if (wsaerr == 10061){
				FINFO("Connection Refused\n");
			}
		} else {
			FINFO("Connect is OK!\n");
			return 0;
		}
		p= p->ai_next;
	}

	freeaddrinfo(res);

	return 1;
}

//Sends the *N* items in freqList[] and vals[] as key,value pairs out over the network. assumes openComs() was successful.
int sig_dog::sendVals(int N, double freqList[], double vals[]){

	std::stringstream ss;

	ss<<"host="<<hostname<<SimpleProtocol::pair_delim_str[0];
	ss<<SimpleProtocol::msg_start_str[0];
	ss<<"Temp"<<SimpleProtocol::key_value_delim_str[0]<<sig_dog::temp<<SimpleProtocol::pair_delim_str[0];
	ss<<"Error"<<SimpleProtocol::key_value_delim_str[0]<<sig_dog::cur_error<<SimpleProtocol::pair_delim_str[0];
	for (int i=0;i<N;i++){
		ss<<freqList[i]<<SimpleProtocol::key_value_delim_str[0]<<vals[i];
		if (i<N-1)
			ss<<SimpleProtocol::pair_delim_str[0];
	}
	ss<<SimpleProtocol::msg_stop_str[0];
	int sent=ss.str().length();
	int recd=0;
	sent = send(serverSocket,ss.str().c_str(),ss.str().length()+1,0);
	FINFO("Sent:%d (%s)\n",sent,ss.str().c_str());
	char buf[BUF_SIZE];
	memset(buf,0,BUF_SIZE);
	if ((recd = recv(serverSocket,buf,BUF_SIZE,0)) == -1){
		perror("recv");
	} else {
		FINFO("Rec:%d (%s)\n",recd,buf);
		if (strstr(buf,"dcc_on=true")){
			sig_dog::DCC_ON=true;
		} else if (strstr(buf,"dcc_on=false")){
			sig_dog::DCC_ON=false;
		}
	}


	return 0;
}

//opens SH with useful defaults. non-zero means error.
int sig_dog::openSH(){
	int numTries=10;

	INFO("Starting!\n");
	start_timer(T_s);
	status = bbOpenDevice(&devID);
	FINFO("bbOpenDevice: %f\n",stop_timer(T_s));
	if (!er(status,devID)){return -1;}

	unsigned int sn;
	status = bbGetSerialNumber(devID,&sn);
	FINFO("Serial Number: %d\n",sn);

	//status=bbConfigureAcquisition(devID,BB_MIN_AND_MAX,BB_LOG_SCALE);
	//status=bbConfigureAcquisition(devID,BB_MIN_AND_MAX,BB_VOLTAGE);
	status=bbConfigureAcquisition(devID,BB_AVERAGE,BB_VOLTAGE);
	if (!er(status,devID)){return -1;}

	//Ref level = -20 db, auto attenuation
	status=bbConfigureLevel(devID,-2.0e1,-1);
	if (!er(status,devID)){return -1;}

	status=bbConfigureGain(devID,BB_AUTO_GAIN);
	if (!er(status,devID)){return -1;}

	// 49.6 Mhz center, 1 Mhz span
	status = bbConfigureCenterSpan(devID,49.6e6,.5e6);
	if (!er(status,devID)){return -1;}

	//RBW = .005 Mhz, VBW = RBW (necessary)
	status=bbConfigureSweepCoupling(devID,5e3,5e3,1.0e-3,BB_NATIVE_RBW,BB_NO_SPUR_REJECT);
	if (!er(status,devID)){return -1;}

	status=bbConfigureWindow(devID,BB_BLACKMAN);
	if (!er(status,devID)){return -1;}

	//status=bbConfigureProcUnits(devID, BB_LOG);
	status=bbConfigureProcUnits(devID, BB_VOLTAGE);
	if (!er(status,devID)){return -1;}

	start_timer(T_s);
	status=bbInitiate(devID,BB_SWEEPING,0);
	//status=bbInitiate(devID,BB_REAL_TIME,0);
	FINFO("bbInitiate: %f\n",stop_timer(T_s));
	if (!er(status,devID)){return -1;}

	status = bbQueryTraceInfo(devID,&traceSize,&binSize,&startFreq);
	if (!er(status,devID)){return -1;}

	FINFO("Trace Size:%d, start freq: %lf Bin: %lf\n",traceSize,startFreq,binSize);
	FINFO("Gives: %lf - %lf span\n",startFreq, startFreq+traceSize*binSize);

	/** Allocate room for query **/
	mins = new double[traceSize];
	maxs = new double[traceSize];
	avg = new double[traceSize];

	/** Initialize buckets **/
	buckets = new SigBucket[traceSize];
	for (unsigned int i=0;i<traceSize;i++){
		buckets[i].initialize_bucket (startFreq + i*binSize,startFreq+(i+1)*binSize);
		buckets[i].idx =(int) i;
	}
	return 0;
}
//call whenever you want to profile the background. Fills the bucket array's freq_var fields.
int sig_dog::sampleBKG( double bkgSampleTime){
	double sweeptime=0.0;
	double maxsweeptime=0.0;
	double proctime =0.0;
	const int N=8192;
	int idx;
	start_timer(1);
	double** samples=new double*[traceSize];
	for (unsigned int i=0;i<traceSize;i++){
		samples[i]=new double[N];
	}
	WARN("Only using running approx of variance!");
	for (idx =0;idx<N && since_start(1)<bkgSampleTime;idx++){
		start_timer(0);
		status = bbFetchTrace(devID,traceSize,mins,maxs);
		sweeptime += (stop_timer(0)-sweeptime)/(idx+1);
		maxsweeptime=max(maxsweeptime,get_time(0));
		if (!er(status,devID)){return -1;}
		start_timer(2);
		for (unsigned int j = 0;j<traceSize;j++){
			samples[j][idx] = ( .5*maxs[j]+.5*mins[j]);
		}
		proctime+=stop_timer(2);
	}
	//get avgs
	for (int k=0;k<idx;k++){
		for (unsigned int j = 0;j<traceSize;j++){
			samples[j][0] += (samples[j][k] - samples[j][0])/(k+1);
		}
	}
	for (unsigned int j = 0;j<traceSize;j++){
		buckets[j].avg = samples[j][0];
	}
	//get vars
	for (int k=0;k<idx;k++){
		for (unsigned int j = 0;j<traceSize;j++){
			buckets[j].freq_var += pow(samples[j][k]-buckets[j].avg, 2.0)/(k+1);
		}
	}

	FINFO("Background check: Over %lf seconds, %d sweeps. Avg sweep: %lf, Max: %lf. proc: %lf\n",stop_timer(1),idx+1,sweeptime,maxsweeptime,proctime);
	for (unsigned int i=0;i<traceSize;i++){
		delete[] samples[i];
	}
	delete samples;
	return 0;
}
//main loop should include this, which calls the SH to get frequency samples. returns signal strength in the vals[] array. Uses
//the bucket arrays, avg, mins, maxs arrays as working space so don't touch those. assumes openSH was successful.
int sig_dog::goSigDog(int numFreq,double*freqList,double l_vals[], double sampleTime, int notag, double notagList[]){

	/** Time keeping **/
	double sweeptime=0.0;
	double maxsweeptime=0.0;
	double proctime =0.0;

	/** Max queries **/
	int numTries = 1,idx,N=1000;

	for (unsigned int i=0;i<traceSize;i++){
		buckets[i].max_trace=-99999.9;
	}
	for (int run=0;run<numTries;run++){
		start_timer(1);
		for (idx =0;idx<SigBucket::kHistMax && idx<N && since_start(1)<sampleTime;idx++){
			start_timer(0);
			status = bbFetchTrace(devID,traceSize,mins,maxs);
			sweeptime += (stop_timer(0)-sweeptime)/(idx+1);
			maxsweeptime=max(maxsweeptime,get_time(0));
			if (!er(status,devID)){return -1;}
			start_timer(2);
			for (unsigned int j = 0;j<traceSize;j++){
				//buckets[j].hist[idx]= .5*maxs[j]+.5*mins[j];
				buckets[j].hist[idx]= maxs[j];
			}
			proctime+=stop_timer(2);
		}

	}
	double f, var, mx, avg;
	for (int i=0;i<numFreq;i++){
		f = freqList[i];
		int bidx = (int) floor( (f*1e6-startFreq)/binSize );
		avg=0.0;
		mx=-990.0;
		var=0.0;
		for (int j = 0;j<idx;j++){
			double v = buckets[bidx].hist[j];
			avg += v/idx;
			mx = max(mx,v);
		}
		for (int j = 0;j<idx;j++){
			double v = buckets[bidx].hist[j];
			var += (v - avg)*(v-avg)/(idx);
		}
		FINFO("%f: mx: %f, var: %f, sig: %f, avg: %f\n",f,mx,var,sqrt(var),avg);
		//l_vals[i]=(mx-avg)/(sqrt(var));//sig_dog::SS(&buckets[idx]);
		l_vals[i]=mx*100;
	}


	return 0;
}

//shutdown the SH, deallocate working memory
void sig_dog::closeSH(){
	delete[] buckets;
	delete[] avg;
	delete[] mins;
	delete[] maxs;
}

/**************************************************MAIN******************************/

int main(){
	sig_dog::DCC_ON = false;
	sig_dog::temp = -1;
	double bkgSampleTime = 0.0; //sec
	double sampleTime = 5.0; //sec
	int numTries=10;
	WARN("Using pre-defined values of numFreq and freqList");
	const int numFreq=4;
	double freqList[] = {49.611,49.641,49.691,49.661}; //Mhz  Need at least one bin --> .3 / 3 = .10 bin size
	double notag[] = {49.777, 48.411};
	double mvals[numFreq];

	for (int i=0;i<numFreq;i++){
		mvals[i]=0.0;
	}
	sig_dog::cur_error = 0;

	if (sig_dog::openSH()){FINFO("SH Failed to open\n");return 1;}
	//if (sig_dog::sampleBKG(bkgSampleTime)){FINFO("Error in sampleBKG()\n");return 1;}
	for (;;){
		start_timer(sig_dog::T_c);
		bbStatus status = bbQueryDiagnostics(sig_dog::devID,&sig_dog::temp,&sig_dog::v1,&sig_dog::v2,&sig_dog::usbVoltage,&sig_dog::usbCurrent);
		if (sig_dog::DCC_ON){
			if(sig_dog::goSigDog(numFreq,freqList,mvals,sampleTime)){
				FINFO("Error in goSigDog()\n");
				sig_dog::cur_error = 1;
				sig_dog::DCC_ON = false;
				Sleep(2000);
			} else {
				sig_dog::cur_error = 0;
				FINFO("Fetch: %f\n",stop_timer(sig_dog::T_c));
				for (int i=0;i<numFreq;i++){
					FINFO("  * %f=%lf\n",freqList[i],mvals[i]);
				}
			}
		} else {
			FINFO("SigDog off\n");
			Sleep(2000);
			for (int i=0;i<numFreq;i++){
				mvals[i]=0;
			}
		}
		if (sig_dog::openComs()){
			FINFO("Error in opencoms()\n");
			if (sig_dog::DCC_ON){
				FINFO("  * Shutting down SigHound for now .. \n");
				sig_dog::DCC_ON = false;
			} else {
				FINFO("  * Signal hound already off\n");
			}
		} else {
			if (sig_dog::sendVals(numFreq,freqList,mvals)){FINFO("Error in sending\n");}
			if (sig_dog::closeComs()){FINFO("Error in closing coms\n");}
		}
	}
	sig_dog::closeSH();
	//delete mvals;
	return 0;
}


////////////////////////////////////////////////////OTHER IMPL


///////////////////////////////IMPL//////////////////////////////////
#ifdef WIN32
#define WINDOWS
#endif

#ifdef _WIN32
#define WINDOWS
#endif

#ifdef _WIN64
#define WINDOWS
#endif

#ifdef WINDOWS
/* To cooperate with windows programs */
	#include <Windows.h>
	#include <time.h>

	typedef struct timeval TTYPE;
	void toWC(const char* msg,wchar_t *buf);
	LARGE_INTEGER getFILETIMEoffset();
	int	clock_gettime(int X, TTYPE *tv);
	#define CLOCK_MONOTONIC 1
#else
	typedef struct timespec TTYPE;
#endif

#ifndef _MAX_TIMERS
#define _MAX_TIMERS 16
#endif

TTYPE _start_timers[_MAX_TIMERS];
TTYPE _stop_timers[_MAX_TIMERS];
double _running_timers[_MAX_TIMERS];
int _isrunning[_MAX_TIMERS];

void _dropout(const int errcode){
	exit(errcode);
}

#ifdef WINDOWS
void _fmecho(const char* pref, const char* str,const char* file,int line, int nl,const char* s,...){
	wchar_t buf[4096];
	wchar_t obuf[4096];
	wchar_t fname[4096];
	wchar_t pbuf[64];
	char fbuf[4096];
	wchar_t wfbuf[4096];
	va_list args;
	va_start(args,s);
	vsprintf_s(fbuf,4096,s,args);

	toWC(fbuf,wfbuf);
	toWC(file,fname);
	toWC(str,buf);
	toWC(pref,pbuf);
	if (nl){
		swprintf(obuf,4096,L"%s: (%s,%d) %s %s\n",pbuf,fname,line,buf,wfbuf);
	}
	else{
		swprintf(obuf,4096,L"%s: (%s,%d) %s %s",pbuf,fname,line,buf,wfbuf);
	}
	OutputDebugStringW(obuf);
	wprintf(obuf);
}
void _mecho(const char* pref, const char* str,const char* file,int line, int nl){
	wchar_t buf[4096];
	wchar_t obuf[4096];
	wchar_t fname[4096];
	wchar_t pbuf[64];
	toWC(file,fname);
	toWC(str,buf);
	toWC(pref,pbuf);
	if (nl){
		swprintf(obuf,4096,L"%s: (%s,%d) %s\n",pbuf,fname,line,buf);
	}
	else{
		swprintf(obuf,4096,L"%s: (%s,%d) %s",pbuf,fname,line,buf);
	}
	OutputDebugStringW(obuf);
	wprintf(obuf);
}
#else
void _fmecho(const char* pref, const char* str,const char* fs,int ll, int nl,const char*s, ...){
	char fbuf[4096];
	va_list args;
	va_start(args,s);
	vsnprintf(fbuf,4096,s,args);
#ifdef WIRITING_MPI
	if (nl)
		printf("%s: %s, %d/%d (%s,%d) %s %s\n",pref,proc_name,myid,P-1,fs,ll,str,fbuf);
	else
		printf("%s: %s, %d/%d (%s,%d) %s %s",pref,proc_name,myid,P-1,fs,ll,str,fbuf);
#else
	if (nl)
		printf("%s: (%s,%d) %s\n",pref,fs,ll,str);
	else
		printf("%s: (%s,%d) %s",pref,fs,ll,str);
#endif
       
}
void _mecho(const char* pref, const char* str,const char* fs,int ll, int nl){
#ifdef WIRITING_MPI
	if (nl)
		printf("%s: %s, %d/%d (%s,%d) %s\n",pref,proc_name,myid,P-1,fs,ll,str);
	else
		printf("%s: %s, %d/%d (%s,%d) %s",pref,proc_name,myid,P-1,fs,ll,str);
#else
	if (nl)
		printf("%s: (%s,%d) %s\n",pref,fs,ll,str);
	else
		printf("%s: (%s,%d) %s",pref,fs,ll,str);
#endif
       
}
#endif

#ifdef WINDOWS
void toWC(const char* msg,wchar_t *buf){
	if (msg!=NULL){
		size_t origsize = strlen(msg) + 1;
		size_t convertedChars = 0;
		mbstowcs_s(&convertedChars, buf, origsize, msg, _TRUNCATE);
	} else {
		//wcscat_s(buf, 5, L"NULL");
	}
}


LARGE_INTEGER
getFILETIMEoffset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

int
clock_gettime(int X, TTYPE *tv)
{
    LARGE_INTEGER           t;
    FILETIME            f;
    LONGLONG                  microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 1;

    if (!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter) {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        } else {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) QueryPerformanceCounter(&t);
    else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (LONGLONG)( t.QuadPart / frequencyToMicroseconds);
    t.QuadPart = microseconds;
    tv->tv_sec = (long)( t.QuadPart / 1000000L);
    tv->tv_usec = t.QuadPart % 1000000L;
    return (0);
}
#endif

#ifdef WRITING_MPI
void _mpi_settings(int argc, char** argv){
	// Initialize MPI
	MPI_Init(&argc, &argv);
	// Obtain the number of processes
	MPI_Comm_size(MPI_COMM_WORLD, &P);
	// Obtain the process id
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
	// Obtain name of machine process is executing on
	MPI_Get_processor_name(proc_name, &name_len);
}
#endif

//////////////////////////TIME HELP BELOW//////////////////////////////////

double since_start(const int which){

	TTYPE *start;
	TTYPE now;
	double elapsed;

	start=&(_start_timers[which]);
	clock_gettime(CLOCK_MONOTONIC,&now);
	elapsed  = now.tv_sec;
	elapsed = elapsed - start->tv_sec;

#ifndef WINDOWS
	elapsed += (now.tv_nsec - start->tv_nsec) / 1000000000.0;
#else
	elapsed += (now.tv_usec - start->tv_usec) / 1000000.0;
#endif

	return elapsed;
}

static double _get_time_diff(const int which){
	TTYPE *start,*finish;
	double elapsed;
	start=&_start_timers[which];
	finish=&_stop_timers[which];
	elapsed = (finish->tv_sec - start->tv_sec);
#ifndef WINDOWS
	elapsed += (finish->tv_nsec - start->tv_nsec) / 1000000000.0;
#else
	elapsed += (finish->tv_usec - start->tv_usec) / 1000000.0;
#endif
	return elapsed;
}

double current_time(const int which){
	if (_isrunning[which]){
		return _get_time_diff(which) + _running_timers[which];
	} else {
		return _running_timers[which];
	}
}
double get_time(const int which){
	return current_time(which);
}
void start_timer(const int which){
	_running_timers[which]=0.0;
	continue_timer(which);
}

void continue_timer(const int which){
	clock_gettime(CLOCK_MONOTONIC, &_start_timers[which]);
}

double stop_timer(const int which){
	clock_gettime(CLOCK_MONOTONIC, &_stop_timers[which]);
	_running_timers[which]+=_get_time_diff(which);
	return get_time(which);
}

#define restart_timer(a) start_timer(a)
#define pause_timer(a) stop_timer(a)
