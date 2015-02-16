#include <dcc/dccmsg.h>
#include <sstream>
#include <string>
#include <string.h>
#include <cstdlib>

dccmsg::~dccmsg(){
	if (data!=NULL){delete[] data;data=NULL;}
}

dccmsg::dccmsg(){
	data = NULL;
	length=0;
}
dccmsg::dccmsg(const char type_byte){
	type = type_byte;
	int nmsg_len = getMaxPacketLength(type_byte);
	data = new char[nmsg_len];
	length = nmsg_len;
	char* dp = data;
	dp[0]=START_COMM;
	dp[1]=nmsg_len;
	dp[2]=type_byte;
	if (nmsg_len>3){
		memset(dp+3,0,nmsg_len-3);
	}
}
dccmsg::dccmsg(const char type_byte,const size_t payloadsz,const char* payload){
	type = type_byte;
	int nmsg_len = getMaxPacketLength(type_byte);
	data = new char[nmsg_len+payloadsz];
	length = nmsg_len+payloadsz;
	char* dp = data;
	dp[0]=START_COMM;
	dp[1]=nmsg_len;
	dp[2]=type_byte;
	if (payloadsz>0){
		memcpy(&dp[3],payload,payloadsz);
	}
}
dccmsg::dccmsg(const dccmsg&other){
	if (other.length>0){
		data = new char[other.length];
		length=other.length;
		type = other.type;
		memcpy(data,other.data,other.length);
	}
}
dccmsg::dccmsg(size_t sz, const char* msg){
	//MARK("dccmsg(char*,len)");
	if (sz > R_MAX_PACKET_SIZE-2) sz = R_MAX_PACKET_SIZE;
	length = sz+2;
	data = new char[sz+2];
	memcpy(&data[2],msg,sz);
	data[0]=START_COMM;
	data[1]=sz+2;
}

const dccmsg dccmsg::Ack(){
	return AckMsg();
}
const dccmsg dccmsg::SetRaw(const char k){
	return dccmsg(SET_RAW,1,&k);
}
const dccmsg dccmsg::GetDevID(){
	return dccmsg(GET_DEV_ID);
}
const dccmsg dccmsg::Reset(){
	return dccmsg(RESET);
}
const dccmsg dccmsg::LogAerial(const char table){
	return dccmsg(LOG_AERIAL,1,&table);
}
const dccmsg dccmsg::LogManual(const int f){
	char tbuf[2];
	char* ip = (char*)(&f);
	tbuf[0]=ip[1];
	tbuf[1]=ip[0];
	dccmsg nmsg(LOG_MANUAL,2,tbuf);
	return nmsg;
}
const dccmsg dccmsg::SetFreq(const int i) {
	char tbuf[2];
	const char* ip = reinterpret_cast<const char*>(&i);
	tbuf[0]=ip[1];
	tbuf[1]=ip[0];
	dccmsg nmsg(SET_FREQ,2,tbuf);
	return nmsg;
}
const dccmsg dccmsg::SetFreq(std::vector<int> freqs){
	int cnt = (int) freqs.size();
	dccmsg nmsg;
	nmsg.type = SET_FREQ;
	int nmsg_len = 2+1+cnt*2;
	nmsg.data = new char[nmsg_len];
	nmsg.length = nmsg_len;
	char* dp = nmsg.data;
	dp[0]=START_COMM;
	dp[1]=nmsg_len;
	dp[2]=SET_FREQ;
	for (int i=0;i<cnt;i++){
		char* cp = (char*)&freqs[i];
		dp[3+2*i]=cp[1];
		dp[3+2*i+1]=cp[0];
	}
	return nmsg;
}
const dccmsg dccmsg::SetFreq(size_t cnt, int* freqs){
	dccmsg nmsg;
	nmsg.type = SET_FREQ;
	int nmsg_len = 2+1+cnt*2;
	nmsg.data = new char[nmsg_len];
	nmsg.length = nmsg_len;
	char* dp = nmsg.data;
	dp[0]=START_COMM;
	dp[1]=nmsg_len;
	dp[2]=SET_FREQ;
	for (size_t i=0;i<cnt;i++){
		char* cp = (char*)&freqs[i];
		dp[3+2*i]=cp[1];
		dp[3+2*i+1]=cp[0];
	}
	return nmsg;
}
const dccmsg dccmsg::SetGain(int gain){
	dccmsg nmsg;
	nmsg.type = CHANGE_GAIN;
	int nmsg_len = getMaxPacketLength(CHANGE_GAIN);
	nmsg.data = new char[nmsg_len];
	nmsg.length = nmsg_len;
	char* dp = nmsg.data;
	dp[0]=START_COMM;
	dp[1]=nmsg_len;
	dp[2]=CHANGE_GAIN;
	char* cp = (char*)&gain;
	dp[3]=cp[1];
	dp[4]=cp[0];
	return nmsg;
}

int dccmsg::getFrequency(){
	if (!isOK()){return -1;}
	int ret=0;
	char* dp = reinterpret_cast<char*>(&ret);
	dp[1] = data[3];
	dp[0] = data[4];
	return ret;
}

int dccmsg::getSS(){
	if (!isOK()){return -1;}
	int ret=0;
	char* dp = reinterpret_cast<char*>(&ret);
	dp[0] = data[8];
	return ret;
}

int dccmsg::getPulse(){
	if (!isOK()){return -1;}
	int ret=0;
	char* dp = reinterpret_cast<char*>(&ret);
	dp[0] = data[7];
	return ret;
}
int dccmsg::size(){
	return length;
}
bool dccmsg::isOK(){
	return (data!=NULL && length>0);
}

char dccmsg::get(int idx){
	if (data!=NULL && idx>=0 && idx<length){return data[idx];}
	else {return -1;}
}

bool dccmsg::isAck(){
	return isOK() && type==ACK;
}

bool dccmsg::isSignal(){
	return (size()==9) && isOK() && data[2]=='S';
}

/*
 * Aerial: 
 * --------------------------
 *  FR 1(00)48000
 *  Period: 0000 0
 *  Pulse Rate: 000   [No|Fix]
 *  SIG 000
 * -------------------------
 * Manual:
 * --------------------------
 *  Freq: XXXXX
 *  Period: 0000 0
 *  Pulse Rate: 000  [No|Fix]
 *  SIG 000
 * --------------------------
 * Bonus pts for [Low|Bat]
 *
 */
bool screenmsg::createFrom(std::list<char>::iterator& b,std::list<char>::iterator& e){
	int len = distance(b,e);
	if (len<80) {return false;}
	std::stringstream ss;
	for (;b!=e;b++){
		ss<<*b;
	}
	std::string s = ss.str();	
	printf("*****************************************\n");
	printf("\t%s\n",s.c_str());
	printf("*****************************************\n");
	printf("d:%Zu\n",distance(b,e));
	return true;
}
bool rawmsg::createFrom(std::list<char>::iterator& b,std::list<char>::iterator& e){
	std::stringstream ss;
	std::list<char>::iterator b2 = b;
	int len = distance(b2,e);
	if (len<80) {
		return false;}
	for (;b2!=e;b2++){
		ss<<*b2;
		//printf("%s",ss.str().c_str());
	}
	std::string s = ss.str();	
	size_t prp,pdp,sigp,fp,fp2=0;
	if ((fp=s.find("FR"))==std::string::npos && 
			(fp2=s.find("Freq:"))==std::string::npos) {
		b=e;	
		return true;}
	if ((prp=s.find("Pulse Rate:"))==std::string::npos) {
		return false;}
	if ((sigp=s.find("SIG"))==std::string::npos || len-sigp<6) {
		return false;}
	if ((pdp=s.find("Period:"))==std::string::npos) {
		return false;}
	//ok, have a message
	std::string frs;
	bool aerial = true;
	if (fp!=std::string::npos){
		frs = s.substr(fp,15);
	} else {
		frs = s.substr(fp2,15);
		fp = fp2;
		aerial=false;
	}
	std::string prs = s.substr(prp,16);	
	std::string pds = s.substr(pdp,14);	
	std::string sigs= s.substr(sigp,10);
	printf("+****************************************\n");
	printf("|\t%s\n",frs.c_str());
	printf("|\t%s\n",pds.c_str());
	printf("|\t%s\n",prs.c_str());
	printf("|\t%s\n",sigs.c_str());
	printf("+****************************************\n");

	if (!aerial){
		freq = atoi(frs.substr(frs.find(':')+1).c_str());
	} else {
		freq = atoi(frs.substr(frs.find(')')+1).c_str());
	}

	pulse = atoi(prs.substr(prs.find(':')+1).c_str());
	str = atoi(sigs.substr(4).c_str());
	int advlen = fp+78;
	while(advlen--){
		b++;}
	printf("f:%d,p:%d,s:%d,d:%Zu\n",freq,pulse,str,distance(b,e));
	return true;
}
bool dccmsg::createFrom(std::list<char>::iterator &b,std::list<char>::iterator &e){
	//printf("Creating packet\n");
	//find START_COMM. Iterating this will remove these items from buffer.
	while(*b!=START_COMM&&b!=e){
		b++;
	}
	if ((*b)!=START_COMM){
		printf("Didn't find packet start\n");
		return true;
	}

	int len = std::distance(b,e);
	//check packet length
	b++;
	int count = (int) *b;
	b--;
	//error condition, return with the pointers where they are.
	if (count>len || len<0 ){return false;}
	length = len;

	if (data!=NULL){delete[] data;data=NULL;}
	data = new char[len];
	int i =0;
	while(len--){
		data[i++]=*b++;
	}
	type = data[2];
	//portbuffer cleans up after us.
	return true;
}

