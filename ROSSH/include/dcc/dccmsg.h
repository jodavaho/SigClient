

#ifndef DCC_DCCMSG_H
#define DCC_DCCMSG_H

#define __DEBUG
#include <serialcoms/PortMessage.h>
#include <dcc/protocol.h>
#include <stdarg.h>
#include <vector>

class dccmsg:public PortMessage{
	public:
		dccmsg();
		dccmsg(const dccmsg&);
		/** empty payload construtor, only inits header, size, and type (ie, first 3 bytes). **/
		dccmsg(const char type_byte);
		/** Will initialize size,header, and type fields, then append extra payload (if any starting at msg[3]). Set payloadsz = 0 if none **/
		dccmsg(const char type_byte,const size_t payloadsz,const char* payload);
		/** Assumes size,type, and header fields are included in payload **/
		dccmsg(size_t sz, const char* paypload);

		/**Static initializers**/
		static const dccmsg Ack();
		static const dccmsg GetDevID();
		static const dccmsg Reset();
		static const dccmsg LogAerial(const char table);
		static const dccmsg LogManual(const int f);
		static const dccmsg SetRaw(const char k);
		static const dccmsg SetFreq(const int) ;
		static const dccmsg SetFreq(size_t,int*);
		static const dccmsg SetFreq(std::vector<int> freqs);
		static const dccmsg SetGain(int) ;
		virtual ~dccmsg();

		/** Other useful stuff **/
		bool isOK();
		char get(int);
		int size();
		bool isAck();
		bool isSignal();
		int getFrequency();
		int getSS();
		int getPulse();

		char type;

		/** 
		 * Called by PortBuffer
		 * Searches over the char list for an appropriate message. When completed, the start = the end of the packet, and end is untouched
		 */
		bool createFrom(std::list<char>::iterator& start,std::list<char>::iterator& end);
	protected:
		char* data;
		int length;
};

class AckMsg:public dccmsg{
	public:
		AckMsg(){
			data=new char[3];
			data[0]='A';
			data[1]=(char)3;
			data[2]='Y';
			length = 3;
		}
};

class rawmsg:public PortMessage{
	public:
		rawmsg():str(-1),freq(-1),pulse(-1){}
		virtual ~rawmsg(){}
		bool isOK(){return str!=-1 && freq!=-1 && pulse!=-1;}
		char get(int){return 0;}
		int size(){return 0;}
		int getFrequency(){return freq;}
		int getSS(){return str;}
		int getPulse(){return pulse;}
		bool createFrom(std::list<char>::iterator& start,std::list<char>::iterator& end);
	private:
		int str,freq,pulse;
};

class screenmsg:public PortMessage{
	public:
		screenmsg(){}
		virtual ~screenmsg(){}
		bool isOK(){return true;}
		char get(int){return 0;}
		int size(){return 0;}
		bool createFrom(std::list<char>::iterator& start,std::list<char>::iterator& end);
};
#endif
