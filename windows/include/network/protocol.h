
#ifndef HB_DELIM
#define HB_DELIM

#include <string.h>
#include <map>
#include <string>
#include <sstream>
#include <cstdlib>

#ifdef WIN32
//OH SO HACKY
#define strtok_r(a,b,c) strtok_s(a,b,c)
//OH SO UNSAFE AND HACKY
//#define strcpy(a,b) strcpy_s(a,4096,b)
#endif

//#define heartbeat_port 49691
//#define repeater_port 48911
//#define exchanger_port 50002

namespace SimpleProtocol{
	/**
	 * The port that the heartbeat service talks over
	 */
	const int heartbeat_port = 49691;
	/**
	 * The port that the repeater_TCP service talks over
	 */
	const int repeater_port= 48911;
	const int exchanger_port= 50002;
	/**
	 * Convenience method for parsing a string that was pulled in from a friend on
	 * the network. It will put the kev,value pairs into the data variable,
	 * including any "meta" data before the msg_start character.
	 */
	void MessageToMap(std::string s, std::map<std::string,std::string> &data, bool includeMeta=true);

	/**
	 * Convenience method for creating a string to send as a message to a friend on
	 * the network. It will put the kev,value pairs into the message from the data
	 * variable, including any "meta" data before the msg_start character.
	 */
	std::string MapToMessage(char* myHostname, std::map<std::string,std::string> &msgData);

	/**
	 * Convenience method for creating a string to send as a message to a friend on
	 * the network. It will put the kev,value pairs into the message from the data
	 * variable, including any "meta" data before the msg_start character.
	 */
	std::string MapToMessage(char* myHostname, std::map<std::string,std::string> &msgData,
			std::map<std::string,std::string> &metaData);

	/** Any of the characters in this string is a valid
	 * delimiter between key,value pairs
	 * Ex: <key><key_value_delim_str><value><pair_delim><key2><key_value_delim_str><value2>
	 */
	const std::string pair_delim_str=",";

	/** Any of the characters in this string is a valid
	 * delimiter between keys and values
	 * Ex: <key><key_value_delim_str><value><pair_delim><key2><key_value_delim_str><value2>
	 */
	const std::string key_value_delim_str="=:";

	/**
	 * Message formatting. any of these characters is a valid message
	 * start byte Anything before this character (such as key,value pairs)
	 * is considered "meta data"
	 */
	const std::string msg_start_str="[";

	/**
	 * Message formatting. any of these characters is a valid message
	 * stop byte.
	 */
	const std::string msg_stop_str="]";
}
#endif
