/** Receiver-Robostix (R4500S) protocol for serial comm. with ros receiver node
 * 57600 8bits - no parity - 1 stop bit
 *
 * Computer to robostix: 'A' INT cmd_bytes
 * 	where 	'A' is start communication
 * 			INT is the total number of bytes in the packet (including start comm)
 * 			cmd_bytes are INT-2 bytes for the command
 * 			command is usually a single character (defined below) possibly followed
 * 			by some args
 *
 * Robostix to computer: can return two types of messages
 * 	Default:	'A' 3 'Y'
 * 			Positive ack. when no other input is requested from robostix in at most
 * 			100 ms. after receiving packet. Resend is ack not received in this time.
 *
 *  Output:		'A' INT output_bytes
 *  		This packet is sent instead of the ack. when the software requests some
 *  		output from robostix (e.g. compass readings).
 *
 *  Created on: Dec 20, 2011
 *      Author: tokekar
 **/

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define R_MAX_PACKET_SIZE 1024
// Packet protocol
#define START_COMM			'A'
#define ACK					'Y'

// Commands
#define GET_DEV_ID			'D' // 'A' 3 'D' returns 'A' 3 2 (dev-id=2)
#define LOG_AERIAL			'L' // 'A' 4 'L' [1-9] (start logging in aerial given table number)
#define LOG_MANUAL			'M' // 'A' 5 'M' MSB LSB (start logging in manual given frequency)
#define RESET				'R' // 'A' 3 'R' (go back to main menu, reset whatever you were doing)
#define SET_RAW				'W' // 'A' 4 'W' <key> (presses given key: use keymap.h to get the key map)

#define CHANGE_GAIN			'G' // 'A' 4 'G' [0-100]
#define SET_FREQ			'F' // e.g. 'A' 9 'F' MSB1 LSB1 MSB2 LSB2 MSB3 LSB3

#define SIGNAL 				'S' // sends dcc signal 'A' 9 'S' freqMSB freqLSB periodMSB periodLSB pulse sig

inline size_t getMaxPacketLength(const char type){
	switch (type){
		case ACK:{return 3;}
		case GET_DEV_ID:{return 3;}
		case LOG_AERIAL:{return 4;}
		case LOG_MANUAL:{return 5;}
		case RESET:{return 3;}
		case SET_RAW:{return 4;}
		case CHANGE_GAIN:{return 4;}
		case SET_FREQ:{return 5;}
		case SIGNAL:{return 9;}
		default:return 0;
	}
}

#endif /* PROTOCOL_H_ */
