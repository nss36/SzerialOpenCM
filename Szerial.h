/*
  Szerial.cpp - Library for interfacing a robot with AnimatLab.
  Based on AnimatSerial, originally written by:
  Copyright (c) 2015 David Cofer, NeuroRobotic Technologies.  All right reserved.
  Modified by Nick Szczecinski, 2018, Case Western Reserve University.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Szerial_h
#define Szerial_h

#include "USBSerial.h"

#define MAX_ANIMAT_BUFFER 128
#define AX_SLOPE 195.5695940713210
#define AX_OFFSET 512
#define MX_SLOPE 651.8986469044033
#define MX_OFFSET 2048
#define TRANSMISSION_THRESHOLD 2 //0, send every update. 1, only if it changes. Any higher number will make updates less frequent.

// Control table address (AX-12)
#define ADDR_MODEL_NUMBER         0
#define ADDR_ID                   3
#define ADDR_RETURN_DELAY_TIME    5
#define ADDR_CW_LIMIT             6
#define ADDR_CCW_LIMIT            8
#define ADDR_TEMP_LIMIT           11
#define ADDR_MAX_TORQUE           14
#define ADDR_ALARM_LED            17
#define ADDR_ALARM_SHUTDOWN       18
#define ADDR_TORQUE_ENABLE        24
#define ADDR_LED_ON               25
#define ADDR_CW_SLOPE             28
#define ADDR_P_GAIN               28
#define ADDR_CCW_SLOPE            29
#define ADDR_GOAL_POSITION        30
#define ADDR_PRESENT_POSITION     36
#define ADDR_PRESENT_SPEED        38  
#define ADDR_PRESENT_LOAD         40
#define ADDR_PRESENT_VOLT         42
#define ADDR_PRESENT_TEMP         43
#define ADDR_IS_MOVING            46

//Sentence structure:
//255, 255 (start), 1 (new info incoming), X (total length of message), A (component ID), B, C (2 byte data), Y (checksum)
//Each comma marks the end of the byte. The example above contains 8 bytes. The text in parentheses is a description. 
//#define PACKET_INFO_SIZE 5  //Size of the header (2), packet size(1), message id(1), and checksum(1) in bytes
//#define START_MESSAGE_INFO_BYTE 4 //Byte in the sentence at which the data begins.
#define HEADER_SIZE 4 //Size of the start (2), message ID (1), and packet size (1).
#define DATA_SIZE 3	//Size of the ID (1) and the data (2).
#define FOOTER_SIZE 1 //Size of the checksum (1).

	
//AnimatData will store the data that needs to be sent back and forth. It will store it as a union
//so it can be accessed as an integer or unsigned character (for transmission).
class AnimatData
{
public:
	union id_tag {
	 unsigned char bval;
	 unsigned char ival;
	} id;
	
	union value_tag {
	 unsigned char bval[2];
	 unsigned short ival;
	} value;
	
	union prev_value_tag {
		unsigned char bval[2];
		unsigned short ival;
	} prev_value;

	AnimatData() {
		id.ival = 0;
		value.ival = 0;
		prev_value.ival = 0;
	}
};

class Szerial
{    
  public:
    Szerial(); 
	Szerial(USBSerial *ss, unsigned int numServos, unsigned int inTotal, unsigned int outTotal); 
	~Szerial();
	
    void begin(unsigned long baud);
    int readMsgs();         // must be called regularly to clean out Serial buffer
	void writeMsgs();
	void writeAllMsgs();
	void writeResendMsg();
	
	bool isChanged();
	bool isChanged(unsigned int id);
	bool getData(unsigned int index, AnimatData &data);
	bool addData(unsigned int id, int val);
	bool getSimStarting();
	void setSimStartingFalse();
	
	unsigned int getInDataTotal() {return inDataTotal;}
	unsigned int getOutDataTotal() {return outDataTotal;}
	unsigned int* getInIDs() {return inIDs;}
	unsigned int* getOutIDs() {return outIDs;}
	
	void clearChanged();
	
	unsigned int readMUXanalog(unsigned int index);	
	const int getMUXdigitalOut(unsigned int index);
	const int getMUXanalogIn(unsigned int index);

	private:

	void setInDataValue(int id, int val);

	void clearInData();
	void clearOutData();

	AnimatData *inData;
	AnimatData *outData;
	bool *inChanged;
	bool dataChanged;
	unsigned int *inIDs;
	unsigned int *outIDs;
	
	unsigned int numServos;
	unsigned int inDataTotal;
	unsigned int outDataTotal;

	unsigned int inDataCount;
	unsigned int outDataCount;
	
	unsigned int inputContains(unsigned int id);
	//bool outputContains(unsigned int id);

	// internal variables used for reading messages
    unsigned char vals[MAX_ANIMAT_BUFFER];  // temporary values, moved after we confirm checksum
    int index;              // -1 = waiting for new packet
    int checksum;
    unsigned char status; 	
	
	int messageID;
	int packetSize;
	
	bool simStarting = false;
	
	//These values are microcontroller-specific.
	const int analogMUXdigitalOut[4] = {6,7,8,9};
	const int numDigitalOutIndices = 4;
	const int analogMUXanalogIn[5] = {0,1,2,3,4};
	const int numAnalogOutIndices = 5;
	const int MUXdigitalOut[18] = {6,7,8,9,6,7,8,9,6,7,8,9,6,7,8,9,6,7};
	const int MUXanalogIn[18] =   {4,4,4,4,3,3,3,3,2,2,2,2,1,1,1,1,0,0};
	void resetMUXanalog(unsigned int maxIndex);
	
	union id_tag {
	 unsigned char bval;
	 unsigned int ival;
	} id;
	
	union value_tag {
	 unsigned char bval[2];
	 unsigned short ival;
	} value;

	union size_tag {
	 unsigned char bval;
	 unsigned char ival;
	} size;
	
	USBSerial *stream;
};

#endif