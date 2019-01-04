/*
  Szerial.cpp - Library for interfacing with AnimatLab.
  Based on AnimatSerial, originally written by:
  Copyright (c) 2015 David Cofer, NeuroRobotic Technologies.  All right reserved.

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

#include <Arduino.h>
#include "Szerial.h"

/* Constructor */
Szerial::Szerial(){
    index = -1;
    status = 0;
	stream = NULL;
	
	inDataTotal = 0;
	outDataTotal = 0;
	inDataCount = 0;
	outDataCount = 0;
	inData = NULL;
	outData = NULL;
	inChanged = NULL;
	dataChanged = false;
	
	messageID  = -1;
	packetSize = 0;
}

Szerial::Szerial(USBSerial *ss, unsigned int inTotal, unsigned int outTotal){
    index = -1;
    status = 0;
	stream = ss;
	
	inDataTotal = inTotal;
	outDataTotal = outTotal;	
	inDataCount = 0;
	outDataCount = 0;
	
	messageID  = -1;
	packetSize = 0;
	
	if(inDataTotal > 0)
	{
		inData = new AnimatData[inDataTotal];
		inChanged = new bool[inDataTotal];
	}
		
	if(outDataTotal > 0)
		outData = new AnimatData[outDataTotal];
		
	clearInData();
	clearChanged();
	clearOutData();
}

Szerial::~Szerial()
{
	if(inData)
	{
		delete[] inData;
		inData = NULL;
	}

	if(outData)
	{
		delete[] outData;
		outData = NULL;
	}
}

bool Szerial::isChanged()
{
	return dataChanged;
}

void Szerial::clearInData()
{
	if(inData)
	{
		for(int i=0; i<inDataTotal; i++)
		{
			inData[i].value.ival = 0;
			inChanged[i] = 0;
		}
	}
}

void Szerial::clearOutData()
{
	if(outData)
	{
		for(int i=0; i<outDataTotal; i++)
			outData[i].value.ival = 0;
	}
}

void Szerial::clearChanged()
{
	dataChanged = false;
	if(inChanged)
	{
		for(int i=0; i<inDataTotal; i++)
			inChanged[i] = false;
	}
}	

bool Szerial::isChanged(unsigned int index)
{
	if(inChanged && index < inDataTotal)
		return inChanged[index];
	else
		return false;
}

bool Szerial::getData(unsigned int index, AnimatData &data)
{
	if(index < inDataTotal)
	{
		data = inData[index];
		return true;
	}
	else
		return false;
}

bool Szerial::addData(unsigned int id, int val)
{
	if(outDataCount < outDataTotal)
	{
		outData[outDataCount].id.ival = id;
		outData[outDataCount].value.ival = val;
		outDataCount++;
		return true;
	}
	else
		return false;
}

void Szerial::setInDataValue(int id, int val) 
{
	if(inData && inChanged && id < inDataTotal)
	{
		inData[id].value.ival = val;
		inChanged[id] = true;
		dataChanged = true;
		
	}
}

void Szerial::begin(unsigned long baud){
	if(stream != NULL)
		stream->begin(baud);
	else
	{
		stream = &Serial;
		stream->begin(baud);
	}
}

/* process messages coming from CommanderHS 
 *  format = 0xFF 0xFF 0x01 ID DATA_LOW_BYTE DATA_HIGH_BYTE CHECKSUM */
int Szerial::readMsgs()
{
	if(stream != NULL) 
	{	
		while(stream->available() > 0)
		{
			//Get first header byte
			if(index == -1)
			{        
				// looking for new packet
				if(stream->read() == 0xff)
				{
					vals[0] = 0xff;
					checksum = (int) vals[0];
					index = 1;
					messageID  = -1;
					packetSize = 0;
				}
			}
			//Get second header byte
			else if(index == 1)
			{
				vals[index] = (unsigned char) stream->read();
				if(vals[index] == 0xff)
				{            
					checksum += (int) vals[index];
					index++;
				}
				else
					index = -1;  //Start over if the second byte is not 0xff
			}
			//Get the message ID
			else if(index==2)
			{
				vals[index] = (unsigned char) stream->read();
				messageID = vals[index];

				checksum += (int) vals[index];
				index++;
			}
			//Get the message size byte 1
			else if(index==3)
			{
				vals[index] = (unsigned char) stream->read();
				packetSize = vals[index];

				//If the message size is greater than 128 then something is wrong. Start over.
				if(packetSize > 128)
					index = -1;
				else
				{
					checksum += (int) vals[index];
					index++;
				}
			}
			else if(index > 3 && index <= packetSize)
			{
				vals[index] = (unsigned char) stream->read();

				if(index == (packetSize-1))
				{ // packet complete
					//The transmitted checksum is the last entry
					int iChecksum = vals[index];
					
					if(checksum%256 != iChecksum)
					{
						Serial3.println("Invalid Checksum. Sending resend msg.");
						writeResendMsg();
						// packet error!
						index = -1;
					}
					else
					{
						if(messageID == 3)
						{
							simStarting = true;
							Serial3.println("SIM IS STARTING!");
						}
						else if(messageID == 2)
							writeAllMsgs();
						else if(messageID == 1)
						{
							int iStop = packetSize - 1; //Exclude the checksum at the end

							for(int iIdx=START_MESSAGE_INFO_BYTE; iIdx<iStop; iIdx+=DATA_SIZE)
							{
								id.bval = vals[iIdx];
								value.bval[0] = vals[iIdx+1];
								value.bval[1] = vals[iIdx+2];
								
								//Message successfully decoded
								//Save the value in the structure used to save servo data.
								setInDataValue(id.ival, value.ival);
								
							}
						}
					}
					//Message complete; flush the stream and restart the index.
					index = -1;
					stream->flush();
					return 1;
				}
				else
				{
					//Tack on this byte to the checksum.
					checksum += (int) vals[index];
					index++;
				}

				if(index >= MAX_ANIMAT_BUFFER)
					index = -1;
			}

		}
	}
	clearInData();
	return 0;
}

void Szerial::writeMsgs(){
	if(stream != NULL && outDataCount > 0) {
		
		//First write the header, and add it to the checksum.
		stream->write((byte) 0xFF);
		stream->write((byte) 0xFF);
		stream->write((byte) 0x01);
		int checksum = 0xFF + 0xFF + 0x01;

		//Second calculate the length of the sentence, write it, and add it to the checksum.
		size.ival = HEADER_SIZE + (DATA_SIZE * outDataCount) + FOOTER_SIZE;
		
		stream->write((byte) size.bval);
		checksum += size.bval;
		
		//For each item to update, send the id, low byte, and high byte, and add them all to the checksum.
		for(int i=0; i<outDataCount; i++) {
			
			stream->write((byte) outData[i].id.bval);
			stream->write((byte) outData[i].value.bval[0]);
			stream->write((byte) outData[i].value.bval[1]);
			
			checksum += outData[i].id.bval;
			checksum += outData[i].value.bval[0];
			checksum += outData[i].value.bval[1];
		}
		
		//Modulo the checksum and send it.
		unsigned char bchecksum = (unsigned char) (checksum%256);
		stream->write(bchecksum);
		
		//Reset the number of items to send.		
		outDataCount = 0;
	}
}

void Szerial::writeResendMsg(){
	if(stream != NULL) {
		int checksum = 0xFF + 0xFF + 0x02;
		
		//First write the header
		stream->write((byte) 0xFF);
		stream->write((byte) 0xFF);
		stream->write((byte) 0x02);
		//Serial.print("Writing Resend message 0xFF, 0xFF, 0x02, ");

		size.ival = HEADER_SIZE + FOOTER_SIZE;
		
		stream->write((byte) size.bval);
		checksum += size.bval;

		unsigned char bchecksum = (unsigned char) (checksum%256);
		stream->write(bchecksum);
	}
}

void Szerial::writeAllMsgs(){
	//Serial.println("Writing resend message");
	if(stream != NULL && outDataTotal > 0) {
		for(int i=0; i<outDataTotal; i++)
			addData(i, outData[i].value.ival);
			
		writeMsgs();
	}
}

bool Szerial::getSimStarting(){
	return simStarting;
}

void Szerial::setSimStartingFalse(){
	simStarting = false;
}