#include "Szerial.h"
#include "DynamixelSDK.h"

#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"       

/////////////////////////////////////////////////////////////////////////
//USER INPUT: SET THE VALUES BELOW TO MATCH YOUR PARTICULAR ROBOT.
/////////////////////////////////////////////////////////////////////////
const uint8_t numServos = 18; //This could be any type of actuator, to which you give a command, and read an output. 
const uint8_t numAnalogSensors = 18; //These are the uController's outputs back to AnimatLab; e.g. potentiometer readings, strain gages, etc. 
int loopDuration = 50; //ms
bool twoStateIO = true; //If this is true, then the second input to szerial must be 2*numServos. Otherwise, it is numServos.
/////////////////////////////////////////////////////////////////////////
//END USER INPUT.
/////////////////////////////////////////////////////////////////////////

//To use USBSerial, we need to change references to HardwareSerial in AnimatSerial.cpp to USBSerial instead.
Szerial szerial(&SerialUSB, 2*numServos, numAnalogSensors, 0);

//Instantiate data and axcm objects.
AnimatData data;

//#define DEBUG 1 //Uncomment this line to enter debugging mode, and print info to Serial3.

//Dynamixel variables
int numInitTries=1;
bool dynamixelExists[numServos];
uint8_t dynamixelList[numServos];
int dynamixelModelNumberList[numServos];
int servoCounter=0;
uint16_t comPos[numServos];
uint16_t comVel[numServos];
bool updateCom[numServos];
uint16_t readPos;
uint16_t readVel;
uint16_t prevReadPos[numServos];
uint16_t prevReadVel[numServos];
uint16_t modelNumber;
uint16_t writeLength;
uint16_t readLength;
uint8_t error;
bool toRead = true;
bool dataChanged;

//Analog Input variables
unsigned int prevReadAnalog[numAnalogSensors];
unsigned int initReadAnalog[numAnalogSensors];

int comVal = 0;

//Timing constants
int loopCounter=0;
bool firstLoop = true;
bool toListen = false;
unsigned long currTime;
unsigned long prevSendTime = 0;


//Placeholders for DynamixelSDK
int dxlCommResult = COMM_TX_FAIL;             // Communication result
bool dxlAddparamResult = false;   
uint8_t bytewiseActuatorCommand[4];
int commRes;


void setup() {
  Serial3.begin(57600);
  while(!Serial3);
  Serial3.println("Starting Szerial setup");
    
  //Initialize button and LED
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);

  /////////////////////////////////////////////////////////////////////////
  //SET UP THE SERVO CONTROL
  /////////////////////////////////////////////////////////////////////////
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(twoStateIO)
  {
    writeLength = 4;
    readLength = 4;
  }
  else
  {
    writeLength = 2;
    readLength = 2;
//    uint8_t bytewiseActuatorCommand[2];
  }

  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, writeLength);
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, readLength);

  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  
  /////////////////////////////////////////////////////////////////////////
  //SET UP COMMUNICATION WITH ANIMATLAB
  /////////////////////////////////////////////////////////////////////////  
  //Begin the szerial connection to AnimatLab.
  szerial.begin(115200);  // 115200

  /////////////////////////////////////////////////////////////////////////
  //INITIALIZE THE SERVOS THEMSELVES
  /////////////////////////////////////////////////////////////////////////

  //Run through the list of servos, 0 to numServos-1.
  //The user should make sure that their servos are numbered this way, starting at 0 and increasing.
  //This is repeated numInitTries times, because the servos do not always respond on the first ping.
  for(int i=0;i<numInitTries;i++)
  {    
    for(uint8_t j=0;j<numServos;j++)
    {
      prevReadPos[i] = 0;
      int id = (uint8_t)j;
      int success = packetHandler->read2ByteTxRx(portHandler,id,ADDR_MODEL_NUMBER,&modelNumber,&error);
      //int success = axcm.getModelNumber(j,&modelNumber, &error);

      //success = 0 if read was successful.
      //The model number determines how the servos are "zeroed," because different model numbers
      //have different position resolution.
      if(success == 0)
      {
        //Success
        if(!dynamixelExists[j])
        {
          if(modelNumber == 12)
          {
            //AX-12
            uint32_t posAndVel = DXL_MAKEDWORD(AX_OFFSET,50);
            packetHandler->write4ByteTxRx(portHandler,id,ADDR_GOAL_POSITION,posAndVel,&error);
            dynamixelModelNumberList[j] = 12;
          }
          else if(modelNumber == 310)
          {
            //MX-64
            uint32_t posAndVel = DXL_MAKEDWORD(MX_OFFSET,50);
            packetHandler->write4ByteTxRx(portHandler,id,ADDR_GOAL_POSITION,posAndVel,&error);
            dynamixelModelNumberList[j] = 64;
          }
          else if(modelNumber == 320)
          {
            //MX-106
            uint32_t posAndVel = DXL_MAKEDWORD(MX_OFFSET,50);
            packetHandler->write4ByteTxRx(portHandler,id,ADDR_GOAL_POSITION,posAndVel,&error);
            dynamixelModelNumberList[j] = 106;
          }
          groupSyncRead.addParam(id);
          
          //We may go through this loop multiple times, so we do not want to 
          //repeatedly add dynamixels that are already in the list.
          dynamixelExists[j] = true;
          updateCom[j] = false;
          comVel[j] = 0;
          dynamixelList[servoCounter] = (uint8_t)j;
          servoCounter++;
        }
      }
      else if(success == -3001)
      {
        //Failure
      }
      
      #ifdef DEBUG
        Serial3.print("ID: ");
        Serial3.print(j);
        Serial3.print("\tSUCCESS: ");
        Serial3.print(success);
        Serial3.print("\tEXISTS: ");
        Serial3.print(dynamixelExists[j]);
        Serial3.print("\tMODEL: ");
        Serial3.print(dynamixelModelNumberList[j]);
        Serial3.print("\tERROR: ");
        Serial3.println(error);
  
//        while(digitalRead(BOARD_BUTTON_PIN)==LOW)
//        {
//          //Wait for a button press before advancing to the next servo, only if in debug mode.
//        }
        delay(500);
      #endif
    }

    for(int i=0;i<numAnalogSensors;i++)
    {
      Serial3.print("AnimatLab's analog input ");
      Serial3.print(i);
      Serial3.print(" corresponds to setting digital out ");
      Serial3.print(szerial.getMUXdigitalOut(i));
      Serial3.print(" to HIGH and reading analog in ");
      Serial3.println(szerial.getMUXanalogIn(i));
      
      pinMode(szerial.getMUXanalogIn(i),INPUT);
      pinMode(szerial.getMUXdigitalOut(i),OUTPUT);
      
      initReadAnalog[i] = szerial.readMUXanalog(i);
    }
  }

  Serial3.println("Setup complete...press board button to initiate loop");
  while(digitalRead(BOARD_BUTTON_PIN)==LOW)
  {
    //Wait for button press before listening for input from the computer.
    //COMMAND EACH SERVO'S POSITION TO BE ITS POSITION IN THE PREVIOUS READ.
  }
  Serial3.println("Loop initiated by user.");

  //Blink the board LED to signify that it is ready to receive commands from AnimatLab.
  //It will blink two times, and then turn on steady. While the LED is on, it will communicate with AnimatLab.
  blinkReady();

  /////////////////////////////////////////////////////////////////////////
  //BEGIN LISTENING FOR COMMANDS FROM ANIMATLAB
  /////////////////////////////////////////////////////////////////////////  
  
  /////////////////////////////////////////////////////////////////////////
  //WAIT FOR THE SIGNAL FROM ANIMATLAB TO START
  /////////////////////////////////////////////////////////////////////////
  bool waiting = true;
  Serial3.println("waiting.");
  /* 
   * Read the messages coming from AnimatLab. 
   * If the simulation is starting, then it will set a flag in the Szerial object, 
   * which we can access via Szerial.getSimStarting().
   */
  while(waiting)
  {
    //Need to read messages to know if the sim has started.
    szerial.readMsgs();
    if(szerial.getSimStarting())
    {
      waiting = false;
    }
    //blinkDone();
  }
  toListen = true;
  loopCounter = 0;

  while(toListen){
    /////////////////////////////////////////////////////////////////////////
    //RECEIVE DATA FROM ANIMATLAB
    /////////////////////////////////////////////////////////////////////////
 
    //Query if there are new messages from AnimatLab.
    szerial.readMsgs();
    toRead = szerial.isChanged();
  
    //If there are new messages, then dole them out to the outputs.
    if(toRead)
    {    
      //Loop through the servos for position commands
      for(int i=0; i<numServos; i++)
      {
        int id = dynamixelList[i];        
  
        //If the value has changed from last time, and if there is data to read, read it and
        //use it as the command position for the servo.
        if(szerial.isChanged(id) && szerial.getData(id, data))
        {
          comPos[i] = data.value.ival;
          updateCom[i] = true;
        }
        else
        {
          //Serial3.println("unchanged.");
        }
      }
      

      if(twoStateIO)
      {
        //Loop through the servos for velocity commands
        for(int i=0; i<numServos; i++)
        {
          /*
           * For velocity, or whatever the second input command is,
           * we need to shift the id upwards.
           */
          int id = numServos+dynamixelList[i];
    
          //If the value has changed from last time, and if there is data to read, read it and
          //use it as the command position for the servo.
          if(szerial.isChanged(id) && szerial.getData(id, data))
          {
            comVel[i] = data.value.ival;
            updateCom[i] = true;
          }
        }
      }

      for(int i=0; i<numServos; i++)
      {
        uint8_t id = dynamixelList[i];
        
        if(updateCom[i])
        {
          if(!twoStateIO)
          {
            bytewiseActuatorCommand[0] = DXL_LOBYTE(comPos[i]);
            bytewiseActuatorCommand[1] = DXL_HIBYTE(comPos[i]);
            #ifdef DEBUG
            Serial3.print("Write: ");
            Serial3.print(id);
            Serial3.print(" ");
            Serial3.println(comPos[i]);
            #endif
          }
          else
          {
            bytewiseActuatorCommand[0] = DXL_LOBYTE(comPos[i]);
            bytewiseActuatorCommand[1] = DXL_HIBYTE(comPos[i]);
            bytewiseActuatorCommand[2] = DXL_LOBYTE(comVel[i]);
            bytewiseActuatorCommand[3] = DXL_HIBYTE(comVel[i]);
            #ifdef DEBUG
            Serial3.print("Write: ");
            Serial3.print(id);
            Serial3.print(" ");
            Serial3.print(comPos[i]);
            Serial3.print(" ");
            Serial3.println(comVel[i]);
            #endif
          }
          dxlAddparamResult = groupSyncWrite.addParam(id, bytewiseActuatorCommand);
          updateCom[i] = false;
        }
      }
      dxlCommResult = groupSyncWrite.txPacket();
      groupSyncWrite.clearParam();
  
      //Clear the flags that signal that new data has been received.
      szerial.clearChanged();
    }else{
      //do nothing
    }

    /////////////////////////////////////////////////////////////////////////
    //RECORD THE ROBOT'S STATES
    /////////////////////////////////////////////////////////////////////////
    
    //We will only return data to AnimatLab if a state has changed. Thus, initialize
    //a boolean as false, and set it to true if we determine that anything has changed.
    bool dataToSend = false;
  
    //Loop through the servos, read their positions, and if they have changed more than
    //the TRANSMISSION_THRESHOLD, then add the data to the next message we send to AnimatLab.
    for(int i=0; i<numServos; i++)
    {
      int id = dynamixelList[i];
      
      if(!twoStateIO)
      {
        commRes = packetHandler->read2ByteTxRx(portHandler,id,ADDR_PRESENT_POSITION,&readPos,&error);
        dataChanged = (loopCounter < 1 || abs(readPos - prevReadPos[id])>=TRANSMISSION_THRESHOLD);
      }
      else
      {
        uint32_t outputWord;
        commRes = packetHandler->read4ByteTxRx(portHandler,id,ADDR_PRESENT_POSITION,&outputWord,&error);
        readPos = DXL_LOWORD(outputWord);
        readVel = DXL_HIWORD(outputWord);

        /*
         * Dynamixel returns positive velocity 0-1023, negative velocity 1024-2047.
         * To preserve the "sign," we'll add 1024 to positive velocity, and compute
         * negative velocity as 2048 - x.
         */
        if(readVel >= 0 && readVel < 1024)
        {
          readVel += 1024;
        }
        else
        {
          readVel = 2048 - readVel;
        }
        
        #ifdef DEBUG
        Serial3.print("Read: ");
        Serial3.print(id);
        Serial3.print(" ");
        Serial3.print(readPos);
        Serial3.print(" ");
        Serial3.println(readVel);
        #endif

        dataChanged = (loopCounter < 1 || abs(readPos - prevReadPos[id])>=TRANSMISSION_THRESHOLD || abs(readVel - prevReadVel[id])>=TRANSMISSION_THRESHOLD);
      }
  
      //If the servo position was read successfully, and if this is the first time step of the AnimatLab
      //controller or if the value has changed significantly from the previous value, add this data
      //to the sentence to send back to AnimatLab.
      if(commRes == 0 && dataChanged )
      {
        //Set the flag to true, so we send the sentence when it has been constructed.
        dataToSend = true;
  
        //Add the ID and position to the next sentence.
        szerial.addData(id,readPos);
        //Save the new read value as the next "previous" value.
        prevReadPos[id] = readPos;

        if(twoStateIO)
        {
          //Add the ID and position to the next sentence.
          szerial.addData(id+numServos,readVel);
          
          //Save the new read value as the next "previous" value.
          prevReadVel[id] = readVel;
        }
      }
      else
      {
        //do nothing
      }
    }

    for(int i=0;i<numAnalogSensors;i++)
    {
      //i is the analog pin index (given the list in Szerial.h).
      //dataIndex is the index in the AnimatData object.
      //animatID is the id of the item (AnimatData.id.ival). This is the value set in the AnimatLab GUI.
      int animatID = i+100; //100 because analog output
      int reading = szerial.readMUXanalog(i);
      
      if(reading >= 0 && abs(reading - prevReadAnalog[i])>=TRANSMISSION_THRESHOLD)
      {
        dataToSend = true;
        szerial.addData(animatID,reading);
        prevReadAnalog[i] = reading;
        #ifdef DEBUG
          Serial3.print("Analog index ");
          Serial3.print(animatID);
          Serial3.print(": ");
          Serial3.print(reading);
          Serial3.print("\t");
        #endif
      }
    }

    /////////////////////////////////////////////////////////////////////////
    //DECIDE WHETHER TO REPORT ANY DATA TO ANIMATLAB
    /////////////////////////////////////////////////////////////////////////

    loopCounter++;
    
    #ifdef DEBUG
    if (dataToSend)
      Serial3.println();
    #endif
  
    //If one of the robot's states has changed, send it to AnimatLab.
    if(dataToSend)
    {
      szerial.writeMsgs(); 
    }
  
    //If we have made it this far, then the AnimatLab controller is no longer
    //on its first loop. Therefore, set this flag to false.
    szerial.setSimStartingFalse();
    
    /////////////////////////////////////////////////////////////////////////
    //LISTEN FOR LOOP TERMINATION, AND WAIT OUT THE REST OF THE LOOP TIME
    /////////////////////////////////////////////////////////////////////////
  
    //If the board button is depressed, then stop communication with AnimatLab.
    //Stop issuing new commands to the servos, and stop sending their positions over serial.
    //We will continue to read the positions, but the data will go nowhere.
    int buttonState = digitalRead(BOARD_BUTTON_PIN);
    if(buttonState==HIGH)
    {
      toListen = false;
      szerial.writeMsgs();
      Serial3.println("Loop terminated by user.");
      digitalWrite(BOARD_LED_PIN,HIGH);
    }
  
    //If there is still more time until the end of this iteration, do nothing
    //until we reach the appropriate time.
    while(millis() - prevSendTime < loopDuration)
    {
      //wait
    }
    prevSendTime = millis();
  }

  /////////////////////////////////////////////////////////////////////////
  //SET ALL SERVOS TO "DAMPER" MODE
  //Basically, command the servo to go to its current position.
  //If you move the servo, it will follow your push.
  /////////////////////////////////////////////////////////////////////////
  while(true)
  {
    for(int i=0; i<numServos; i++)
    {
      uint8_t id = dynamixelList[i];
      
      if(updateCom[i])
      {
        if(!twoStateIO)
        {
          bytewiseActuatorCommand[0] = DXL_LOBYTE(comPos[i]);
          bytewiseActuatorCommand[1] = DXL_HIBYTE(comPos[i]);
        }
        else
        {
          bytewiseActuatorCommand[0] = DXL_LOBYTE(comPos[i]);
          bytewiseActuatorCommand[1] = DXL_HIBYTE(comPos[i]);
          bytewiseActuatorCommand[2] = DXL_LOBYTE(comVel[i]);
          bytewiseActuatorCommand[3] = DXL_HIBYTE(comVel[i]);
        }
        dxlAddparamResult = groupSyncWrite.addParam(id, bytewiseActuatorCommand);
        updateCom[i] = false;
      }
      
      if(!twoStateIO)
      {
        commRes = packetHandler->read2ByteTxRx(portHandler,id,ADDR_PRESENT_POSITION,&readPos,&error);
        comPos[i] = readPos;
      }
      else
      {
        uint32_t outputWord;
        commRes = packetHandler->read4ByteTxRx(portHandler,id,ADDR_PRESENT_POSITION,&outputWord,&error);
        readPos = DXL_LOWORD(outputWord);
        readVel = DXL_HIWORD(outputWord);
        
        if(readVel >= 0 && readVel < 1024)
        {
          //do nothing
        }
        else
        {
          readVel -= 1024;
        }

        comPos[i] = readPos;
        comVel[i] = readVel;
        updateCom[i] = true;
      }
    }
    dxlCommResult = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();

  }
}

void loop()
{
  blinkDone();
}

void blinkReady()
{
  for(int i=0;i<3;i++)
  {
    digitalWrite(BOARD_LED_PIN,HIGH);
    delay(125);
    digitalWrite(BOARD_LED_PIN,LOW);
    delay(125);
  }
}

void blinkDone()
{
  int totalPeriod = 250;
  int pulsePeriod = 5;
  for(int i=0;i<pulsePeriod;i++)
  {
    for(int j=0;j<totalPeriod;j+=pulsePeriod)
    {
      digitalWrite(BOARD_LED_PIN,HIGH);
      delay(pulsePeriod-i);
      digitalWrite(BOARD_LED_PIN,LOW);
      delay(i);
    }
  }
  for(int i=0;i<pulsePeriod;i++)
  {
    for(int j=0;j<totalPeriod;j+=pulsePeriod)
    {
      digitalWrite(BOARD_LED_PIN,HIGH);
      delay(i);
      digitalWrite(BOARD_LED_PIN,LOW);
      delay(pulsePeriod-i);
    }
  }
}
