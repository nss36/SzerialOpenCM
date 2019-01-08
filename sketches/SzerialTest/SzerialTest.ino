#include "Szerial.h"
#include "axcm.h"

/////////////////////////////////////////////////////////////////////////
//USER INPUT: SET THE VALUES BELOW TO MATCH YOUR PARTICULAR ROBOT.
/////////////////////////////////////////////////////////////////////////
const uint8_t numServos = 3; //This could be any type of actuator, to which you give a command, and read an output. 
const uint8_t numInAnalog = 1; //These are the uController's inputs; e.g. actuator commands, PWM for servos or LEDs, etc. 
const uint8_t numOutAnalog = 1; //These are the uController's outputs back to AnimatLab; e.g. potentiometer readings, strain gages, etc. 
int loopDuration = 10; //ms
/////////////////////////////////////////////////////////////////////////
//END USER INPUT.
/////////////////////////////////////////////////////////////////////////

//To use USBSerial, we need to change references to HardwareSerial in AnimatSerial.cpp to USBSerial instead.
Szerial szerial(&SerialUSB, numServos, numInAnalog, numOutAnalog);

//#define DEBUG 1 //Uncomment this line to enter debugging mode, and print info to Serial3.

//Instantiate data and axcm objects.
AnimatData data;
AXCM axcm;

//Dynamixel variables
int numInitTries=1;
bool dynamixelExists[numServos];
int dynamixelList[numServos];
int dynamixelModelNumberList[numServos];
int servoCounter=0;
uint16_t comPos;
uint16_t readPos;
uint16_t prevReadPos[numServos];
uint16_t modelNumber;
uint8_t error;

//Analog Input variables
int prevReadAnalog[numInAnalog];
int initReadAnalog[numInAnalog];

int comVal = 0;

//Timing constants
int loopCounter=0;
bool firstLoop = true;
unsigned long currTime;
unsigned long prevSendTime = 0;

void setup() {
  Serial3.begin(57600);
  while(!Serial3);
  Serial3.println("Starting Szerial setup");
    
  //Initialize button and LED
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);

  //Initialize the servo control object
  axcm.initialize();

  //Run through the list of servos, 0 to numServos-1.
  //The user should make sure that their servos are numbered this way, starting at 0 and increasing.
  //This is repeated numInitTries times, because the servos do not always respond on the first ping.
  for(int i=0;i<numInitTries;i++)
  {
    for(int j=0;j<numServos;j++)
    {
      prevReadPos[i] = 0;
      int success = axcm.getModelNumber(j,&modelNumber, &error);

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
            axcm.setPositionSpeed(j,AX_OFFSET,50);
            dynamixelModelNumberList[j] = 12;
          }
          else if(modelNumber == 310)
          {
            //MX-64
            axcm.setPositionSpeed(j,MX_OFFSET,50);
            dynamixelModelNumberList[j] = 64;
          }
          else if(modelNumber == 320)
          {
            //MX-106
            axcm.setPositionSpeed(j,MX_OFFSET,50);
            dynamixelModelNumberList[j] = 106;
          }
          
          //We may go through this loop multiple times, so we do not want to 
          //repeatedly add dynamixels that are already in the list.
          dynamixelExists[j] = true;
          dynamixelList[servoCounter] = j;
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
  
        while(digitalRead(BOARD_BUTTON_PIN)==LOW)
        {
          //Wait for a button press before advancing to the next servo, only if in debug mode.
        }
        delay(500);
      #endif
    }

    for(int i=0;i<numInAnalog;i++)
    {
      Serial3.print("Setting pin ");
      Serial3.print(szerial.getAnalogOutIndex(i));
      Serial3.println(" to input to AnimatLab/output from robot.");
      pinMode(szerial.getAnalogOutIndex(i),INPUT);
      initReadAnalog[i] = analogRead(szerial.getAnalogOutIndex(i));
    }

    for(int i=0;i<numInAnalog;i++)
    {
      //Remember, inputs from AnimatLab, but outputs to the robot.
      Serial3.print("Setting pin ");
      Serial3.print(szerial.getAnalogInIndex(i));
      Serial3.println(" to input from AnimatLab/output to robot.");
      pinMode(szerial.getAnalogInIndex(i),OUTPUT);
    }
  }

  //Begin the szerial connection to AnimatLab.
  szerial.begin(115200);  // 115200
  Serial3.println("Setup complete...press board button to initiate loop");

  while(digitalRead(BOARD_BUTTON_PIN)==LOW)
  {
    //Wait for button press before listening for input from the computer.
  }
  Serial3.println("Loop initiated by user.");

  //Blink the board LED to signify that it is ready to receive commands from AnimatLab.
  //It will blink two times, and then turn on steady. While the LED is on, it will communicate with AnimatLab.
  blinkReady();
}

void loop()
{
  /////////////////////////////////////////////////////////////////////////
  //RECEIVE DATA FROM ANIMATLAB
  /////////////////////////////////////////////////////////////////////////

  //Query if there are new messages from AnimatLab.
  szerial.readMsgs();

  //If there are new messages, then dole them out to the outputs.
  if(szerial.isChanged())
  {      
    //Loop through the servos
    for(int i=0; i<numServos; i++)
    {
      int id = dynamixelList[i];

      //If the value has changed from last time, and if there is data to read, read it and
      //use it as the command position for the servo.
      if(szerial.isChanged(id) && szerial.getData(id, data))
      {
        comPos = data.value.ival;
        int success = axcm.setPositionSpeed(id,comPos,(uint16_t)1000);
      }
    }

    //Loop through the analog inputs (inputs from AnimatLab, but outputs to the robot hardware).
    for(int i=0; i<numInAnalog; i++)
    {
      //i is the analog pin index (given the list in Szerial.h).
      //dataIndex is the index in the AnimatData object.
      //animatID is the id of the item (AnimatData.id.ival). This is the value set in the AnimatLab GUI.
      int dataIndex = i+numServos;
      int animatID = i+200; //200 because it's analog input from AnimatLab
      
      if(szerial.isChanged(dataIndex) && szerial.getData(dataIndex,data))
      {
        //animatID == data.id.ival
        comVal = data.value.ival;
        analogWrite(szerial.getAnalogInIndex(i),comVal);
      }
    }

    //Clear the flags that signal that new data has been received.
    szerial.clearChanged();
  }else{
    //Nothing
  }

  /////////////////////////////////////////////////////////////////////////
  //SEND DATA BACK TO ANIMATLAB
  /////////////////////////////////////////////////////////////////////////
  
  //We will only return data to AnimatLab if a state has changed. Thus, initialize
  //a boolean as false, and set it to true if we determine that anything has changed.
  bool dataToSend = false;

  //Loop through the servos, read their positions, and if they have changed more than
  //the TRANSMISSION_THRESHOLD, then add the data to the next message we send to AnimatLab.
  for(int i=0; i<numServos; i++)
  {
    int id = dynamixelList[i];
    
    int commRes = axcm.getPosition(id,&readPos); 

    //We may want special behavior when the AnimatLab controller begins running.
    //This function will let us detect when that happens.
    if(szerial.getSimStarting())
      loopCounter = 0;

    //If the servo position was read successfully, and if this is the first time step of the AnimatLab
    //controller or if the value has changed significantly from the previous value, add this data
    //to the sentence to send back to AnimatLab.
    if(commRes == 0 && (loopCounter < 1 || abs(readPos - prevReadPos[id])>=TRANSMISSION_THRESHOLD) )
    {
      //Set the flag to true, so we send the sentence when it has been constructed.
      dataToSend = true;

      //Add the ID and position to the next sentence.
      szerial.addData(id,readPos);

      //Save the new read value as the next "previous" value.
      prevReadPos[id] = readPos;
      
      #ifdef DEBUG
//        Serial3.print(id);
//        Serial3.print(": ");
//        Serial3.print(readPos);
//        Serial3.print("\t");
      #endif
    }else{
      #ifdef DEBUG
//        Serial3.println();
      #endif
    }
  }

  //Loop through other inputs.
  for(int i=0;i<numOutAnalog;i++)
  {
    //i is the analog pin index (given the list in Szerial.h).
    //dataIndex is the index in the AnimatData object.
    //animatID is the id of the item (AnimatData.id.ival). This is the value set in the AnimatLab GUI.
    int dataIndex = i+numServos;
    int animatID = i+100; //100 because analog output
    int reading = analogRead(szerial.getAnalogOutIndex(i)) - initReadAnalog[i];
    
    if(reading >= 0 && abs(reading - prevReadAnalog[i])>=TRANSMISSION_THRESHOLD)
    {
      dataToSend = true;
      //szerial.addData(dataIndex,reading);
      szerial.addData(animatID,reading);
      prevReadAnalog[i] = reading;
      #ifdef DEBUG
        Serial3.print(szerial.getAnalogOutIndex(i));
        Serial3.print(": ");
        Serial3.print(reading);
        Serial3.print("\t");
      #endif
    }
  }
  
  loopCounter++;
  
  #ifdef DEBUG
  if (dataToSend)
    Serial3.println();
  #endif

  //If one of the robot's states has changed, send it to AnimatLab.
  if(dataToSend)
    szerial.writeMsgs();

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
    szerial.writeMsgs();
    Serial3.println("Loop terminated by user.");
    //stop communicating with animatlab
    while(true)
    {
      //do nothing
      digitalWrite(BOARD_LED_PIN,HIGH);
    }
  }

  //If there is still more time until the end of this iteration, do nothing
  //until we reach the appropriate time.
  while(millis() - prevSendTime < loopDuration)
  {
    //wait
  }
  prevSendTime = millis();
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


