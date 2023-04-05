
// Logging.ino
// Andrew Doucet

// When "play" is pressed on the IR remote, begins logging IMU data to an SD card
// Written to be compatible with main code

// IR Remote library & definition
#define DECODE_NEC
#include <IRremote.h>

// SD card libraries & definitiona
#include <SD.h>
#include <SPI.h>
#define chipSelect BUILTIN_SDCARD

// IMU libray & object
//#include <DFRobot_BMX160.h>
//DFRobot_BMX160 bmx160;

//File object
File myFile;

//Timing variables in miliseconds
unsigned long last_time = 0;
#define INTERVAL 100

// I/O Pins
#define LED_PIN 13
#define IR_PIN 8

//Button Mapping
#define PLAY_BUTTON 64

//Not a button but ensures that a 0 shows up
//as a continued press as opposed to an
//unrecognized key
#define CONTINUED_PRESS       0

// "Is the arduino recording data?"
bool recording = false;

// Angle of the servo
double angle = 0;

void setup() 
{
  //Start Serial for troubleshooting
  Serial.begin(9600);

  //Connect SD card
  if (!SD.begin(chipSelect)) 
  {
    Serial.println("No SD card!");
    return;
  }

  //Clears previous data file
  SD.remove("data.txt");

  //if (!bmx160.begin())
  //{
  //  Serial.println("No IMU");
  //}

  //Start receiving from IR remote
  IrReceiver.begin(IR_PIN);

  //LED used to display recording
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

//Functions for IR Remote play button
void play_button()
{
  if(!recording)
  {
    //Begins data recording
    myFile = SD.open("data.txt", FILE_WRITE);
    digitalWrite(LED_PIN, HIGH);
    recording = true;
  }
  else
  {
    //Ends data recording
    digitalWrite(LED_PIN, LOW);
    recording = false;
    myFile.close();
  }
}

void unrecognized_key(){
  ;
}

//Main decoding loop

void loop() {
  //Decode once
  if(IrReceiver.decode()){
    //Go back to listening
    IrReceiver.resume();
    //Decode command
    
    int command = IrReceiver.decodedIRData.command;
        Serial.println(command);

    //if(command == 0) command = last_command;
    
    //Switch to deal with different buttons
    //Each button is mapped to a function
    //Unmapped buttons will have no effect
    switch(command){
      case PLAY_BUTTON:
        play_button();
        break;
      default:
        unrecognized_key();
    }
  }

  //Logs data at evrey interval if recording
  if (recording)
  {
    unsigned long time = millis();
    if (time - last_time >= INTERVAL) 
    {
      //Writes time in ms
      myFile.print(time);
      myFile.print(",");

      //Collects gyroscope data
      //sBmx160SensorData_t Omagn, Ogyro, Oaccel;
      //bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

      //Writes gyro data in rpm
      //myFile.print(Ogyro.x);
      myFile.print(",");
      //myFile.print(Ogyro.y);
      myFile.print(",");
      //myFile.print(Ogyro.z);
      myFile.print(",");

      //Writes servo angle
      myFile.println(angle);
      
      //Resest time since last log
      last_time = time;
    }
  }
}
