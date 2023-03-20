//RSS CMG Prototype command program
//A script to control the RSS CMG Prototype based on
//symbols recieved from the IR remote.
//Written by Jasper Grant and Andrew Doucet
//March 20th 2023 for the RSS CMG Capstone

//Include library for IR remote
#include <IRremote.h>

//Define pin for IR reciever
#define IR_PIN 8

//Button Mappings
#define POWER_BUTTON         69
#define VOLUME_PLUS_BUTTON   70
#define VOLUME_MINUS_BUTTON  21
#define REWIND_BUTTON        68
#define FAST_FORWARD_BUTTON  67
//Not a button but ensures that a 0 shows up
//as a continued press as opposed to an
//unrecognized key
#define CONTINUED_PRESS       0

void setup() {
  //Start Serial
  Serial.begin(9600);
  //Start receiving from IR remote
  IrReceiver.begin(IR_PIN);

  //Keep LED on for debugging purposes
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

}

//ANDY CHANGE THESE!

//Functions for specific buttons on the IR remote

void power_button(){
  Serial.println("Emergency Stop");
}

void volume_plus_button(){
  Serial.println("Turning on CMG rotation");
}

void volume_minus_button(){
  Serial.println("Turning off CMG rotation");
}

void rewind_button(){
  Serial.println("Turning CMG CCW");
}

void fast_forward_button(){
  Serial.println("Turning CMG CW");
}

void continued_press_case(){
  Serial.println("I can see you are holding down a key");
}

void unrecognized_key(){
  ;
}

//int last_command = 0;

//Main decoding loop

void loop() {
  //Decode once
  if(IrReceiver.decode()){
    //Go back to listening
    IrReceiver.resume();
    //Decode command
    int command = IrReceiver.decodedIRData.command;
    //if(command == 0) command = last_command;
    
    //Switch to deal with different buttons
    //Each button is mapped to a function
    //Unmapped buttons will have no effect
    switch(command){
      case POWER_BUTTON:
        power_button();
        break;
      case VOLUME_PLUS_BUTTON:
        volume_plus_button();
        break;
      case VOLUME_MINUS_BUTTON:
        volume_minus_button();
        break;
      case REWIND_BUTTON:
        rewind_button();
        break;
      case FAST_FORWARD_BUTTON:
        fast_forward_button();
        break;
      default:
        unrecognized_key();
    }
    //last_command = command;
  }
}
