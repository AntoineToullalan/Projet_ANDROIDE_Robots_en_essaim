/*
  Simple_collision_avoider.ino - Usage of the libraries Example
  Using the Mona_ESP library in C style.
  Created by Bart Garcia, December 2020.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
*/
//Include the Mona_ESP library
#include <Wire.h>
#include "Mona_ESP_lib.h"

//Variables
bool IR_object[5] = {false, false, false, false, false};
int IR_values[5] = {250, 250, 250, 250, 250};

int default_speed = 100;

int turn_speed = default_speed;
int forward_speed = default_speed;


int ceil_stop = 210;

//Threshold value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there
//false detections.
int threshold = 30;
//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state, old_state;

void setup()
{
  //Initialize the MonaV2 robot
  Mona_ESP_init();
  //Initialize variables
  state=0;
  old_state=0;
  Serial.begin(9600); // Initialize communication to PC
  // Huge Green light to show the robo is ready
  Set_LED(1, 0, 255, 0);
  Set_LED(2, 0, 255, 0);
}


void loop(){

  
  //--------------Motors------------------------
  // Set motors movement based on the state value.
  switch(state){
    case -1 : Motors_forward(0); forward_speed = default_speed; break;
    case 0 : Motors_forward(forward_speed); turn_speed = default_speed; break;
    case 1 : Motors_forward(forward_speed); turn_speed = default_speed; break;
    case 2 : Motors_spin_right(turn_speed); forward_speed = default_speed; break;
    case 3 : Motors_spin_left(turn_speed); forward_speed = default_speed; break;
  }
  
  //--------------IR sensors------------------------
  //Decide future state:
  //Read IR values to determine maze walls
  //Serial.println("New values");
  for(int i = 0; i<5; i++){
    IR_object[i] = Detect_object(i+1,threshold);
    IR_values[i] = Read_IR(i+1);
    //Serial.println(IR_values[i]);
  }

  state = update_dir();
  
  forward_speed += 2;
  turn_speed += 2;
  
  delay(100);
}

//--------------State Machine------------------------
//Use the retrieved IR values to set state
//Check for frontal wall, which has priority
int update_dir(){
  
  for(int i = 0; i < 5; i++){
    if (IR_values[i] < ceil_stop){
      Set_LED(2, 100, 0, 0); // Red
      Set_LED(1, 100, 0, 0);
      return -1;
    }
  }
  if(IR_object[2]){
    Set_LED(1, 0, 0, 250); // Blue
    Set_LED(2, 0, 0, 250);
    return 1;
  }
  else if(IR_object[0] or IR_object[1]){ //Check for left proximity
    Set_LED(2, 0, 0, 0); // Orange
    Set_LED(1, 13, 7, 0);    
    return 3;
  }
  else if(IR_object[3] or IR_object[4]){// Check for right proximity
    Set_LED(1, 0, 0, 0); // Orange
    Set_LED(2, 13, 7, 0);  
    return 2;
  }
  Set_LED(1, 0, 0, 10); // Blue
  Set_LED(2, 0, 0, 10);
  return 0;
}
