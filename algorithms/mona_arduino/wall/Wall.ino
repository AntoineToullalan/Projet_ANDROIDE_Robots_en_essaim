#include <Wire.h>
#include "Mona_ESP_lib.h"
#include <time.h>
#include <stdlib.h>


bool IR_object[5] = {false, false, false, false, false};
int threshold = 50;
int threshold_robot = 30;
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state, side, r;
float delay_reset;

int turn_speed = 100;
int forward_speed = 150;


void setup(){
  Mona_ESP_init();
  state = 0;
  side = 0;
  r = 0;
  delay_reset = - 20.0 / 100;
  Serial.begin(9600); // Initialize communication to PC
  srand(time(NULL));
}


void loop(){

  compute_motor();

  // Get all the IR values
  for(int i = 0; i<5; i++){
    IR_object[i] = Detect_object(i+1,threshold);
  }

  if (Get_IR(1) < threshold_robot){
    state = 2;
  }
  else if (Get_IR(5) < threshold_robot){
    state = 3;
  }
  else if (IR_object[1] or IR_object[2] or IR_object[3]){
    // Check if there is a wall in front of the robot
    if (side == 0) {
      // If we haven't choose a side right now, we need to choose one
      if(IR_object[1] > IR_object[3]){
        side = 1;
      }else{
        side = 2;
      }
    }
    delay_reset = 0;
    state = 1;
  }
  else if (IR_object[0]){ //Check for left proximity
    delay_reset = 0;
    side = 2;
    state = 0;
  }
  else if (IR_object[4]){// Check for right proximity
    delay_reset = 0;
    side = 1;
    state = 0;
  }
  else{ //If there are no proximities, move forward
    if (delay_reset >= 2) {
      side = 0;
      delay_reset = 0;
    }

    delay_reset = delay_reset + 20.0 / 100;
    state = 0;
  }

  Serial.println(delay_reset);

  delay(20);
}


//--------------Motors------------------------
// Set motors movement based on the state value.
void compute_motor(){
  switch(state){
    case 0 : Motors_forward(forward_speed); break;
    case 1 : side_choice(); break;
    case 2 : Motors_spin_right(turn_speed); break;
    case 3 : Motors_spin_left(turn_speed); break;
  }
}


void side_choice(){
  if(side == 1){
    Motors_spin_left(turn_speed);
  }
  if(side == 2){
    Motors_spin_right(turn_speed);
  }
}
