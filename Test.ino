//MIPS MIPS
//Include the Mona_ESP library
#include <Wire.h>
#include "Mona_ESP_lib.h"

//Variables
bool obstacle[5] = {false, false, false, false, false};
int ir_state[5] = {false, false, false, false, false};
bool obstacle_bool = false;

//Threshold value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there
//false detections.
int threshold1 = 50;
int threshold2 = 50;
int threshold3 = 50;
int threshold4 = threshold2;
int threshold5 = threshold1;
//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state, old_state, lr_random, turn_delay;
//Min and Max values for the robot détection
int min_value_IR = 0; //TODO : determine the value
int max_value_IR = 245; //TODO : determine the value
//Speed
int forward_speed = 240;
int turn_speed = 120;
//Min and Max values for the turn delay
int min_turn_delay = 500;
int max_turn_delay = 1500;
int gap = 30;
int stop_loop = 0;
int old_time = millis();
int tried =0;
int truth_value[5];
int truth_value_old[5];
int nb_robots=0;

int verbose=1;
void setup()
{
  
  Serial.begin(9600);
  Mona_ESP_init();
  state = 0;
  old_state = 0;
}


void loop() {
  int now = millis();
  if (state == 2)
  {
    stop_loop = 1;

    Serial.println(tried);
    int new_time = millis();
    
    Set_LED(1, 10, 10, 50);
    Set_LED(2, 10, 10, 50);
    Motors_stop();
    if (new_time-old_time > 30 and tried < 150 )
    {
    for (int i = 1; i <= 5; i++)
    {
      Disable_IR(i);
      ir_state[i-1] = Read_IR(i);
      Serial.println(ir_state[i-1]);
    }
    /*
    ir_state[0] = Read_IR(1);
    ir_state[1] = Read_IR(2);
    ir_state[2] = Read_IR(3);
    ir_state[3] = Read_IR(4);
    ir_state[4] = Read_IR(5);
    */
    if (not((ir_state[0] > min_value_IR and ir_state[0] < max_value_IR) || (ir_state[1] > min_value_IR and ir_state[1] < max_value_IR) || (ir_state[2] > min_value_IR and ir_state[2] < max_value_IR) || (ir_state[3] > min_value_IR and ir_state[3] < max_value_IR) || (ir_state[4] > min_value_IR and ir_state[4] < max_value_IR)))
    {
      stop_loop=0;
      tried=0;
      state=1;
    }
    old_time=new_time;
    tried+=1;
    }
    else if (tried==150)
    {
      tried+=1;
      lr_random = random(2); //0: left 1: right
      //turn_delay = random(min_turn_delay, max_turn_delay); //Turn delay between min and max
      turn_delay=2000;
      if (lr_random == 0)
      {
        Motors_spin_left(turn_speed); //Left
      }
      else
      {
        Motors_spin_right(turn_speed); //Right
      }
      delay(turn_delay);
      Motors_stop();
      Set_LED(1, 13, 7, 0);
      Set_LED(2, 13, 7, 0);    
      for (int i = 1; i <= 5; i++)
      {
        Enable_IR(i);  
      }
    }
  }
  else 
  {
  for (int i = 1; i <= 5; i++)
  {
    Enable_IR(i);
  }
  delay(30);
  for (int i = 1; i <= 5; i++)
  {
      Disable_IR(i);
  }
  //delay(10);
  //--------------Motors------------------------
  //Set motors movement based on the state machine value.
  
  if (state == 0) { //FORWARD
    Motors_forward(forward_speed);
    Set_LED(1, 0, 20, 0);
    Set_LED(2, 0, 20, 0);
  }
  if (state == 1) { //TURN (random)
    lr_random = random(2); //0: left 1: right
    turn_delay = random(min_turn_delay, max_turn_delay); //Turn delay between min and max
    if (lr_random == 0)
    {
      Motors_spin_left(turn_speed); //Left
    }
    else
    {
     Motors_spin_right(turn_speed); //Right
    }
    Set_LED(1, 13, 7, 0);
    Set_LED(2, 13, 7, 0);
    delay(turn_delay);

  }

  for (int i = 1; i <= 5; i++)
  {
    Enable_IR(i);
  }
  delay(30);
  }
  if (not(stop_loop))
  {
      int old_time = millis();
      int new_time = millis();
    //--------------IR sensors------------------------
    //Decide future state:
    //Read IR state to determine maze walls
    for (int i = 1; i <= 5; i++)
    {
      Disable_IR(i);
    }
    delay(1);

    obstacle[0] = Detect_object(1, threshold1);
    obstacle[1] = Detect_object(2, threshold2);
    obstacle[2] = Detect_object(3, threshold3);
    obstacle[3] = Detect_object(4, threshold4);
    obstacle[4] = Detect_object(5, threshold5);
    ir_state[0] = Read_IR(1);
    ir_state[1] = Read_IR(2);
    ir_state[2] = Read_IR(3);
    ir_state[3] = Read_IR(4);
    ir_state[4] = Read_IR(5);
    obstacle_bool = false;
    for (int j = 1; j <= 5; j++)
    {
     Enable_IR(j);
    }
    delay(30);
    //Print for debug
    
      Serial.print("Read_IR");
      Serial.print(Read_IR(1));
      Serial.print("\n");
      Serial.print(Read_IR(2));
      Serial.print("\n");
      Serial.print(Read_IR(3));
      Serial.print("\n");
      Serial.print(Read_IR(4));
      Serial.print("\n");
      Serial.print(Read_IR(5));
      Serial.print("\n");
    

      if (verbose)
      {
        new_time = millis();
        //Serial.print("Temps pris 1 mesures, en ms:");
        //Serial.println(new_time-old_time);
      }
    //--------------State Machine------------------------
    //Use the retrieved IR state to set the robot state
    //Check the obstacles

    if (obstacle[0] or obstacle[1] or obstacle[2] or obstacle[3] or obstacle[4])
    { //Obstacle detected
      //Wall detected
      Serial.print("obstacle detected \n");
      state = 1; //TURN
      obstacle_bool = true;
    }


    for (int j=0;j<5;j++)
    {
      truth_value[j]=(ir_state[j] > min_value_IR and ir_state[j] < max_value_IR);
      if (truth_value[j])
      {
        nb_robots+=1;
      }
    }
    if ((truth_value[0] || truth_value[1] || truth_value[2] || truth_value[3] ||truth_value[4]))
    {
      Serial.println("Robot ou source détecté");
      Motors_stop();

      //Robot detected
      int source = 1;
      //10 detection
      int old_time = millis();
      int new_time = millis();
      for (int i=0;i<5;i++)
      {
        for (int j = 1; j <= 5; j++)
        {
          Disable_IR(j);
        }
        delay(40);
        ir_state[0] = Read_IR(1);
        ir_state[1] = Read_IR(2);
        ir_state[2] = Read_IR(3);
        ir_state[3] = Read_IR(4);
        ir_state[4] = Read_IR(5);
        if (not((ir_state[0] > min_value_IR and ir_state[0] < max_value_IR) || (ir_state[1] > min_value_IR and ir_state[1] < max_value_IR) || (ir_state[2] > min_value_IR and ir_state[2] < max_value_IR) || (ir_state[3] > min_value_IR and ir_state[3] < max_value_IR) || (ir_state[4] > min_value_IR and ir_state[4] < max_value_IR)))
        {
          source=0;
        }
        for (int j = 1; j <= 5; j++)
        {
          Enable_IR(j);
        }
        delay(30);
      }
      
      if (verbose)
      {
        new_time = millis();
        Serial.print("Temps pris par les 5 mesures, en ms, idéalement plus de 80 ms :");
        Serial.println(new_time-old_time);
      }
      if (source)
      {
        state = 2; //it was the source
        Serial.println("Source détectée");
      }

      else
      {
        state = 1; //TURN, it was a robot
        //Serial.println("Robot detécté");
        Set_LED(1, 50, 0, 0);
        Set_LED(2, 50, 0, 0);
        delay(nb_robots*500);
        Serial.println("Nombre de robots :");
        Serial.print(nb_robots);

        nb_robots=0;
      }
      for (int j = 1; j <= 5; j++)
        {
          Disable_IR(j);
        }
        
      //Serial.print("Robot detected \n");
      obstacle_bool = true;
    }

    if (!obstacle_bool)
    { //No obstacle detected
      state = 0; //FORWARD
    }

    int fin = millis();
    //Serial.println(fin - now);
  }
}