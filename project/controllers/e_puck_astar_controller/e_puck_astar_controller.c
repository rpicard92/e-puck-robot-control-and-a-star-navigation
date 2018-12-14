/***************************************************************************

  e-puck_line -- Base code for a practical assignment on behavior-based
  robotics. When completed, the behavior-based controller should allow
  the e-puck robot to follow the black line, avoid obstacles and
  recover its path afterwards.
  Copyright (C) 2006 Laboratory of Intelligent Systems (LIS), EPFL
  Authors: Jean-Christophe Zufferey
  Email: jean-christophe.zufferey@epfl.ch
  Web: http://lis.epfl.ch

  This program is free software; any publications presenting results
  obtained with this program must mention it and its origin. You
  can redistribute it and/or modify it under the terms of the GNU
  General Public License as published by the Free Software
  Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
  USA.

***************************************************************************/

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/led.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Global defines - preset
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0            // for wb_robot_get_mode() function
#define REALITY 2               // for wb_robot_get_mode() function
#define TIME_STEP  32           // [ms]


/////////// Epuck definitions......

// epuck wheel speed
#define LFM_FORWARD_SPEED 200

// gain for line following steering correction
#define LFM_K_GS_SPEED 0.3

// Threasholds are used as cutoff values for detecting the track and the board.

// USE FOR REAL EPUCK
#define Board_Min_Threshold 550
#define Track_Max_Threshold 400

// USE FOR SIMULATION
//#define Board_Min_Threshold 700
//#define Track_Max_Threshold 600

// Grid Size
#define GRID_ROWS 12
#define GRID_COLS 12
#define QUEUE_PARAMS 7
#define DIRECTIONS 4

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

////////// random pause definitions.....

// 1 for random pauses, 0 for none
#define PAUSE 0
int pause_flag = PAUSE; 


///////// Lab related adjustable parameters.....


// lab
#define INITIAL_X 1
#define INITIAL_Y 4


// LAB
#define GOAL_X 9
#define GOAL_Y 4


// LAB
int grid[GRID_ROWS][GRID_COLS] = {
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,1,0,0,0,0,0,0,0},
    {0,1,0,0,1,0,0,0,0,0,0,0},
    {0,1,0,0,1,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,0,0,0,0},
    {0,1,0,0,0,0,0,1,0,0,0,0},
    {0,1,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,1,1,1,1,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0}
  };
 

///////// Driving related items.........

WbDeviceTag gs[NB_GROUND_SENS]; // ground sensors
unsigned short gs_value[NB_GROUND_SENS]={0,0,0}; // storage for ground sensor data
int lfm_speed[2]; // wheel speeds storage
int nodeCount = 0; // keeps track of node count on a per command basis
int stillOnCurrentNode = 0; // flag for still on current node
int turningLock = 0; // flag to lock for turning
int turningTimer = 0; // later set as a timing delay for turning
int turnInit = 0; // flag to marking that a turn has been initiated
int speed[2]; // store speed globally
int i; // index declared
int Mode=1; // flag for switching form simulation to real epuck
int lineJustPassedNodeDetected = 0; 
int passedNodeFlag = 0;
int down = 1; // directions
int up = 2; // directions
int left = 3; // directions
int right = 4; // directions
int straight = 1; // for naming purposes make straight equivalent to down
int doubleCheckNode = -0; // be sure you at a node by testing multiple times

//////////// Astar related items below..... 
  
const int null = -1; // null constant for graphs

// graph neighbor arrays

// up
int graph_direction_up[GRID_ROWS][GRID_COLS];  
int graph_neighbor_row_up[GRID_ROWS][GRID_COLS];  
int graph_neighbor_col_up[GRID_ROWS][GRID_COLS];

// down
int graph_direction_down[GRID_ROWS][GRID_COLS];  
int graph_neighbor_row_down[GRID_ROWS][GRID_COLS];  
int graph_neighbor_col_down[GRID_ROWS][GRID_COLS];

// left
int graph_direction_left[GRID_ROWS][GRID_COLS];    
int graph_neighbor_row_left[GRID_ROWS][GRID_COLS];  
int graph_neighbor_col_left[GRID_ROWS][GRID_COLS];

// right
int graph_direction_right[GRID_ROWS][GRID_COLS];    
int graph_neighbor_row_right[GRID_ROWS][GRID_COLS];  
int graph_neighbor_col_right[GRID_ROWS][GRID_COLS];  

int currentX = INITIAL_X; // set current to inital at beginning
int currentY = INITIAL_Y; // set current to inital at beginning
int finished = 0; // flag for finished astar
int visited[GRID_ROWS][GRID_COLS];
int directions[GRID_ROWS*GRID_COLS];
int key; //will hold top of keys chain
int queue[GRID_ROWS*GRID_COLS*DIRECTIONS][QUEUE_PARAMS]; // array disguised as a queue
int reverse_array[GRID_ROWS*GRID_COLS]; // gets retrieved from queue in reverse


///////////// Functions begin.....


// this function tells whether a sensor is detects a line
int isLineSensed(int GroundSensorValue){
  // if the sensor detects a line
  if(GroundSensorValue<=Track_Max_Threshold){
    return 1;
  // if the sensor does not detect line
  } else if(GroundSensorValue>=Board_Min_Threshold){
    return 0;
  // if unknown
  } else {
    return -1;
  }
}

// this function tells whether the robot is on a node
int isNodeSensed(int left, int center, int right){
  // if all sensors are on line
  if((left == 1) && (center == 1) && (right == 1)){
    return 1;
  }
  // not on a node
  return 0;
}


// this function tells whether the robot detects a new node
void detectNode(int left, int center, int right){
  // if the robot has left the nodenode
  if (stillOnCurrentNode == 0){
    // find out if the robot senses a new node
    int node = isNodeSensed(left, center, right);
    
      // if the robot is senses a new node
      if(node == 1){
          if(doubleCheckNode < 5){
            doubleCheckNode = doubleCheckNode + 1;
          } else if(doubleCheckNode >=5){
            doubleCheckNode = 0;
            // add the node to count
            nodeCount = nodeCount + 1;
            // mark that the robot is on a new node
            stillOnCurrentNode = 1;
            // logger
            printf("\nNode # %d Found\n",nodeCount);
             
             
            int timer = 10;
            while(timer > 0){
              timer = timer - 1;
            }
        }
      } 
    }
}


// this function detects when the robot has just left a node
int detectLineJustPassedNode(int left, int center, int right){
 // if the robot has been on a node
 if(stillOnCurrentNode == 1){

    // find out if the robot is still on the node
    int node = isNodeSensed(left, center, right);
    // if the robot is not still on the node
    if(node == 0){
      // mark that the robot is no longer on the node
      stillOnCurrentNode = 0;
      // logger
      printf("\nBack On Line\n");
      return 1;
    }
  }
  return 0;
};

// this function contain the turn logic
void turn(int turnDirection, int LeftSensorOnTrack, int CenterSensorOnTrack, int RightSensorOnTrack){
    // if the robot is not currently turning
  if(turningLock==0){
      // mark that the robot is turning
      turningLock = 1;
      // set a timer for 25. this will allow the robot get completely off a line before attempting to
      // sense the line it is tuning onto.
      turningTimer = 30;
    // if the robot is turning left
    if(turnDirection==0){
      // hard coded turning constants
      lfm_speed[LEFT]  = -130;
      lfm_speed[RIGHT] = 180;
      // logger
      printf("\nLEFT TURN START\n");
      // if the robot is turning right
    } else if(turnDirection==1){
      // hard coded turning constants
      lfm_speed[LEFT]  = 180;
      lfm_speed[RIGHT] = -130;
      // logger
      printf("\nRIGHT TURN START\n");
    }
  // if the robot is still turning
  } else if(turningLock==1){
    // if turn timer has not reached 0
    if(turningTimer !=0){
      // decrement the timer
      turningTimer = turningTimer - 1;
    }
    // if turn timer has reached 0
    if(turningTimer == 0){
      // if a left turn has been completed
     if(((turnDirection == 0) && (LeftSensorOnTrack==0) && (CenterSensorOnTrack==0) && (RightSensorOnTrack == 1)) ||
     ((turnDirection == 0) && (LeftSensorOnTrack==0) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 1)) || 
     ((turnDirection == 0) && (LeftSensorOnTrack==1) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 1))){
        // mark that the robot is no longer turning
        turningLock = 0;
        // logger
        printf("\nLEFT TURN FINISHED\n");
      // if a right turn has been completed
     } else if(((turnDirection == 1) && (LeftSensorOnTrack==1) && (CenterSensorOnTrack==0) && (RightSensorOnTrack == 0)) ||
     ((turnDirection == 1) && (LeftSensorOnTrack==1) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 0)) ||
      ((turnDirection == 1) && (LeftSensorOnTrack==1) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 1))){
        // mark that the robot is no longer turning
         turningLock = 0;
        // logger
        printf("\nRIGHT TURN FINISHED\n");
     }
    }
  }
}

// this function drives just passed node (used in conjuction with the drive function)
void driveJustPassed(){
     // get the ground sensor values
    int LeftSensorReading = gs_value[GS_LEFT];
    int CenterSensorReading = gs_value[GS_CENTER];
    int RightSensorReading = gs_value[GS_RIGHT];
    // logger
    //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
    
    // find out which sensors are on the line
    int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
    int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
    int RightSensorOnTrack = isLineSensed(RightSensorReading);
    // logger
    // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    
   
    // find out if the robot has just passed the node
    lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);

    // drive straight
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED;    
}


// this function drives the robot straigh for N nodes
void driveStraight(int numberOfNodes){
     // get the ground sensor values
    int LeftSensorReading = gs_value[GS_LEFT];
    int CenterSensorReading = gs_value[GS_CENTER];
    int RightSensorReading = gs_value[GS_RIGHT];
    // logger
    //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
    
    // find out which sensors are on the line
    int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
    int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
    int RightSensorOnTrack = isLineSensed(RightSensorReading);
    // logger
    // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    
    // find out if robot is detects a new node
    detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
   
    // find out if the robot has just passed the node
    lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
 
  // loop till the robot has passed the number of nodes
  if(nodeCount<numberOfNodes){

    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  } 
}

// this function turns the robot to left
void leftTurn(){
  // get the ground sensor values
  int LeftSensorReading = gs_value[GS_LEFT];
  int CenterSensorReading = gs_value[GS_CENTER];
  int RightSensorReading = gs_value[GS_RIGHT];
  // logger
  //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
  
  // find out which sensors are on the line
  int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
  int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
  int RightSensorOnTrack = isLineSensed(RightSensorReading);
  // logger
  // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);

  // if not locked for turning
  if(turningLock != 1){
    // find out if robot is detects a new node
    detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
      
    // find out if the robot has just passed the node
    lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
   }
  // loop till the robot has passed the number of nodes
  if((lineJustPassedNodeDetected == 1) || (turningLock == 1)){
    // logger
    // printf("\JustPassedNode: %d, lineJustPassedNodeDetected);
    // turn the robot
    turn(0,LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    turnInit=1;
  } else {
    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  }
}

// this function turns the robot to left
void immediateLeftTurn(){
  // get the ground sensor values
  int LeftSensorReading = gs_value[GS_LEFT];
  int CenterSensorReading = gs_value[GS_CENTER];
  int RightSensorReading = gs_value[GS_RIGHT];
  // logger
  //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
  
  // find out which sensors are on the line
  int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
  int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
  int RightSensorOnTrack = isLineSensed(RightSensorReading);
  // logger
  // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);


  // loop till the robot has passed the number of nodes
  if((turnInit==0) || (turningLock == 1)){
    // logger
    // printf("\JustPassedNode: %d, lineJustPassedNodeDetected);
    // turn the robot
    turn(0,LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    turnInit=1;
  } else {
    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  }
}

// this function turns the robot to the right
void rightTurn(){
  // get the ground sensor values
  int LeftSensorReading = gs_value[GS_LEFT];
  int CenterSensorReading = gs_value[GS_CENTER];
  int RightSensorReading = gs_value[GS_RIGHT];
  // logger
  //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
  
  // find out which sensors are on the line
  int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
  int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
  int RightSensorOnTrack = isLineSensed(RightSensorReading);
  // logger
  // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
   
   // if not locked for turning
  if(turningLock != 1){
    // find out if robot is detects a new node
    detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
      
    // find out if the robot has just passed the node
    lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
  }
  // loop till the robot has passed the number of nodes
  if((lineJustPassedNodeDetected == 1) || (turningLock == 1)){
    // logger
    // printf("\JustPassedNode: %d, lineJustPassedNodeDetected);
    // turn the robot
    turn(1,LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    turnInit=1;
  } else {
    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  }
}

// this function turns the robot to the right
void immediateRightTurn(){
  // get the ground sensor values
  int LeftSensorReading = gs_value[GS_LEFT];
  int CenterSensorReading = gs_value[GS_CENTER];
  int RightSensorReading = gs_value[GS_RIGHT];
  // logger
  //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
  
  // find out which sensors are on the line
  int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
  int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
  int RightSensorOnTrack = isLineSensed(RightSensorReading);
  // logger
  // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
   
  // loop till the robot has passed the number of nodes
  if((turnInit == 0) || (turningLock == 1)){
    // logger
    // printf("\JustPassedNode: %d, lineJustPassedNodeDetected);
    // turn the robot
    turn(1,LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    turnInit=1;
  } else {
    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  }
}

// this function drives the robot in direction for N nodes
void drive(int numberOfNodes,int direction){

  // used for random pauses
  int pause_number = (rand() % 10)*10;
  printf("\nPause #: %d\n", pause_number);
  int pause_count = 0;

  // while nodes are left
  while(nodeCount<numberOfNodes){
    // read sensors value - this must be called here in the loop
    for(i=0;i<NB_GROUND_SENS;i++) gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    
    
    //  down
    if(direction == down){
      driveStraight(numberOfNodes);
    // if left
    } else if(direction == left){
      if((turnInit==0)||(turningLock==1)){
        immediateLeftTurn();
      } else if((turnInit==1)||(turningLock==0)){
        driveStraight(numberOfNodes);
      }
    // if right
    } else if(direction == right){
      if((turnInit==0)||(turningLock==1)){
        immediateRightTurn();
      } else if((turnInit==1)||(turningLock==0)){
        driveStraight(numberOfNodes);
      }
    }
    
    // if random pauses are enabled
    if(PAUSE == 1){
      pause_count = pause_count + 1;
      if(pause_count == pause_number){
        int time = (rand() % 10)*15;
        int timer = time;
        printf("\nPause for: %d\n", time);
        while(timer > 0){
          timer = timer -1;
          wb_differential_wheels_set_speed(0,0);
          wb_robot_step(TIME_STEP);
        }
      } 
    }
    // drive
    wb_differential_wheels_set_speed(lfm_speed[LEFT],lfm_speed[RIGHT]);
    wb_robot_step(TIME_STEP);
  }
  
  // set flag 
  if(passedNodeFlag != 1){
    lineJustPassedNodeDetected=0;
  } 
  
  // after the robot has reached the node of the command is finish driving to just pass the node until a line is detected
  while(lineJustPassedNodeDetected != 1){

    // read sensors value - this must be called here in the loop
    for(i=0;i<NB_GROUND_SENS;i++) gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    driveJustPassed();
    
    // drive
    wb_differential_wheels_set_speed(lfm_speed[LEFT],lfm_speed[RIGHT]);
    wb_robot_step(TIME_STEP);
  }
  
  // pause just after last node is passed (helps with stability before turns)
  int timer = 10;
  while(timer > 0){
    timer = timer -1;
    wb_differential_wheels_set_speed(0,0);
    wb_robot_step(TIME_STEP);
  }
  
  // drive forward slightly to deconflict sensor readings (help with stability before turns)
  timer = 5;
  while(timer > 0){
    timer = timer -1;
   
    // read sensors value - this must be called here in the loop
    for(i=0;i<NB_GROUND_SENS;i++) gs_value[i] = wb_distance_sensor_get_value(gs[i]);
      
    int LeftSensorReading = gs_value[GS_LEFT];
    int CenterSensorReading = gs_value[GS_CENTER];
    int RightSensorReading = gs_value[GS_RIGHT];
   
    // drive
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
    wb_differential_wheels_set_speed(lfm_speed[LEFT],lfm_speed[RIGHT]);
    wb_robot_step(TIME_STEP);
  } 
  
  // set flag
  if(lineJustPassedNodeDetected==1){
   passedNodeFlag = 0;
  }
  
  // reset
  turnInit=0;
  nodeCount=0;
  
  // stop
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);

}

// this function recusively retrieves the data from astar queue solution
int retrieveRoute(int key, int count){
  if(key > 0){
    int direction = queue[key][2];
    reverse_array[count] = direction;
    int previous_key = queue[key][5];
    count = count + 1;
    count = retrieveRoute(previous_key, count);
  }
  // return the number keys retrieved
  return count;
}

// This function provides the robot with a driving route
void driveRoute(int key){
  printf("\nKEY: %d\n", key);
  int direction;
  int number_of_nodes = 1;
  int count = 0;
  count = retrieveRoute(key, count);
  printf("\nCOUNT: %d\n", count);

  // reverse the route (it's initially backwards)
  int correct_array[GRID_ROWS*GRID_COLS];
  for(int i = 0; i < count; i++){
    int direction = reverse_array[i];
    correct_array[count-1-i] = direction;
  }
  
  // print the route
  for(int i = 0; i < count; i++){
    int direction = correct_array[i];
    printf("\nDIRECTION: %d\n", direction);
  }

  
  // drive the directions and correct for orientation
  int index = 0;
  while(index <= count){
    int direction = correct_array[index];
    // if this is not the first direct
    if(index != 0){
      int previouseDirection = correct_array[index-1];
      printf("\nINDEX %d, COUNT %d\n", index, count);
      printf("\nDIRECTION: %d, PREVIOUS: %d\n", direction, previouseDirection);
      if((direction == left) && (previouseDirection == down)){
          int turn_node_count = 0;
          while((index+turn_node_count <= count) && (correct_array[index + turn_node_count] == left)){
            turn_node_count = turn_node_count + 1;
          }
          index = index + turn_node_count-1;
          printf("\nNodes %d, Dir %d\n", turn_node_count, direction);
          drive(turn_node_count, left);
      } else if((direction == right) && (previouseDirection == down)){
          int turn_node_count = 0;
          while((index+turn_node_count <= count) && (correct_array[index + turn_node_count] == right)){
            turn_node_count = turn_node_count + 1;
          }
          index = index + turn_node_count-1;
          printf("\nNodes %d, Dir %d\n", turn_node_count, direction);
          drive(turn_node_count, right);
      } else if((direction == down) && (previouseDirection == left)){
          int turn_node_count = 0;
          while((index+turn_node_count <= count) && (correct_array[index + turn_node_count] == down)){
            turn_node_count = turn_node_count + 1;
          }
          index = index + turn_node_count-1;
          printf("\nNodes %d, Dir %d\n", turn_node_count, direction);
          drive(turn_node_count, right);
      } else if((direction == down) && (previouseDirection == right)){
          int turn_node_count = 0;
          while((index+turn_node_count <= count) && (correct_array[index + turn_node_count] == down)){
            turn_node_count = turn_node_count + 1;
          }
          index = index + turn_node_count-1;
          printf("\nNodes %d, Dir %d\n", turn_node_count, direction);
          drive(turn_node_count, left);
      }
    // if it is the first direction 
    } else {
      if(direction == down){
          int turn_node_count = 0;
          while((index+turn_node_count <= count) && (correct_array[index + turn_node_count] == down)){
            turn_node_count = turn_node_count + 1;
          }
          index = index + turn_node_count - 1;
          printf("\nNodes %d, Dir %d\n", turn_node_count, direction);
          drive(turn_node_count, down);
      }
    }
    
  printf("\nINDEX %d, COUNT %d\n", index, count);
  index = index + 1;
  }       
}


// this function converts the grid to arrays that function as a graph
// it assumes the epuck start at the top corner
void convertGridToGraphs(){
  int width = GRID_ROWS;
  int height = GRID_COLS;
  for(int row = 0; row < width; row++){
      for(int col = 0; col < height; col++){
        // if the cell to the right is a path
        if((row < height - 1) && (grid[row + 1][col] == 1)){          
          graph_direction_down[row][col] = down;
          graph_neighbor_row_down[row][col] = row + 1;
          graph_neighbor_col_down[row][col] = col;         
        // otherwise null           
        } else if ((row < height - 1) && (grid[row + 1][col] == 0)){        
          graph_direction_down[row][col] = null;
          graph_neighbor_row_down[row][col] = row + 1;
          graph_neighbor_col_down[row][col] = col;
        }
        
        // if the cell below is a path
        if((col < width -1) && (grid[row][col + 1] == 1)){        
          graph_direction_right[row][col] = left;
          graph_neighbor_row_right[row][col] = row;
          graph_neighbor_col_right[row][col] = col + 1;          
        // otherwise null
        } else if((col < width -1) && (grid[row][col + 1] == 0)){        
          graph_direction_right[row][col] = null;
          graph_neighbor_row_right[row][col] = row;
          graph_neighbor_col_right[row][col] = col + 1;
        }
        
        // if the cell below is a path
        if((col < width -1) && (grid[row][col - 1] == 1)){     
          graph_direction_left[row][col] = right;
          graph_neighbor_row_left[row][col] = row;
          graph_neighbor_col_left[row][col] = col - 1;
        // otherwise null
        } else if((col < width -1) && (grid[row][col - 1] == 0)){        
          graph_direction_left[row][col] = null;
          graph_neighbor_row_left[row][col] = row;
          graph_neighbor_col_left[row][col] = col - 1;          
        }
    }
  }
}

// this function returns the absolute value of an integer
int absoluteValue(int number){
  if(number < 0){
    return -1*number;
  } else if(number > 0){
    return number;
  } 
  return 0;
}

// this function is the heuristic for the astar
int heuristic(int neighborX, int neighborY){
  return (absoluteValue(neighborX - GOAL_X) + absoluteValue(neighborY - GOAL_Y));
}

// this function runs the astar algorithm and fills the queue with directions
void aStar(){
  int width = GRID_ROWS;
  int height = GRID_COLS;
  int path_cost = 0;
  int path = 0;
  
  // up
  int neighbor_up = 0;
  int neighbor_row_up = 0;
  int neighbor_col_up = 0;
  
  // down
  int neighbor_down = 0;
  int neighbor_row_down = 0;
  int neighbor_col_down = 0;
  
  // left
  int neighbor_left = 0;
  int neighbor_row_left = 0;
  int neighbor_col_left = 0;

  // right
  int neighbor_right = 0;
  int neighbor_row_right = 0;
  int neighbor_col_right = 0;
  
  
  int est_dist_up = heuristic(currentX, currentY);
  int est_dist_down = heuristic(currentX, currentY);
  int est_dist_left = heuristic(currentX, currentY);
  int est_dist_right = heuristic(currentX, currentY);


  int push_count = 0;
  int pop_count = 0;
  queue[push_count][0] = 0;
  queue[push_count][1] = 0;
  queue[push_count][2] = 0;
  queue[push_count][3] = currentX;
  queue[push_count][4] = currentY;
  queue[push_count][5] = 0;
  queue[push_count][6] = push_count;
  push_count = push_count + 1;

  while(finished == 0){
    
    // pop queue
    int est_dist = queue[pop_count][0];
    int path_cost = queue[pop_count][1];
    int dir = queue[pop_count][2];
    currentX = queue[pop_count][3];
    currentY = queue[pop_count][4];
    key = queue[pop_count][6];
    pop_count = pop_count + 1;
    
    printf("\nPOP: %d, DIRECTION %d, CurrentX %d, CurrentY %d", push_count, dir, currentX, currentY);

    // if it's found the destination
    if((currentX == GOAL_X) && (currentY == GOAL_Y)){
      printf("\nFINAL!: POP: %d, DIRECTION %d, CurrentX %d, CurrentY %d, Key %d\n", push_count, dir, currentX, currentY, key);
      finished = 1;
    // otherwise keep seraching
    } else if(visited[currentX][currentY] == 0) {
          
          // mark as visited
          visited[currentX][currentY] = 1;
      
          // get up info
          neighbor_up = graph_direction_up[currentX][currentY];
          neighbor_row_up = graph_neighbor_row_up[currentX][currentY];
          neighbor_col_up = graph_neighbor_col_up[currentX][currentY];
          est_dist_up = est_dist_up + heuristic(neighbor_row_up, neighbor_col_up);
          
          // if not null push to queue
          if(neighbor_up != null){
            queue[push_count][0] = est_dist_up;
            queue[push_count][1] = path_cost + 1;
            queue[push_count][2] = neighbor_up;
            queue[push_count][3] = neighbor_row_up;
            queue[push_count][4] = neighbor_col_up;
            queue[push_count][5] = key; // previous key
            queue[push_count][6] = push_count; // current key

            push_count = push_count + 1;
            printf("\nUP: PUSH: %d, DIRECTION %d, CurrentX %d, CurrentY %d, Key %d", push_count, dir*10 + neighbor_up, currentX, currentY, key);
          }
          
          // get down info
          neighbor_down = graph_direction_down[currentX][currentY];
          neighbor_row_down = graph_neighbor_row_down[currentX][currentY];
          neighbor_col_down = graph_neighbor_col_down[currentX][currentY];
          est_dist_down = est_dist_down + heuristic(neighbor_row_up, neighbor_col_up);
         
          // if not null push to queue
          if(neighbor_down != null){
            queue[push_count][0] = est_dist_down;
            queue[push_count][1] = path_cost + 1;
            queue[push_count][2] = neighbor_down;
            queue[push_count][3] = neighbor_row_down;
            queue[push_count][4] = neighbor_col_down;
            queue[push_count][5] = key; // previous key
            queue[push_count][6] = push_count; // current key
            push_count = push_count + 1;
            printf("\nDOWN: PUSH: %d, DIRECTION %d, CurrentX %d, CurrentY %d, Key %d", push_count, dir*10 + neighbor_up, currentX, currentY, key);
          }

          // get left info
          neighbor_left = graph_direction_left[currentX][currentY];
          neighbor_row_left = graph_neighbor_row_left[currentX][currentY];
          neighbor_col_left = graph_neighbor_col_left[currentX][currentY];
          est_dist_left = est_dist_left + heuristic(neighbor_row_up, neighbor_col_up);
          
          // if not null push to queue
          if(neighbor_left != null){
            queue[push_count][0] = est_dist_left;
            queue[push_count][1] = path_cost + 1;
            queue[push_count][2] = neighbor_left;
            queue[push_count][3] = neighbor_row_left;
            queue[push_count][4] = neighbor_col_left;
            queue[push_count][5] = key; // previous key
            queue[push_count][6] = push_count; // current key
            push_count = push_count + 1;
            printf("\nLEFT PUSH: %d, DIRECTION %d, CurrentX %d, CurrentY %d, Key %d", push_count, dir*10 + neighbor_up, currentX, currentY, key);
          }

          
          // get right info
          neighbor_right = graph_direction_right[currentX][currentY];
          neighbor_row_right = graph_neighbor_row_right[currentX][currentY];
          neighbor_col_right = graph_neighbor_col_right[currentX][currentY];
          est_dist_right = est_dist_right + heuristic(neighbor_row_up, neighbor_col_up);
          
          // if not null push to queue
          if(neighbor_right != null){
            queue[push_count][0] = est_dist_right;
            queue[push_count][1] = path_cost + 1;
            queue[push_count][2] = neighbor_right;
            queue[push_count][3] = neighbor_row_right;
            queue[push_count][4] = neighbor_col_right;
            queue[push_count][5] = key; // previous key
            queue[push_count][6] = push_count; // current key
            push_count = push_count + 1;
            printf("\nRIGHT PUSH: %d, DIRECTION %d, CurrentX %d, CurrentY %d, Key %d", push_count, dir*10 + neighbor_up, currentX, currentY, key);
          }

    }
  }
}



////////////////////////////////////////////
// Main
int main() {

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];

  // grab ground sensors
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i],TIME_STEP);
  }

  // Reset all BB variables when switching from simulation to real robot and back
  if (Mode!=wb_robot_get_mode())
  {
    Mode = wb_robot_get_mode();
    if (Mode == SIMULATION) {
      wb_differential_wheels_set_speed(0,0); wb_robot_step(TIME_STEP); // Just run one step to make sure we get correct sensor values
      printf("\n\n\nSwitching to SIMULATION and reseting all BB variables.\n\n");
      printf("\n\n\nSwitching to REALITY and reseting all BB variables.\n\n");
    } else if (Mode == REALITY) {
      wb_differential_wheels_set_speed(0,0); wb_robot_step(TIME_STEP); // Just run one step to make sure we get correct sensor values
      printf("\n\n\nSwitching to REALITY and reseting all BB variables.\n\n");
      printf("\n\nSwitching to REALITY and reseting all BB variables.\n\n");
    }
  }
 
  // Reset all BB variables when switching from simulation to real robot and back
  if (Mode!=wb_robot_get_mode())
  {
    Mode = wb_robot_get_mode();
    if (Mode == SIMULATION){
      printf("\nSwitching to SIMULATION and reseting all BB variables.\n\n");
    } else if (Mode == REALITY){
      printf("\nSwitching to REALITY and reseting all BB variables.\n\n");
    }
  }

  
   ////////////////////E-PUCK A Star Control Lab/////////////////////////
   
   
   /////// PART A - hard code simple command path /////////
   //
     // INFO: Use the following command in theis section: drive(NumberOfNodes, Direction).
     // INFO: Note: Direction can be left, right, or straight. The NumberOfNodes is an integer value.
     // INFO: Example:  drive(3, left);
     //Answer goes here....
     
     // SOLUTION FOR TA:
      //drive(10, straight);
      //drive(11, left);
   //
   
   ////// PART B - hard code complex command path /////////
   //
     // INFO: Use the following command in theis section: drive(NumberOfNodes, Direction).
     // INFO: Note: Direction can be left, right, or straight. The NumberOfNodes is an integer value.
     // INFO: Example:  drive(3, left);
     //Answer goes here....

     // SOLUTION FOR TA:
       //drive(3, straight);
       //drive(3, left);
       //drive(3, right);
       //drive(3, right);
       //drive(2, left);
   //
    
   ////// PART C  - Running A Star /////////
   //
      // Info: Uncomment the 3 commands below and run the program.
      //convertGridToGraphs();
      //aStar();
      //driveRoute(key);
   //
   
  wb_robot_step(TIME_STEP);
  return 0;
}
