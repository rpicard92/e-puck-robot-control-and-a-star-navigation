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

// Global defines
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

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

// ground sensors
WbDeviceTag gs[NB_GROUND_SENS];
unsigned short gs_value[NB_GROUND_SENS]={0,0,0};

// grid
unsigned short map[12][12] = {
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,0},
    {0,0,0,0,0,0,0,0,0,0,0,0}
  };
// wheel speeds
int lfm_speed[2];

// epuck wheel speed
#define LFM_FORWARD_SPEED 200

// gain for line following steering correction
#define LFM_K_GS_SPEED 0.1

// Threasholds are used as cutoff values for detecting the track and the board. -RonPicard
#define Board_Min_Threshold 700
#define Track_Max_Threshold 500

// Keep track of # of nodes detected -RonPicard
int nodeCount = 0;
int stillOnCurrentNode = 0;
int turningLock = 0;
int turningTimer = 0;
int turnInit = 0;
int i;
int speed[2];
int Mode=1;


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
int detectNode(int left, int center, int right){
  // if the robot has left the nodenode
  if (stillOnCurrentNode == 0){
    // find out if the robot senses a new node
    int node = isNodeSensed(left, center, right);
    // if the robot is senses a new node
    if(node == 1){
      // add the node to count
      nodeCount = nodeCount + 1;
      // mark that the robot is on a new node
      stillOnCurrentNode = 1;
      // logger
      printf("\nNode # %d Found\n",nodeCount);
      return 1;
    }
  }
  // the robot is still on the current node
  return 0;
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
      // set a timer for 25 seconds. this will allow the robot get completely off a line before attempting to
      // sense the line it is tuning onto.
      turningTimer = 25;
    // if the robot is turning left
    if(turnDirection==0){
      // hard coded turning constants
      lfm_speed[LEFT]  = -70;
      lfm_speed[RIGHT] = 180;
      // logger
      printf("\nLEFT TURN START\n");
      // if the robot is turning right
    } else if(turnDirection==1){
      // hard coded turning constants
      lfm_speed[LEFT]  = 180;
      lfm_speed[RIGHT] = -70;
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
      if((turnDirection == 0) && (LeftSensorOnTrack==0) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 1)){
        // mark that the robot is no longer turning
        turningLock = 0;
        // logger
        printf("\nLEFT TURN FINISHED\n");
      // if a right turn has been completed
      } else if((turnDirection == 1) && (LeftSensorOnTrack==1) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 0)){
        // mark that the robot is no longer turning
        turningLock = 0;
        // logger
        printf("\nRIGHT TURN FINISHED\n");
      }
    }
  }
}

void driveStraight(int numberOfNodes){
  // loop till the robot has passed the number of nodes
  if(nodeCount<=numberOfNodes){
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
    int nodeDetected = detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
    // logger
    // printf("\Node: %d, nodeDetected);
  
    // find out if the robot has just passed the node
    int lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    // logger
    // printf("\JustPassedNode: %d, lineJustPassedNodeDetected);

    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  } else {
    lfm_speed[LEFT]=0;
    lfm_speed[RIGHT]=0;
  }
}

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
    
  // find out if robot is detects a new node
  int nodeDetected = detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
  // logger
  // printf("\Node: %d, nodeDetected);
  
  // find out if the robot has just passed the node
  int lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    
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
    
  // find out if robot is detects a new node
  int nodeDetected = detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
  // logger
  // printf("\Node: %d, nodeDetected);
  
  // find out if the robot has just passed the node
  int lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
    
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

void navigate(int numberOfNodes,int direction){

  while(nodeCount<=numberOfNodes){
    // run one simulation step
    wb_robot_step(TIME_STEP);
    // read sensors value
    for(i=0;i<NB_GROUND_SENS;i++) gs_value[i] = wb_distance_sensor_get_value(gs[i]);
  
    if(direction == 2){
      driveStraight(numberOfNodes);
    } else if(direction == 0){
      if((turnInit==0)||(turningLock==1)){
        leftTurn();
      } else if((turnInit==1)||(turningLock==0)){
        driveStraight(numberOfNodes);
      }
    } else if(direction == 1){
      if((turnInit==0)||(turningLock==1)){
        rightTurn();
      } else if((turnInit==1)||(turningLock==0)){
        driveStraight(numberOfNodes);
      }
    }
    wb_differential_wheels_set_speed(lfm_speed[LEFT],lfm_speed[RIGHT]);
  }
  printf("/nFINISHED: %d nodes, %d direction",numberOfNodes,direction);
  turnInit=0;
  nodeCount=0;
  wb_differential_wheels_set_speed(0,0);
}


// this function adjust the driving logic
void drive()
{
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
  int nodeDetected = detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
  // logger
  // printf("\Node: %d, nodeDetected);

  // find out if the robot has just passed the node
  int lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
  // logger
  // printf("\JustPassedNode: %d, lineJustPassedNodeDetected);

  // hardcode turn direction (temporary)
  int turnDirection = 0;
  
  // if the robot has just entered a node and the robot is not currently turning
  if((nodeDetected == 1) && (turningLock == 0)){
    // drive straight
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED;
  // if the robot just passed a node or the the robot is currently turning
  }else if((lineJustPassedNodeDetected == 1) || (turningLock == 1)){
     // turn the robot
     turn(turnDirection,LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
  // if the robot is not turning
  }else if(turningLock == 0){
    // stay on the line (Braitenberg-like behavior)
    int DeltaS=0;
    DeltaS = RightSensorReading-LeftSensorReading;
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
  }
  
  
  
  
}


//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
int main()
{

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];

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
      } else if (Mode == REALITY) {
        wb_differential_wheels_set_speed(0,0); wb_robot_step(TIME_STEP); // Just run one step to make sure we get correct sensor values
        printf("\n\n\nSwitching to REALITY and reseting all BB variables.\n\n");
      }
    }

    // Reset all BB variables when switching from simulation to real robot and back
    if (Mode!=wb_robot_get_mode())
    {
      Mode = wb_robot_get_mode();
      if (Mode == SIMULATION) printf("\nSwitching to SIMULATION and reseting all BB variables.\n\n");
      else if (Mode == REALITY) printf("\nSwitching to REALITY and reseting all BB variables.\n\n");
    }

    speed[LEFT] = 0;
    speed[RIGHT] = 0;
    // Navigation logic
    navigate(2,2);
    navigate(3,0);
    navigate(5,1);
    //drive();
  return 0;
}
