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
WbDeviceTag gs[NB_GROUND_SENS]; /* ground sensors */
unsigned short gs_value[NB_GROUND_SENS]={0,0,0};

// LEDs
#define NB_LEDS    8
WbDeviceTag led[NB_LEDS];

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// LFM - Line Following Module
//
// This module implements a very simple, Braitenberg-like behavior in order
// to follow a black line on the ground. Output speeds are stored in
// lfm_speed[LEFT] and lfm_speed[RIGHT].

int lfm_speed[2];

#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.1


/* Threasholds are used as cutoff values for detecting
   the track and the board. -RonPicard
*/
#define Board_Min_Threshold 700
#define Track_Max_Threshold 500

/*  Keep track of # of nodes detected -RonPicard
*/
int nodeCount = 0;
int stillOnCurrentNode = 0;
int turningLock = 0;
int turningTimer = 0;

/* This function tells whether a censor is reading a line, a board, or neither -RonPicard
*/
int isLineSensed(int GroundSensorValue){
  // If the sensor detects a line
  if(GroundSensorValue<=Track_Max_Threshold){
    // on line
    return 1;
  // If the sensor does not detect line
  } else if(GroundSensorValue>=Board_Min_Threshold){
    // not on line
    return 0;
  } else {
    // unknown
    return -1;
  }
}

/* This function tests whether the robot is at on a node, line, or unknown -RonPicard
*/
int isNode(int left, int center, int right){
  // If sensor all sensors a line
  if((left == 1) && (center == 1) && (right == 1)){
    // on node
    return 1;
  } else {
    // not on node
    return 0;
  }
}


// Detected when you pass a node -RonPicard
int detectNode(int left, int center, int right){
  // If the logic does not think it is on a node
  if (stillOnCurrentNode == 0){
    // Find out if the robot is still on a node
    int node = isNode(left, center, right);
    // If the robot is on a node
    if(node == 1){
      // Add node to count
      nodeCount = nodeCount + 1;
      // Change to 1 since the robot is on the node
      stillOnCurrentNode = 1;
      printf("\nNode # %d Found\n",nodeCount);
      return 1;
    }
  }
  return 0;
}

// Detect when the rebot has just passed a node
int detectLineJustPassedNode(int left, int center, int right){
 // If the logic still thinks it's on current node
 if(stillOnCurrentNode == 1){
    // Find out if the robot is still on a node
    int node = isNode(left, center, right);
    // If the robot is not still on a node
    if(node == 0){
      printf("\nBack On Line\n");
      // Change to 0 since the robot is not on the node
      stillOnCurrentNode = 0;
      return 1;
    }
  }
  return 0;
};

// This function contain the turn logic -RonPicard
void turn(int turnDirection, int LeftSensorOnTrack, int CenterSensorOnTrack, int RightSensorOnTrack){
    // If not locked for turning
    if(turningLock==0){
      // Lock for turning
      turningLock = 1;
      // Set intial turn timer to 25
      turningTimer = 25;
      // If this is a left turn
      if(turnDirection==0){
        // Hard coded turning contants
        lfm_speed[LEFT]  = -40;
        lfm_speed[RIGHT] = 180;
        printf("\nLEFT TURN START\n");
      // If this is right turn
      } else if(turnDirection==1){
        // Hard coded turning contants
        lfm_speed[LEFT]  = 180;
        lfm_speed[RIGHT] = -40;
        printf("\nRIGHT TURN START\n");
      }
    // If turn is locked
    } else if(turningLock==1){
      // If initial turn timer is not 0
      if(turningTimer !=0){
        // decrement timer
        turningTimer = turningTimer - 1;
        //printf("\nTIMER: %d ",timer);
      }
      // If timer is finished
      if(turningTimer == 0){
        // For Left turn - If right and center sensor detect the line
        if((turnDirection == 0) && (LeftSensorOnTrack==0) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 1)){
          // Unlock turn
          turningLock = 0;
          printf("\nLEFT TURN FINISHED\n");
        // For Right turn - If left and center sensor detect the line
        } else if((turnDirection == 1) && (LeftSensorOnTrack==0) && (CenterSensorOnTrack==1) && (RightSensorOnTrack == 1)){
          turningLock = 0;
          printf("\nRIGHT TURN FINISHED\n");
        }
       }
    }
}

/* This function adjust the driving logic -RonPicard
*/
void navigate(void)
{
  // Get sensor values
  int LeftSensorReading = gs_value[GS_LEFT];
  int CenterSensorReading = gs_value[GS_CENTER];
  int RightSensorReading = gs_value[GS_RIGHT];
  
  //Log Sensor values -RonPicard
  //printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorReading,CenterSensorReading,RightSensorReading);
  
  // Find out if sensors detect the track
  int LeftSensorOnTrack = isLineSensed(LeftSensorReading);
  int CenterSensorOnTrack = isLineSensed(CenterSensorReading);
  int RightSensorOnTrack = isLineSensed(RightSensorReading);
  
  // 1 if on line, 0 if not on line, -1 if unknown. -RonPicard
  // printf("\nLEFT: %d, Center: %d, Right %d",LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
  
  // Find out if the robot is on a node
  int nodeDetected = detectNode(LeftSensorOnTrack, CenterSensorOnTrack, RightSensorOnTrack);
  
  // Find out if the robot just passed a node
  int lineJustPassedNodeDetected = detectLineJustPassedNode(LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
  
  // Hardcode turn direction (temporary)
  int turnDirection = 1;
  
  // If node is detected and the robot is not turnning
  if((nodeDetected == 1) && (turningLock == 0)){
    // Continue straight
    lfm_speed[LEFT]  = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED;
  // If you the robot just passed a not
  } else if((lineJustPassedNodeDetected == 1) || (turningLock ==1)){
       // Turn the robot
       turn(turnDirection,LeftSensorOnTrack,CenterSensorOnTrack,RightSensorOnTrack);
  // If the robot is not locked for turning
  }else if(turningLock == 0){
    // Stay on the line
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
  int i, speed[2], Mode=1;

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i],TIME_STEP);
  }

  for(;;)   // Main loop
  {
    // Run one simulation step
    wb_robot_step(TIME_STEP);

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

    // read sensors value
    for(i=0;i<NB_GROUND_SENS;i++) gs_value[i] = wb_distance_sensor_get_value(gs[i]);

    // Reset all BB variables when switching from simulation to real robot and back
    if (Mode!=wb_robot_get_mode())
    {
      Mode = wb_robot_get_mode();
      if (Mode == SIMULATION) printf("\nSwitching to SIMULATION and reseting all BB variables.\n\n");
      else if (Mode == REALITY) printf("\nSwitching to REALITY and reseting all BB variables.\n\n");
    }

    // Speed initialization
    speed[LEFT] = 0;
    speed[RIGHT] = 0;

    // Navigation logic
    navigate();

    // Speed computation
    speed[LEFT]  = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];


    // Set wheel speeds
    //wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);

  }
  return 0;
}
