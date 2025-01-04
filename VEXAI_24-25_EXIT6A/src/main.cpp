/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Rian Thomas                                               */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
// Robot configuration code.
motor leftDrive = motor(PORT3, ratio18_1, true);
motor rightDrive = motor(PORT4, ratio18_1, false);

gps GPS = gps(PORT12, -127, -165, distanceUnits::mm, 180);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 1);
// Controls arm used for raising and lowering rings
motor Arm = motor(PORT10, ratio18_1, false);
// Controls the chain at the front of the arm
// used for pushing rings off of the arm
motor Chain = motor(PORT8, ratio18_1, false);
static AI_RECORD  local_map;


// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT1, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT1, "robot_32456_1", linkType::worker );
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Isolation(void) {
  int32_t loop_time = 33;

  thread t1(dashboardTask);

  this_thread::sleep_for(loop_time);
  
  bool reverseable = false;
  bool catchr = false;
  bool rings = false;
  int count = 0;
  while (true){
    if (jetson_comms.get_packets() > 0){
      if (local_map.detectionCount > 0 && !reverseable){ //Add a for loop here *********** vvvvvvvvv
        for (int i = 0; i < local_map.detectionCount; i++){
          if (local_map.detections[i].classID == 0 && !reverseable){ //Checks if it is a mobile goal
                Drivetrain.driveFor(reverse, (distanceTo(local_map.detections[i].mapLocation.x * 1000, local_map.detections[i].mapLocation.y * 1000)) - 200, mm);
                reverseable = true;
          }
        }
      }
      else if (!reverseable && !rings){
        Drivetrain.setTurnVelocity(10, percent);
        Drivetrain.turnFor(left, 7, degrees, false);
      }
      else if (reverseable && !rings){
        Drivetrain.turnFor(right, 1080, degrees, false);
        rings = true;
      }


      //When it has a mobile goal latched onto it
      if (rings){
        if (local_map.detectionCount > 0){
          for (int i = 0; i < local_map.detectionCount; i++){ //Add a for loop here *********** vvvvvvvvv
            if (local_map.detections[i].classID == 1 && !catchr){ //Checks if it is a mobile goal
              if (local_map.detections[i].mapLocation.y >= 0.2){ //Goes toward the goal
                Drivetrain.driveFor(reverse, local_map.detections[i].mapLocation.y * 1000, mm);
                catchr = true;
              }
              else{
                Drivetrain.turnFor(left, 7, degrees, false);
              }
            }
          }
        }
      }

      //Vex link to send taken data
    }
    
    jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
    link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

      // request new data    
      // NOTE: This request should only happen in a single task.    
    jetson_comms.request_map();

      // Allow other tasks to run
    this_thread::sleep_for(loop_time);
  }
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Add functions for interaction phase
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}

int main() {
  void vexcodeInit(void);

  auto_Isolation();
}
