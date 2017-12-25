/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "aq.h"
#include "fixed_wing.h"
#include "imu.h"
#include "nav_ukf.h"
#include "radio.h"
#include "util.h"
#include "config.h"
#include "aq_timer.h"
#include "nav.h"
#include "comm.h"
#include "gps.h"
#include "supervisor.h"
#include "motors.h"
#include "run.h"
#include "alt_ukf.h"
#include <string.h>
#include <CoOS.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#ifndef __CC_ARM
#include <intrinsics.h>
#endif


fixedWingStruct_t fixedWingData __attribute__((section(".ccm")));

void fixedWingInit(void) {

    
    AQ_NOTICE("Fixed wing init\n");

    memset((void *)&fixedWingData, 0, sizeof(fixedWingData));

    int8_t chkTim, initPitchPort, initRollPort, initRuddPort, initFlapPort;
    fixedWingData.flapsActivated = 0; // initialize flaps to be deactivated
    fixedWingData.fixedWingSemiAutomode = 0; // initialize semi automatic mode to be deactivated
    fixedWingData.fixedWingWayPointMode = 0; 
    fixedWingData.timeSpent = 0;
    fixedWingData.counter = 0;
    fixedWingData.legNum = 1; // start with first waypoint in missions legs
     fixedWingData.distanceToTarget = 10000.0f;
    //p[FIXW_ALT_OFFS] = 1500;

    // values for PID controller altitude
    p[FIXW_ALT_POS_P] = 30.0f;
    p[FIXW_ALT_POS_I] = 3.0f;
    p[FIXW_ALT_POS_D] = 12.0f;
    p[FIXW_ALT_POS_PM] = 40.0f;
    p[FIXW_ALT_POS_IM] = 60.0f;
    p[FIXW_ALT_POS_DM] = 40.0;
    p[FIXW_ALT_POS_OM] = 130.0f;

    // values for PID controller roll
    p[FIXW_ROLL_POS_P] =  8.0f; //2.0f;
    p[FIXW_ROLL_POS_I] = 0.01f; //1.85f;
    p[FIXW_ROLL_POS_D] = 0.0f;
    p[FIXW_ROLL_POS_PM] = 400.0f; //1000.0f;
    p[FIXW_ROLL_POS_IM] = 15.0f; //700.0f;
    p[FIXW_ROLL_POS_DM] = 0.0f;
    p[FIXW_ROLL_POS_OM] = 500.0f;

    p[FIXW_ROLL_SETP] = 0.0f;

    // values for PID controller heading
    p[FIXW_YAW_POS_P] =  2.5f; //2.0f;
    p[FIXW_YAW_POS_I] = 0.01f; //1.85f;
    p[FIXW_YAW_POS_D] = 0.001f;
    p[FIXW_YAW_POS_PM] = 35.0f; //1000.0f;
    p[FIXW_YAW_POS_IM] = 15.0f; //700.0f;
    p[FIXW_YAW_POS_DM] = 2.0f;
    p[FIXW_YAW_POS_OM] = 35.0f;

 
    
    fixedWingData.d1 = 100.0f;
    fixedWingData.d2 = 99.0f;
#ifdef USE_FIXED_WING
    p[FIXW_PITCH_PORT] = 8;
    p[FIXW_ROLL_PORT] = 6;
    p[FIXW_RUDD_PORT] = 7;
    p[FIXW_FLAP_PORT] = 0;
    p[FIXW_PWM_FREQ] = 50;
#endif

    initializeWayPoints(); // football field

    if (!p[FIXW_PWM_FREQ]){
	AQ_NOTICE("Fixed wing functions disabled.\n");
	return;}

    chkTim = ((int)p[FIXW_PWM_FREQ] != MOTORS_PWM_FREQ);

    // check if any timers are conflicting
    initPitchPort = (p[FIXW_PITCH_PORT] && (!chkTim || !pwmCheckTimer(p[FIXW_PITCH_PORT]-1)));
    initRollPort = (p[FIXW_ROLL_PORT] && (!chkTim || !pwmCheckTimer(p[FIXW_ROLL_PORT]-1)));
    initRuddPort = (p[FIXW_RUDD_PORT] && (!chkTim || !pwmCheckTimer(p[FIXW_RUDD_PORT]-1)));
    initFlapPort = (p[FIXW_FLAP_PORT] && (!chkTim || !pwmCheckTimer(p[FIXW_FLAP_PORT]-1)));

    if (initPitchPort){
        fixedWingData.pitchPort = pwmInitOut(p[FIXW_PITCH_PORT]-1, 1000000, (int)p[FIXW_PWM_FREQ], p[FIXW_NTRL_PITCH], -1);
	if (fixedWingData.pitchPort) {
            AQ_NOTICE("Fixed wing PITCH control port initialized.\n");
	    yield(100);
	}
    }

    if (initRollPort){
	fixedWingData.rollPort = pwmInitOut(p[FIXW_ROLL_PORT]-1, 1000000, (int)p[FIXW_PWM_FREQ], p[FIXW_NTRL_ROLL], -1);
	if (fixedWingData.rollPort)
	    AQ_NOTICE("Fixed wing ROLL control port initialized.\n");
    }

    if (initRuddPort){
	fixedWingData.ruddPort = pwmInitOut(p[FIXW_RUDD_PORT]-1, 1000000, (int)p[FIXW_PWM_FREQ], p[FIXW_NTRL_PITCH], -1);
	if (fixedWingData.ruddPort)
	    AQ_NOTICE("Fixed wing RUDDER control port initialized.\n");
    }
    
    // initialize PID controller for altitude
    fixedWingData.altPosPID = pidInit(&p[FIXW_ALT_POS_P], &p[FIXW_ALT_POS_I], &p[FIXW_ALT_POS_D], 0, &p[FIXW_ALT_POS_PM], &p[FIXW_ALT_POS_IM], &p[FIXW_ALT_POS_DM], &p[FIXW_ALT_POS_OM], 0, 0, 0, 0);
    // initialize PID controller for roll
    fixedWingData.rollPosPID = pidInit(&p[FIXW_ROLL_POS_P], &p[FIXW_ROLL_POS_I], &p[FIXW_ROLL_POS_D], 0, &p[FIXW_ROLL_POS_PM], &p[FIXW_ROLL_POS_IM], &p[FIXW_ROLL_POS_DM], &p[FIXW_ROLL_POS_OM], 0, 0, 0, 0);
    // initialize PID controller for yaw/heading
    fixedWingData.headingPosPID = pidInit(&p[FIXW_YAW_POS_P], &p[FIXW_YAW_POS_I], &p[FIXW_YAW_POS_D], 0, &p[FIXW_YAW_POS_PM], &p[FIXW_YAW_POS_IM], &p[FIXW_YAW_POS_DM], &p[FIXW_YAW_POS_OM], 0, 0, 0, 0);
    
#ifdef USE_FLAPS
    if (initFlapPort){
	fixedWingData.flapPort = pwmInitOut(p[FIXW_FLAP_PORT]-1, 1000000, (int)p[FIXW_PWM_FREQ], p[FIXW_NTRL_PITCH], -1);
	if (fixedWingData.flapPort){
	    AQ_NOTICE("Fixed wing FLAPS control port initialized.\n");
            *fixedWingData.flapPort->ccr = 1500; // set neutral position for flaps
        }
    }
#endif

    fixedWingData.timeSpent = timerMicros(); // initialize reference for timer
    
    
}

void fixedWingUpdate(void)
{
    uint16_t pwm;

    // NEED cleaning up - change hardcoded values to use config 
    if (timerMicros() < 5e6f)
	return;

#ifdef USE_PRES_ALT // If barometer is used, then altitude is retrieved from RunData
        fixedWingData.pressureHeight = ALT_POS;//;+fixedWingData.heightOffset;
        // PITCH passtrough (use autopilot if enabled otherwise use manual control)
        if (p[FIXW_PITCH_PORT]){
            if(fixedWingData.fixedWingSemiAutomode && (RADIO_PITCH < 300) && (RADIO_PITCH > -300)){
                fixedWingData.pitchPwm = (-(pidUpdate(fixedWingData.altPosPID,fixedWingData.desiredHeight,fixedWingData.pressureHeight))+((float)RADIO_PITCH+1500.0f)); // calculate and offset pwm
                pwm = (uint16_t)fixedWingData.pitchPwm;
                *fixedWingData.pitchPort->ccr = pwm;
            }
            else{
                  pwm = constrainInt(((RADIO_PITCH+1500)),750,2250);
                  *fixedWingData.pitchPort->ccr = pwm;
                  fixedWingData.pitchPwm = (float)pwm;
            }
        }
#else // Use GPS DATA for altitude
        fixedWingData.heightGPS = gpsData.height-(fixedWingData.heightOffsetGPS);
        // PITCH passtrough (use autopilot if enabled otherwise use manual control)
        if (p[FIXW_PITCH_PORT]){
            if(fixedWingData.fixedWingSemiAutomode && (RADIO_PITCH < 300) && (RADIO_PITCH > -300)){
                fixedWingData.pitchPwm =(pidUpdate(fixedWingData.altPosPID,fixedWingData.desiredHeight,fixedWingData.heightGPS)+((float)RADIO_PITCH+1500.0f)); // calculate and offset pwm
                pwm = (uint16_t)fixedWingData.pitchPwm;
                *fixedWingData.pitchPort->ccr = pwm;
            }
            else{
                  pwm = constrainInt(((RADIO_PITCH+1500)),750,2250);
                  *fixedWingData.pitchPort->ccr = pwm;
                  fixedWingData.pitchPwm = (float)pwm;
            }
        }
#endif

        // ROLL passthough (use autopilot if enabled otherwise use manual control)
        if (p[FIXW_ROLL_PORT]){
            if(fixedWingData.fixedWingSemiAutomode && (RADIO_ROLL < 300) && (RADIO_ROLL > -300)){
               //pwm = (uint16_t)(pidUpdate(fixedWingData.rollPosPID,fixedWingData.setPoints[fixedWingData.counter],AQ_ROLL)+1500.0f); // calculate and offset pwm
               fixedWingData.rollPwm =(pidUpdate(fixedWingData.rollPosPID,p[FIXW_ROLL_SETP],AQ_ROLL)+((float)RADIO_ROLL+1500.0f)); // calculate and offset pwm
               pwm = (uint16_t)fixedWingData.rollPwm;
                *fixedWingData.rollPort->ccr = pwm;
            }
            else{
                  pwm = constrainInt((RADIO_ROLL+1500),750,2250);
                  *fixedWingData.rollPort->ccr = pwm;
                  fixedWingData.rollPwm = (float)pwm;
            }
        }
         
         if(fixedWingData.fixedWingWayPointMode){ // only check for goals when waypoint mode is activated
            if(!areWeThereYet()){ // check if we reach way point or endgoal
                updateTargetHeadingFixedWing(); // update heading continously while we are not at goal
                fixedWingData.rollSetP = pidUpdate(fixedWingData.headingPosPID,fixedWingData.desiredHeading,gpsData.heading);
                //p[FIXW_ROLL_SETP] = fixedWingData.rollSetP; // update roll setpoint with calculated PID
                
            }
         }
        // YAW passthough
        if (p[FIXW_RUDD_PORT]){
             if(fixedWingData.fixedWingSemiAutomode){
              //fixedWingData.rollSetP = pidUpdate(fixedWingData.headingPosPID,fixedWingData.desiredHeading,gpsData.heading);
                
                if(fixedWingData.rollSetP > 5.0f || fixedWingData.rollSetP < -5.0f){
                  p[FIXW_ROLL_SETP] = fixedWingData.rollSetP; // update roll setpoint with calculated PID

                  }
                  else p[FIXW_ROLL_SETP] = 0.0f;
             // get current heading
             // current heading
             // get current setpoint
             // calculate difference
             // use PID controller to make bank turn. Roll setpoint should not exceed 45 degress in first test
             }
             else{

                pwm = constrainInt((RADIO_RUDD+1550),750,2250);
                *fixedWingData.ruddPort->ccr = pwm; 
              }
        }

#ifdef USE_FLAPS
         // FLAPS control
        if (p[FIXW_FLAP_PORT]){
              if(RADIO_AUX2 < -250){
                   if(!fixedWingData.flapsActivated)
                      activateFlaps();
              } 
              else if(RADIO_AUX2 > -250 &&  fixedWingData.flapsActivated)
                      deactivateFlaps();
        }
#endif


#ifdef USE_FIXEDWING_WAYPOINT
             // Wayppoint control - only hardcoded at the moment
            if(RADIO_AUX2 < -250){
                 if(!fixedWingData.fixedWingWayPointMode)
                      activateFixedWingWayPointMode();
            } 
            else if(RADIO_AUX2 > -250 && fixedWingData.fixedWingWayPointMode) 
                      deactivatefixedwingwaypointmode();
#else // else use only semi-mode
        // Semi-automode control
        if(RADIO_AUX2 > 250){
              if(!fixedWingData.fixedWingSemiAutomode)
                  activateSemiAutomaticmode();
        }
        else if(RADIO_AUX2 < 250 && fixedWingData.fixedWingSemiAutomode)
                  deactivateSemiAutomaticmode();
#endif           
        // for testing different setpoints<
        //fixedWingData.timeMicros = timerMicros();
        //fixedWingData.differenceTime = fixedWingData.timeMicros - fixedWingData.timeSpent;
        //if((fixedWingData.differenceTime > 7e6) && fixedWingData.fixedWingSemiAutomode)
          //changeFixPoint();

}

int areWeThereYet(){
   
    // check if leg is a wanted type, otherwise return
    //if(!fixedWingData.curLeg->type)
    //   return 1; // just accept being at goal if we meet an invalid type or not defined
    
    //calculate distance from current position to target position for current waypoint
    fixedWingData.distanceToTarget = navCalcDistance(gpsData.lat, gpsData.lon,fixedWingData.curLeg->targetLat, fixedWingData.curLeg->targetLon);
    
    if(fixedWingData.distanceToTarget < 20.0f) // if we are closer than X meters to the target, load a new leg
        fixedWingData.curLeg = &navData.missionLegs[fixedWingData.legNum++];
    
    // if we are not close enough, then update heading
}

void updateTargetHeadingFixedWing(){
     fixedWingData.desiredHeading = navCalcBearing(fixedWingData.curLeg->targetLat, fixedWingData.curLeg->targetLon, gpsData.lat, gpsData.lon); 
     fixedWingData.desiredHeading * RAD_TO_DEG; 
}

void activateFixedWingWayPointMode(){
      fixedWingData.curLeg = &navData.missionLegs[fixedWingData.legNum++];
      fixedWingData.fixedWingWayPointMode = 100.0f;
      activateSemiAutomaticmode(); // when activating waypoint mode, also use the semi-mode to hold target roll and altitude
}

void deactivatefixedwingwaypointmode(){
      fixedWingData.legNum = 1;
      fixedWingData.fixedWingWayPointMode = 0;
      deactivateSemiAutomaticmode();
}

void activateFlaps(){
      *fixedWingData.flapPort->ccr = 2250; //pwm;  // set flap to maximum position when activated
      fixedWingData.flapsActivated = 1;
      AQ_NOTICE("Flaps activated.\n");
}

void deactivateFlaps(){
      *fixedWingData.flapPort->ccr = 1500; // need to be neutral position when flaps is not activated
      fixedWingData.flapsActivated = 0;
      AQ_NOTICE("Flaps deactivated\n");
}

void activateSemiAutomaticmode(){
    fixedWingData.fixedWingSemiAutomode = 100.0f; // value 100 only used to improve the visual in QGC
     
#ifdef USE_PRES_ALT
      fixedWingData.desiredHeight = fixedWingData.pressureHeight; // save current height to stay in the same height.
#else
      fixedWingData.desiredHeight = fixedWingData.heightGPS; 
#endif
      fixedWingData.desiredHeading = gpsData.heading;
     AQ_NOTICE("Fixed wing semi automatic mode activated\n");
}

void deactivateSemiAutomaticmode(){
     fixedWingData.fixedWingSemiAutomode = 0.0f;
     AQ_NOTICE("Fixed wing semi automatic mode deactivated.\n");
     //debug_printf("Fixed wing semi automatic mode deactivated %x\n", 1);
}

void changeFixPoint(){ //this routine is for flight mode with changing setpoints in air
    fixedWingData.counter++;
    if(fixedWingData.counter == 14)
      fixedWingData.counter = 0;
    fixedWingData.timeSpent = timerMicros(); // set new reference
}

void motorCommandsFixedWing(float throtCommand, float pitchCommand, float rollCommand, float ruddCommand) // NOT USED CURRENTLY
{
    fixedWingData.throtCommand = (throtCommand);
    fixedWingData.pitchCommand = (pitchCommand);
    fixedWingData.rollCommand = (rollCommand);
    fixedWingData.ruddCommand = (ruddCommand);
}

void initializeWayPoints(){

    navData.missionLegs[1].type = NAV_LEG_GOTO;
    navData.missionLegs[1].targetLat = 54.9298420343803429;
    navData.missionLegs[1].targetLon = 10.7036876678466797;
    navData.missionLegs[1].targetRadius = 20.0f;

    navData.missionLegs[2].type = NAV_LEG_GOTO;
    navData.missionLegs[2].targetLat = 54.9298420343803429;
    navData.missionLegs[2].targetLon = 10.7036876678466797;
    navData.missionLegs[2].targetRadius = 20.0f;

    navData.missionLegs[3].type = NAV_LEG_GOTO;
    navData.missionLegs[3].targetLat = 54.9290344683367948;
    navData.missionLegs[3].targetLon = 10.7061123847961426;
    navData.missionLegs[3].targetRadius = 20.0f;

    navData.missionLegs[4].type = NAV_LEG_GOTO;
    navData.missionLegs[4].targetLat = 54.9293550308442633;
    navData.missionLegs[4].targetLon = 10.7087409496307373;
  

    navData.missionLegs[5].type = NAV_LEG_GOTO;
    navData.missionLegs[5].targetLat = 54.9305509530395852;
    navData.missionLegs[5].targetLon = 10.7091593742370605;

    navData.missionLegs[6].type = NAV_LEG_GOTO;
    navData.missionLegs[6].targetLat = 54.9299468318386914;
    navData.missionLegs[6].targetLon = 10.7043099403381348;

 
}

void initializeWayPoints1(){

    navData.missionLegs[1].type = NAV_LEG_GOTO;
    navData.missionLegs[1].targetLat = 55.0579160170189894;
    navData.missionLegs[1].targetLon = 10.5723077058792114;
    navData.missionLegs[1].targetRadius = 10.0f;

    navData.missionLegs[2].type = NAV_LEG_GOTO;
    navData.missionLegs[2].targetLat = 55.0574029128502076;
    navData.missionLegs[2].targetLon = 10.5726134777069092;
    navData.missionLegs[2].targetRadius = 20.0f;
}


        // intialize test array setpoints
//    fixedWingData.setPoints[0] = 0.0f;
//    fixedWingData.setPoints[1] = 0.0f;
//    fixedWingData.setPoints[2] = 30.0f;
//    fixedWingData.setPoints[3] = 0.0f;
//    fixedWingData.setPoints[4] = -30.0f;
//    fixedWingData.setPoints[5] = 0.0f;
//    fixedWingData.setPoints[6] = 45.0f;
//    fixedWingData.setPoints[7] = 0.0f;
//    fixedWingData.setPoints[8] = -45.0f;
//    fixedWingData.setPoints[9] = 0.0f;
//    fixedWingData.setPoints[10] = 60.0f;
//    fixedWingData.setPoints[11] = 0.0f;
//    fixedWingData.setPoints[12] = -60.0f;
//    fixedWingData.setPoints[13] = 0.0f;

