#ifndef _fixed_wing_h
#define _fixed_wing_h


#include "pwm.h"
#include "pid.h"
#include "nav.h"


#define FIXW_DEGREES_TO_PW	(2000 - 1000)		// degrees multiplier to obtain PWM pulse width before scaling
//#define USE_FLAPS // uncomment when using flaps
//#define USE_FIXEDWING_WAYPOINT

typedef struct {

    float throtCommand;
    float pitchCommand;
    float rollCommand;
    float ruddCommand;
    float test;
    int flapsActivated;
    float fixedWingSemiAutomode;
    float fixedWingWayPointMode;
    float desiredHeight;
    float desiredHeading;
    float diff;
    float pressureHeight;
    float heightGPS;
    float heightOffset;
    float heightOffsetGPS;
    float rollSetP;
    float rollPwm;
    float pitchPwm;
    float distanceToTarget;
    unsigned long timeMicros;
    unsigned long timeSpent;
    unsigned long differenceTime;
    int counter;

    float setPoints[14];

    pwmPortStruct_t *pitchPort;
    pwmPortStruct_t *rollPort;
    pwmPortStruct_t *ruddPort;
    pwmPortStruct_t *flapPort;

    float d1;
    float d2;

    pidStruct_t *altPosPID;
    pidStruct_t *rollPosPID;
    pidStruct_t *headingPosPID;

    // for navigation
    uint8_t legNum;
    navMission_t *curLeg; 

} fixedWingStruct_t;

extern fixedWingStruct_t fixedWingData;

extern void activateFlaps();
extern void deactivateFlaps();
extern void fixedWingInit(void);
extern void fixedWingUpdate(void);
extern void changeFixPoint();
extern void activateFixedWingWayPointMode();
extern void deactivateFixedWingWayPointMode();
extern void updateTagetHeadingFixedWing();
extern int areThereYet(); // 
extern void motorCommandsFixedWing(float throtCommand, float pitchCommand, float rollCommand, float ruddCommand);


#endif