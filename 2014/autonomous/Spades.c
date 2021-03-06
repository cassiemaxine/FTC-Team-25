#pragma config(StandardModel, "teddy")
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Autonomous Mode Code Template
//
// This file contains a template for simplified creation of an autonomous program for an TETRIX robot
// competition.
//
// You need to customize two functions with code unique to your specific robot.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "../library/auto_defs.h"
#include "../library/sensors/drivers/hitechnic-sensormux.h"
#include "../library/sensors/drivers/hitechnic-irseeker-v2.h"
#include "../library/sensors/drivers/hitechnic-compass.h"
#include "../library/sensors/drivers/hitechnic-protoboard.h"
#include "../library/DrivetrainSquare.c"
#include "../library/dead_reckon.h"

const tMUXSensor IRSeeker = msensor_S2_4;

#include "../library/light_strip.h"
#include "../library/rnrr_start.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of autonomous mode, you may want to perform some initialization on your robot.
// Things that might be performed during initialization include:
//   1. Move motors and servos to a preset position.
//   2. Some sensor types take a short while to reach stable values during which time it is best that
//      robot is not moving. For example, gyro sensor needs a few seconds to obtain the background
//      "bias" value.
//
// In many cases, you may not have to add any code to this function and it will remain "empty".
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                         Main Task
//
// The following is the main code for the autonomous robot operation. Customize as appropriate for
// your specific robot.
//
// The types of things you might do during the autonomous phase (for the 2008-9 FTC competition)
// are:
//
//   1. Have the robot follow a line on the game field until it reaches one of the puck storage
//      areas.
//   2. Load pucks into the robot from the storage bin.
//   3. Stop the robot and wait for autonomous phase to end.
//
// This simple template does nothing except play a periodic tone every few seconds.
//
// At the end of the autonomous period, the FMS will autonmatically abort (stop) execution of the program.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

int travelDistance[] = {17, 10, 19, 9};
int compensation[] = {-5, -4, -8, -5};

void dead_reckon_from_b0(void)
{
    init_path();

    add_segment(-56, 0, 100);
    add_segment(-40, -90, 100);
    add_segment(-38, -90, 100);
    add_segment(0, 0, 0);

    dead_reckon();
}

void dead_reckon_from_b1(void)
{
    init_path();

    add_segment(-46, 0, 100);
    add_segment(-40, -90, 100);
    add_segment(-38, -90, 100);
    add_segment(0, 0, 0);

    dead_reckon();
}

void dead_reckon_from_b2(void)
{
    init_path();

    add_segment(-30, 0, 100);
    add_segment(-40, -90, 100);
    add_segment(-38, -90, 100);
    add_segment(0, 0, 0);

    dead_reckon();
}

void dead_reckon_from_b3(void)
{
    init_path();

    add_segment(-15, 0, 100);
    add_segment(-42, -90, 100);
    add_segment(-38, -90, 100);
    add_segment(0, 0, 0);

    dead_reckon();
}

void initializeRobot()
{
    tHTIRS2DSPMode mode = DSP_1200;
    //HTIRS2setDSPMode(IRSeeker, mode);

    servoChangeRate[blockDump] = 5;

	servo[blockDump] = BLOCK_SERVO_RETRACTED;

    lightStripInit(0x1F);
    displayRestingPulse();
}

void moveToRamp(int basket)
{
    switch (basket) {
    case 0:
        dead_reckon_from_b0();
        break;
    case 1:
        dead_reckon_from_b1();
        break;
    case 2:
        dead_reckon_from_b2();
       break;
    case 3:
        dead_reckon_from_b3();
        break;
    }
}

void dumpBlock()
{
    servoChangeRate[blockDump] = 10;
    servo[blockDump] = BLOCK_SERVO_EXTENDED;
    wait1Msec(1500);

    servoChangeRate[blockDump] = 10;

    for (int i = 0; i < 5; i++) {
        servo[blockDump] = BLOCK_SERVO_EXTENDED - 20;
        wait1Msec(100);
        servo[blockDump] = BLOCK_SERVO_EXTENDED;
    }

    servoChangeRate[blockDump] = 5;
    servo[blockDump] = BLOCK_SERVO_RETRACTED;
}

void showIRSegment(int dir)
{
    switch (dir) {

    case 5:
       flashGreen();
       break;
    default:
       wipeRed();
       break;
    }
}

int findIRBeacon()
{
    int dir;
    int i;

    for (i = 0; i < 4; i++) {
	    move(travelDistance[i], DIR_BACKWARD, 100);
        dir = HTIRS2readACDir(IRSeeker);
        showIRSegment(dir);
        wait1Msec(500);
        nxtDisplayTextLine(2, "Reading %d", dir);

	    if (dir == 5) {
	        return i;
	    }
    }

    return -1;
}

void rangeFind()
{
}

int findIRBeacon_v2()
{
    int acS1, acS2, acS3, acS4, acS5, _dirAC = 0;
    int max = 0;
    bool done;
    int dir;

    tHTIRS2DSPMode _mode = DSP_1200;
    //HTIRS2setDSPMode(IRSeeker, _mode);

    move(0, BACKWARD, 50);

    while (!done) {
	    HTIRS2readAllACStrength(IRSeeker, acS1, acS2, acS3, acS4, acS5);
	    dir = HTIRS2readACDir(IRSeeker);
	    if (dir == 5) {
            rangeFind();
        }
    }

    return 0;
}


task main()
{
    int basketNumber;
    bool done = false;

    initializeRobot();

    RNRR_waitForStart(); // Wait for the beginning of autonomous phase.

    disableDiagnosticsDisplay();
    eraseDisplay();

    basketNumber = findIRBeacon();
    nxtDisplayTextLine(5, "Beacon #%d", basketNumber);

    wait1Msec(300);

    if (basketNumber != -1) {
        if (compensation[basketNumber] < 0) {
            move(abs(compensation[basketNumber]), FORWARD, 60);
        } else if (compensation[basketNumber] > 0) {
            move(compensation[basketNumber], BACKWARD, 60);
        }
        /*for (int i = 0; i <= basketNumber; i++) {
            displayCaution();
            wait1Msec(500);
            displayBackward();
            wait1Msec(500);
        }
        displayRestingPulse();
        */
		dumpBlock();
    } else {
        basketNumber = 3;
    }

    moveToRamp(basketNumber);

    while (true)
    {}
}
