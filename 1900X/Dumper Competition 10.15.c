#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    catapultPot,    sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  platformLock,   sensorDigitalOut)
#pragma config(Sensor, dgtl2,  drumPosEnc,     sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  platformDown,   sensorTouch)
#pragma config(Sensor, dgtl5,  platformUp,     sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           lDriveFront,   tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,           lDriveBack,    tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           rDump23,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           rDump1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           lDump1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           lDump23,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           rDriveBack,    tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rDriveFront,   tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//platformLock - 0 = out, 1 = in


// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
//ENABLE THIS LATER
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "..\Vex_Competition_Includes.c"
#include "LCD Selection Wizard.c"
void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	bDisplayCompetitionStatusOnLcd = false;
	bLCDBacklight = true;

	startTask(lcdSelection);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

	//gyro calibration
	//wait1Msec(2000);
}

const int UP = 1;
const int DOWN = 0;


//fundamental movement functions
void setDumperMotors (float power) {
	motor[lDump1] = power;
	motor[lDump23] = power;
	motor[rDump1] = power;
	motor[rDump23] = power;
}

void setLeftDtMotors(float lFront, float lBack) {
	motor[lDriveFront] = lFront;
	motor[lDriveBack] = lBack;
}

void setRightDtMotors(float rFront, float rBack) {
	motor[rDriveFront] = rFront;
	motor[rDriveBack] = rBack;
}


void toggleSolenoid(int sensor) {
	int currVal = SensorValue[sensor],
			newVal = abs(currVal - 1); //If currVal is 1, newVal is 0.  If currVal is 0, newVal is abs(-1) = 1

	SensorValue[sensor] = newVal;
}

//TODO: macro functions
void movePlatformDown() {
	//TODO: move platform down until platformDown limit switch pressed.  Then extend platform lock.  Then turn off motors
}

void movePlatformUp() {
	//TODO: move platform all the way up (to dump) until platformUp limit switch presed.	If platformDown is pressed at start, release the platform lock.
	//          At top, set the motors to their resting power
}


task driverDumper() {
	bool btn5UPressed = false;
	while(1) {
		//this is set up such that movement functions will suspend this task while they execute, thus disabling other catapult controls in that time
		if (vexRT[Btn8U]) {
			//prepareCatapult();
		} else if (vexRT[Btn5D] && vexRT[Btn6U] == 0) {
			//fireCatapult();
		} else if (vexRT[Btn7D]) {
			//setCatapultPosition(0);
		} else if (vexRT[Btn7L]) {
		  //setCatapultPosition(1);
		} else if (vexRT[Btn7U]) {
			//setCatapultPosition(2);
		} else if (vexRT[Btn7R]) {
			//setCatapultPosition(3);
		} else if (vexRT[Btn6D] && vexRT[Btn6U] == 0) {
			//prepareCatapult();
		}/*	else if (vexRT[Btn8L]) {
			setDrumMotors(-127);
		} else if (vexRT[Btn8R]) {
			setDrumMotors(127);
		} else {
			setDrumMotors(0);
		}*/

		//platform lock
		/*if (vexRT[Btn5U]) {
			SensorValue[platformLock] = 0;
		} else if (vexRT[Btn5D]) {
			SensorValue[platformLock] = 1;
		}*/

		//6U/6D manual drum control
		//8D done. toggle hang lock
		//8U toggle tongue
		//5U ratchet
		//8U held platform lock



		//FOR PLATFORM LOCK
		if (vexRT[Btn5U] && !btn5UPressed) {
			toggleSolenoid(platformLock);
			btn5UPressed = true;
		} else if (!vexRT[Btn5U] && btn5UPressed) {
			btn5UPressed = false;
		}

		wait1Msec(25);
	}
}

#include "Clyde Auton 10.15.c"


//shouldn't need to be changed
task autonomous()
{
	catapultInit(); //make sure we can use the catapult	in any autonomous play

	if (autonChoices.waitTime > 0) {
		wait1Msec(autonChoices.waitTime*1000);
	}

	if (autonChoices.startingTile == "left") {
			startTask(autonFence);
	} else if (autonChoices.startingTile == "right") {
		startTask(autonFence);
	} else if (autonChoices.startingTile == "programmingSkills") {
		startTask(autonSkills);
	}


	//autonomous play selection goes here
	//startTask(autonFence);
	//startTask(autonSkills);
	//setCatapultPosition(2);
	//fireCatapult();
	//resetDrumPosition();

}

const bool DEBUG_AUTON = false;

task usercontrol()
{
	catapultInit(); //make sure catapult can be controlled

	if (DEBUG_AUTON) {
		//to select the play
		autonChoices.waitTime = 0; //time in seconds, so 0,1,3 are values supported by LCD selection wizard
		autonChoices.startingTile = "left";
		startTask(autonomous);
		stopTask(usercontrol);
	}
	startTask(driverCatapult);

	//resetDrumPosition();
	bLCDBacklight = true;
	clearLCDLine(0);
	int LY = 0;
	int LX = 0;
	int RY = 0;
	int RX = 0;
	int threshold = 15;
	while (1) {
		//8L/8R circle strafe

		//TODO: probably don't need circle strafing anymore
		if (vexRT[Btn8L]) {
			setLeftDtMotors(-55,127);
			setRightDtMotors(55,-127);
		} else if (vexRT[Btn8R]) {
			setLeftDtMotors(55,-127);
			setRightDtMotors(-55,127);
		} else {
			//for deadzones; when the joystick value for an axis is below the threshold, the motors controlled by that joystick will be set to 0
			LY = (abs(vexRT[Ch3]) > threshold) ? vexRT[Ch3] : 0;
			LX = (abs(vexRT[Ch4]) > threshold) ? vexRT[Ch4] : 0;
			RY = (abs(vexRT[Ch2]) > threshold) ? vexRT[Ch2] : 0;
			RX = (abs(vexRT[Ch1]) > threshold) ? vexRT[Ch1] : 0;
			motor[lDriveFront] = LY + LX;
			motor[lDriveBack] = LY - LX;
			motor[rDriveFront] = RY - RX;
			motor[rDriveBack] = RY + RX;
			wait1Msec(20);

			if (vexRT[Btn6U]) {
				while (vexRT[Btn6U]) {
					if(vexRT[Btn6D] == 1)
					{
						SensorValue[drumPosEnc] = 0;
					}
					if(vexRT[Btn5D] == 1)
					{
						drumResetForward();
					}
					else if(vexRT[Btn5U] == 1)
					{
						drumResetBackward();
					}
				}
			}
		}
	}
}
