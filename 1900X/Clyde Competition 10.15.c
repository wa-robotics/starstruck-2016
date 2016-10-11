#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    catapultPot,    sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  platformLock,   sensorDigitalOut)
#pragma config(Sensor, dgtl2,  drumPosEnc,     sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  platformDown,   sensorTouch)
#pragma config(Sensor, dgtl5,  drumZero,       sensorTouch)
#pragma config(Sensor, dgtl6,  drumRatchet,    sensorDigitalOut)
#pragma config(Sensor, dgtl7,  tongue,         sensorDigitalOut)
#pragma config(Sensor, dgtl8,  hangLock,       sensorDigitalOut)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           lDriveFront,   tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,           lDriveBack,    tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           rCatapult23,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           rCatapult1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           lCatapult1,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           lCatapult23,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           rDriveBack,    tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rDriveFront,   tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//platformLock - 0 = out, 1 = in
//tongue - 0 = in, 1 = out
//drumRatchet - 0 = up, 1 = down


// This code is for the VEX cortex platform
#pragma platform(VEX2)
bool DEBUG_ENABLE = true;

// Select Download method as "competition"
//ENABLE THIS LATER
//#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "..\Vex_Competition_Includes.c"

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

	//gyro calibration
	//wait1Msec(2000);
}

const int UP = 1;
const int DOWN = 0;


//fundamental movement functions
void setDrumMotors (float power) {
	motor[lCatapult1] = power;
	motor[lCatapult23] = power;
	motor[rCatapult1] = power;
	motor[rCatapult23] = power;
}

void setLeftDtMotors(float lFront, float lBack) {
	motor[lDriveFront] = lFront;
	motor[lDriveBack] = lBack;
}

void setRightDtMotors(float rFront, float rBack) {
	motor[rDriveFront] = rFront;
	motor[rDriveBack] = rBack;
}


//CATAPULT SPECIFIC
//if pos is 0(DOWN), the ratchet will be down
//if pos is 1 (UP), the ratchet will be released, which may require moving the catapult down briefly to release the ratchet
void setRatchetPos(int pos) {
	if (pos == UP) {
		setDrumMotors(127);
		wait1Msec(100); //spin the drum downwards to relaese the ratchet
		SensorValue[drumRatchet] = 0;
		wait1Msec(350); //this time should be slightly longer since this period is crucial to the ratchet releasing properly
		setDrumMotors(0);
	} else { //pos == 0
		SensorValue[drumRatchet] = 1;
	}
}

//full rev = 360*7
//automatically releases ratchet first
void moveCatapultDrumDist (int count, int direction) {
	int target;
	if (direction == DOWN) {
		setRatchetPos(DOWN);
		target = SensorValue[drumPosEnc] + count;
		writeDebugStreamLine("down target: %d",target);
		while(SensorValue[drumPosEnc] < target) {
			setDrumMotors(127);
			wait1Msec(25);
		}
	} else { //direction == UP
		//move the catapult down a little to release the ratchet
		setRatchetPos(UP);
		//move drum as requested
		target = SensorValue[drumPosEnc] - count;
		writeDebugStreamLine("up target %d",target);
		while(SensorValue[drumPosEnc] < target) {
			setDrumMotors(-127);
			wait1Msec(25);
		}
	}
	setDrumMotors(0);
}

void resetDrumPosition() {
	setRatchetPos(1);
	wait1Msec(400); //should probably reduce this time; one second to make sure the human has lifted the ratchet
	writeDebugStreamLine("resetDrumPosition called");
	while (!SensorValue[drumZero] || SensorValue[drumPosEnc] > 100 || vexRT[Btn6D]) {
		writeDebugStreamLine("Inside while loop: SensorValue[drumZero] = %d, SensorValue[drumPosEnc] = %d",SensorValue[drumZero],SensorValue[drumPosEnc]);
		setDrumMotors(-127);
	}
	writeDebugStreamLine("Exited while loop");
	setDrumMotors(0);
	writeDebugStreamLine("Set drum motors to 0");
	SensorValue[drumPosEnc] = 0;
}

int catapultPositions[4];



void catapultInit() {
	catapultPositions[0] = 0;
	catapultPositions[1] = 1500;
	catapultPositions[2] = 2600;
	catapultPositions[3] = 2780;
}

void setCatapultPosition(int pos) {
	if (pos == 0) {
		resetDrumPosition();
	} else {
		int currPos = SensorValue[drumPosEnc];
		int desiredPos = catapultPositions[pos];
		writeDebugStreamLine("Catapult desired position: %d",desiredPos);
		writeDebugStreamLine("Current enc val: %d",SensorValue[drumPosEnc]);
		int distReq = desiredPos - currPos;
		writeDebugStreamLine("Dist req: %d",distReq);
		if (distReq < 0) { // move UP
			moveCatapultDrumDist(abs(distReq), UP);
		} else if (distReq > 0) {
			moveCatapultDrumDist(abs(distReq),DOWN);
		}
	}
}

void prepareCatapult() {
	//platformLock - 0 = out, 1 = in
	//tongue - 0 = in, 1 = out
	//drumRatchet - 0 = up, 1 = down

	//if it doesn't look like the platform is down, release the lock so it will be able to get to the bottom
	if (!SensorValue[platformDown]) {
		SensorValue[platformLock] = 1;
	}
	//if the drum is not at 0, reset it; the encoder check is because the drum makes more than one rev in some cases and we still need to reset then
	//  even if drumZero bumper switch is pressed
	if (!SensorValue[drumZero] || SensorValue[drumPosEnc] > 100) {
		SensorValue[tongue] = 0;
		resetDrumPosition();
	}

	wait1Msec(250); //let things settle out

	if (SensorValue[platformDown]) { //make sure the platform is down now, then lock the platform and stick out the tongue
		SensorValue[platformLock] = 0; //lock the platform
		SensorValue[tongue] = 1; //stick the tongue out
	}
}

void fireCatapult() {
	if (SensorValue[drumPosEnc] > 100) {
		SensorValue[tongue] = 0; //move the tongue in before shooting; it will stay out until the platform is all the way up
		SensorValue[platformLock] = 1; //release the platform to fire objects

    bool catapultUpTimedOut = false;
    int catapultTimeOutTime = 3000; //in milliseconds; wait 3 seconds at most for the catapult to go all the way up
    time1[T1] = 0; //start timing how long we've been waiting for the catapult to go all the way up
		while(SensorValue[catapultPot] < 2550) { //when the catapult is up - CHECK VAL
			if (time1[T1] > catapultTimeOutTime) {
				catapultUpTimedOut = true;
				break; //exit this while loop
			}
			wait1Msec(25);
		}

		//once catapult is up, start resetting the drum
		 //this should be inside the next if statement

		if (!catapultUpTimedOut) { //if catapult up didn't time out
			resetDrumPosition();
			time1[T1] = 0;
			int catapultResetTimeOutTime = 7000; //check this value
			while (!SensorValue[platformDown] || SensorValue[catapultPot] > 2550) { //when platform is attached and catapult is down, move one; CHECK VAL
				if (time1[T1] > catapultResetTimeOutTime) {
					break; //exit this while loop
				}
				wait1Msec(25);
			}
		}

		//if either wait operation times out, the catapult will stop moving and the platform solenoid will extend out; the variable platform released is not used
		SensorValue[platformLock] = 0; //reattach platform
		SensorValue[tongue] = 1;

		}

}

void toggleSolenoid(int sensor) {
	int currVal = SensorValue[sensor],
			newVal = abs(currVal - 1); //If currVal is 1, newVal is 0.  If currVal is 0, newVal is abs(-1) = 1

	SensorValue[sensor] = newVal;
}

//press tongue
//hold platform lock
task button8UController() {
	while(1) {
		if (vexRT[Btn8U]) {
			time1[T1] = 0;
			while (vexRT[Btn8U]) {
					if (time1[T1] >= 500) { break; }
					wait1Msec(50); //don't hog the CPU
			}
			if (time1[T1] >= 500) { //toggle platform lock
				toggleSolenoid(platformLock);
				while (vexRT[Btn8U]) { //wait to continue until the button is released so the action only gets triggered once
					wait1Msec(50);
				}
			} else {
				toggleSolenoid(tongue);
			}

		}
	}
}

task driverCatapult() {
	bool btn8UPressed = false,
			 btn8DPressed  = false,
			 btn5UPressed = false;
	while(1) {
		//this is set up such that movement functions will suspend this task while they execute, thus disabling other catapult controls in that time
		if (vexRT[Btn8U]) {
			//prepareCatapult();
		} else if (vexRT[Btn5D]) {
			fireCatapult();
		} else if (vexRT[Btn7D]) {
			setCatapultPosition(0);
		} else if (vexRT[Btn7L]) {
		  setCatapultPosition(1);
		} else if (vexRT[Btn7U]) {
			setCatapultPosition(2);
		} else if (vexRT[Btn7R]) {
			setCatapultPosition(3);
		} else if (vexRT[Btn6D]) {
			resetDrumPosition();
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





		//hang lock
		if (vexRT[Btn8D] && !btn8DPressed) {
			toggleSolenoid(hangLock);
			btn8DPressed = true;
		} else if (!vexRT[Btn8D] && btn8DPressed) {
			btn8DPressed = false;
		}

		//drum ratchet
		if (vexRT[Btn5U] && !btn5UPressed) {
			toggleSolenoid(drumRatchet);
			btn5UPressed = true;
		} else if (!vexRT[Btn5U] && btn5UPressed) {
			btn5UPressed = false;
		}

		wait1Msec(25);
	}
}

#include "Clyde Auton 10.15.c"

task autonomous()
{
	catapultInit(); //make sure we can use the catapult	in any autonomous play
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

	//setCatapultPosition(0);
	//wait1Msec(1000);
	//fireCatapult();
	if (DEBUG_AUTON) {
		startTask(autonomous);
		stopTask(usercontrol);
	}
	startTask(driverCatapult);
	startTask(button8UController);
	//resetDrumPosition();
	bLCDBacklight = true;
	clearLCDLine(0);
	int LY = 0;
	int LX = 0;
	int RY = 0;
	int RX = 0;
	int threshold = 15;
	while (1) {
		displayLCDNumber(0,1,SensorValue[drumPosEnc],1);
		displayLCDNumber(1,0,SensorValue[drumZero],1);


		//8L/8R circle strafe
		//like strafenostraightening(55/127)

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
	}
}
