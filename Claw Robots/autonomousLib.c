//this file assumes that
//some constants for claw PID
float kp = 0.5; //these constants might change down the road, but they are good for now
float ki = 0.01;
float kd = 0.00001;
int error;
int integral = 0;
int derivative;
int lastError;
int PIDDrive;
int target;
bool compensate = false
//function to control claw compensation
task clawCompensate()
{
	while(1)
	{
		while(compensate == true) //allows us to control when to turn off compensation
		{
			error = target - SensorValue[claw];
			integral += error;
			derivative = error - lastError;
			PIDDrive = kp*error + ki*integral + kd*derivative;
			motor[clawRight] = PIDDrive;
			motor[clawLeft] = PIDDrive;
			lastError = error;
		}
	}
}
//functions for auto plays, and driver too I guess. (Evan's poor naming scheme not mine) -- Crawford
void setDumpMotors(int power) {
	if (power > 127) {
		power = 127;
		} else if (power < -127) {
		power = -127;
	}
	motor[lDump12] = power;
	motor[lDump3] = power;
	motor[rDump12] = power;
	motor[rDump3] = power;
}

void setClawMotors(int power) {
	motor[leftClaw] = power;
	motor[rightClaw] = power;
}

void setLeftDtMotors(int power) {
	motor[lDriveFront] = power;
	motor[lDriveBack] = power;
}

void setRightDtMotors(int power) {
	motor[rDriveFront] = power;
	motor[rDriveBack] = power;
}

//2-motor forward right diagonal or backward left diagonal
void diagonalLeft(int power, int dist) {
	SensorValue[lDriveEnc] = 0;
	while(abs(SensorValue[lDriveEnc]) < dist) {
		writeDebugStreamLine("%d",SensorValue[lDriveEnc]);
		motor[lDriveFront] = power;
		motor[rDriveBack] = power;
	}
	if (power > 0) {
		motor[lDriveFront] = -10;
		motor[rDriveBack] = -10;
		} else if (power < 0) {
		motor[lDriveFront] = 10;
		motor[rDriveBack] = 10;
	}
	wait1Msec(75);
	motor[lDriveFront] = 0;
	motor[rDriveBack] = 0;
}

void straight(int power, int dist) {
	SensorValue[lDriveEnc] = 0;
	while(abs(SensorValue[lDriveEnc]) < dist) {
		setLeftDtMotors(power);
		setRightDtMotors(power);
	}
	if (power > 0) {
		setLeftDtMotors(-10);
		setRightDtMotors(-10);
		} else if (power < 0) {
		setLeftDtMotors(10);
		setRightDtMotors(10);
	}
	wait1Msec(75);
	setLeftDtMotors(0);
	setRightDtMotors(0);
}

//high pot value = bottom
//low pot value = top
void liftToPotTarget(int target, int maxPower) {
	int potStart = SensorValue[arm];
	if (potStart > target) { //potentiometer value is above target.  New target is UP
		while (SensorValue[arm] > target + 500) {
			setDumpMotors(maxPower);
		}
		while (SensorValue[arm] > target) {
			setDumpMotors(.3*maxPower);
		}
		setDumpMotors(-20)
		wait1Msec(125);
		setDumpMotors(0);
	}
}
void killCompensation() // lets us turn off all compensation
{
	compensate = false
	setClawMotors(0);
}
void moveClaw(int power, int potValue)//allows us to move the claw in auto and compensate
{
	killCompensation(); //very important to do this before manually controlling the motors because PID is doing its thing
	if(SensorValue[claw] < potValue)
	{
		while(SensorValue[claw] < potValue)
		{
			setClawMotors(abs(power)); //if the claw needs to be opened, this makes sure you are using a positive power so that it doesn't try to move forever
		}
	}
	else if(SensorValue[claw] > potValue)
	{
		while(SensorValue[claw] > potValue)
		{
			setClawMotors(-1*abs(power)); //the exact opposite of the above codition for positive input (credit to Evan for remembering the number -1 exists)
		}
	}
	target = potValue
	compensate = true //turns on the compensation code for the claw
}
void manualCompensation(); //allows us to use the single power compensation
{
	killCompensation(); //very important to do this before manually controlling the motors because PID is doing its thing
	setClawMotors(15);
}
