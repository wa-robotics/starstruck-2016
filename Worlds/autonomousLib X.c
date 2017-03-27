//START: global movement functions for basic robot subsystem movements
//parameter is a float because sometimes this gets powers via a PID function
void setLeftDtMotors(float power) {
	motor[lDriveFront] = power;
	motor[lDriveBack] = power;
}

void setRightDtMotors(float power) {
	motor[rDriveFront] = power;
	motor[rDriveBack] = power;
}

void setDumpMotors(float power) {
	motor[dumpTopY] = power;
	motor[dumpBottomY] = power;
	motor[dumpOutsideLeft] = power;
	motor[dumpOutsideRight] = power;
}

void setClawMotors(float power) {
	motor[leftClaw] = power;
	motor[rightClaw] = power;
}
//END: global functions

//function to control claw compensation
int target; //this variable needs global scope so PID can be controlled from outside this task.  Other variables are inside so they reset each time
task clawCompensatePID()
{
	//some constants for claw PID
	float kp = 0.5; //these constants might change down the road, but they are good for now
	float ki = 0.01;
	float kd = 0.00001;
	int error;
	int integral = 0;
	int derivative;
	int lastError = 0;
	int PIDDrive;

	while(1)
	{
			error = target - SensorValue[claw];
			integral += error;
			derivative = error - lastError;
			PIDDrive = kp*error + ki*integral + kd*derivative;
			setClawMotors(PIDDrive);
			lastError = error;
			wait1Msec(25); //don't hog the CPU
	}
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
	SensorValue[rDriveEnc] = 0;
	while(abs(SensorValue[rDriveEnc]) < dist) {
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
void strafeRight(int target, int speed)
{
	SensorValue[rDriveEnc] = 0;
	while(abs(sensorValue[rDriveEnc]) < target)
	{
		motor[lDriveFront] = speed;
		motor[lDriveBack] = -speed;
		motor[rDriveFront] = -speed;
		motor[rDriveBack] = speed;
	}
	motor[lDriveFront] = 0;
	motor[lDriveBack] = 0;
	motor[rDriveFront] = 0;
	motor[rDriveBack] = 0;
}
void strafeLeft(int target, int speed)
{
	SensorValue[rDriveEnc] = 0;
	while(abs(sensorValue[rDriveEnc]) < target)
	{
		motor[lDriveFront] = -speed;
		motor[lDriveBack] = speed;
		motor[rDriveFront] = speed;
		motor[rDriveBack] = -speed;
	}
	motor[lDriveFront] = 0;
	motor[lDriveBack] = 0;
	motor[rDriveFront] = 0;
	motor[rDriveBack] = 0;
}
//high pot value = bottom
//low pot value = top
void liftToPotTarget(int target, int maxPower) {
	int potStart = SensorValue[arm];
	int difference = abs(target - potStart);
	if (potStart > target && difference > 50) { //potentiometer value is above target.  New target is UP
		while (SensorValue[arm] > target + 500 || SensorValue[arm] < target) {
			setDumpMotors(abs(maxPower));
		}
		while (SensorValue[arm] > target || SensorValue[arm] < target - 50) {
			setDumpMotors(abs(.3*maxPower));
		}
		setDumpMotors(-20);
		wait1Msec(125);
		setDumpMotors(0);
	}
	else if (potStart < target) { //potentiometer value is above target.  New target is UP
		while (SensorValue[arm] < target - 500) {
			setDumpMotors(-abs(maxPower));
		}
		while (SensorValue[arm] < target) {
			setDumpMotors(-abs(.3*maxPower));
		}
		setDumpMotors(0);
	}
	setDumpMotors(12);

}
void killClawCompensation() // lets us turn off all compensation
{
	stopTask(clawCompensatePID);
	setClawMotors(0);
}

void startClawCompensation(int newTarget) {
	target = newTarget;
	startTask(clawCompensatePID);
}
void manualCompensation() //allows us to use the single power compensation
{
	killClawCompensation(); //very important to do this before manually controlling the motors because PID is doing its thing
	setClawMotors(15);
}
void moveClaw(int power, int potValue)//allows us to move the claw in auto and compensate
{
	int lastVal = SensorValue[claw];
	//killClawCompensation(); //very important to do this before manually controlling the motors because PID is doing its thing
	if(SensorValue[claw] > potValue)
	{
		while(SensorValue[claw] > potValue || abs(SensorValue[claw] - lastVal) > 100)
		{
			lastVal = SensorValue[claw];
			setClawMotors(-abs(power)); //if the claw needs to be opened, this makes sure you are using a positive power so that it doesn't try to move forever
		}
	}
	else if(SensorValue[claw] < potValue)
	{
		while(SensorValue[claw] < potValue)
		{
			lastVal = SensorValue[claw];
			setClawMotors(abs(power)); //the exact opposite of the above codition for positive input (credit to Evan for remembering the number -1 exists)
		}
	}
	//manualCompensation();
	//startClawCompensation(potValue); //turns on the compensation code for the claw
}


void waitForLift(int target, int variance)
{
	while(SensorValue[liftEnc] < target-variance && SensorValue[LiftEnc] > target+variance)
	{
		wait1Msec(25);
	}
}

void waitForClaw(int target, int variance)
{
	writeDebugStreamLine("inside waiting for claw");
	writeDebugStreamLine("%d,%d,%d",SensorValue[claw],target+variance,target+variance);
	int lower = target - variance;
	int upper = target + variance;
	while(SensorValue[claw] > upper || SensorValue[claw] < lower)
	{
		writeDebugStreamLine("waiting for claw");
		wait1Msec(25);
	}
}
