//START: global movement functions for basic robot subsystem movements
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
		setDumpMotors(-20);
		wait1Msec(125);
		setDumpMotors(0);
	}

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

void moveClaw(int power, int potValue)//allows us to move the claw in auto and compensate
{
	killClawCompensation(); //very important to do this before manually controlling the motors because PID is doing its thing
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
			setClawMotors(-abs(power)); //the exact opposite of the above codition for positive input (credit to Evan for remembering the number -1 exists)
		}
	}
	startClawCompensation(potValue); //turns on the compensation code for the claw
}

void manualCompensation() //allows us to use the single power compensation
{
	killClawCompensation(); //very important to do this before manually controlling the motors because PID is doing its thing
	setClawMotors(15);
}

void waitForLift(int target, int variance)
{
	while(SensorValue[arm]> target+variance || SensorValue[arm]< target-variance)
	{
		wait1Msec(25);
	}
}

void waitForClaw(int target, int variance)
{
	while(SensorValue[claw]>target+variance || SensorValue[claw] < target-variance)
	{
		wait1Msec(25);
	}
}


void rotateDegrees(int position, int direction) {//This function is for turning
	SensorValue[gyro] = 0;
	//Clear gyro
	if(direction == 1){
		//If direction == Left
		while(abs(SensorValue[gyro]) < position){
			//While the gyro is less than a set degrees, turn Left
			setRightDtMotors(45);
			setLeftDtMotors(-45);
		}
		setRightDtMotors(-30);
		setLeftDtMotors(30);
		wait1Msec(100); //brief brake
	}
	//end of LEFT turn
	else{
		//if direction == right
		while(abs(SensorValue[gyro]) < position){
			//While the gyro is less than a set degrees, turn right
			setRightDtMotors(-45);
			setLeftDtMotors(45);
		}

		setRightDtMotors(30);
		setLeftDtMotors(-30);
		wait1Msec(100); //brief brake
	} //end of RIGHT turn
	setRightDtMotors(0);
	setLeftDtMotors(0);

}
