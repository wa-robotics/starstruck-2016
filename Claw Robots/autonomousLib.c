//this file assumes that
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
