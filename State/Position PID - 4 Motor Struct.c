
float positionKp = .8, //proportional constant for positional PID
			straighteningKpLeft = 0.29,//.43,//.195, //proportional constant for straightening response for the left side
			straighteningKpRight = -0.29,//.22,//.16, //proportional constant for straightening response for the right side
			straighteningKpLeftTurn = 0.4,//.43,//.195, //proportional constant for straightening response for the left side when turning
			straighteningKpRightTurn = 0.4,//.22,//.16, //proportional constant for straightening response for the right side when turning
			positionKi = 0.000350, //integral constant
			positionKd = 4.25; //derivative constant

float liftKp,
			liftKi,


void setLeftDtMotors(float power) {
	motor[lDriveFront] = power;
	motor[lDriveBack] = power;
}

void setRightDtMotors(float power) {
	motor[rDriveFront] = power;
	motor[rDriveBack] = power;
}
//Structure to hold all position PID related data
typedef struct _position_pid_controller {
    // Encoder
    long            e_current;              ///< current encoder count
    long            e_last;                 ///< last encoder count
    // PID control algorithm variables
    int            target;
    int						 slewRateLimit;
    float           error;                  ///
    float           last_error;             ///< error last time update called
    float						errorSum;
    float						Kp;
    float						Ki;
    float						Kd;
    float						p;
    float						i;
    float						d;

    // final motor drive
    long            motor_drive;            ///< final motor control value
} posPID;

static int STRAIGHT = 2; //the 2 here shouldn't matter as long as no variables are multiplied by 'direction' in driveDistancePID
static int STRAFE = 3;
static int ROTATE_LEFT = -1;
static int ROTATE_RIGHT = 1;

void posPidInit(posPID *pos, int target, float kP, float kI, float kD, float slewRate) {
	pos->Kp = kP;
	pos->Ki = kI;
	pos->Kd = kD;
	pos->target = target;
	pos->last_error = 0;
	pos->errorSum = 0;
	pos->e_last = 0;
	pos->slewRateLimit = slewRate;
}

//Check if motor power is > 127 or < -127, and if so, fix it
float clampPower(float power) {
	if (power > 127) {
		return 127;
	} else if (power < -127) {
		return -127;
	}
	return power;
}

float posPidNextPower (posPID *pos,int encoderVal) {
	pos->e_current = encoderVal;
	pos->error = pos->target - pos->e_current;
	pos->errorSum += pos->error;

	int oldMotorDrive = pos->motor_drive;

	pos->p = pos->error * (float) pos->Kp;
	pos->i = pos->errorSum * (float) pos->Ki;
	pos->d = (pos->error - pos->last_error) * (float)pos->Kd;

	pos->last_error = pos->error;
	float newMotorDrive = pos->p + pos->i + pos->d;

	newMotorDrive = clampPower(newMotorDrive);

	//apply a slew rate to limit acceleration/deceleration
	if(abs(newMotorDrive-oldMotorDrive) > pos->slewRateLimit) {
		if(newMotorDrive > oldMotorDrive) { //if the power is increasing (and the difference is greater than the slew rate allows)
			newMotorDrive = oldMotorDrive + pos->slewRateLimit; //increment the power to only add
		} else { //if the power is decreasing (and the difference is greater than the slew rate allows)
			newMotorDrive = oldMotorDrive - pos->slewRateLimit;
		}
	}
	pos->motor_drive = newMotorDrive;
	//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,pos->target,pos->error,SensorValue[lDriveEnc], SensorValue[rDriveEnc],pos->p,pos->i,pos->d,pos->motor_drive);
	return newMotorDrive;
}

int getReversedPot() {
	return 4095 - SensorValue[arm];
}

void liftToTargetPID(int target, int time, float kP, float kI, float kD) {
	posPID liftPID;
	posPidInit(liftPID,target,kP,kI,kD,15);
	time1[T1] = 0;
	writeDebugStreamLine("getReversedPot(), target, liftPID.p,liftPID.i,liftPID.d,power");
	float lastPower;
	while (time1[T1] < time) {
		float power = posPidNextPower(liftPID,getReversedPot());
		setDumpMotors(power);
		lastPower = power;
		writeDebugStreamLine("%d,%d,%d,%d,%d,%d",getReversedPot(), liftPID.error, liftPID.p,liftPID.i,liftPID.d,power);
		wait1Msec(25);
	}
	setDumpMotors(lastPower);
}

//@param time The maximum time in milliseconds to complete the operation
void driveDistancePID(int encoderCounts, int direction, int time) {
	int slewRateLimit = 15;
	posPID drivePID;
	posPidInit(drivePID,encoderCounts,positionKp,positionKi,positionKd,slewRateLimit);
	writeDebugStreamLine("nPgmTime,target,error,SensorValue[lDriveEnc], SensorValue[rDriveEnc],pTerm,iTerm,dTerm,lPower,rPower");
	//reset encoder values
	SensorValue[lDriveEnc] = 0;
	SensorValue[rDriveEnc] = 0;



	time1[T1] = 0;
	while(time1[T1] < time) {

		if (direction == STRAIGHT) {
			float encAverage = (SensorValue[lDriveEnc] + SensorValue[rDriveEnc])/2.0;
			float power = posPidNextPower(drivePID,encAverage);

			//adjust the powers sent to each side if the encoder values don't match
			int straighteningError = SensorValue[lDriveEnc] - SensorValue[rDriveEnc];
			float rPower, lPower;

			writeDebugStreamLine("%f",straighteningError;
			if (straighteningError > 0) { //left side is ahead, so speed up the right side
				rPower = power + straighteningError*straighteningKpLeft;
			} else { //otherwise, just set the right side to the power
				rPower = power;
			}
			if (straighteningError < 0) { //right side is ahead, so speed up the left side
				lPower = power - straighteningError*straighteningKpRight;
			} else { //otherwise, just set the right side to the power
				lPower = power;
			}

			writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,drivePID.target,drivePID.error,SensorValue[lDriveEnc], SensorValue[rDriveEnc],drivePID.p,drivePID.i,drivePID.d,lPower,rPower);
			setLeftDtMotors(lPower);
			setRightDtMotors(rPower);
			wait1Msec(25);

		} //end if (direction == STRAIGHT)
		wait1Msec(25);

		setLeftDtMotors(0);
		setRightDtMotors(0);

	} //end while time1[T1]

//	if (direction == STRAIGHT) {
//		while (time1[T1] < time) {
//			//update error terms
//			error = target - (nMotorEncoder[lDriveFront] + nMotorEncoder[rDriveFront])/2.0;


//			pTerm = error * (float) positionKp;
//			iTerm = errorSum * (float) positionKi;
//			dTerm = (error - lastError) * (float) positionKd; //calculate motor power
//			power = pTerm + iTerm + dTerm;

//			//limit the values of the power term to only be those that can be taken by the motors
//			if (power > 127) {
//				power = 127;
//			} else if (power < -127) {
//				power = -127;
//			}

//			lastError = error; //update last error

//			//apply a slew rate to limit acceleration/deceleration
//			if(abs(power-lastPower) > slewRateLimit) {
//				if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
//					power = lastPower + slewRateLimit; //increment the power to only add
//				} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
//					power = lastPower - slewRateLimit;
//				}
//			}

//			//adjust the powers sent to each side if the encoder values don't match
//			straighteningError = nMotorEncoder[lDriveFront] - nMotorEncoder[rDriveFront];

//			if (straighteningError > 0) { //left side is ahead, so speed up the right side
//				rPower = power + straighteningError*straighteningKpLeft;
//			} else { //otherwise, just set the right side to the power
//				rPower = power;
//			}
//			if (straighteningError < 0) { //right side is ahead, so speed up the left side
//				lPower = power - straighteningError*straighteningKpRight;
//			} else { //otherwise, just set the right side to the power
//				lPower = power;
//			}

//			lastPower = power; //update the last power
//			writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
//			setLDriveMotors(lPower);
//			setRDriveMotors(rPower);
//			wait1Msec(25);
//		}
//	} else if (direction == ROTATE_LEFT || direction == ROTATE_RIGHT) {
//		while (time1[T1] < time) {
//			//update error terms
//			error = target - (abs(nMotorEncoder[lDriveFront]) + abs(nMotorEncoder[rDriveFront]))/2; //need to use absolute values here because one of
//			errorSum += error;

//			pTerm = error * (float) positionKp;
//			iTerm = errorSum * (float) positionKi;
//			dTerm = (error - lastError) * (float) positionKd; //calculate motor power
//			power = pTerm + iTerm + dTerm;

//			//limit the values of the power term to only be those that can be taken by the motors
//			if (power > 127) {
//				power = 127;
//			} else if (power < -127) {
//				power = -127;
//			}

//			lastError = error; //update last error

//			//apply a slew rate to limit acceleration/deceleration
//			if(abs(power-lastPower) > slewRateLimit) {
//				if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
//					power = lastPower + slewRateLimit; //increment the power to only add
//				} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
//					power = lastPower - slewRateLimit;
//				}
//			}

//			//adjust the powers sent to each side if the encoder values don't match
//			straighteningError = abs(nMotorEncoder[lDriveFront]) - abs(nMotorEncoder[rDriveFront]);

//			if (straighteningError > 0) { //left side is ahead, so speed up the right side
//				rPower = power + straighteningError*straighteningKpLeftTurn;
//			} else { //otherwise, just set the right side to the power
//				rPower = power;
//			}
//			if (straighteningError < 0) { //right side is ahead, so speed up the left side
//				lPower = power - straighteningError*straighteningKpRightTurn;
//			} else { //otherwise, just set the right side to the power
//				lPower = power;
//			}

//			lastPower = power; //update the last power
//			writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
//			setLDriveMotors(lPower * direction); //for a left turn, ROTATE_LEFT = -1 so this moves the left side backwards for a left turn. For a right turn will go forward since ROTATE_RIGHT = 1
//			setRDriveMotors(rPower * -1 * direction); //same idea as for a left turn, except this side needs to go the opposite way as the left side in order to turn, hence the * -1 in the calculation
//			wait1Msec(25);
//	}
//}
}

//rotate the robot a certain amount
//@param deg The number of degrees to turn; positive values are counterclockwise, negative values are clockwise.
//@param direction The direction to turn in to get to the position; 1 is counterclockwise, -1 is clockwise
//void rotateDegrees(int position, int direction) {//This function is for turning
//	SensorValue[gyro] = 0;
//	//Clear gyro
//	if(direction == 1){
//		//If direction == Left
//		while(abs(SensorValue[gyro]) < position){
//			//While the gyro is less than a set degrees, turn Left
//			setRDriveMotors(45);
//			setLDriveMotors(-45);
//		}
//		setRDriveMotors(-30);
//		setLDriveMotors(30);
//		wait1Msec(100); //brief brake
//	}
//	//end of LEFT turn
//	else{
//		//if direction == right
//		while(abs(SensorValue[gyro]) < position){
//			//While the gyro is less than a set degrees, turn right
//			setRDriveMotors(-45);
//			setLDriveMotors(45);
//		}

//		setRDriveMotors(30);
//		setLDriveMotors(-30);
//		wait1Msec(100); //brief brake
//	} //end of RIGHT turn
//	setRDriveMotors(0);
//	setLDriveMotors(0);

//}
