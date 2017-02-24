
float positionKp = .8, //proportional constant for positional PID
			straighteningKpLeft = 0.29,//.43,//.195, //proportional constant for straightening response for the left side
			straighteningKpRight = 0.29,//.22,//.16, //proportional constant for straightening response for the right side
			straighteningKpLeftTurn = 0.4,//.43,//.195, //proportional constant for straightening response for the left side when turning
			straighteningKpRightTurn = 0.4,//.22,//.16, //proportional constant for straightening response for the right side when turning
			positionKi = 0.000350, //integral constant
			positionKd = 4.25; //derivative constant


void setLeftDtMotors(float power) {
	motor[lDriveFront] = power;
	motor[lDriveBack] = power;
}

void setRightDtMotors(float power) {
	motor[rDriveFront] = power;
	motor[rDriveBack] = power;
}

void setLeftDtMotorsStrafe(float power) {
		motor[lDriveFront] = power;
		motor[lDriveBack] = -power;
}

void setRightDtMotorsStrafe(float power) {
		motor[rDriveFront] = -power;
		motor[rDriveBack] = power;
}

//Structure to hold all position PID related data
typedef struct _position_pid_controller {
    // Encoder
    int            e_current;              ///< current encoder count
    int            e_last;                 ///< last encoder count
    // PID control algorithm variables
    int            target;
    float						 slewRateLimit;
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
static int STRAFE_LEFT = 3;
static int STRAFE_RIGHT = 4;
static int STRAFE = 5;
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
void liftToTargetPIDEnc(int target, int time, float kP, float kI, float kD) {
	int error = 0,
	errorSum = 0,
	lastError = 0,
	slewRateLimit = 15;

	float pTerm,
	iTerm,
	dTerm,
	power,
	lastPower = 0;

	time1[T1] = 0;

	while (time1[T1] < time) {
		//update error terms
		error = target - SensorValue[liftEnc];
		errorSum += error;

		pTerm = error * (float) kP;
		iTerm = errorSum * (float) kI;
		dTerm = (error - lastError) * (float) kD; //calculate motor power
		power = pTerm + iTerm + dTerm;

		//limit the values of the power term to only be those that can be taken by the motors
		if (power > 127) {
			power = 127;
		} else if (power < -127) {
			power = -127;
		}

		lastError = error; //update last error

		//apply a slew rate to limit acceleration/deceleration
		if(abs(power-lastPower) > slewRateLimit) {
			if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
				power = lastPower + slewRateLimit; //increment the power to only add
			} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
				power = lastPower - slewRateLimit;
			}
		}

		lastPower = power; //update the last power
		writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,SensorValue[liftEnc],pTerm,iTerm,dTerm,power);
		setDumpMotors(power);
		wait1Msec(25);
	}
}

//positive powers will go forward or right
//negative powers values go backwards or left
//encoder counts is how many counts to go, always positive
//power is the power to run the motors at before straightening is applied
//time is a maximum time to complete the operation
void driveDistancePID(int encoderCounts, int direction, int time) {
	writeDebugStreamLine("nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower");
	//reset encoder values
	Sensorvalue[lDriveEnc] = 0;
	SensorValue[rDriveEnc] = 0;

	int error = 0,
	straighteningError = 0,
	errorSum = 0,
	lastError = 0,
	target = encoderCounts,
	slewRateLimit = 15;

	float pTerm,
	iTerm,
	dTerm,
	straighteningCorrection,
	power,
	lPower,
	rPower,
	lastPower = 0;

	time1[T1] = 0;
	if (direction == STRAIGHT || direction == STRAFE || direction == ROTATE_LEFT || direction == ROTATE_RIGHT) { //validate direction

			if (direction == STRAIGHT) {
				while (time1[T1] < time) {
					//update error terms
					error = target - (SensorValue[lDriveEnc] + SensorValue[rDriveEnc])/2.0;
					errorSum += error;

					pTerm = error * (float) positionKp;
					iTerm = errorSum * (float) positionKi;
					dTerm = (error - lastError) * (float) positionKd; //calculate motor power
					power = pTerm + iTerm + dTerm;

					//limit the values of the power term to only be those that can be taken by the motors
					if (power > 127) {
						power = 127;
					} else if (power < -127) {
						power = -127;
					}

					lastError = error; //update last error

					//apply a slew rate to limit acceleration/deceleration
					if(abs(power-lastPower) > slewRateLimit) {
						if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
							power = lastPower + slewRateLimit; //increment the power to only add
						} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
							power = lastPower - slewRateLimit;
						}
					}

					//adjust the powers sent to each side if the encoder values don't match
					straighteningError = SensorValue[lDriveEnc] - SensorValue[rDriveEnc];

					if (straighteningError < 0) { //right side is ahead, so speed up the right side
						rPower = power + straighteningError*straighteningKpRight;
					} else { //otherwise, just set the right side to the power
						rPower = power;
					}
					if (straighteningError > 0) { //left side is ahead, so speed up the left side
						lPower = power - straighteningError*straighteningKpLeft;
					} else { //otherwise, just set the right side to the power
						lPower = power;
					}

					lastPower = power; //update the last power
					writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,SensorValue[lDriveEnc], SensorValue[rDriveEnc],pTerm,iTerm,dTerm,lPower,rPower);
					setLeftDtMotors(lPower);
					setRightDtMotors(rPower);
					wait1Msec(25);
				}

			} else if (direction == STRAFE) {
					while(time1[T1] < time) {
						//update error terms
						if (target >= 0) {
							 error = target - (SensorValue[lDriveEnc] + SensorValue[rDriveEnc]*-1)/2.0;

						}	else {
							 error = target + (SensorValue[lDriveEnc] + SensorValue[rDriveEnc]*-1)/2.0;

					  }
						errorSum += error;

						pTerm = error * (float) positionKp;
						iTerm = errorSum * (float) positionKi;
						dTerm = (error - lastError) * (float) positionKd; //calculate motor power
						power = pTerm + iTerm + dTerm;

						lastError = error; //update last error

						//apply a slew rate to limit acceleration/deceleration
						if(abs(power-lastPower) > slewRateLimit) {
							if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
								power = lastPower + slewRateLimit; //increment the power to only add
							} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
								power = lastPower - slewRateLimit;
							}
						}

						//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(SensorValue[lDriveEnc]) - abs(SensorValue[rDriveEnc]);
						writeDebugStreamLine("%d",straighteningError);
					//if (straighteningError < 0) { //right side is ahead, so speed up the right side
					//	rPower = power + straighteningError*straighteningKpRight;
					//} else { //otherwise, just set the right side to the power
					//	rPower = power;
					//}
					//if (straighteningError > 0) { //left side is ahead, so speed up the left side
					//	lPower = power - straighteningError*straighteningKpLeft;
					//} else { //otherwise, just set the right side to the power
					//	lPower = power;
					//}
						rPower = power;
						lPower = power;

						lastPower = power;
						setLeftDtMotorsStrafe(lPower);
						setRightDtMotorsStrafe(rPower);
						writeDebugStreamLine("4: %d,%f,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,SensorValue[lDriveEnc], SensorValue[rDriveEnc],pTerm,iTerm,dTerm,lPower,rPower);
						wait1Msec(25);
					}
					setLeftDtMotors(0);
					setRightDtMotors(0);
			} else if (direction == ROTATE_LEFT || direction == ROTATE_RIGHT) {
					while (time1[T1] < time) {
						//update error terms
						error = target - (abs(SensorValue[lDriveEnc]) + abs(SensorValue[rDriveEnc]))/2; //need to use absolute values here because one of
						errorSum += error;

						pTerm = error * (float) positionKp;
						iTerm = errorSum * (float) positionKi;
						dTerm = (error - lastError) * (float) positionKd; //calculate motor power
						power = pTerm + iTerm + dTerm;

						//limit the values of the power term to only be those that can be taken by the motors
						if (power > 127) {
							power = 127;
						} else if (power < -127) {
							power = -127;
						}

						lastError = error; //update last error

						//apply a slew rate to limit acceleration/deceleration
						if(abs(power-lastPower) > slewRateLimit) {
							if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
								power = lastPower + slewRateLimit; //increment the power to only add
							} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
								power = lastPower - slewRateLimit;
							}
						}

						//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(SensorValue[lDriveEnc]) - abs(SensorValue[rDriveEnc]);

						if (straighteningError > 0) { //left side is ahead, so speed up the right side
							rPower = power + straighteningError*straighteningKpLeftTurn;
						} else { //otherwise, just set the right side to the power
							rPower = power;
						}
						if (straighteningError < 0) { //right side is ahead, so speed up the left side
							lPower = power - straighteningError*straighteningKpRightTurn;
						} else { //otherwise, just set the right side to the power
							lPower = power;
						}

						lastPower = power; //update the last power
						writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,error,SensorValue[lDriveEnc],SensorValue[rDriveEnc],pTerm,iTerm,dTerm,lPower,rPower);
						setLeftDtMotors(lPower * direction); //for a left turn, ROTATE_LEFT = -1 so this moves the left side backwards for a left turn. For a right turn will go forward since ROTATE_RIGHT = 1
						setRightDtMotors(rPower * -1 * direction); //same idea as for a left turn, except this side needs to go the opposite way as the left side in order to turn, hence the * -1 in the calculation
						wait1Msec(25);
				}
				setLeftDtMotors(0);
				setRightDtMotors(0);
			}
		}
	}
