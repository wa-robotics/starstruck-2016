#pragma config(Motor,  port4,           FL,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           FR,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           BR,            tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           BL,            tmotorVex393TurboSpeed_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*------------------------------------------------------

Motor Pin Diagram:

FL - front left motor
FR - front right motor
BR - back right motor
BL - back left motor

Motion / Mechanics Notes

Entire left side is reversed so that robot can go forward
List (FL Speed, BL speed, BR speed, FR) - remember FL and BL are reversed
Forward: 127, 127, 127, 127 - both joysticks forward
Backwards: -127, -127, -127, -127 - both joysticks backwards
Strafe Left: -127, 127, 127, -127 - both joysticks left
Strafe Right: 127, -127, -127, 127 - both joysticks right
Rotate Left: -127, -127, 127, 127 - left joystick down right joystick up
Rotate Right: 127, 127, -127,-127 - left joystick up right joystick down


------------------------------------------------------*/

int threshold = 15;

task main()
{
	while(true){

		//If joysticks are deadzones
		if ((vexRT[Ch1] < threshold || vexRT[Ch4] < threshold) && (vexRT[Ch2] < threshold || vexRT[Ch3] < threshold)){
			motor[FR] = 0;
			motor[BR] = 0;
			motor[FL] = 0;
			motor[BL] = 0;
		}
		// Joystick Control
		motor[FR] = vexRT[Ch2] - vexRT[Ch1];
		motor[FL] = vexRT[Ch3] + vexRT[Ch4];
		motor[BR] = vexRT[Ch2] + vexRT[Ch1];
		motor[BL] = vexRT[Ch3] - vexRT[Ch4];

	}
}