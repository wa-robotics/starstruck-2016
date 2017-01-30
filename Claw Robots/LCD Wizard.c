void clearLCD() {
	clearLCDLine(0);
	clearLCDLine(1);
}

void showOnLCD(const string line0, const string line1) {
	clearLCD();
	displayLCDCenteredString(0,line0);
	displayLCDCenteredString(1,line1);
}

void waitForPress() {
	while(nLCDButtons == 0) {
		wait1Msec(25);
	}
}

void waitForRelease() {
	while (nLCDButtons > 0) {
		wait1Msec(25);
	}
}

int waitForButtonPress() {
	int btnPressed;
	waitForPress();
	btnPressed = nLCDButtons;
	waitForRelease();
	return btnPressed; //return the buttons that were pressed
}

task LCDSelect();

string playConfirmName = "";
task confirmChoice() {
	bool choiceConfirmed = false;
	while(!choiceConfirmed) {
			showOnLCD("Confirm choice:",playConfirmName);
			wait1Msec(1000);
			showOnLCD("Center - Confirm","L/R - Restart");
			wait1Msec(1000);
	}
}

task restartAutonSelection() {
	stopTask(LCDSelect);
	//reset global variables
	playConfirmName = "";
	AUTON_PLAY = 0;
	AUTON_SIDE = 0;
	startTask(LCDSelect);
	stopTask(restartAutonSelection);
}
	//L-R-Sk
	//#1 Big
	//#2 Small
	//#3 Cube (score middle cube and block)
	//#4 Fence (3 stars, corner)
	//#5 Nothing
	//#6 Prog skills
bool showVersion = true;
task LCDSelect() {
	if (showVersion) {
		clearLCD();
		displayLCDCenteredString(0,"LCD Select v1.0"); //briefly display version for debug purposes
		wait1Msec(750);
	}
	showVersion = false; //don't show the version again if autonSelection is ever reset


	//Q1: Which side?
	showOnLCD("Which side?","Left  Right  Sk.");
	waitForPress();
	int q1Response = nLCDButtons;
	waitForRelease();
	int q2Page = 1; //1 - Big/Small choices shown, 2 - Cube/Fence choices shown, 3 - Nothing choice shown
	int q2Response = 0;
	int q2PlayChosen = 0;

	if (q1Response != 4) {

		while (q2PlayChosen == 0) {

			if (q2Page == 1) {
				showOnLCD("Choose play: 1/3","Big   Small   >>");
				wait1Msec(250); //short delay to prevent button press from triggering multiple times
				waitForPress();
				q2Response = nLCDButtons;
				if (q2Response == 4) {
					q2Page = 2;
				} else if (q2Response == 1) {
					q2PlayChosen = 1;
				} else if (q2Response == 2) {
					q2PlayChosen = 2;
				}
				waitForRelease();
				if (q2Response == 1 ||q2Response == 2) {
					break;
				}

			} else if (q2Page == 2) {
				showOnLCD("Choose play: 2/3","Cube   Fence  >>");
				wait1Msec(250);
			  waitForPress();
				q2Response = nLCDButtons;
				if (q2Response == 4) {
					q2Page = 3;
				} else if (q2Response == 1) {
					q2PlayChosen = 3;
				} else if (q2Response == 2) {
					q2PlayChosen = 4;
				}
				waitForRelease();
				if (q2Response == 1 ||q2Response == 2) {
					break;
				}
			} else if (q2Page == 3) {
				showOnLCD("Choose play: 3/3","Nothing       >>");
				wait1Msec(250);
			  waitForPress();
				q2Response = nLCDButtons;
				if (q2Response == 4) {
					q2Page = 1;
				} else if (q2Response == 1) {
					q2PlayChosen = 5;
				}
				waitForRelease();
				if (q2Response == 1 ||q2Response == 2) {
					break;
				}
			} //end else if
		} //end while
	} //end if


	if (q1Response == 4) {
		AUTON_PLAY = 6;
		playConfirmName = "Prog. skills";
	} else {
		if (q1Response == 1) {
			AUTON_SIDE = LEFT;
		} else { //q1Response == 2
			AUTON_SIDE = RIGHT;
		}
		switch (q2PlayChosen) {
			case 1:
				AUTON_PLAY = 1;
				playConfirmName = "Big";
				break;
			case 2:
				AUTON_PLAY = 2;
				playConfirmName = "Small";
				break;
			case 3:
				AUTON_PLAY = 3;
				playConfirmName = "Cube";
				break;
			case 4:
				AUTON_PLAY = 4;
				playConfirmName = "Fence";
				break;
			case 5:
				AUTON_PLAY = 5;
				playConfirmName = "NOTHING";
				break;
		}
	}

	startTask(confirmChoice);
	wait1Msec(250);
	int confirmResult = waitForButtonPress();
	stopTask(confirmChoice);
	if (confirmResult == 2) {
		if (AUTON_PLAY == 5) {
			showOnLCD("NOTHING","No auton!");
		} else {
			showOnLCD(playConfirmName,"will run!");
		}
	} else {
		startTask(restartAutonSelection);
	}



} //end task
