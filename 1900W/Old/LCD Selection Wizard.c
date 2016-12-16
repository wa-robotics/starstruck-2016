

typedef struct LcdResponse {
	string startingTile;
	int waitTime;
} LcdResponse;

LcdResponse autonChoices;

void waitForPress() {
	while(nLCDButtons == 0) {
		wait1Msec(50);
	}
}

void waitForRelease() {
	while(nLCDButtons > 0) {
		wait1Msec(50);
	}
}

task lcdSelection();

task stopLcdSelectionMonitor() {
	while(1) {
		if (abs(vexRT[Ch1]) > 35 || abs(vexRT[Ch2]) > 35 || abs(vexRT[Ch3]) > 35 || abs(vexRT[Ch4]) > 35) {
			clearLCDLine(0);
			clearLCDLine(1);
			stopTask(lcdSelection);
			stopTask(stopLcdSelectionMonitor);
		}
		wait1Msec(250);
	}
}

task lcdSelection()
{
	startTask(stopLcdSelectionMonitor);
	bDisplayCompetitionStatusOnLcd = false;
	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);

	displayLCDCenteredString(0,"Starting tile:");
	displayLCDCenteredString(1,"Left  Sk.  Right");
	waitForPress();
	int startingTileBtn = nLCDButtons;
	waitForRelease();
	switch (startingTileBtn) {
		case 1:
			autonChoices.startingTile = "left";
			break;
		case 2:
			autonChoices.startingTile = "programmingSkills";
			break;
		case 3:
			autonChoices.startingTile = "right";
			break;
	}

	displayLCDString(0,0,autonChoices.startingTile);
	displayLCDCenteredString(1,"selected");

}
