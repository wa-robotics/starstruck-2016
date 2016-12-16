typedef enum _LCDButton {
	LEFT, CENTER, RIGHT
} LCDButton;

typedef struct _Answer {
		LCDButton button;
		char text;
		int functionToRun;
		int nextMenuId;
} Answer;

typedef struct _LCDQuestion {
		int id;
		char question;
		Answer left;
		Answer center;
		Answer right;
	} LCDQuestion;
LCDQuestion questions[1];
void newQuestion(LCDQuestion& q, int menuNum, char* question, Answer* leftAnswer, Answer* centerAnswer, Answer* rightAnswer) {
	q.id = menuNum;
	q.question = (char) question;

	q.left = (Answer) leftAnswer;
	q.center = (Answer)centerAnswer;
	q.right = (Answer)rightAnswer;
}

Answer *getQuestionAnswer(char* name, int functionID, LCDButton btn, int nextID) {
	static Answer questionAnswer;
	questionAnswer.button = btn;
	questionAnswer.text = name;
	questionAnswer.functionToRun = functionID;
	questionAnswer.nextMenuId = nextID;
	return questionAnswer;
}

void lcdExecuteFunction (int num) {
	switch (num) {
		case 1:
			break;
	}
}

LCDQuestion q1;

task lcdWizard() {
	int qId = 0;
	string question, leftAns, centerAns, rightAns;
	//while (1) {
		//question = questions[0].question;
		//leftAns = questions[qId].left.text;
		//centerAns = questions[qId].center.text;
		//rightAns = questions[qId].right.text;
		//displayLCDCenteredString(0, question);
		//displayLCDString(1,0,leftAns);
		//displayLCDString(1,6,centerAns);
		//displayLCDString(1,11,rightAns);
		writeDebugStreamLine("%s %s %s %s",q1.question, leftAns, centerAns, rightAns);
	//}
}

task main()
{
	newQuestion(q1,0,"Question",getQuestionAnswer("test",2,LEFT,3),getQuestionAnswer("test",2,CENTER,3),getQuestionAnswer("test",2,RIGHT,3));
	//startTask(lcdWizard);
	Answer test;
	//test = (Answer) getQuestionAnswer("test",2,CENTER,3);
	writeDebugStreamLine("%s",test.button);

while(1){}
}
