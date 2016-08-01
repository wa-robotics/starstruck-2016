typedef enum _LCDButton {
	LEFT, CENTER, RIGHT
} LCDButton;

typedef struct _Answer {
		LCDButton button;
		string text;
		int functionToRun;
		int nextMenuId;
} Answer;

typedef struct _LCDQuestion {
		int id;
		string question;
		Answer left;
		Answer center;
		Answer right;
	} LCDQuestion;

void newQuestion(LCDQuestion *q, int menuNum, char* question, char* leftAnswerText, int leftAnswerFunction, int leftNextMenu, char* centerAnswerText, int centerAnswerFunction, int centerNextMenu,char* rightAnswerText, int rightAnswerFunction, int rightNextMenu) {
	q->id = menuNum;
	q->question = question;

	Answer left, center, right;
	left.button = LEFT;
	left.text = leftAnswerText;
	left.functionToRun = leftAnswerFunction;
	left.nextMenuId = leftNextMenu;

	center.button = CENTER;
	center.text = centerAnswerText;
	center.functionToRun = centerAnswerFunction;
	center.nextMenuId = centerNextMenu;

	right.button = RIGHT;
	right.text = rightAnswerText;
	right.functionToRun = rightAnswerFunction;
	right.nextMenuId = rightNextMenu;

	q->left = left;
	q->center = center;
	q->right = right;
}

LCDQuestion q1, q2, q3;
task main()
{
	newQuestion(q1,3,"Question","A1",0,0,"A2",0,0,"A3",0,0);
	LCDQuestion test[5];
	test[0] = q1;


}
