#pragma once
#include "BugAlgorithm.h"
class Bug2 :
	public BugAlgorithm
{

private:	
	
	//state machine states
	enum state { FOLLOW_LINE = 0, MOVE_AROUND_OBS = 1};
	state cState;
	int objToFollow;
	
	int moved;

	Point getMovementVector(Point baseVector);
	void addMovement(const Point& vct,Box& robot);
	
	bool followLine(Box obstacle[], Box& robot, int nObst);
	bool moveAroundObs(Box obstacle[], Box& robot, int nObst);
	bool backToLine(Box obstacle[], Box& robot, int nObst);
	Point rotate(Point vct, double deg);
	double perpendicularDist(Point lp1, Point lp2, Point refPoint);

	Point getClosestPointOnSegment(Point a, Point b, Point p);

	bool sanityCheck(Box obstacle[], Box robot[], int nObst);

	bool sanityCheckDone = false;



public:
	Bug2(const string& name);
	~Bug2();
	bool update(Box obstacle[], Box robot[], int nObst);
};

