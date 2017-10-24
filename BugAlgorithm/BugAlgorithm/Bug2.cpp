#include "Bug2.h"



Bug2::Bug2(const string& name) : BugAlgorithm(name), cState(Bug2::FOLLOW_LINE)
{
}

Point Bug2::getMovementVector(Point baseVector)
{

	double length = sqrt((baseVector.x * baseVector.x) + (baseVector.y * baseVector.y));

	Point normVct;
	normVct.x = baseVector.x / length;
	normVct.y = baseVector.y / length;
	normVct.z = 0;
	normVct.SetLength(speed);

	return normVct;

}

void Bug2::addMovement(const Point& vct,Box& robot)
{
	actPoint = actPoint + vct;
	robot.Set(actPoint);
}

bool Bug2::followLine(Box obstacle[], Box& robot, int nObst) {
	//line between the start an the end pos
	Point line = goalPosition - startPosition;

	//check for collisions
	int obsIndex = obstacleInWay(obstacle, robot, nObst);

	if (obsIndex == -1)
	{
		//no object
		Point mVct = getMovementVector(line);

		addMovement(mVct, robot);
		
		if (distanceEuclid(actPoint, goalPosition) < speed*2) {
			actPoint = goalPosition;
			return true;
		}
		else {
			return false;
		}	

	}
	else
	{
		hitPoint.push_back(actPoint);
		objToFollow = obsIndex;
		moved = 0;
		cState = MOVE_AROUND_OBS;
		return false;
	}
}




Point Bug2::rotate(Point vct, double deg)
{
	double radians = deg* (M_PI / 180);

	double ca = cos(radians);
	double sa = sin(radians);

	Point rotatedVct;
	rotatedVct.x = ca*vct.x - sa*vct.y;
	rotatedVct.y = sa*vct.x + ca*vct.y;

	return rotatedVct;
}

bool Bug2::moveAroundObs(Box obstacle[], Box& robot, int nObst)
{
	Point* pt = new Point();
	
	double dist= robot.distance(obstacle[objToFollow], pt);

	moved++;	

	Point rVct = rotate((*pt), -90);

	Point mVct = getMovementVector(rVct);

	//check where we would end up	
	Box tmp = robot;
	Point p;
	p.x = actPoint.x + mVct.x;
	p.y = actPoint.y + mVct.y;
	p.z = 0;
	tmp.Set(p);
	dist = tmp.distance(obstacle[objToFollow], pt);
	
	if (dist > speed * 2)
	{
		double length = sqrt((pt->x * pt->x) + (pt->y * pt->y));
		Point normVct;
		normVct.x = pt->x / length;
		normVct.y = pt->y / length;
		normVct.z = 0;		
		double l = dist - (speed * 1.2);
		normVct.SetLength(l);
		p = p + normVct;
		p.z = 0;
		robot.Set(p);
		actPoint = p;
	}
	else
	{
		addMovement(mVct, robot);
	}

		
	if (moved > 3)
	{
		double distToLine = perpendicularDist(startPosition, goalPosition, actPoint);

		if (distToLine < speed)
		{
			//move onto the closest point on the line
			Point nextPos = getClosestPointOnSegment(startPosition, goalPosition, actPoint);

			leavePoint.push_back(actPoint);

			actPoint = nextPos;
			robot.Set(actPoint);
			
			cState = FOLLOW_LINE;
			return false;
		}

		if (distanceEuclid(hitPoint.at(hitPoint.size() - 1), actPoint)<speed) {

			cout << "no way found" << endl;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

Point Bug2::getClosestPointOnSegment(Point a, Point b, Point p)
{
	//implementation inspired by
	//https://stackoverflow.com/questions/3120357/get-closest-point-to-a-line

	Point aToP = p - a;  
	Point aToB = b - a;  

	aToB.z = 0;
	aToP.z = 0;

	double atb2 = pow(aToB.x, 2) + pow(aToB.y, 2);	

	double dot = aToP.Dot(aToB);

	double t = dot / atb2; 

	Point pOnLine;

	pOnLine.x = a.x + aToB.x * t;
	pOnLine.y = a.y + aToB.y * t;
	pOnLine.z = 0;

	return pOnLine;
}

bool Bug2::backToLine(Box obstacle[], Box& robot, int nObst) {
	return false;
}

double Bug2::perpendicularDist(Point lp1, Point lp2, Point refPoint) {

	//https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

	double dist = abs((lp2.y - lp1.y)*refPoint.x - (lp2.x - lp1.x)*refPoint.y + (lp2.x*lp1.y) - (lp2.y*lp1.x))/sqrt(pow(lp2.y-lp1.y,2)+pow(lp2.x - lp1.x,2));

	return dist;

}

bool Bug2::update(Box obstacle[], Box robot[], int nObst)
{

	if (cState == FOLLOW_LINE) {
		return followLine(obstacle, robot[0], nObst);
	}
	else if (cState == MOVE_AROUND_OBS) {
		return moveAroundObs(obstacle, robot[0], nObst);
	}




	return true;
}

Bug2::~Bug2()
{
}
