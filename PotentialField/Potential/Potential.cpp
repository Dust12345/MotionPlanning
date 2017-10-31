#include <iostream>
#include "Potential.h"

using namespace std;

/*********************************************************************************************************************************/
static const double INKR = 0.01;        // step size for robot along gradient
static const double DIST_MIN = 0.05;    // minimum distance between the robot and the goal
static const double GOAL_ERROR = 0.01;  // distance between the robot and the goal

//scaling factors and thresholds
static const double C_REP_THRES = 0.1;
static const double C_REP_SCALING = 0.00000025;

static const double B_REP_THRES = 0.01;
static const double B_REP_SCALING = 0.000000000025;

static const double ATTRAC_SCALING = 0.025;
static const double ATTRAC_THES = 0.2;

/*********************************************************************************************************************************/
Potential::Potential(const std::string& name)
    : goalPosition(1.0, 1.0, 0.0) //set the goal position
    , startPosition(0.0, 0.0, 0.0) //set the start position
{
}

void Potential::setGoalPosition(double x, double y, double z)
{
    goalPosition.x = x;
    goalPosition.y = y;
    goalPosition.z = z;
}

void Potential::setStartPosition(double x, double y, double z)
{
    startPosition.x = x;
    startPosition.y = y;
    startPosition.z = z;
}

void Potential::setActPoint(Point p)
{
    actPoint.x = p.x;
    actPoint.y = p.y;
    actPoint.z = p.z;
}

Point Potential::getGoalPosition()
{
    return goalPosition;
}

Point Potential::getStartPosition()
{
    return startPosition;
}

Point Potential::getRobPos()
{
    return actPoint;
}

bool Potential::goalReached(Point robotPos, Point goalPos, double distError)
{
    return (robotPos.Distance(goalPos) <= distError);
}

/*************************************************************************************************************************/
bool Potential::update_box(Box obstacle[], Box robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        return true;
    }

	//get the movement vector
	Point movementVector = getOverallPotential(robotPos,obstacle,robot,nObst);

	//invert to move in the correct direction
	movementVector.x = movementVector.x * -1;
	movementVector.y = movementVector.y * -1;
	movementVector.z = 0;

	//apply the movement vector
	actPoint = actPoint + movementVector;
	robot[0].Set(actPoint);

    return false;
}

/*************************************************************************************************************************/
bool Potential::update_cylinder(Cylinder obstacle[], Cylinder robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        return true;
    }

   
	//get the movement vector
	Point movementVector = getOverallPotential(robotPos, obstacle, robot, nObst);

	//invert to move in the correct direction
	movementVector.x = movementVector.x * -1;
	movementVector.y = movementVector.y * -1;
	movementVector.z = 0;

	//apply the movement vector
	actPoint = actPoint + movementVector;
	robot[0].SetCenter(actPoint);

    return false;
}

/*************************************************************************************************************************/
bool Potential::update_cylinder_navigation(Cylinder obstacle[], Cylinder robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        //cout << "at goal, smile :)\n";

        return true;
    }

    actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

    return false;
}

Point Potential::attractivPotential(Point q,double scalingFactor,double switchThreshold)
{
	//implemented the combined attactive method

	double distToGoal = (goalPosition - q).Magnitude();

	Point gradient;
	gradient.x = 0;
	gradient.y = 0;
	gradient.z = 0;

	//decide on which potential should be used
	if (distToGoal <= switchThreshold)
	{
		//quadratic potention
		gradient.x = (q.x - goalPosition.x)*scalingFactor;
		gradient.y = (q.y - goalPosition.y)*scalingFactor;

	}
	else
	{
		//conical potential
		gradient.x = ((q.x - goalPosition.x)*scalingFactor*switchThreshold)/ distToGoal;
		gradient.y = ((q.y - goalPosition.y)*scalingFactor*switchThreshold)/ distToGoal;
	}

	return gradient;
}

Point Potential::repulsionToObject(Point q, Box robot, Box object, double theshold, double scalingFactor)
{

	Point repGradient;

	repGradient.x = 0;
	repGradient.y = 0;
	repGradient.z = 0;

	Point* ptr = new Point();
	double distToObject = robot.distance(object, ptr);	
	
	//the center of the object
	Point c = object.getCenter();
	delete ptr;

	//decide if the object is too far away to matter or not
	if (distToObject > theshold)
	{
		//object does not matter
		return repGradient;
	}
	else
	{
		Point gradOfDist;
		gradOfDist.x = (q.x - c.x) / distToObject;
		gradOfDist.y = (q.y - c.y) / distToObject;
		gradOfDist.z = 0;

		repGradient = scalingFactor*((1 / theshold) - (1 / distToObject))*(1 / pow(distToObject, 2))*gradOfDist;
		repGradient.z = 0;
		return repGradient;
	}
}

Point Potential::repulsivePotential(Point q, Box obstacle[], Box robot[], int nObst)
{
	Point r;
	r.x = 0;
	r.y = 0;
	r.z = 0;

	for (int i = 0; i < nObst; i++) {
		r = r + repulsionToObject(q, robot[0], obstacle[i], B_REP_THRES, B_REP_SCALING);
	}

	return r;
}

Point Potential::getOverallPotential(Point q, Box obstacle[], Box robot[], int nObst)
{
	Point potential = repulsivePotential(q, obstacle, robot, nObst) + attractivPotential(q, ATTRAC_SCALING, ATTRAC_THES);
	potential.z = 0;

	return potential;
}


Point Potential::repulsionToObject(Point q, Cylinder robot, Cylinder object, double theshold, double scalingFactor)
{
	Point repGradient;

	repGradient.x = 0;
	repGradient.y = 0;
	repGradient.z = 0;

	Point* ptr = new Point();
	double distToObject = robot.distance(object, ptr);

	//not sure if c is the center of the object or the closest point
	Point c = object.GetCenter();
	delete ptr;

	if (distToObject > theshold)
	{
		return repGradient;
	}
	else
	{
		Point gradOfDist;
		gradOfDist.x = (q.x - c.x) / distToObject;
		gradOfDist.y = (q.y - c.y) / distToObject;
		gradOfDist.z = 0;

		repGradient = scalingFactor*((1 / theshold) - (1 / distToObject))*(1 / pow(distToObject, 2))*gradOfDist;
		repGradient.z = 0;
		return repGradient;
	}
}


Point Potential::getOverallPotential(Point q, Cylinder obstacle[], Cylinder robot[], int nObst)
{	
	Point potential = repulsivePotential(q, obstacle, robot, nObst) + attractivPotential(q, ATTRAC_SCALING, ATTRAC_THES);
	potential.z = 0;

	return potential;
}
Point Potential::repulsivePotential(Point q, Cylinder obstacle[], Cylinder robot[], int nObst)
{
	Point r;
	r.x = 0;
	r.y = 0;
	r.z = 0;

	for (int i = 0; i < nObst; i++) {
		r = r + repulsionToObject(q, robot[0], obstacle[i], C_REP_THRES, C_REP_SCALING);
	}

	return r;
}