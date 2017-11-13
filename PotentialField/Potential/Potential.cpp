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

static const double B_REP_THRES = 0.15;
static const double B_REP_SCALING = 0.0000000000020;

static const double ATTRAC_SCALING_C = 0.08;
static const double ATTRAC_THES_C = 0.2;

static const double ATTRAC_SCALING_B = 0.008;
static const double ATTRAC_THES_B = 0.2;

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

	//get the movement vector based on the gradient at the current position
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
	const double K = 12;

	cnt++;

	if (goalReached(robotPos, goalPosition, GOAL_ERROR))
	{
		actPoint = goalPosition;
		return true;
	}

	/*Part 1 Formula: 2(q - qgoal) * [d(q - qgoal)^2K + ß(q)]^1/K *********************************************************************************/
	Point partOneFormula;
	partOneFormula.x = 2 * (robotPos.x - goalPosition.x);
	partOneFormula.y = 2 * (robotPos.y - goalPosition.y);
	partOneFormula.z = 0;

	partOneFormula.x = partOneFormula.x * (pow(pow(robotPos.Distance(goalPosition), 2 * K) + navigationDistanceToAllObstacles(obstacle, robot[0], nObst), 1 / K));
	partOneFormula.y = partOneFormula.y * (pow(pow(robotPos.Distance(goalPosition), 2 * K) + navigationDistanceToAllObstacles(obstacle, robot[0], nObst), 1 / K));


	/*Part 2 Formula: d^2(g,goal) * 1/K * [d(q,qgoal)^2K + ß(q)]^1/K-1 *********************************************************************************/
	double partTwoFormula;

	double partTwoFirstProduct = robotPos.SquareDistance(goalPosition)*(1 / K);

	double partTwoSecondProduct = pow(pow(robotPos.Distance(goalPosition), 2 * K) + navigationDistanceToAllObstacles(obstacle, robot[0], nObst), (1 / K) - 1);

	partTwoFormula = partTwoFirstProduct * partTwoSecondProduct;

	/*Part 3 [2 * K * d(q,qgoal)^2*K-2 * (q - qgoal) + gradient ß(g)] *********************************************************************************/
	Point partThreeFormula;
	double partThreeResultOne;
	Point partThreeResultTwo;

	partThreeResultOne = 2 * K*pow(robotPos.Distance(goalPosition), (2 * K) - 2);

	partThreeResultTwo.x = robotPos.x - goalPosition.x;
	partThreeResultTwo.y = robotPos.y - goalPosition.y;

	partThreeFormula.x = partThreeResultTwo.x * partThreeResultOne;
	partThreeFormula.y = partThreeResultTwo.y * partThreeResultOne;

	Point partThreeGradientBeta = navigationGradientDistanceToAllObstacle(robot[0], obstacle, nObst);

	partThreeFormula.x = partThreeFormula.x + partThreeGradientBeta.x;
	partThreeFormula.y = partThreeFormula.y + partThreeGradientBeta.y;
	partThreeFormula.z = 0;

	/*Part 4 [d (q,qgoal)^2*K +ß(g)]^2/K *********************************************************************************/
	double partFour = pow(robotPos.Distance(goalPosition), 2 * K) + navigationDistanceToAllObstacles(obstacle, robot[0], nObst);
	partFour = pow(partFour, 2 / K);

	/*Result *********************************************************************************/
	Point movementVector;
	movementVector.x = partThreeFormula.x * partTwoFormula;
	movementVector.y = partThreeFormula.y * partTwoFormula;

	movementVector.x = partOneFormula.x - movementVector.x;
	movementVector.y = partOneFormula.y - movementVector.y;

	movementVector.x = movementVector.x / partFour;
	movementVector.y = movementVector.y / partFour;
	movementVector.z = 0;

	//invert to move in the correct direction
	movementVector.x = movementVector.x * -1;
	movementVector.y = movementVector.y * -1;
	movementVector.z = 0;

	actPoint.Mac(movementVector.Normalize(), INKR);
	robot[0].SetCenter(actPoint);

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
	
	//the closest point of the obstical to the robot
	Point c = q + (*ptr);
	delete ptr;

	//decide if the object is too far away to matter or not
	if (distToObject > theshold)
	{
		//object does not matter
		return repGradient;
	}
	else
	{
		//object is close enougth to matter
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

	//sum up the repulsive force of all obsticals
	for (int i = 0; i < nObst; i++) {
		r = r + repulsionToObject(q, robot[0], obstacle[i], B_REP_THRES, B_REP_SCALING);
	}

	return r;
}

Point Potential::getOverallPotential(Point q, Box obstacle[], Box robot[], int nObst)
{
	Point potential = repulsivePotential(q, obstacle, robot, nObst) + attractivPotential(q, ATTRAC_SCALING_B, ATTRAC_THES_B);
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

	
	Point c =  object.GetCenter();
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
	Point potential = repulsivePotential(q, obstacle, robot, nObst) + attractivPotential(q, ATTRAC_SCALING_C, ATTRAC_THES_C);
	potential.z = 0;

	return potential;
}
Point Potential::repulsivePotential(Point q, Cylinder obstacle[], Cylinder robot[], int nObst)
{
	Point r;
	r.x = 0;
	r.y = 0;
	r.z = 0;

	//sum up the repulsive force of all obsticals
	for (int i = 0; i < nObst; i++) {
		r = r + repulsionToObject(q, robot[0], obstacle[i], C_REP_THRES, C_REP_SCALING);
	}

	return r;
}

double Potential::navigationDistanceToObstacle(Cylinder robot, Cylinder obstacle, int obstacleIndex)
{
	Point pt = robot.GetCenter();
	if (obstacleIndex == 0) {

		// Formula: -d^2 (q,qi) + ri^2
		return -pt.Sub(obstacle.GetCenter()).SquareMagnitude() + (pow(obstacle.GetRadius() + robot.GetRadius(), 2));
	}
	else {
		// Formula: d^2 (q,qi) - ri^2
		return pt.Sub(obstacle.GetCenter()).SquareMagnitude() - (pow(obstacle.GetRadius() + robot.GetRadius(), 2));
	}
}

double Potential::navigationDistanceToAllObstacles(Cylinder obstacle[], Cylinder robot, int nObst)
{
	double product = 1;

	// Formula: ß(q) = Π ßi(q) 
	for (int i = 0; i < nObst; i++) {
		product = product * navigationDistanceToObstacle(robot, obstacle[i], i);
	}

	return product;
}

double Potential::navigationDistanceToAllObstacles(Cylinder obstacle[], Cylinder robot, int nObst, int ignore)
{
	double product = 1;

	// Formula: ß(q) = Π ßi(q)
	for (int i = 0; i < nObst; i++) {

		if (i != ignore) {
			product = product * navigationDistanceToObstacle(robot, obstacle[i], i);
		}
	}
	return product;
}

Point Potential::navigationGradientDistanceToObstacle(Cylinder robot, Cylinder obstacle, int obstacleIndex) {
	Point point;

	Point roboPointC = robot.GetCenter();
	Point obstaclePointC = obstacle.GetCenter();

	if (obstacleIndex == 0) {
		// Formula: -2(q-qi)
		point.x = -2 * (roboPointC.x - obstaclePointC.x);
		point.y = -2 * (roboPointC.y - obstaclePointC.y);
		point.z = 0;
		return point;
	}
	else {
		// Formula: 2(q-qi)
		point.x = 2 * (roboPointC.x - obstaclePointC.x);
		point.y = 2 * (roboPointC.y - obstaclePointC.y);
		point.z = 0;
		return point;
	}
}

Point Potential::navigationGradientDistanceToAllObstacle(Cylinder robot, Cylinder obstacle[], int obstacleNumber) {
	Point sum(0.0, 0.0, 0.0);

	// Formula: Sum ßi(q) * Product ßi(q)  Condition: i != j 
	for (int i = 0; i < obstacleNumber; i++) {
		Point tmp = navigationGradientDistanceToObstacle(robot, obstacle[i], i);
		double tmpDeltaDistanceProduct = navigationDistanceToAllObstacles(obstacle, robot, obstacleNumber, i);
		sum = sum.Add(tmp.Mult(tmpDeltaDistanceProduct));
		sum.z = 0;
	}
	return sum;
}
