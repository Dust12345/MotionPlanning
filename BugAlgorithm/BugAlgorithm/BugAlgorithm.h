#ifndef __BUGALGORITHM_H__
#define __BUGALGORITHM_H__

#include <iostream>
#include <vector>

#include "Point.h"
#include "Box.h"

using namespace std;

/*********************************************************************************************************************************/
static const double DIST_MIN = 0.01;            //minimum distance between the robot and the goal
static const double DIST_ERROR = DIST_MIN / 2.; //minimum allowed distance between the robot and the obstacle divided by 2
static const double M_PI = 3.1415926;

class BugAlgorithm
{
public:
    enum { CLOCKWISE = true, COUNTERCLOCKWISE = false };

protected:
    bool firstMove; //used to avoid multiple definitions of hit points around and obstacle
    bool direction, leftCheck, rightCheck, hitDefined, firstHit;
    double totalDistance;
    Point goalPosition;     //the goal position
    Point startPosition;    //start point position
    Point heading;          // direction to move
    Point intermediate;     // intermediate Point
    vector<Point> hitPoint;
    vector<Point> leavePoint;
    Point oldPoint;
    Point actPoint;
	double speed;



public:
    BugAlgorithm::BugAlgorithm(const std::string& name);

    // ------ Virtuelle Methoden ----- //
    virtual void goToLeavePoint(Point) {}
    virtual bool researchComplete(Point, Point, Point) { return false; }
    virtual bool leavePointFound() { return false; }
    virtual void findLeavePoint(Point, Point) {}
    virtual void wallFollowing(bool, Box) {};

    void setIntermediatePoint(Point pt);
    void setHeading(Point dir);
    void setHeading(double x, double y, double z = 0.f);
    void setGoalPosition(double x, double y);
    void setStartPosition(double x, double y);
    void setActPoint(Point p);
    Point getGoalPosition();
    Point getStartPosition();
    Point getHeading();
    Point getRobPos();

    // euklidischer Abstand
    double distanceEuclid(Point positionA, Point positionB);

    void setTotalDistance(double value);
    double getTotalDistance();

    bool update(Box obstacle[], Box robot[], int nObst);
    int obstacleInWay(Box obstacle[], Box& robot, int nObst);
    void move();
    void wallFollowingToPoint(Point p, bool dir, Box box);
    void motionToGoal();
    void halt();

    //----- Obstacle Avoidance ----//
    bool completeCycleAroundObstacle(Point robotPos, Point hitPoint);
    void identifyLeavePoint(bool direction, Point robotPos, Point goalPos, Box box);
    bool goalReached(Point robotPos, Point goalPos, double distError);
    Point getFirstHitPoint();
    Point getLastHitPoint();
    Point getFirstLeavePoint();
    Point getLastLeavePoint();
    void recordHitPoint(Point point);
    void recordLeavePoint(Point point);
    bool obstacleInFrontOfTheRobot();
    bool goalIsReachable(Point robotPos, Point goalPos, double distError);

	double getSpeed();
	void setSpeed(double newSpeedValue);

    /****************************************************************************************************
     * Return value: true, if line from point p1, p2 intersects line from point p3, p4
     * if not, the function returns false
     * in case of intersection, the intersection point intersection is calculated,
     * also
     * t1 the parameter value between [0,1]: intersection = p1 + t*(p2-p1)
     * t2 the parameter value between [0,1]: intersection = p3 + t*(p4-p3)
     */
    bool IntersectionLineLine(Point p1, Point p2, Point p3, Point p4, Point *intersection, double *t1, double *t2);

    /****************************************************************************************************
     * Find the points of intersection of a circle and a line segment. There max. 2 intersection points
     * returns number of intersection points
     */
    int FindLineCircleIntersections(Point center, double radius, Point point1, Point point2, Point *intersection1, Point *intersection2);
};

#endif /* __BUGALGORITHM_H__ */
