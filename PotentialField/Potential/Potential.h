#ifndef __POTENTIAL_H__
#define __POTENTIAL_H__

#include <string>


#include "Point.h"
#include "Box.h"
#include "Cylinder.h"

class Potential
{
public:
    enum Object { BOX, CYLINDER };

protected:
    Point goalPosition;     // the goal position
    Point startPosition;    // start point position
    Point actPoint;

public:
    Potential(const std::string& name);

    void setGoalPosition(double x, double y, double z = 0.);
    void setStartPosition(double x, double y, double z = 0.);
    void setActPoint(Point p);
    Point getGoalPosition();
    Point getStartPosition();
    Point getRobPos();

    bool goalReached(Point robotPos, Point goalPos, double distError);

    bool update_box(Box obstacle[], Box robot[], int nObst);
    bool update_cylinder(Cylinder obstacle[], Cylinder robot[], int nObst);
    bool update_cylinder_navigation(Cylinder obstacle[], Cylinder robot[], int nObst);

private:

	Point repulsionToObject(Point q, Box robot,Box object,double theshold, double scalingFactor);
	Point getOverallPotential(Point q, Box obstacle[], Box robot[], int nObst);
	Point attractivPotential(Point q, double scalingFactor, double switchThreshold);
	Point repulsivePotential(Point q, Box obstacle[], Box robot[], int nObst);

	Point repulsionToObject(Point q, Cylinder robot, Cylinder object, double theshold, double scalingFactor);
	Point getOverallPotential(Point q, Cylinder obstacle[], Cylinder robot[], int nObst);
	Point repulsivePotential(Point q, Cylinder obstacle[], Cylinder robot[], int nObst);

};

#endif /* __POTENTIAL_H__ */
