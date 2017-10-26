#ifndef __CYLINDER_H__
#define __CYLINDER_H__

#include "Point.h"

class Cylinder
{
public:
    // Constructor / Destructor
    Cylinder::Cylinder();
    Cylinder(const Point p, double pr, double ph);
    ~Cylinder();

    // Data access
    Point& GetCenter(); // gets the coordinates of the i-th vertex of the cube
    void SetCenter(const Point);
    void SetCenter(double x, double y, double z); // sets the origin of box to x,y,z
    void SetRadius(const double pr);
    double GetRadius(void);
    void Translate(double x, double y, double z); // translates the box with coordinates x,y,z
    void Translate(Point p);

    //void Set(const Point p);
    //Point& getLowestDistancePoint();
    //Point& getLowestDistanceVector();
    double distance(Cylinder cyl2, Point *);
    double distance_sqr(Cylinder cyl2);

protected:
    Point center;               // center of cylinder
    double r, h;                // radius and height of cylinder
    //Point lowestDistancePoint;  // Point with the lowest distance to the robot
    //Point lowestDistanceVector; // Vector from the lowest distance point to the robot lowest distance point
};

#endif /* __CYLINDER_H__ */
