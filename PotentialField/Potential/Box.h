#ifndef __BOX_H__
#define __BOX_H__

#include "Point.h"

class Box
{
public:
    // Constructor / Destructor
    Box(); // contructs a unit cube at the origin
    ~Box();

    // Data access
    Point& GetVertex(int i); // gets the coordinates of the i-th vertex of the cube
    void SetVertex(int i, const Point);

    void Scale(double sx, double sy, double sz); // scales the box
    void Translate(double x, double y, double z ); // translates the box with coordinates x,y,z
    void Translate(Point p);
    void Set(double x, double y, double z); // sets the origin of box to x,y,z
    void Set(const Point p);
    Box MinkowskiDifference(const Box box); // builds the Minkowsky Difference this - box
    int getNextVertex();
    void setNextVertex(int index);
    Point& getLowestDistancePoint();
    double distance(Box box2, Point *pt);

	

	
	Point getCenter();

protected:
    Point m_Verts[8];           // vertices of the box
    Point p;                    // length of each side of the box (p.x, p.y, p.z)
    Point lowestDistancePoint;  // Point with the lowest distance to the robot
    int nextVertex;             // index of next vertex wrt lowestDistnacePoint when travelling CCW

	Point center;
};

#endif /* __BOX_H__ */
