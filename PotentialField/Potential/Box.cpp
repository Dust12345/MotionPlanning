/******************************************************************************
    file:       Box.cpp
    created:    12.4.2004
******************************************************************************/
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include "Box.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Box::Box( )
{
    // Vertices ( Eckpunkte )
    m_Verts[0].Set( 0.0, 0.0, 0.0);
    m_Verts[1].Set( 1.0, 0.0, 0.0);
    m_Verts[2].Set( 1.0, 1.0, 0.0);
    m_Verts[3].Set( 0.0, 1.0, 0.0);
    m_Verts[4].Set( 0.0, 0.0, 1.0 );
    m_Verts[5].Set( 1.0, 0.0, 1.0 );
    m_Verts[6].Set( 1.0, 1.0, 1.0 );
    m_Verts[7].Set( 0.0, 1.0, 1.0 );
    p.x = p.y = p.z = 1.0;
	center = Point();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Box::~Box()
{ }

Point& Box::GetVertex(int i)
{
    return m_Verts[i];
}

Point Box::getCenter()
{
	return center;
}

void Box::SetVertex(int i, const Point pt)
{
    m_Verts[i] = pt;
}

// Returns the MinkowskiDifference of a axis aligned box (AABB), the result is also a AABB
Box Box::MinkowskiDifference(const Box box)
{
    Box minkowski_difference(*this);
    Point p1(-box.p.x, 0.0, 0.0);
    Point p2(0.0, -box.p.y, 0.0);

    minkowski_difference.SetVertex(0, minkowski_difference.GetVertex(0) + p1 + p2);
    minkowski_difference.SetVertex(1, minkowski_difference.GetVertex(1) + p2);
    minkowski_difference.SetVertex(3, minkowski_difference.GetVertex(3) + p1);

    minkowski_difference.nextVertex = this->nextVertex;

    minkowski_difference.p = this->p + box.p;
    //minkowski_difference.p.y = this->p.y;
    //minkowski_difference.p.z = this->p.z;

    return (minkowski_difference);
}

void Box::Scale(double sx, double sy, double sz)
{
    // Skalieren
    p.x = sx;
    p.y = sy;
    p.z = sz;

    for (int i = 0; i < 8; ++i)
    {
        m_Verts[i].x *= sx;
        m_Verts[i].y *= sy;
        m_Verts[i].z *= sz;
    }
}

void Box::Translate(double x, double y, double z)
{
	center.x = center.x + x;
	center.y = center.y + y;
	center.z = 0;

    // Verschieben
    for (int i = 0; i < 8; ++i)
    {
        m_Verts[i].x += x;
        m_Verts[i].y += y;
        m_Verts[i].z += z;
    }
}

void Box::Translate(Point p)
{
	center.x = center.x + p.x;
	center.y = center.y + p.y;
	center.z = 0;

    // Verschieben
    for (int i = 0; i < 8; ++i)
    {
        m_Verts[i].x += p.x;
        m_Verts[i].y += p.y;
        m_Verts[i].z += p.z;
    }
}

void Box::Set(const Point pt)
{
	center.x = pt.x;
	center.y = pt.y;
	center.z = 0;

    // Setzen
    m_Verts[0] = pt + Point(0.0, 0.0, 0.0);
    m_Verts[1] = pt + Point(p.x, 0.0, 0.0);
    m_Verts[2] = pt + Point(p.x, p.y, 0.0);
    m_Verts[3] = pt + Point(0.0, p.y, 0.0);
    m_Verts[4] = pt + Point(0.0, 0.0, p.z);
    m_Verts[5] = pt + Point(p.x, 0.0, p.z);
    m_Verts[6] = pt + Point(p.x, p.y, p.z);
    m_Verts[7] = pt + Point(0.0, p.y, p.z);
}

void Box::Set(double px, double py, double pz)
{
	center.x = px;
	center.y = py;
	center.z = 0;

    // Setzen
    Point pt(px, py, pz);
    m_Verts[0] = pt + Point(0.0, 0.0, 0.0);
    m_Verts[1] = pt + Point(p.x, 0.0, 0.0);
    m_Verts[2] = pt + Point(p.x, p.y, 0.0);
    m_Verts[3] = pt + Point(0.0, p.y, 0.0);
    m_Verts[4] = pt + Point(0.0, 0.0, p.z);
    m_Verts[5] = pt + Point(p.x, 0.0, p.z);
    m_Verts[6] = pt + Point(p.x, p.y, p.z);
    m_Verts[7] = pt + Point(0.0, p.y, p.z);
}

int Box::getNextVertex()
{
    return nextVertex;
}

void Box::setNextVertex(int index)
{
    nextVertex = index;
}

Point& Box::getLowestDistancePoint()
{
    return(lowestDistancePoint);
}

/*********************************************************************************************************************
 * distance()
 * Diese Funktion berechnet den minimalen Abstand der beiden Objekte, box1 und box2
 * und gibt den kürzesten Abstandsvektor zurück
 */
double Box::distance(Box robot, Point *pt)
{
    int dist3(int nvi, int nvj, double center_i[], double center_j[], double zi[][3], double zj[][3], int iwant[], double eps,
        double zisol[], double zjsol[], double zsol[], double *dist, int *nv_s,
        int ris[], int rjs[], double als[], int *ncy, double *gfinal, double zbi[][3],
        double zbj[][3], double neworg[], double *di, double *dj, int *ierror);

    //void gilbert(int nvi, int nvj, double[][3], double[][3], double zisol[3], double zjsol[3]);
    double gfinal, di, dj, zi[8][3], zj[8][3], zbi[8][3], zbj[8][3], zisol[3], zjsol[3], zsol[3], dist;
    double center_i[3], center_j[3], new_org[3];
    int ierror, iwant[5] = { 0,0,0,0,0 };
    int ncy, nv_s, ris[5], rjs[5];
    double als[5];
    int max_index = 0; // max. Index der Konvexkombination im Hindernis (rjs[])
    int cnt = 0; // Anzahl der Indices kleiner 4 (untere Fläche von der Box)

    for (int i = 0; i < 8; i++)
    {
        zi[i][0] = this->GetVertex(i).x; // obstacle
        zi[i][1] = this->GetVertex(i).y;
        zi[i][2] = this->GetVertex(i).z;
        zj[i][0] = robot.GetVertex(i).x;
        zj[i][1] = robot.GetVertex(i).y;
        zj[i][2] = robot.GetVertex(i).z;
    }

    int ret = dist3(8, 8, center_i, center_j, zi, zj, iwant, 0.000005, zisol, zjsol, zsol, &dist,
        &nv_s, ris, rjs, als, &ncy, &gfinal, zbi, zbj, new_org, &di, &dj, &ierror);

    lowestDistancePoint.x = zisol[0];
    lowestDistancePoint.y = zisol[1];
    lowestDistancePoint.z = 0.0;

    //double min_dist = 1e16;
    //int index = 0;
    //for (int i = 0; i < 4; i++)
    //{
    //  double d = m_Verts[i].SquareDistance(lowestDistancePoint);
    //  if (d < min_dist)
    //  {
    //      min_dist = d;
    //      index = i;
    //  }
    //}
    //nextVertex = index;
    //  lowestDistancePoint.z = zisol[2];

    pt->x = -zsol[0];
    pt->y = -zsol[1];
    //pt->z = -zsol[2];
    pt->z = 0.0;

    double alpha = atan2(pt->y, pt->x);
    nextVertex = 0; // set default next vertex (direction down)
    if (fabs (alpha) < 0.01 )
    {
        nextVertex = 2; // set next vertex if pt directs up
    }
    else if (fabs(alpha - M_PI_2) < 0.01)
    {
        nextVertex = 3; // set next vertex if pt directs left
    }
    else if (fabs(alpha + M_PI_2) < 0.01)
    {
        nextVertex = 1; // set next vertex if pt directs rigth
    }

    //cout << "Distance: " << dist << " Nahpkt: " << pt->x << " " << pt->y << " " << pt->z << endl;
    //for (int i = 0; i < nv_s; i++)
    //{
    //  if (ris[i] < 4)
    //  { // only indices 0-3 are investigated
    //      if (ris[i] > max_index)
    //      {
    //          max_index = ris[i];
    //          cnt++;
    //      }
    //  }
    //}
    //nextVertex = max_index + 1;
    //
    //if (cnt == 0)
    //  nextVertex = (max_index + 1) % 4;

    return dist;
}
