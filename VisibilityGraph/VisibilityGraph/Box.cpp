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

