/******************************************************************************
file:      VisibilityGraph.h
created:   2016-10-23
author:    Thomas Horsch

description: it is a brute force algorithm O(n^3), testing the visibility
of each pair of edges
******************************************************************************/
#ifndef __VISIBILITYGRAPH_H__
#define __VISIBILITYGRAPH_H__

#include <vector>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "Point.h"

#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/geometry/geometries/segment.hpp> 
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <deque>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/geometry/geometries/register/point.hpp>

// additional Information for each vertex
struct VertexProperty
{
    Point pt; // point coordinates of vertex
};


namespace bg = boost::geometry;

struct MyPoint {
	double x, y;

	
};




typedef boost::geometry::model::d2::point_xy<double> pp;

typedef boost::geometry::model::polygon<pp> MyPolygon;

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

std::vector<Point> VisibilityGraph(Graph g, const int nHind, Point startPos, Point goal);
void write_gnuplot_file(Graph g, std::string filename);

void test();

bool areEqual(float a, const float& b, float epsilon);
bool pointsArequal(Point p1, pp p2);

float getEdgeW(Edge e, Graph g);

std::vector<Point> getPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal);

bool polySegmentIntersect(Point a, Point b, MyPolygon poly);

std::vector<Edge>  getVisibleEdges(Graph& g,Point startPoint,const int nHind, Point goal,std::vector<Edge>& edges, std::vector<MyPolygon>& poly);

bool segmentIntersect(Point a, Point b, Point c, Point d);
bool isVisible(Graph& g,Point& a, Point& b, int aIndex, int bIndex,std::vector<Edge>& edges, std::vector<MyPolygon>& poly);

#endif /* __VISIBILITYGRAPH_H__ */

