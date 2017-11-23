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

typedef boost::geometry::model::d2::point_xy<double> MyPoint;
typedef boost::geometry::model::polygon<MyPoint> MyPolygon;
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

std::vector<Point> VisibilityGraph(Graph g, const int nHind, Point startPos, Point goal);
void write_gnuplot_file(Graph g, std::string filename);

std::vector<MyPolygon> getObsPolys(Graph g, const int nHind);
std::vector<Edge> getInitialEdges(Graph g, const int nHind);
std::vector<float> calcEdgeWeigths(Graph g, std::vector<Edge> edges);

bool areEqual(float a, const float& b, float epsilon);
bool pointsArequal(Point p1, MyPoint p2);

float getEdgeWeigth(Edge e, Graph g);

std::vector<Point> getShortestPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal);

bool polySegmentIntersect(Point a, Point b, MyPolygon poly);

std::vector<Edge>  getVisibleEdges(Graph& g,Point startPoint,const int nHind, Point goal,std::vector<Edge>& edges, std::vector<MyPolygon>& poly);

bool checkIfEdgeIsKnown(int indexA, int indexB, std::vector<Edge> lines);

bool isVisible(Graph& g,Point& a, Point& b, int aIndex, int bIndex,std::vector<Edge>& edges, std::vector<MyPolygon>& poly);

std::vector<Point> getShortestPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal);

#endif /* __VISIBILITYGRAPH_H__ */

