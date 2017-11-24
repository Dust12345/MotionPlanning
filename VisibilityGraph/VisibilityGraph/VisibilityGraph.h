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


//typedefs which are used by the graph boost lib
namespace bg = boost::geometry;
typedef boost::geometry::model::d2::point_xy<double> MyPoint;
typedef boost::geometry::model::polygon<MyPoint> MyPolygon;
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

//returns a path from the start to the goal, path is empty if no such path exists
std::vector<Point> VisibilityGraph(Graph g, const int nHind);

//writes the graph into a gnupolot file !!!DOES NOT WORK !!!
void write_gnuplot_file(Graph g, std::string filename);

//returns the obsticals in the graph as polygons
std::vector<MyPolygon> getObsPolys(Graph g, const int nHind);

// returns the edges which are formed by the obsticals
std::vector<Edge> getInitialEdges(Graph g, const int nHind);

//returns a list of weights that correspons to the list of edges, weigth i is the weigth for edge i and so on
std::vector<float> calcEdgeWeigths(Graph g, std::vector<Edge> edges);

//check wether two floats are equal
bool areEqual(float a, const float& b, float epsilon);

//check wether two points are equal
bool pointsArequal(Point p1, MyPoint p2, float epsilon);
bool pointsArequal(Point p1, Point p2, float epsilon);

//claclulates the weigth of an edge based on it length
float getEdgeWeigth(Edge e, Graph g);

//runs dijkstra and returns the shortest path
std::vector<Point> getShortestPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal);

//checks if the goal is reachable or if it is inside of a polygon
bool goalIsReachable(MyPoint goal, std::vector<MyPolygon> obsticals);

//checks if a polygon an a line intersect. allows for hitting the border of the polygon
bool polySegIntersection(Point a, Point b, MyPolygon poly);

//adds all visible edges to the given edge vector
void getVisibleEdges(Graph& g,const int nHind,std::vector<Edge>& edges, std::vector<MyPolygon>& poly);

//check if the edge is already in reverse order in the list
bool checkIfEdgeIsKnown(int indexA, int indexB, std::vector<Edge> lines);

//checks if the given segment intersects with a polygon, allow for hitting the border
bool isVisible(Point& a, Point& b, int aIndex, int bIndex,std::vector<Edge>& edges, std::vector<MyPolygon>& poly);

#endif /* __VISIBILITYGRAPH_H__ */

