/******************************************************************************
    file:      VisibilityGraph.cpp
    created:   2016-10-23
    author:    Thomas Horsch

    description: it is a brute force algorithm O(n^3), testing the visibility
    of each pair of edges
******************************************************************************/

#include <iostream>
#include <fstream>
#include "VisibilityGraph.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <string>
#include <list>
#include <limits> // for numeric_limits
#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>

#include "Dijkstra.h"

#define SOLUTION

using namespace std;

float getEdgeWeigth(Edge e, Graph g)
{
	// a = sqrt(b�+c�)
	Point p1 = g[e.first].pt;
	Point p2 = g[e.second].pt;
	float d = sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2));
	return d;
}

std::vector<MyPolygon> getObsPolys(Graph g, const int nHind)
{
	std::vector<MyPolygon> obsticals;
	for (int i = 0; i < nHind; i++)
	{
		//i iterates through the obsticals
		MyPolygon poly;	
		std::vector<MyPoint> points;

		for (int j = i * 4; j < (i * 4) + 4; j++)
		{
			//j iterates through the points of the obstical
			points.push_back(MyPoint(g[j].pt.x, g[j].pt.y));
		}

		//dont forget to add the first point again to close the polygon
		points.push_back(MyPoint(g[i * 4].pt.x, g[i * 4].pt.y));		

		boost::geometry::assign_points(poly, points);
		obsticals.push_back(poly);
	}

	return obsticals;
}

std::vector<Edge> getInitialEdges(Graph g, const int nHind)
{
	std::vector<Edge> edges;

	for (int i = 0; i < nHind; i++)
	{
		for (int j = i * 4; j < (i * 4) + 3; j++)
		{
			edges.push_back(Edge(j, j + 1));
		}
		//dont forget the last edge, which is formed by the first and the last corner of the obstical
		edges.push_back(Edge((i * 4) + 3,i * 4));
	}

	return edges;
}

std::vector<float> calcEdgeWeigths(Graph g, std::vector<Edge> edges)
{
	std::vector<float> weigths;

	for (int i = 0; i < edges.size(); i++)
	{
		float d = getEdgeWeigth(edges[i], g);
		weigths.push_back(d);
	}

	return weigths;
}

bool goalIsReachable(MyPoint goal, std::vector<MyPolygon> obsticals)
{
	for (int i = 0; i < obsticals.size(); i++)
	{
		if (boost::geometry::within(goal, obsticals[i])) {
			return false;
		}
	}

	return true;
}

vector<Point> VisibilityGraph(Graph g, const int nHind, bool reduce)
{
	std::vector<Point> path;

	//check if start and goal are the same
	if (pointsArequal(g[nHind * 4].pt, g[(nHind * 4) + 1].pt,0.00001))
	{
		return path;
	}	

	//get the edges of the obsticals
	std::vector<Edge> edges= getInitialEdges(g, nHind);

	//build the obsticals from the graph
	std::vector<MyPolygon> obsticals = getObsPolys(g, nHind);

	//check if the goal is reachable
	MyPoint destination = MyPoint(g[(nHind * 4) + 1].pt.x, g[(nHind * 4) + 1].pt.y);	
	if (!goalIsReachable(destination, obsticals))
	{
		std::cout << "goal is unreachable" << std::endl;
		return path;
	}

	//check if the goal can be reached directly
	if (isVisible(g[nHind * 4].pt, g[(nHind * 4) + 1].pt, (nHind * 4), (nHind * 4) + 1, edges, obsticals)) {
		path.push_back(g[(nHind * 4) + 1].pt);
		return path;
	}

	//run the actual visibility graph algo
	getVisibleEdges(g, nHind, edges, obsticals);

	//reduce the graph
	if (reduce) {
		reduceGraph(g, nHind, edges, obsticals, g[(nHind * 4)].pt, g[(nHind * 4) + 1].pt);
	}

	//get the edges weigth
	std::vector<float> weigths = calcEdgeWeigths(g, edges);	

	//run dijkstra
	path = getShortestPath(g, nHind, edges, weigths, (nHind * 4), (nHind * 4) + 1);

	write_gnuplot_file(g, "VisibilityGraph.dat", edges);

    return path;
}

bool polySegIntersection(Point a, Point b, MyPolygon poly) 
{
	float threshold = 0.005;

	//typedefs needed for boost graph
	typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> point_t;
	typedef boost::geometry::model::segment<point_t> Segment;	
	typedef boost::geometry::model::linestring<MyPoint> Linestring;

	//build the segment
	Linestring ls;
	MyPoint p1 = MyPoint(a.x, a.y);
	MyPoint p2 = MyPoint(b.x, b.y);
	ls.push_back(p1);
	ls.push_back(p2);

	std::vector<MyPoint> intersectionPoints;	
	//calc the intersection
	boost::geometry::intersection(ls, poly, intersectionPoints);

	//we have to invastigate the intersections points, because intersections can happen directly on the border of an obstical
	//such an intersections whould be allowed in our case

	if (intersectionPoints.size() > 1)
	{
		//because intersections we allow can only accour at the ends of the segment, so more then one intersection means there was defenetly an obstical in the way
		return true;
	}
	else {
		//check of the intersections are at the end of the segments
		for (int j = 0; j < intersectionPoints.size(); j++)
		{				
			if (pointsArequal(a, intersectionPoints[j],0.01) || pointsArequal(b, intersectionPoints[j], 0.01))
			{				
			}
			else {
				return true;
			}
		}
		return false;
	}
}

bool pointsArequal(Point p1, Point p2,float epsilon)
{
	bool xSame = areEqual(p1.x, p2.x, epsilon);
	bool ySame = areEqual(p1.y, p2.y, epsilon);
	return xSame && ySame;
}

bool pointsArequal(Point p1, MyPoint p2, float epsilon)
{
	bool xSame = areEqual(p1.x, p2.x(), epsilon);
	bool ySame = areEqual(p1.y, p2.y(), epsilon);
	return xSame && ySame;
}

bool areEqual(float a,const float& b,float epsilon)
{
	float diff = fabs(a - b);
	return diff < epsilon;
}


bool isVisible(Point& a, Point& b, int aIndex, int bIndex, std::vector<Edge>& edges, std::vector<MyPolygon>& poly)
{	
	for (int i = 0; i < poly.size(); i++)
	{
		bool intersect = polySegIntersection(a, b, poly[i]);
		if (intersect) {
			return false;
		}
	}
	return true;
}

bool checkIfEdgeIsKnown(int indexA, int indexB, std::vector<Edge> lines)
{
	bool alreadyKnown = false;

	for (int h = 0; h < lines.size(); h++)
	{		
		if (lines[h].first == indexB&&lines[h].second == indexA) {
			alreadyKnown = true;
		}
	}
	return alreadyKnown;
}

void getVisibleEdges(Graph& g, const int nHind, std::vector<Edge>& edges, std::vector<MyPolygon>& poly)
{	

	for (int i = 0; i < nHind; i++)
	{
		//i is the index of an obstrical
		for (int j = (i*4); j < (i*4) + 4; j++)
		{			
			Point& a = g[j].pt;	
			//j iterates through the edges of an obstical
			for (int k = 0; k < (nHind * 4)+2; k++)
			{
				//k goes through all point on the graph
				Point& b = g[k].pt;
			
				//make sure we dont add edges that are part of the same obstical
				if (k >= (i * 4) && k < (i * 4) + 4)
				{					
					//edge is part of the same object, do nothing
				}
				else{

					if (isVisible(a, b, j, k, edges, poly)) {

						//check if this combination is already there in reverse order
						if (!checkIfEdgeIsKnown(j, k, edges))
						{						
							edges.push_back(Edge(j, k));
						}						
					}

				}
			}
		}
	}	
}

void write_gnuplot_file(Graph g, string filename, std::vector<Edge> edges)
{
    ofstream myfile;
    myfile.open(filename);

	myfile << "set title 'Visibility Graph'" << endl;
	myfile <<"set size ratio 1.0"<< endl;
	myfile << "set xrange[0:1]" << endl;
	myfile << "set yrange[0:1]" << endl;

	for (int i = 0; i < edges.size();i++)
	{
		myfile << g[edges[i].first].pt.x << " " << g[edges[i].first].pt.y << endl;
		myfile << g[edges[i].second].pt.x << " " << g[edges[i].second].pt.y << endl << endl;
	}
    myfile.close();
}

std::vector<Point> getShortestPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal)
{
	std::vector<Point> shortestPath;

	//build ajacency matrix
	Dijkstra::adjacency_list_t adjacency_list((nHind * 4) + 2);

	for (int i = 0; i < edges.size(); i++)
	{
		adjacency_list[edges[i].first].push_back(Dijkstra::neighbor(edges[i].second, weights[i]));
		adjacency_list[edges[i].second].push_back(Dijkstra::neighbor(edges[i].first, weights[i]));
	}

	Dijkstra dijkstra;
	std::vector<Dijkstra::weight_t> min_distance;
	std::vector<Dijkstra::vertex_t> previous;
	dijkstra.DijkstraComputePaths(startIndex, adjacency_list, min_distance, previous);
	std::list<Dijkstra::vertex_t> path = dijkstra.DijkstraGetShortestPathTo(goal, previous);

	//convert the path into the format we want
	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		Point p = g[(*it)].pt;
		shortestPath.push_back(p);
	}

	return shortestPath;
}

void reduceGraph(Graph& g, const int nHind, std::vector<Edge>& edges, std::vector<MyPolygon>& poly,Point start,Point goal) {

	vector<Edge> edgesToRemove;
	vector<int>ObsNodeBegin;

	for (int y = 0; y < poly.size();y++) {

		if (y == 0) {
			ObsNodeBegin.push_back(0);
		}
		else {
			ObsNodeBegin.push_back(boost::geometry::num_points(poly[y])-1+ObsNodeBegin[y - 1]);
		}
		
	}

	for (int i = 0; i < edges.size(); i++) {
		int pointIndexStart = edges[i].first, pointIndexEnd = edges[i].second;

		//check if the pointIndexStart and pointIndexEnd are the start or end point
		if (pointIndexStart != nHind*4 && pointIndexStart != (nHind * 4)+1 && pointIndexEnd != nHind * 4 && pointIndexEnd != (nHind * 4) + 1) {
			int obsIndexStart = 0, obsIndexEnd = 0;

			if (pointIndexStart < 4) {
				obsIndexStart = 0;
			}
			else {
				obsIndexStart = pointIndexStart / 4;
			}

			if (pointIndexEnd < 4) {
				obsIndexEnd = 0;
			}
			else {
				obsIndexEnd = pointIndexEnd / 4;
			}

			//check if the edge are not from the same Obs
			if (obsIndexStart != obsIndexEnd) {
				bool isSupportLineResult = isSupportingLine(g, pointIndexStart, pointIndexEnd, obsIndexStart, obsIndexEnd, boost::geometry::num_points(poly[obsIndexStart]) - 1, boost::geometry::num_points(poly[obsIndexEnd]) - 1, ObsNodeBegin);
				bool isSeparatingLineResult = isSeparatingLine(g, pointIndexStart, pointIndexEnd,obsIndexStart, obsIndexEnd, boost::geometry::num_points(poly[obsIndexStart]) - 1, boost::geometry::num_points(poly[obsIndexEnd]) - 1, ObsNodeBegin);

				if (!isSupportLineResult && !isSeparatingLineResult) {				
					edgesToRemove.push_back(edges[i]);
				}
			}
		}
	}
	// delete edges that aren't supporting or separating lines 
	for (int j = 0; j < edgesToRemove.size();j++) {
		for (std::_Vector_iterator<std::_Vector_val<std::_Simple_types<Edge>>> it = edges.begin(); it != edges.end();)
		{
			if (*it._Ptr == edgesToRemove[j]){
				it = edges.erase(it);
				break;
			}
			else {
				++it;
			}

		}
	}
}


bool isSupportingLine(Graph g, const int aPointIndex, const int bPointIndex, const int ObsIndexA, const int ObsIndexB, const int numberOfPointsOfObsa, const int numberOfPointsOfObsb,const vector<int>ObsNodeBegin) {
	Point lineStartPoint = g[aPointIndex].pt, lineEndPoint = g[bPointIndex].pt;
	Point PointFromObsA,PointFromObsB;

	for (int i = 0; i < numberOfPointsOfObsa; i++) {
		//take the first point of obs a

		for (int j = 0; j < numberOfPointsOfObsb; j++) {
			//check the point from obs a with all other points in obs b

			PointFromObsA = g[ObsNodeBegin[ObsIndexA] + i].pt;
			PointFromObsB = g[ObsNodeBegin[ObsIndexB] + j].pt;

			if (!PointFromObsA.equals(lineStartPoint) && !PointFromObsA.equals(lineEndPoint) && !PointFromObsB.equals(lineStartPoint) && !PointFromObsB.equals(lineEndPoint)) {
				if (!isOnTheSameLine(lineStartPoint, lineEndPoint, PointFromObsA, PointFromObsB)) {
					return false;
				}
			}
		}
	}


	return true;
}

bool isSeparatingLine(Graph g, const int aPointIndex, const int bPointIndex, const int ObsIndexA, const int ObsIndexB, const int numberOfPointsOfObsa, const int numberOfPointsOfObsb, const vector<int>ObsNodeBegin) {
	
	Point lineStartPoint = g[aPointIndex].pt, lineEndPoint = g[bPointIndex].pt;
	Point PointFromObsA, PointFromObsB;

	for (int i = 0; i < numberOfPointsOfObsa; i++) {
		//take the first point of obs a

		for (int j = 0; j < numberOfPointsOfObsb; j++) {
			//check the point from obs a with all other points in obs b

			PointFromObsA = g[ObsNodeBegin[ObsIndexA] + i].pt;
			PointFromObsB = g[ObsNodeBegin[ObsIndexB] + j].pt;

			if (!PointFromObsA.equals(lineStartPoint) &&  !PointFromObsA.equals(lineEndPoint) && !PointFromObsB.equals(lineStartPoint) && !PointFromObsB.equals(lineEndPoint)) {
				if (isOnTheSameLine(lineStartPoint, lineEndPoint, PointFromObsA, PointFromObsB)) {
					return false;
				}
			}
		}
	}
	return true;
}

bool isOnTheSameLine(Point beginLine, Point endLine, Point a, Point b) {

	double aResult = (a.y - beginLine.y)*(endLine.x - beginLine.x) - (endLine.y - beginLine.y)*(a.x - beginLine.x);
	double bResult = (b.y - beginLine.y)*(endLine.x - beginLine.x) - (endLine.y - beginLine.y)*(b.x - beginLine.x);

	// sign check
	if ((aResult >= 0 && bResult >= 0) || (aResult < 0 && bResult < 0)) {
		return true;
	}

	if (aResult == 0 || bResult == 0) {
		return true;
	}

	return false;
}