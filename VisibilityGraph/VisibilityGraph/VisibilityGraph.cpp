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
		MyPolygon poly;	
		std::vector<MyPoint> points;

		for (int j = i * 4; j < (i * 4) + 4; j++) {
			points.push_back(MyPoint(g[j].pt.x, g[j].pt.y));
		}
		points.push_back(MyPoint(g[i * 4].pt.x, g[i * 4].pt.y));		

		boost::geometry::assign_points(poly, points);
		obsticals.push_back(poly);
	}



	return obsticals;
}

std::vector<Edge> getInitialEdges(Graph g, const int nHind)
{
	std::vector<Edge> edges;

	for (int i = 0; i < nHind; i++) {
		//std::cout << "obstical "<< i<< std::endl;

		for (int j = i * 4; j < (i * 4) + 3; j++) {
			edges.push_back(Edge(j, j + 1));
			

			//std::cout << g[j].pt.x << "/" << g[j].pt.y << " to " << g[j+1].pt.x << "/" << g[j+1].pt.y << std::endl;
		}

		//std::cout << g[i * 4].pt.x << "/" << g[i * 4].pt.y << " to " << g[(i * 4) + 3].pt.x << "/" << g[(i * 4) + 3].pt.y << std::endl;
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

vector<Point> VisibilityGraph(Graph g, const int nHind,Point startPos, Point goal)
{
	std::vector<Edge> edges= getInitialEdges(g, nHind);
	std::vector<MyPolygon> obsticals = getObsPolys(g, nHind);	

	getVisibleEdges(g, startPos, nHind, goal, edges, obsticals);
	
	for (int i = 0; i < edges.size(); i++)
	{
		Edge& e = edges[i];
		//std::cout << g[e.first].pt.x << "/" << g[e.first].pt.y << " to " << g[e.second].pt.x << "/" << g[e.second].pt.y <<std::endl;
	}

	

	std::vector<float> weigths = calcEdgeWeigths(g, edges);	

	std::vector<Point> path = getShortestPath(g, nHind, edges, weigths, (nHind * 4), (nHind * 4) + 1);

	for (int i = 0; i < path.size(); i++) {
		//std::cout << path[i].x << "/" << path[i].y << std::endl;
	}

	

	write_gnuplot_file(g, "VisibilityGraph.dat");

    return path;
}

bool polySegmentIntersect(Point a, Point b, MyPolygon poly) 
{
	float threshold = 0.005;

	typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> point_t;

	typedef boost::geometry::model::segment<point_t> Segment;

	
	typedef boost::geometry::model::linestring<MyPoint> Linestring;

	Linestring ls;

	MyPoint p1 = MyPoint(a.x, a.y);
	MyPoint p2 = MyPoint(b.x, b.y);

	//std::cout << a.x << "/" << a.y << " to " << b.x << "/" << b.y << std::endl;

	ls.push_back(p1);
	ls.push_back(p2);

	std::vector<MyPoint> result;	

	boost::geometry::intersection(ls, poly, result);

	if (result.size() > 2) {
		return true;
	}
	else {
		//check of the intersections are at the end of the segments
		for (int j = 0; j < result.size(); j++)
		{
			
				
			if (pointsArequal(a, result[j]) || pointsArequal(b, result[j])) {
				
			}
			else {
				return true;
			}
		}
		return false;
	}
}

bool pointsArequal(Point p1, MyPoint p2) {

	float epsilon = 0.01;	

	bool xSame = areEqual(p1.x, p2.x(), epsilon);
	bool ySame = areEqual(p1.y, p2.y(), epsilon);

	return xSame && ySame;
}

bool areEqual(float a,const float& b,float epsilon)
{
	float diff = fabs(a - b);

	return diff < epsilon;
}


bool isVisible(Graph& g, Point& a, Point& b, int aIndex, int bIndex, std::vector<Edge>& edges, std::vector<MyPolygon>& poly)
{
	//int i = 0;
	for (int i = 0; i < poly.size(); i++)
	{
		bool result = polySegmentIntersect(a, b, poly[i]);
		if (result) {
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

std::vector<Edge> getVisibleEdges(Graph& g, Point startPoint, const int nHind, Point goal, std::vector<Edge>& edges, std::vector<MyPolygon>& poly)
{
	std::vector<Edge> lines;	

	

	for (int i = 0; i < nHind; i++)
	{
		
		

		//i is the index of an obstrical
		for (int j = (i*4); j < (i*4) + 4; j++)
		{
	
			
			Point& a = g[j].pt;

		

			//int k = 4;

			//j iterates through the edges of an obstical
			for (int k = 0; k < (nHind * 4)+2; k++)
			{
				//k goes through all point on the graph
				Point& b = g[k].pt;

			
				//make sure we dont add edges that are part of the same obstical
				if (k >= (i * 4) && k < (i * 4) + 4)
				{
					
					//edge is part of the same object
				}
				else{

					bool lineIsVisible = isVisible(g, a, b,j,k, edges,poly);

					if (lineIsVisible) {
	
						

						//check if this combination is already there in reverse order
						bool alreadyKnown = checkIfEdgeIsKnown(j, k, lines);

						if (!alreadyKnown)
						{
							lines.push_back(Edge(j, k));
							edges.push_back(Edge(j, k));
						}
						
					}
					else {
					
					}
				}
			}
		}
	}

	return lines;
}

/**************************************************************************/
// Ausgabe einer Plotdatei für gnuplot:
// Aufruf in gnuplot: plot 'visibilitygraph.data' using 1:2 with lines
void write_gnuplot_file(Graph g, string filename)
{
    ofstream myfile;
    myfile.open(filename);

    // Iterate through the edges and print them out
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> ep;
    edge_iter ei, ei_end;

    int cnt = 0; // edge counter

    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
        myfile << g[ei->m_source].pt.x << " " << g[ei->m_source].pt.y << endl;
        myfile << g[ei->m_target].pt.x << " " << g[ei->m_target].pt.y << endl << endl;
        cnt++;
    }

    cout << "Number of edges: " << cnt <<  endl;
    myfile.close();
}

std::vector<Point> getShortestPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal)
{
	Dijkstra::adjacency_list_t adjacency_list((nHind * 4) + 2);

	std::vector<Point> pathToGo;

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

	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		Point p = g[(*it)].pt;
		pathToGo.push_back(p);
	}

	return pathToGo;
}