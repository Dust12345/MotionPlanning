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

#define SOLUTION

using namespace std;

float getEdgeW(Edge e, Graph g)
{
	Point p1 = g[e.first].pt;
	Point p2 = g[e.second].pt;

	float d = sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2));

	return d;
}


vector<Point> VisibilityGraph(Graph g, const int nHind,Point startPos, Point goal)
{
    vector<Point> path; // create a point vector for storing the path

    // Example for access to the coordinates of the vertices
    for (int i = 0; i < nHind * 4 + 2; i++)
    {
        cout << g[i].pt.x << " " << g[i].pt.y << endl;
    }	

	std::vector<Edge> edges;
	std::vector<MyPolygon> obsticals;
	std::vector<EdgeWeightProperty> ew;


	for (int i = 0; i < nHind; i++)
	{
		MyPolygon poly;

		std::vector<pp> points;
	
			

		for (int j = i * 4; j < (i * 4) + 4; j++) {
			points.push_back(pp(g[j].pt.x, g[j].pt.y));
		}
		points.push_back(pp(g[i * 4].pt.x, g[i * 4].pt.y));
		boost::geometry::assign_points(poly, points);
		obsticals.push_back(poly);
	}

	for (int i = 0; i < nHind; i++) {
		for (int j = i * 4; j < (i * 4) + 3; j++) {
			edges.push_back(Edge(j, j + 1));
		}

		edges.push_back(Edge(i * 4, (i*4)+4));
	}	
	

	getVisibleEdges(g, startPos, nHind, goal, edges, obsticals);
	
	std::vector<float> weigths;

	for (int i = 0; i < edges.size(); i++)
	{
		float d = getEdgeW(edges[i], g);
		weigths.push_back(d);
	}	

	std::vector<Point> mpath = getPath(g, nHind, edges, weigths, 0, (nHind * 4) + 1);



	for (int i = 0; i < mpath.size(); i++)
	{
		//std::cout << mpath[i].x << "/" << mpath[i].y << std::endl;
	}
	


    //write_gnuplot_file(g, "VisibilityGraph.dat");

	

    return mpath;
}

bool polySegmentIntersect(Point a, Point b, MyPolygon poly) 
{
	float threshold = 0.005;

	typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> point_t;

	typedef boost::geometry::model::segment<point_t> Segment;
	Segment AB(point_t(a.x, a.y), point_t(b.x, b.y));
	
	typedef boost::geometry::model::linestring<pp> Linestring;

	Linestring ls;

	pp p1 = pp(a.x, a.y);
	pp p2 = pp(b.x, b.y);

	

	ls.push_back(p1);
	ls.push_back(p2);

	std::vector<pp> result;

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

bool pointsArequal(Point p1, pp p2) {

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

bool segmentIntersect(Point a, Point b,Point c, Point d)
{

	

	//code from here
	//https://stackoverflow.com/questions/19867265/using-boost-geometry-to-check-if-two-lines-have-an-intersection


	typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> point_t;	

	typedef boost::geometry::model::segment<point_t> Segment;
	Segment AB(point_t(a.x, a.y), point_t(b.x, b.y));
	Segment CD(point_t(c.x, c.y), point_t(d.x, d.y));

	bool result = boost::geometry::intersects(AB, CD);

	return result;
}

bool isVisible(Graph& g, Point& a, Point& b, int aIndex, int bIndex, std::vector<Edge>& edges, std::vector<MyPolygon>& poly)
{
	for (int i = 0; i < poly.size(); i++)
	{
		bool result = polySegmentIntersect(a, b, poly[i]);
		if (result) {
			return false;
		}
	}

	return true;
}

std::vector<Edge> getVisibleEdges(Graph& g, Point startPoint, const int nHind, Point goal, std::vector<Edge>& edges, std::vector<MyPolygon>& poly)
{

	std::vector<Edge> lines;
	
	/*int j = 7;
	int k = 10;
	Point& a = g[j].pt;
	Point& b = g[k].pt;
	bool r = isVisible(g, a, b, j, k, edges,poly);

	std::cout << "should be false, but is " << r << std::endl;

	return lines;*/

	for (int i = 0; i < nHind; i++) {
		
		//i is the index of an obstrical
		for (int j = (i*4); j < (i*4) + 4; j++) {
			Point& a = g[j].pt;
			//j iterates through the edges of an obstical
			for (int k = 0; k < (nHind * 4)+2; k++) {
				//k goes through all point on the graph
				Point& b = g[k].pt;
				//make sure we dont add edges that are part of the same obstical
				if (k >= (i * 4) && k < (i * 4) + 4)
				{
					//edge is part of the same object
				}
				else {
//					std::cout << "Combination " << j << " to " << k << std::endl;

					bool r = isVisible(g, a, b,j,k, edges,poly);

					if (r) {
	//					std::cout << "Combination " << j << " to " << k << std::endl;
						//if (j == 7 && k == 10)
						{
							//std::cout << "found at " << g[j].pt.x << "/" << g[j].pt.y << " and " << g[k].pt.x << "/" << g[k].pt.y << std::endl;
						}
						
						//check if this combination is already there in reverse order

						bool alreadyKnown = false;

						for (int h = 0; h < lines.size(); h++)
						{
							if (lines[h].first == k&&lines[h].second == j) {
								alreadyKnown = true;
							}
						}

						if (!alreadyKnown)
						{
							lines.push_back(Edge(j, k));
							edges.push_back(Edge(j, k));
						}
						
					}
				}
			}
		}
	}

	//start and goal are done in a seperate loop
	/*for (int i = nHind * 4; i < (nHind * 4) + 2; i++) {
		Point& a = g[i].pt;
		
		for (int k = 0; k < nHind * 4; k++)
		{
			Point& b = g[k].pt;
			bool r = isVisible(g, a, b, edges);
			if (r) {
				lines.push_back(Edge(i, k));
			}
		}
	}*/



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










#include <iostream>
#include <vector>
#include <string>
#include <list>

#include <limits> // for numeric_limits

#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>


//code for dijkstra taken from:
//https://rosettacode.org/wiki/Dijkstra%27s_algorithm#C.2B.2B

typedef int vertex_t;
typedef double weight_t;

const weight_t max_weight = std::numeric_limits<double>::infinity();

struct neighbor {
	vertex_t target;
	weight_t weight;
	neighbor(vertex_t arg_target, weight_t arg_weight)
		: target(arg_target), weight(arg_weight) { }
};

typedef std::vector<std::vector<neighbor> > adjacency_list_t;


void DijkstraComputePaths(vertex_t source,
	const adjacency_list_t &adjacency_list,
	std::vector<weight_t> &min_distance,
	std::vector<vertex_t> &previous)
{
	int n = adjacency_list.size();
	min_distance.clear();
	min_distance.resize(n, max_weight);
	min_distance[source] = 0;
	previous.clear();
	previous.resize(n, -1);
	std::set<std::pair<weight_t, vertex_t> > vertex_queue;
	vertex_queue.insert(std::make_pair(min_distance[source], source));

	while (!vertex_queue.empty())
	{
		weight_t dist = vertex_queue.begin()->first;
		vertex_t u = vertex_queue.begin()->second;
		vertex_queue.erase(vertex_queue.begin());

		// Visit each edge exiting u
		const std::vector<neighbor> &neighbors = adjacency_list[u];
		for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
			neighbor_iter != neighbors.end();
			neighbor_iter++)
		{
			vertex_t v = neighbor_iter->target;
			weight_t weight = neighbor_iter->weight;
			weight_t distance_through_u = dist + weight;
			if (distance_through_u < min_distance[v]) {
				vertex_queue.erase(std::make_pair(min_distance[v], v));

				min_distance[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.insert(std::make_pair(min_distance[v], v));

			}

		}
	}
}


std::list<vertex_t> DijkstraGetShortestPathTo(
	vertex_t vertex, const std::vector<vertex_t> &previous)
{
	std::list<vertex_t> path;
	for (; vertex != -1; vertex = previous[vertex])
		path.push_front(vertex);
	return path;
}

void test() {
	adjacency_list_t adjacency_list(4);
	// 0 = a
	adjacency_list[0].push_back(neighbor(1, 5));
	;
	// 1 = b
	adjacency_list[1].push_back(neighbor(2, 2));
	adjacency_list[1].push_back(neighbor(3, 3));

	// 2 = c
	adjacency_list[2].push_back(neighbor(1, 2));
	adjacency_list[2].push_back(neighbor(3, 17));
	
	// 3 = d
	adjacency_list[3].push_back(neighbor(1, 3));
	adjacency_list[3].push_back(neighbor(2, 7));
	

	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;
	DijkstraComputePaths(0, adjacency_list, min_distance, previous);
	std::cout << "Distance from 0 to 4: " << min_distance[2] << std::endl;
	std::list<vertex_t> path = DijkstraGetShortestPathTo(2, previous);
	std::cout << "Path : ";
	std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
	std::cout << std::endl;

}

std::vector<Point> getPath(Graph g, const int nHind, std::vector<Edge>edges, std::vector<float> weights, int startIndex, int goal)
{
	adjacency_list_t adjacency_list((nHind * 4) + 2);

	std::vector<Point> pathToGo;

	//pathToGo.push_back(g[startIndex].pt);

	for (int i = 0; i < edges.size(); i++)
	{
		adjacency_list[edges[i].first].push_back(neighbor(edges[i].second, weights[i]));
		adjacency_list[edges[i].second].push_back(neighbor(edges[i].first, weights[i]));
	}

	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;
	DijkstraComputePaths(startIndex, adjacency_list, min_distance, previous);
	std::list<vertex_t> path = DijkstraGetShortestPathTo(goal, previous);

	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		Point p = g[(*it)].pt;
		pathToGo.push_back(p);
	}

	//pathToGo.push_back(g[goal].pt);

	return pathToGo;
}