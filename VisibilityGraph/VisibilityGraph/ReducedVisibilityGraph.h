#pragma once

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

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;
typedef std::pair<int, int> Edge;

class ReducedVisibilityGraph
{
public:
	ReducedVisibilityGraph();
	~ReducedVisibilityGraph();
	void reduce(Graph& g, Point startPoint, const int nHind, Point goal, std::vector<Edge>& edges);
};

