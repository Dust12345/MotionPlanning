#include "stdafx.h"
#include "KDT.h"
#include "utils.h"
#include "nanoflann.hpp"
#include <math.h>

using namespace nanoflann;
KDT::KDT()
{
}

/*
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
}*/



void KDT::getKNN(std::vector<Eigen::Vector5d> vct, std::vector<KDT::nodeKnn>& nodeNNVct,int startIndex,int k)
{
	//build an populate the tree
	PointCloud cloud;

	for (int i = 0; i < vct.size(); i++) {
		cloud.pts.push_back(Element(vct[i], i));
	}
	my_kd_tree_t tree(5 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	tree.buildIndex();


	//get the nn for all points

	for (int i = startIndex; i < vct.size(); i++)
	{
		Eigen::Vector5d& v = vct[i];

		double query_pt[5] = { v[0], v[1], v[2] ,v[3],v[4] };
		size_t num_results = k;
		std::vector<size_t>   ret_index(num_results);
		std::vector<double> out_dist_sqr(num_results);

		num_results = tree.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

		// In case of less points in the tree than requested:
		ret_index.resize(num_results);
		out_dist_sqr.resize(num_results);


		KDT::nodeKnn res;
		res.index = i;
		double avrgDist = 0;

		for (size_t j = 0; j < num_results; j++)
		{

			if (ret_index[j] != res.index)
			{
				res.nn.push_back(ret_index[j]);
				avrgDist = avrgDist + out_dist_sqr[j];
			}
			
		}
			
		res.avrgDist = avrgDist / (k-1);
		nodeNNVct.push_back(res);
	}
}

void KDT::getKNNWithEuclid(std::vector<Eigen::Vector5d> vct, std::vector<KDT::nodeKnn>& nodeNNVct, int startIndex, int k)
{
	//build an populate the tree
	PointCloud cloud;

	for (int i = 0; i < vct.size(); i++) {
		cloud.pts.push_back(Element(vct[i], i));
	}
	my_kd_tree_t tree(5 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	tree.buildIndex();


	//get the nn for all points

	for (int i = startIndex; i < vct.size(); i++)
	{
		Eigen::Vector5d& v = vct[i];

		double query_pt[5] = { v[0], v[1], v[2] ,v[3],v[4] };
		size_t num_results = k;
		std::vector<size_t>   ret_index(num_results);
		std::vector<double> out_dist_sqr(num_results);

		num_results = tree.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

		// In case of less points in the tree than requested:
		ret_index.resize(num_results);
		out_dist_sqr.resize(num_results);


		KDT::nodeKnn res;
		res.index = i;
		double avrgEuclidDist = 0;

		for (size_t j = 0; j < num_results; j++)
		{

			if (ret_index[j] != res.index)
			{
				Eigen::Vector5d nnPoint = vct[ret_index[j]];
				res.nn.push_back(ret_index[j]);

				// Dinstance about all 5 values ( x, y and 3 rotational)
				double euclidDistance = sqrt(pow((query_pt[0] - nnPoint[0]), 2) + pow((query_pt[1] - nnPoint[1]), 2)+pow((query_pt[2] - nnPoint[2]), 2) + pow((query_pt[3] - nnPoint[3]), 2) + pow((query_pt[4] - nnPoint[4]), 2));

				avrgEuclidDist = avrgEuclidDist + euclidDistance;
			}

		}

		res.avrgDist = avrgEuclidDist / (k - 1);
		nodeNNVct.push_back(res);
	}
}

KDT::~KDT()
{
}
