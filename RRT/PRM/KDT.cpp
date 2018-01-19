#include "stdafx.h"
#include "KDT.h"
#include "nanoflann.hpp"
#include <math.h>

using namespace nanoflann;
KDT::KDT()
{
}


void KDT::getKNN(std::vector<Eigen::Vector5d> vct, std::vector<Eigen::Vector5d> toConnect,std::vector<KDT::nodeKnn>& nodeNNVct, int startIndex, int k)
{
	//build an populate the tree
	PointCloud cloud;

	for (int i = 0; i < vct.size(); i++) {
		cloud.pts.push_back(Element(vct[i], i));
	}
	my_kd_tree_t tree(5 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	tree.buildIndex();


	//get the nn for all points

	for (int i = startIndex; i < toConnect.size(); i++)
	{
		Eigen::Vector5d& v = toConnect[i];

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

		res.avrgDist = avrgDist / (k - 1);
		nodeNNVct.push_back(res);
	}
}

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

void KDT::getKNN(std::vector<Eigen::Vector5d> vct, std::vector<KDT::nodeKnn>& nodeNNVct, int startIndex, int endIndex, int k)
{
	//build an populate the tree
	PointCloud cloud;

	for (int i = 0; i < vct.size(); i++) {
		cloud.pts.push_back(Element(vct[i], i));
	}
	my_kd_tree_t tree(5 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	tree.buildIndex();


	//get the nn for all points

	for (int i = startIndex; i <= endIndex; i++)
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

		res.avrgDist = avrgDist / (k - 1);
		nodeNNVct.push_back(res);
	}
}

KDT::~KDT()
{
}
