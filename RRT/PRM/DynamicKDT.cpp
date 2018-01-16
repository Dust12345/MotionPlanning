#include "stdafx.h"
#include "DynamicKDT.h"


DynamicKDT::DynamicKDT()
{
	index = 0;
	tree = new dynamic_kd_tree(5 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

}


DynamicKDT::~DynamicKDT()
{
}

void DynamicKDT::addPoint(Eigen::Vector5d vct) {
	Element e = Element(vct, index);

	cloud.pts.push_back(e);

	tree->addPoints(index, index);
	index++;
}
int DynamicKDT::getNN(Eigen::Vector5d vct)
{

	double query_pt[5] = { vct[0], vct[1], vct[2] ,vct[3],vct[4] };

	// do a knn search
	const size_t num_results = 1;
	size_t ret_index;
	double out_dist_sqr;
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_index, &out_dist_sqr);
	tree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

	return ret_index;
}