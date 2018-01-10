#pragma once
//entire class taken and somewhat modified from https://rosettacode.org/wiki/K-d_tree
#include <vector>
#include "cell.h"

class KDTree
{

/* global variable, so sue me */
int visited;

#define MAX_DIM 5
	struct kd_node_t
	{
		kd_node_t() {

		}

		kd_node_t(Eigen::Vector5d v,int indexOfVector)
		{
			if (MAX_DIM == 5)
			{
				x[0] = v[0];
				x[1] = v[1];
				x[2] = v[2];
				x[3] = v[3];
				x[4] = v[4];
			}
			index = indexOfVector;
			
		}

		double x[MAX_DIM];
		kd_node_t *left, *right;
		int index;
	};


private:
	double dist(kd_node_t *a, kd_node_t *b, int dim);
	void swap(kd_node_t *x, kd_node_t *y);
	kd_node_t* find_median(kd_node_t *start, kd_node_t *end, int idx);
	kd_node_t* make_tree(kd_node_t *t, int len, int i, int dim);
	void nearest(kd_node_t *root, kd_node_t *nd, int i, int dim, kd_node_t **best, double *best_dist);

	kd_node_t *root;

public:
	KDTree();
	~KDTree();

	void populateTree(std::vector<Eigen::Vector5d> vct);
};

