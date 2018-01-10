#include "stdafx.h"
#include "KDTree.h"


KDTree::KDTree()
{
}


KDTree::~KDTree()
{
}

double KDTree::dist(kd_node_t *a, kd_node_t *b, int dim)
{
	double t, d = 0;
	while (dim--) {
		t = a->x[dim] - b->x[dim];
		d += t * t;
	}
	return d;
}

void KDTree::swap(kd_node_t *x, kd_node_t *y)
{
	double tmp[MAX_DIM];
	memcpy(tmp, x->x, sizeof(tmp));
	memcpy(x->x, y->x, sizeof(tmp));
	memcpy(y->x, tmp, sizeof(tmp));
}

KDTree::kd_node_t* KDTree::find_median(kd_node_t *start, kd_node_t *end, int idx)
{
	if (end <= start) return NULL;
	if (end == start + 1)
		return start;

	kd_node_t *p, *store, *md = start + (end - start) / 2;
	double pivot;
	while (1) {
		pivot = md->x[idx];

		swap(md, end - 1);
		for (store = p = start; p < end; p++) {
			if (p->x[idx] < pivot) {
				if (p != store)
					swap(p, store);
				store++;
			}
		}
		swap(store, end - 1);

		/* median has duplicate values */
		if (store->x[idx] == md->x[idx])
			return md;

		if (store > md) end = store;
		else        start = store;
	}
}

KDTree::kd_node_t* KDTree::make_tree(kd_node_t *t, int len, int i, int dim)
{
	kd_node_t *n;

	if (!len) return 0;

	if ((n = find_median(t, t + len, i))) {
		i = (i + 1) % dim;
		n->left = make_tree(t, n - t, i, dim);
		n->right = make_tree(n + 1, t + len - (n + 1), i, dim);
	}
	return n;
}

void KDTree::nearest(kd_node_t *root, kd_node_t *nd, int i, int dim, kd_node_t **best, double *best_dist)
{
	double d, dx, dx2;

	if (!root) return;
	d = dist(root, nd, dim);
	dx = root->x[i] - nd->x[i];
	dx2 = dx * dx;

	visited++;

	if (!*best || d < *best_dist) {
		*best_dist = d;
		*best = root;
	}

	/* if chance of exact match is high */
	if (!*best_dist) return;

	if (++i >= dim) i = 0;

	nearest(dx > 0 ? root->left : root->right, nd, i, dim, best, best_dist);
	if (dx2 >= *best_dist) return;
	nearest(dx > 0 ? root->right : root->left, nd, i, dim, best, best_dist);
}


void KDTree::populateTree(std::vector<Eigen::Vector5d> vct)
{
	kd_node_t* nodes = new kd_node_t[vct.size()];

	for (int i = 0; i < vct.size(); i++)
	{
		nodes = new kd_node_t(vct[i], i);
		nodes++;
	}	
	root = make_tree(nodes, sizeof(nodes) / sizeof(nodes[1]), 0, 4);
}