#pragma once
#include "nanoflann.hpp"
#include <vector>
#include "cell.h"

using namespace nanoflann;
class KDT
{
public:

	struct nodeKnn {
	public:
		int index;
		std::vector<int> nn;
		double avrgDist;

	};
	
	struct Element {	
		Element() {

		}

		Element(Eigen::Vector5d vct,int elementIndex)
		{
			index = elementIndex;
			values.push_back(vct[0]);
			values.push_back(vct[1]);
			values.push_back(vct[2]);
			values.push_back(vct[3]);
			values.push_back(vct[4]);

		}
		std::vector<double> values;
		int index;
		
	};

	struct PointCloud
	{

	public:
		std::vector<Element> pts;

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return pts.size(); }

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline double kdtree_get_pt(const size_t idx, int dim) const
		{
			if (dim >= pts[idx].values.size()) {
				return  pts[idx].values.at(pts[idx].values.size()-1);
			}
			else {
				return  pts[idx].values.at(dim);
			}

		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

	};

	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<double, PointCloud >,
		PointCloud,
		5 /* dim */
	> my_kd_tree_t;


private:
	

public:
	KDT();
	~KDT();

	void getKNN(std::vector<Eigen::Vector5d> vct,std::vector<nodeKnn>& nodeNNVct,int startIndex, int k);
};

