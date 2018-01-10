#ifndef __RRTSimple_H__
#define __RRTSimple_H__

#include <vector>
#include "cell.h"
#include "KDT.h"


using namespace std;
class RRTSimple
{
	private:
		typedef pair<int, int> Edge;

		struct Metrics {
			int numberOfNodes;
			int numberOfEdges;
		};

		struct Tree {
			vector<Eigen::Vector5d> nodes;
			vector<Edge> edges;
		};

		Eigen::Vector5d START_POINT;
		Metrics metrics;
		Tree tree;
		uniform_real_distribution<double> dis;
		mt19937_64 rng;

		vector<Eigen::Vector5d> getSamples(int numberofSample);
		vector<Edge> connectNodes(vector<Eigen::Vector5d>nodes, int startIndex);
		Eigen::Vector5d getSample();
		void connectNode(Eigen::Vector5d node, vector<KDT::nodeKnn> nodeNNVct, int nodeIndex);
		double vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b);
		double scalar_product(Eigen::Vector5d a, Eigen::Vector5d b);
		double getDistance(Eigen::Vector5d point, Eigen::Vector5d lineStart, Eigen::Vector5d lineEnd, Eigen::Vector3d &pointOnLine);
		double distPointToLine(Eigen::Vector5d PV, Eigen::Vector5d PV0, Eigen::Vector5d PV1, Eigen::Vector5d &orthogonalPoint);
		vector<int> getAllRelationNodes(int nodeIndex, vector<Edge> &edges);

public:
	RRTSimple();
	~RRTSimple();
	void createTree(const Eigen::Vector5d start, const int numberOfSample);
	void writeGnuplotFile(std::vector<Eigen::Vector5d> &points, string filename, vector<Edge> &edges);
};

#endif /* __RRTSimple_H__ */