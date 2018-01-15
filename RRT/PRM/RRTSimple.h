#ifndef __RRTSimple_H__
#define __RRTSimple_H__

#include <vector>
#include "cell.h"
#include "KDT.h"


using namespace std;
class RRTSimple
{
	typedef pair<int, int> Edge;

	public:
		struct SimpleRRTMetrics {
			int numberOfNodes = 0;
			int numberOfEdges = 0;
			int numberOfNodesOnTheLine = 0;
			int numberOfLineSplitts = 0;
			double runtime = 0;
		};

		struct Tree {
			vector<Eigen::Vector5d> nodes;
			vector<Edge> edges;
		};

		RRTSimple();
		~RRTSimple();
		Tree createTree(const Eigen::Vector5d start, const int numberOfSample, RRTSimple::SimpleRRTMetrics & metrics);
		void writeGnuplotFile(std::vector<Eigen::Vector5d> &points, string filename, vector<Edge> &edges);
		void printResult(std::vector<Eigen::Vector5d> &nodes, RRTSimple::SimpleRRTMetrics &metrics, bool printNodes, bool printMetrics);

	protected:
		Tree tree;
		uniform_real_distribution<double> dis;
		mt19937_64 rng;

		double vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b);


	private:
		SimpleRRTMetrics *metrics;
		vector<Eigen::Vector5d> getSamples(int numberofSample);
		vector<Edge> connectNodes(vector<Eigen::Vector5d>nodes);
		Eigen::Vector5d getSample();
		void connectNode(Eigen::Vector5d node, vector<KDT::nodeKnn> nodeNNVct, int nodeIndex);
		double distPointToLine(Eigen::Vector5d PV, Eigen::Vector5d PV0, Eigen::Vector5d PV1, Eigen::Vector5d &orthogonalPoint);

		vector<int> getAllRelationNodes(int currentNodeIndex, vector<Edge> &edges);
		void splittEdges(int nodeIndexOne, int nodeIndexTwo, int nodeIndexBetween, vector<Edge> &edges);
};

#endif /* __RRTSimple_H__ */