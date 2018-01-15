#ifndef __RRT5Dof_H__
#define __RRT5Dof_H__

#include "RRTSimple.h"
#include "cell.h"
#include <string>
#include <unordered_map>

class RRT5Dof : public RRTSimple
{
	typedef pair<int, int> Edge;

public:

	struct Result {
		Tree tree;
		vector<Eigen::VectorXd> path;
	};

	struct RRT5dofMetrics {
		int numberOfNodes = 0;
		int numberOfEdges = 0;
		int numberOfNodesOnTheLine = 0;
		int numberOfGoalNodes = 0;
		int numberOfLineSplitts = 0;
		double runtime = 0;
	};

	RRT5Dof(Eigen::VectorXd START, Eigen::VectorXd GOAL);
	~RRT5Dof();
	Result getPath(WormCell &cell, int numberOfSamples, RRT5dofMetrics &metric, Eigen::VectorXd stepSize, double percent);
	void printResult(std::vector<Eigen::Vector5d> &nodes, RRT5Dof::RRT5dofMetrics &metrics, bool printNodes, bool printMetrics);

private:
	Eigen::VectorXd START, GOAL;
	WormCell *cell;
	Eigen::VectorXd stepSize;
	double percent = 0;
	RRT5dofMetrics *metric;
	int goalIndex = -1;

	unordered_map<string, int> generatedNodes;

	vector<Eigen::Vector5d> getSamples(int numberofSample);
	bool isNextRandomSampleAsGoal(double percent);
	vector<Edge> connectNodes(vector<Eigen::Vector5d>nodes);
	double distPointToLine(Eigen::Vector5d P, Eigen::Vector5d P0, Eigen::Vector5d P1, Eigen::Vector5d &orthogonalPoint);
	void connectNode(Eigen::Vector5d node, vector<KDT::nodeKnn> nodeNNVct, int nodeIndex, bool isNodeNew);
	string convertVector5dToString(Eigen::Vector5d node);
	Eigen::Vector5d stopconfiguration(Eigen::Vector5d node, Eigen::Vector5d nearestNeighbourNode,WormCell *cell, Eigen::Vector5d stepSize);
	vector<Eigen::Vector5d> getShortestPath(std::vector<Edge>edges, std::vector<Eigen::Vector5d>& samplePoint, int startIndex, int endIndex);


	vector<int> getAllRelationNodes(int currentNodeIndex, vector<Edge> &edges);
	void splittEdges(int nodeIndexOne, int nodeIndexTwo, int nodeIndexBetween, vector<Edge> &edges);
};



#endif /* __RRT5Dof_H__ */#pragma once
