#ifndef __RRT5Dof_H__
#define __RRT5Dof_H__

#include "RRTSimple.h"
#include "cell.h"
#include <string>
#include <unordered_map>
#include "DynamicKDT.h"

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
	Result getPath(WormCell &cell, int numberOfSamples, RRT5dofMetrics &metric, float stepSize, double percent);
	void printResult(std::vector<Eigen::Vector5d> &nodes, RRT5Dof::RRT5dofMetrics &metrics, bool printNodes, bool printMetrics);

//private:
	Eigen::VectorXd START, GOAL;
	WormCell *cell;
	float stepSize;
	double percent = 0;
	RRT5dofMetrics *metric;
	int goalIndex = -1;
	DynamicKDT dkdt;
	uniform_real_distribution<double> dis2;

	unordered_map<string, int> generatedNodes;

	bool goalFound = false;
	bool neverDone = true;
	int nearSamlpleRate = 11;

	vector<Eigen::Vector5d> getSamples(int numberofSample);
	bool isNextRandomSampleAsGoal(double percent,int number);
	vector<Edge> connectNodes(vector<Eigen::Vector5d>nodes);
	double distPointToLine(Eigen::Vector5d P, Eigen::Vector5d P0, Eigen::Vector5d P1, Eigen::Vector5d &orthogonalPoint);
	void connectNode(Eigen::Vector5d node, int nearestNeighbourIndex, int nodeIndex, bool isNodeNew,int sampleIndex);
	string convertVector5dToString(Eigen::Vector5d node);
	Eigen::Vector5d stopconfiguration(Eigen::Vector5d node, Eigen::Vector5d nearestNeighbourNode,WormCell *cell, float stepSize);
	vector<Eigen::Vector5d> getShortestPath(std::vector<Edge>edges, std::vector<Eigen::Vector5d>& samplePoint, int startIndex, int endIndex);
	Eigen::Vector5d stopconfiguration(Eigen::Vector5d node, Eigen::Vector5d nearestNeighbourNode, WormCell *cell);

	vector<int> getAllRelationNodes(int currentNodeIndex, vector<Edge> &edges);
	void splittEdges(int nodeIndexOne, int nodeIndexTwo, int nodeIndexBetween, vector<Edge> &edges);

	void mergeTrees(Tree& t1, Tree& t2);
	bool canConnect(WormCell& mw, Eigen::Vector5d a, Eigen::Vector5d b);
	std::vector<Edge> checkConnections(WormCell& mw, std::vector<Eigen::Vector5d>& t1, std::vector<Eigen::Vector5d>& t2, KDT::nodeKnn node);
};



#endif /* __RRT5Dof_H__ */#pragma once
