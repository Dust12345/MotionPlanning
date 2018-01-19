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
	
	// returns the path from the start to the goal, if there is one
	Result getPath(WormCell &cell, int numberOfSamples, RRT5dofMetrics &metric, float stepSize, double percent);
	
	//prints the metrics of the path search on the console
	void printResult(std::vector<Eigen::Vector5d> &nodes, RRT5Dof::RRT5dofMetrics &metrics, bool printNodes, bool printMetrics);

private:
	Eigen::VectorXd START, GOAL;
	WormCell *cell;
	float stepSize;
	double percent = 0;
	RRT5dofMetrics *metric;
	int goalIndex = -1;
	DynamicKDT dkdt;
	
	//secondary distribution to sample points close to another one
	uniform_real_distribution<double> dis2;

	unordered_map<string, int> generatedNodes;

	bool goalFound = false;
	bool foundPathToGoal = true;
	int nearSamlpleRate = 11;

	int goalSampleRate = 30;

	//generates the given number of random sample points
	vector<Eigen::Vector5d> getSamples(int numberofSample);

	//determines if the next sample should be the goal
	bool isNextRandomSampleAsGoal(double percent,int number);

	//attemps to build an RRT with the given nodes
	vector<Edge> connectNodes(vector<Eigen::Vector5d>nodes);

	//calculates the closest from a point to a line, also returns the orthogonalPoint
	double distPointToLine(Eigen::Vector5d P, Eigen::Vector5d P0, Eigen::Vector5d P1, Eigen::Vector5d &orthogonalPoint);
	
	//attemps to connect two points
	void connectNode(Eigen::Vector5d node, int nearestNeighbourIndex, int nodeIndex, bool isNodeNew,int sampleIndex);

	//returns a string representing the node
	string convertVector5dToString(Eigen::Vector5d node);
	
	//calculates the shortest path from start to goal, uses dijkstra
	vector<Eigen::Vector5d> getShortestPath(std::vector<Edge>edges, std::vector<Eigen::Vector5d>& samplePoint, int startIndex, int endIndex);
	
	//returns the stop configuration from the nearestNeighbour to the node
	Eigen::Vector5d stopconfiguration(Eigen::Vector5d node, Eigen::Vector5d nearestNeighbourNode, WormCell *cell);

	// get all nodesindexes which have a relation with the current node (nodeindex)
	vector<int> getAllRelationNodes(int currentNodeIndex, vector<Edge> &edges);

	//splits an edge and inserts a point at the split
	void splittEdges(int nodeIndexOne, int nodeIndexTwo, int nodeIndexBetween, vector<Edge> &edges);

	//check if the two nodes can be connected without colliding with an obstical
	bool canConnect(WormCell& mw, Eigen::Vector5d a, Eigen::Vector5d b);

};



#endif /* __RRT5Dof_H__ */#pragma once
