#pragma once

#include "cell.h"
#include <vector>
#include "KDT.h"

//incapsulates the prm functionality
class PRM
{	
protected:
	typedef std::pair<int, int> Edge;

	//combined the index of a point and the number of failed connection attempts
	struct NodeAttemptPair
	{
		public:
			int index;
			int failedAttemps;
			double avrgDist;

	};

	//struct for a connected component
	//simply a vector which contains the indecies of the points, that make of the graph
	struct CC
	{
		public:
			std::vector<int> nodesInCC;
	};

	//calculates the distance between two vectors
	double vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b);

	//algo params
	int initSampleSize;
	int k;
	int resamplePointNumbers;
	int ccLowThreshold;

	//calculates the number of connected components in the given graph
	int getConectedComponentNumber(std::vector<Edge>& edges);

	//check if the connected component contains the given node
	bool inCC(CC cc, int nodeIndex);
	//takes the edge and adds it to the vector of connected components
	//if this means adding a new CC, compining two CCs or something else, depends on the edge itself
	void addEdgeToCCs(std::vector<CC>& cc, Edge& edge);

	//creates random vectors within the given distribution, all sampels are valid configurations and do not collide with an obstical
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, std::vector<Eigen::Vector5d>& samples);
	
	//creates random vectors within the given distribution around the given base vector, all sampels are valid configurations and do not collide with an obstical
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples);
	//checks which points are connected and returns the edges between the points
	std::vector<Edge> connectionTesting(WormCell& mw, std::vector<NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints, int startIndex);

	//takes a node and its nearest neigtbors and check which nn point in connected to the node, also keeps track of the number of failed attempts
	std::vector<Edge> checkConnections(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, KDT::nodeKnn node, NodeAttemptPair& metric);
	
	//check wether two nodes can be connected or not
	bool canConnect(WormCell& mw,Eigen::Vector5d a, Eigen::Vector5d b);

	//resamples more points around points which had a high rate of failed connnection attempts
	std::vector<Edge> reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct);

	//calculates the weigth of the edge between two vectors
	double calcWeigth(Eigen::Vector5d a, Eigen::Vector5d b);

	//calculates via dijkstra the shortest path between the startnode and the endnode
	std::vector<Eigen::Vector5d> getShortestPath(std::vector<Edge>edges, std::vector<Eigen::Vector5d>& samplePoint);


public:

	//holds the interessting metrecies
	struct PRMMetrics {
		int numberOfNodes;
		int numberOfEdges;
		int numberOfNN;
		double runtime;
		int numberCC;
	};

	//constuctor
	PRM(int initSampleSize, int k, int resamplePointNumbers, int ccLowThreshold);
	
	//destructor
	~PRM();
	//prints the collected metrecies
	void printResult(std::vector<Eigen::VectorXd> path, PRM::PRMMetrics metrics, bool printPath, bool printMetrics);
	
	//calculates and returns the shortest path between the start and goal configuration
	std::vector<Eigen::VectorXd> getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, PRM::PRMMetrics& prmMetrics);
};

