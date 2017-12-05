#pragma once

#include "cell.h"
#include <vector>
#include "KDT.h"

class PRM
{

	typedef std::pair<int, int> Edge;
	
private:

	struct NodeAttemptPair
	{
		public:
			int index;
			int failedAttemps;
			double avrgDist;

	};

	double vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b);

	//algo params
	int initSampleSize;
	int k;
	int resamplePointNumbers;

	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, std::vector<Eigen::Vector5d>& samples);
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples);

	std::vector<Edge> connectionTesting(WormCell& mw, std::vector<NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints, int startIndex);

	std::vector<Edge> checkConnections(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, KDT::nodeKnn node, NodeAttemptPair& metric);
	bool canConnect(WormCell& mw,Eigen::Vector5d a, Eigen::Vector5d b);

	std::vector<Edge> reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct);

	double calcWeigth(Eigen::Vector5d a, Eigen::Vector5d b);

	std::vector<Eigen::Vector5d> getShortestPath(std::vector<Edge>edges, std::vector<Eigen::Vector5d>& samplePoint);

public:


	PRM();
	~PRM();

	std::vector<Eigen::VectorXd> getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal);
};

