#pragma once
#include "cell.h"
#include <vector>
#include "KDT.h"
#include "PRM.h"
#include "stdafx.h"

class OBPRM : public PRM
{
	typedef std::pair<int, int> Edge;

private:
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, std::vector<Eigen::Vector5d>& samples);
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples);
	Eigen::Vector5d  moveOutOfInsect(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, std::vector<Eigen::Vector5d>& samples, Eigen::Vector5d & insectPoint);
	Eigen::Vector5d  moveOutOfInsect(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, std::vector<Eigen::Vector5d>& samples, Eigen::Vector5d & insectPoint, Eigen::Vector5d base);
	std::vector<PRM::Edge> reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct);
	std::vector<PRM::Edge> connectionTesting(WormCell& mw, std::vector<PRM::NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints, int startIndex);
public:

	struct OBPRMMetrics {
		int numberOfNodes;
		int numberOfEdges;
		int numberOfNN;
		double runtime;
		int numberCC;
	};

	OBPRM(const int initSampleSize, const int k, const int resamplePointNumbers, int ccLowThreshold);
	~OBPRM();
	void printResult(std::vector<Eigen::VectorXd> path, OBPRM::OBPRMMetrics metrics, bool printPath, bool printMetrics);
	std::vector<Eigen::VectorXd> getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, OBPRM::OBPRMMetrics& prmMetrics);
};