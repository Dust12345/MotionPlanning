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
	// creates random vectors within the given distribution, all sampels are valid configurations and do not collide with an obstical
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, std::vector<Eigen::Vector5d>& samples);
	//creates random vectors within the given distribution around the given base vector, all sampels are valid configurations and do not collide with an obstical
	void getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples);
	// move a invalid Position to a valid Position.
	Eigen::Vector5d  moveOutOfInsect(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, std::vector<Eigen::Vector5d>& samples, Eigen::Vector5d & insectPoint, Eigen::Vector5d base);
	//resamples more points around points which had a high rate of failed connnection attempts
	std::vector<PRM::Edge> reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct);
	//checks which points are connected and returns the edges between the points
	std::vector<PRM::Edge> connectionTesting(WormCell& mw, std::vector<PRM::NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints, int startIndex);
public:

	OBPRM(const int initSampleSize, const int k, const int resamplePointNumbers, int ccLowThreshold);
	~OBPRM();
	//prints the collected metrecies
	void printResult(std::vector<Eigen::VectorXd> path, PRM::PRMMetrics metrics, bool printPath, bool printMetrics);
	// calculates and returns the shortest path between the start and goal configuration
	std::vector<Eigen::VectorXd> getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, PRM::PRMMetrics& prmMetrics);
};