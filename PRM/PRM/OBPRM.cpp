#include "stdafx.h"
#include "PRM.h"
#include "OBPRM.h"
#include "KDT.h"
#include "Dijkstra.h"
#include "time.h"

OBPRM::OBPRM(const int initSampleSize, const int k, const int resamplePointNumbers, int ccLowThreshold) : PRM::PRM(initSampleSize,k, resamplePointNumbers, ccLowThreshold)
{

}

OBPRM::~OBPRM(){

}

void OBPRM::printResult(std::vector<Eigen::VectorXd> path, OBPRM::OBPRMMetrics metrics, bool printPath, bool printMetrics) {
	if (path.size() > 1 && printPath)
	{
		for (int i = 0; i < path.size(); i++)
		{
			std::cout << "---------------------------------------------" << std::endl;
			std::cout << path[i] << std::endl;
			std::cout << "---------------------------------------------" << std::endl;

		}
	}

	if (printMetrics) {
		std::cout << "------- Metric -------" << std::endl;
		std::cout << "Number of nodes: " << metrics.numberOfNodes << std::endl;
		std::cout << "Number of edges: " << metrics.numberOfEdges << std::endl;
		std::cout << "Runtime: " << metrics.runtime << "sec" << std::endl;
	}
}

std::vector<Eigen::VectorXd> OBPRM::getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, OBPRM::OBPRMMetrics& prmMetrics) {
	clock_t clockStart = clock();

	std::mt19937_64 rng(0);
	std::uniform_real_distribution<double> dis(0, 1.0);

	std::cout << "Starting init sample" << std::endl;
	std::vector<Eigen::Vector5d> initSampels;
	initSampels.push_back(goal);
	initSampels.push_back(start);
	getSample(mw, rng, dis,initSampleSize, initSampels);
	std::vector<PRM::NodeAttemptPair> nodeFailVct;

	std::cout << "getting connection" << std::endl;
	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, initSampels, 0);

	std::cout << "resample" << std::endl;
	std::vector<PRM::Edge> newEdges = reSample(mw, initSampels, nodeFailVct);

	for (int i = 0; i < newEdges.size(); i++) {
		edges.push_back(newEdges[i]);
	}

	std::cout << "number of Edges: " << edges.size() << std::endl;

	std::vector<Eigen::Vector5d> path = getShortestPath(edges, initSampels);

	//convert the path

	std::vector<Eigen::VectorXd> p;

	//path of size one means dijstra found no path
	if (path.size() > 1)
	{
		for (int i = 0; i < path.size(); i++)
		{
			p.push_back(path[i]);
		}
	}

	prmMetrics.numberOfNodes = initSampels.size();
	prmMetrics.numberOfEdges = edges.size();
	double t = (clock() - clockStart) / CLOCKS_PER_SEC;
	prmMetrics.runtime = t;
	return p;
}

void OBPRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, std::vector<Eigen::Vector5d>& samples) {
	int added = 0;
	Eigen::Vector5d moveVectorForward;
	Eigen::Vector5d nextForwardPoint, nextBackwardPoint,lastBackwardPoint;
	bool findCollision = false, findFreeSpace = false;

	while (true)
	{
		//check if we are done
		if (added >= sampleSize)
		{
			//get enough sample points
			return;
		}
		else {
			//not enough, try to get another one
			Eigen::Vector5d sample = MyWorm::Random(rng, dis);

			//check if the sample point is in free space
			if (!mw.CheckPosition(sample))
			{
				Eigen::Vector5d randomDirectionPoint = MyWorm::Random(rng, dis);
				Eigen::Vector5d directionPoint = (randomDirectionPoint - sample)*0.1;
				nextForwardPoint = directionPoint;

				while (!findFreeSpace) {	
					//If the next position is in free spacee
					if (!mw.CheckPosition(nextForwardPoint)) {
						//find the first obs collision
						nextBackwardPoint = nextForwardPoint;

						while (!findCollision) {
							if (!mw.CheckPosition(nextBackwardPoint)) {
								sample = lastBackwardPoint;
								findFreeSpace = true;
								findCollision = true;
							}
							else {
								lastBackwardPoint = nextBackwardPoint;
								nextBackwardPoint = nextBackwardPoint - (directionPoint / 2);
							}
						}
					}
					else {
						nextForwardPoint = nextForwardPoint + directionPoint;
					}
				}
			}

			samples.push_back(sample);
			findFreeSpace = false;
			findCollision = false;
			added++;
		}
	}
}

void OBPRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples) {
	
}
