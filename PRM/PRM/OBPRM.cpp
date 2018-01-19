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

void OBPRM::printResult(std::vector<Eigen::VectorXd> path, PRM::PRMMetrics metrics, bool printPath, bool printMetrics) {
	
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
		std::cout << "------- Metric OBPRM -------" << std::endl;
		std::cout << "Number of nodes: " << metrics.numberOfNodes << std::endl;
		std::cout << "Number of edges: " << metrics.numberOfEdges << std::endl;
		std::cout << "Number of nearest neighbours: " << metrics.numberOfNN << std::endl;
		std::cout << "Number of connected components: " << metrics.numberCC << std::endl;
		std::cout << "Number of invalid generated nodes: " << metrics.numberOfGeneratedSampleInvalid << std::endl;
		std::cout << "Number of nodes in path: " << metrics.numberOfPathNodes << std::endl;
		std::cout << "Runtime: " << metrics.runtime << "sec" << std::endl;
	}
}

std::vector<Eigen::VectorXd> OBPRM::getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, PRM::PRMMetrics& prmMetrics) {
	//start a clock to keep track on how long it took
	clock_t clockStart = clock();

	//set up the rng and random distribution for the sampleing
	std::mt19937_64 rng(0);
	std::uniform_real_distribution<double> dis(0, 1);

	std::cout << "Starting init sample" << std::endl;
	std::vector<Eigen::Vector5d> configurationsPoints;

	//get the inital sample points
	OBPRM::getSample(mw, rng, dis, initSampleSize, configurationsPoints);
	std::vector<PRM::NodeAttemptPair> nodeFailVct;

	std::cout << "checking connection" << std::endl;
	//test the connectifity of the sample points
	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, configurationsPoints, 0);

	//resample until the number of connected components is below the threshold
	while (true)
	{
		//resample
		std::cout << "resampling points" << std::endl;
		std::vector<PRM::Edge> newEdges = reSample(mw, configurationsPoints, nodeFailVct);

		//add the edges we got from the resampling to the ones we already know
		for (int i = 0; i < newEdges.size(); i++)
		{
			edges.push_back(newEdges[i]);
		}

		//check if we are done
		//we do the check at the end, to resample at least once
		int numberOfCC = getConectedComponentNumber(edges);
		if (numberOfCC <= ccLowThreshold) {
			break;
		}
	}
	std::cout << "adding start and goal to the graph" << std::endl;
	//add the start and goal to the graph
	std::vector<Eigen::VectorXd> p;

	//check if start and goal are within the boundry
	if (goal[0] < 0 || goal[0]>1 || goal[1] < 0 || goal[1]>1
		|| start[0] < 0 || start[0]>1 || start[1] < 0 || start[1]>1)
	{
		std::cout << "unreachable goal or start" << std::endl;


		prmMetrics.numberOfNN = k;
		prmMetrics.numberCC = getConectedComponentNumber(edges);
		prmMetrics.numberOfNodes = configurationsPoints.size();
		prmMetrics.numberOfEdges = edges.size();
		double t = (clock() - clockStart) / CLOCKS_PER_SEC;
		prmMetrics.runtime = t;
		prmMetrics.numberOfGeneratedSampleInvalid = numberOfSampleInvalid;
		prmMetrics.numberOfPathNodes = 0;
		return p;
	}


	configurationsPoints.push_back(goal);
	configurationsPoints.push_back(start);
	std::vector<PRM::Edge> newEdges = connectionTesting(mw, nodeFailVct, configurationsPoints, configurationsPoints.size() - 2);
	for (int i = 0; i < newEdges.size(); i++)
	{
		edges.push_back(newEdges[i]);
	}

	std::cout << "calculating the shortest path" << std::endl;
	std::vector<Eigen::Vector5d> path = getShortestPath(edges, configurationsPoints);
	
	//run the Dijkstra again wiht the last solution nodes
	std::cout << "calculating the shortest path again" << std::endl;

	//delete goal and start Position because they have the wrong index (0 and path.size()-1)
	for (std::_Vector_iterator<std::_Vector_val<std::_Simple_types<Eigen::Vector5d>>> it = path.begin(); it != path.end();)
	{
		if (*it._Ptr == start || *it._Ptr == goal) {
			it = path.erase(it);
		}
		else {
			++it;
		}

	}

	//add goal and start 
	path.push_back(goal);
	path.push_back(start);
	
	std::vector<PRM::NodeAttemptPair> newNodeFailVct;

	newEdges = connectionTesting(mw, newNodeFailVct, path, 0);
	path = getShortestPath(newEdges, path);

	// add Metric informations
	std::cout << "set Metric informations" << std::endl;
	//path of size one means dijstra found no path
	if (path.size() > 1)
	{
		for (int i = 0; i < path.size(); i++)
		{
			p.push_back(path[i]);
		}
	}
	else {
		std::cout << "No path found" << std::endl;
	}
	int numberOfCC = getConectedComponentNumber(newEdges);

	prmMetrics.numberOfNN = k;
	prmMetrics.numberCC = numberOfCC;
	prmMetrics.numberOfNodes = configurationsPoints.size();
	prmMetrics.numberOfEdges = newEdges.size()+edges.size();
	double t = (clock() - clockStart) / CLOCKS_PER_SEC;
	prmMetrics.runtime = t;
	prmMetrics.numberOfGeneratedSampleInvalid = numberOfSampleInvalid;
	prmMetrics.numberOfPathNodes = path.size();
	return p;
}

void OBPRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, std::vector<Eigen::Vector5d>& samples)
{
	int added = 0;
	//to prevent an endless loop, we stop at some point no matter what
	int maxTries = sampleSize * 25;

	while (true)
	{
		//check if we are done
		if (added > sampleSize || added >= maxTries)
		{
			//get enough sample points
			return;
		}
		else {
			//not enough, try to get another one
			Eigen::Vector5d sample = MyWorm::Random(rng, dis);

			//check if the sample point is in free space
			if (mw.CheckPosition(sample))
			{		
				//point is free
				samples.push_back(sample);
				added++;
			}
			else {
				//if the point is not free then move the point in a free position
				Eigen::Vector5d newPoint = moveOutOfInsect(mw, rng, dis, samples, sample, sample.base());
				samples.push_back(newPoint);
				added++;
			}
			std::cout << samples.size() << "/" << sampleSize << std::endl;
		}
	}
}

void OBPRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples) {
	int added = 0;
	//to prevent an endless loop, we stop at some point no matter what
	int maxTries = sampleSize * 50;

	while (true) {
		//check if we are done
		if (added > sampleSize || added >= maxTries) {
			//get enough sample points
			return;
		}
		else {
			//not enough, try to get another one
			Eigen::Vector5d sample = MyWorm::Random(base, rng, dis);

			//check if the sample point is in free space
			if (mw.CheckPosition(sample)) {
				//point is free
				samples.push_back(sample);
				added++;
			}
			else {
				//if the point is not free then move the point in a free position
				Eigen::Vector5d newPoint = moveOutOfInsect(mw, rng, dis, samples, sample, base);
				samples.push_back(newPoint);
				added++;
			}
		}
	}
}

Eigen::Vector5d OBPRM::moveOutOfInsect(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, std::vector<Eigen::Vector5d>& samples, Eigen::Vector5d & insectPoint, Eigen::Vector5d base) {
	double jumpForwardScalaMul = 0.4;
	double jumpBackwardScalaDiv = 2;

	//calc a random Point
	Eigen::Vector5d randomDirectionPoint = MyWorm::Random(base, rng, dis);
	randomDirectionPoint[2] = 0;
	randomDirectionPoint[3] = 0;
	randomDirectionPoint[4] = 0;

	//calc a movement vector and multiply with the jumpForwardScalaMul
	Eigen::Vector5d directionPoint = (randomDirectionPoint - insectPoint)*jumpForwardScalaMul;
	//Calc the next Point with the move movement vector
	Eigen::Vector5d nextForwardPoint = insectPoint + directionPoint;
	Eigen::Vector5d nextBackwardPoint, lastBackwardPoint;

	while (true) {
		//If the next position is in free spacee
		if (mw.CheckPosition(nextForwardPoint)) {
			nextBackwardPoint = nextForwardPoint;

			while (true) {
				//find the first obs collision
				if (!mw.CheckPosition(nextBackwardPoint)) {
					return lastBackwardPoint;
				}
				else {
					// Calc the next backward point and save the current next backward point 
					lastBackwardPoint = nextBackwardPoint;
					nextBackwardPoint = nextBackwardPoint - (directionPoint / jumpBackwardScalaDiv);
				}
			}
		}
		else {
			// calc the next forward point
			nextForwardPoint = nextForwardPoint + directionPoint;
		}
	}
}

std::vector<PRM::Edge> OBPRM::reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct) {
	std::mt19937_64 rng(0);

	for (int i = 0; i < nodeFailVct.size(); i++) {
		//do a simple threshold check
		if (nodeFailVct[i].failedAttemps >= k / 2) {
			//the range in which the new samples should be created
			//double range = sqrt(nodeFailVct[i].avrgDist);
			double range = 0.6;
			double min = (range / 2)*-1;
			double max = range / 2;

			if (min < 0) {
				min = 0;
			}

			if (max > 1) {
				max = 1;
			}

			std::uniform_real_distribution<double> dis(min, max);

			//the base point around which the new samples should be created
			Eigen::Vector5d base = samplePoints[nodeFailVct[i].index];

			OBPRM::getSample(mw, rng, dis, resamplePointNumbers, base, samplePoints);
		}
	}
	std::cout << "Testing connections again" << std::endl;

	//test the new samples for their connectifity
	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, samplePoints, initSampleSize);

	return edges;
}

std::vector<PRM::Edge> OBPRM::connectionTesting(WormCell& mw, std::vector<PRM::NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints, int startIndex) {
	std::vector<PRM::Edge> edges;
	KDT kdTree;
	std::vector<KDT::nodeKnn> nodeNNVct;

	//get the nearest neigtbors for all nodes
	//kdTree.getKNNWithEuclid(samplePoints, nodeNNVct, startIndex, k);

	for (int i = 0; i < nodeNNVct.size(); i++) {
		//for each node, check the connectifity to its NN
		PRM::NodeAttemptPair metric;
		metric.avrgDist = nodeNNVct[i].avrgDist;
		std::vector<Edge> edgesForNode = checkConnections(mw, samplePoints, nodeNNVct[i], metric);
		nodeFailVct.push_back(metric);

		//add the adges
		for (int j = 0; j < edgesForNode.size(); j++) {
			edges.push_back(edgesForNode[j]);
		}
	}
	return edges;
}
