#include "stdafx.h"
#include "PRM.h"
#include "KDT.h"
#include "Dijkstra.h"
#include "time.h"

PRM::PRM(const int initSampleSize,const int k,const int resamplePointNumbers)
{
	this->initSampleSize = initSampleSize;
	this->k = k;
	this->resamplePointNumbers = resamplePointNumbers;
}

std::vector<PRM::Edge> PRM::checkConnections(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, KDT::nodeKnn node, PRM::NodeAttemptPair& metric)
{
	std::vector<PRM::Edge> edges;
	Eigen::Vector5d nodeA = samplePoints[node.index];
	int failedAttempts = 0;

	for (int i = 0; i < node.nn.size(); i++)
	{
		Eigen::Vector5d nodeB = samplePoints[node.nn[i]];
		bool connected = canConnect(mw,nodeA,nodeB);

		if (connected) {
			Edge e;
			//std::cout << "adding " << node.index << " and " << node.nn[i] << std::endl;

			e.first = node.index;
			e.second = node.nn[i];
			edges.push_back(e);
		}
		else {
			failedAttempts++;
		}
	}

	metric.index = node.index;
	metric.failedAttemps = failedAttempts;

	return edges;

}
double PRM::vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b)
{
	Eigen::Vector5d c = a - b;

	double sum = pow(c[0], 2)+ pow(c[1], 2)+ pow(c[2], 2)+pow(c[3], 2)+ pow(c[4], 2);

	double length = std::sqrt(sum);

	return length;
}

bool PRM::canConnect(WormCell& mw, Eigen::Vector5d a, Eigen::Vector5d b)
{
	//first check the distance between the two points and, based on that, decide how many check we want to do
	double dist = vec5Distance(a, b);
	int checkPerOneUnit = 20;

	int checks = dist*checkPerOneUnit;

	//do at least 3
	if (checks <= 0) {
		checks = 3;
	}

	checks = 10;

	//bool result = mw.CheckMotion(a, b, checks);

	//return result;

	//simple but not efficent way to check
	Eigen::Vector5d c = a - b;

	c = c / (checks-1);

	for (int i = 0; i < (checks - 1); i++)
	{
		Eigen::Vector5d d = b + (c*i);

		if(!mw.CheckPosition(d))
		{
			return false;
		}
		
	}
	return true;
	
}

std::vector<PRM::Edge> PRM::connectionTesting(WormCell& mw, std::vector<PRM::NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints,int startIndex)
{
	std::vector<PRM::Edge> edges;
	KDT kdTree;
	std::vector<KDT::nodeKnn> nodeNNVct;

	kdTree.getKNN(samplePoints, nodeNNVct, startIndex,k);


	/*for (int h = 0; h < nodeNNVct.size(); h++)
	{	
		std::cout << "----" << std::endl;
		std::cout << nodeNNVct[h].index << std::endl;
		for (int o = 0; o < nodeNNVct[h].nn.size(); o++) {
			std::cout << nodeNNVct[h].nn[o] << std::endl;
		}

	}*/


	for (int i = 0; i < nodeNNVct.size(); i++)
	{
		PRM::NodeAttemptPair metric;
		metric.avrgDist = nodeNNVct[i].avrgDist;
		std::vector<Edge> edgesForNode = checkConnections(mw, samplePoints, nodeNNVct[i], metric);
		nodeFailVct.push_back(metric);
		
		
		//add the adges
		for (int j = 0; j < edgesForNode.size(); j++)
		{
			
			edges.push_back(edgesForNode[j]);
		}

	}

	return edges;
}

void PRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples)
{
	
	int added = 0;

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
			Eigen::Vector5d sample = MyWorm::Random(base,rng, dis);

			//check if the sample point is in free space
			if (mw.CheckPosition(sample))
			{
				added++;
				//point is free
				samples.push_back(sample);
			}
		}
	}
}


double PRM::calcWeigth(Eigen::Vector5d a, Eigen::Vector5d b)
{
	//try simple euklid
	return vec5Distance(a, b);
}

std::vector<Eigen::Vector5d> PRM::getShortestPath(std::vector<PRM::Edge>edges, std::vector<Eigen::Vector5d>& samplePoint)
{
	std::vector<Eigen::Vector5d> shortestPath;

	std::vector<float> weights;

	for (int i = 0; i < edges.size(); i++)
	{
		Edge& e = edges[i];
		weights.push_back(calcWeigth(samplePoint[e.first], samplePoint[e.second]));
	}

	//build ajacency matrix
	Dijkstra::adjacency_list_t adjacency_list(samplePoint.size());

	for (int i = 0; i < edges.size(); i++)
	{
		adjacency_list[edges[i].first].push_back(Dijkstra::neighbor(edges[i].second, weights[i]));
		adjacency_list[edges[i].second].push_back(Dijkstra::neighbor(edges[i].first, weights[i]));
	}

	Dijkstra dijkstra;
	std::vector<Dijkstra::weight_t> min_distance;
	std::vector<Dijkstra::vertex_t> previous;
	dijkstra.DijkstraComputePaths(0, adjacency_list, min_distance, previous);
	std::list<Dijkstra::vertex_t> path = dijkstra.DijkstraGetShortestPathTo(1, previous);

	//convert the path into the format we want
	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		Eigen::Vector5d p = samplePoint[(*it)];
		shortestPath.push_back(p);
	}

	return shortestPath;

}

void PRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis,int sampleSize, std::vector<Eigen::Vector5d>& samples)
{
	
	while (true)
	{
		//check if we are done
		if (samples.size() >= sampleSize)
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
				std::cout << samples.size() << "/" << sampleSize << std::endl;
				//point is free
				samples.push_back(sample);
			}
		}
	}
}

std::vector<PRM::Edge> PRM::reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct)
{
	std::mt19937_64 rng(0);

	for (int i = 0; i < nodeFailVct.size(); i++)
	{
		//do a simple threshold check
		if (nodeFailVct[i].failedAttemps >= k / 2)
		{
			//double range = sqrt(nodeFailVct[i].avrgDist);
			double range = 0.7;
			
			std::uniform_real_distribution<double> dis((range/2)*-1, range / 2);

			Eigen::Vector5d base = samplePoints[nodeFailVct[i].index];

			 getSample(mw, rng, dis, resamplePointNumbers,base, samplePoints);
		}
	}

	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, samplePoints, initSampleSize);

	return edges;

}

std::vector<Eigen::VectorXd> PRM::getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, PRM::PRMMetrics& prmMetrics)
{
	clock_t clockStart = clock();	

	

	std::mt19937_64 rng(0);
	std::uniform_real_distribution<double> dis(0, 1.0);

	std::cout << "Starting init sample" << std::endl;
	std::vector<Eigen::Vector5d> initSampels;
	initSampels.push_back(goal);
	initSampels.push_back(start);

	getSample(mw, rng, dis, initSampleSize, initSampels);
	std::vector<PRM::NodeAttemptPair> nodeFailVct;	

	std::cout << "getting connection" << std::endl;
	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, initSampels,0);

	std::cout << "resample" << std::endl;
	std::vector<PRM::Edge> newEdges = reSample(mw, initSampels,nodeFailVct);

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

void PRM::printResult(std::vector<Eigen::VectorXd> path, PRM::PRMMetrics metrics, bool printPath,bool printMetrics) {
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

PRM::~PRM()
{
}
