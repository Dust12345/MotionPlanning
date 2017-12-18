#include "stdafx.h"
#include "PRM.h"
#include "KDT.h"
#include "Dijkstra.h"
#include "time.h"

PRM::PRM(const int initSampleSize,const int k,const int resamplePointNumbers, int ccLowThreshold)
{
	this->initSampleSize = initSampleSize;
	this->k = k;
	this->resamplePointNumbers = resamplePointNumbers;
	this->ccLowThreshold = ccLowThreshold;
}

std::vector<PRM::Edge> PRM::checkConnections(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, KDT::nodeKnn node, PRM::NodeAttemptPair& conFailMetric)
{
	std::vector<PRM::Edge> edges;
	
	// nodeA is the base node
	Eigen::Vector5d nodeA = samplePoints[node.index];
	int failedAttempts = 0;

	for (int i = 0; i < node.nn.size(); i++)
	{
		//nodeB is one of the base nodes nearest neigtbors
		Eigen::Vector5d nodeB = samplePoints[node.nn[i]];
		
		//check if the nodes can be connected
		bool connected = canConnect(mw,nodeA,nodeB);

		if (connected)
		{
			//nodes can be connected, create an edge
			Edge e;
			e.first = node.index;
			e.second = node.nn[i];
			edges.push_back(e);
		}
		else {
			failedAttempts++;
		}
	}

	//keep track of the failed attemps for later resampeling
	conFailMetric.index = node.index;
	conFailMetric.failedAttemps = failedAttempts;

	return edges;

}
double PRM::vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b)
{
	//simple euklidian distance
	Eigen::Vector5d c = a - b;

	double sum = pow(c[0], 2)+ pow(c[1], 2)+ pow(c[2], 2)+pow(c[3], 2)+ pow(c[4], 2);

	double length = std::sqrt(sum);

	return length;
}

bool PRM::canConnect(WormCell& mw, Eigen::Vector5d a, Eigen::Vector5d b){
	//how many collision checks will be done
	int checks = 10;
	//simple but not efficent way to check
	Eigen::Vector5d c = a - b;

	c = c / (checks-1);

	for (int i = 0; i < (checks - 1); i++){
		Eigen::Vector5d d = b + (c*i);

		if(!mw.CheckPosition(d)){
			return false;
		}	
	}
	return true;
}

std::vector<PRM::Edge> PRM::connectionTesting(WormCell& mw, std::vector<PRM::NodeAttemptPair>& nodeFailVct, std::vector<Eigen::Vector5d>& samplePoints,int startIndex){
	std::vector<PRM::Edge> edges;
	KDT kdTree;
	std::vector<KDT::nodeKnn> nodeNNVct;

	//get the nearest neigtbors for all nodes
	kdTree.getKNN(samplePoints, nodeNNVct, startIndex,k);

	for (int i = 0; i < nodeNNVct.size(); i++){
		//for each node, check the connectifity to its NN
		PRM::NodeAttemptPair metric;
		metric.avrgDist = nodeNNVct[i].avrgDist;
		std::vector<Edge> edgesForNode = checkConnections(mw, samplePoints, nodeNNVct[i], metric);
		nodeFailVct.push_back(metric);
		
		//add the adges
		for (int j = 0; j < edgesForNode.size(); j++){			
			edges.push_back(edgesForNode[j]);
		}
	}
	return edges;
}

void PRM::getSample(WormCell& mw, std::mt19937_64& rng, std::uniform_real_distribution<double>& dis, int sampleSize, Eigen::Vector5d base, std::vector<Eigen::Vector5d>& samples){
	int added = 0;
	//to prevent an endless loop, we stop at some point no matter what
	int maxTries = sampleSize * 50;

	while (true){
		//check if we are done
		if (added > sampleSize||added >= maxTries){
			//get enough sample points
			return;
		}
		else {
			//not enough, try to get another one
			Eigen::Vector5d sample = MyWorm::Random(base,rng, dis);

			//check if the sample point is in free space
			if (mw.CheckPosition(sample)){
				//point is free
				samples.push_back(sample);
				added++;
			}
		}
	}
}


double PRM::calcWeigth(Eigen::Vector5d a, Eigen::Vector5d b)
{
	//dist calculation with correct wrapping for rotations
	double result = sqrt(pow(a[0] + b[0], 2) + pow(a[1] + b[1], 2));

	float pi = 3.14159265358979323846;

	for (int i = 2; i < 5; i++) {
		double noWrapDist = std::abs(a[i] - b[i]);

		double wrapDist = 0;

		if (a[i] < b[i]) {
			double d1 = std::abs((pi*-1) - a[i]);
			double d2 = std::abs((pi)-b[i]);
			wrapDist = d1 + d2;
		}
		else
		{
			double d1 = std::abs((pi*-1) - b[i]);
			double d2 = std::abs((pi)-a[i]);
			wrapDist = d1 + d2;
		}

		if (wrapDist < noWrapDist) {
			result += wrapDist;
		}
		else {
			result += noWrapDist;
		}

	}

	return result;

}

std::vector<Eigen::Vector5d> PRM::getShortestPath(std::vector<PRM::Edge>edges, std::vector<Eigen::Vector5d>& samplePoint)
{
	std::vector<Eigen::Vector5d> shortestPath;

	std::vector<float> weights;

	//calculate the weigths of the edges
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

	//run dijstra
	Dijkstra dijkstra;
	std::vector<Dijkstra::weight_t> min_distance;
	std::vector<Dijkstra::vertex_t> previous;
	dijkstra.DijkstraComputePaths(samplePoint.size()-2, adjacency_list, min_distance, previous);
	std::list<Dijkstra::vertex_t> path = dijkstra.DijkstraGetShortestPathTo(samplePoint.size() - 1, previous);

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
	int added = 0;
	//to prevent an endless loop, we stop at some point no matter what
	int maxTries = sampleSize * 25;
	
	while (true)
	{
		//check if we are done
		if (added > sampleSize||added >= maxTries)
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
				added++;
			}
		}
	}
}

std::vector<PRM::Edge> PRM::reSample(WormCell& mw, std::vector<Eigen::Vector5d>& samplePoints, std::vector<PRM::NodeAttemptPair> nodeFailVct){
	std::mt19937_64 rng(0);

	for (int i = 0; i < nodeFailVct.size(); i++){
		//do a simple threshold check
		if (nodeFailVct[i].failedAttemps >= k / 2){
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

			std::uniform_real_distribution<double> dis(min,max );

			//the base point around which the new samples should be created
			Eigen::Vector5d base = samplePoints[nodeFailVct[i].index];

			getSample(mw, rng, dis, resamplePointNumbers,base, samplePoints);
		}
	}
	std::cout << "Testing connections again" << std::endl;

	//test the new samples for their connectifity
	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, samplePoints, initSampleSize);

	return edges;
}

std::vector<Eigen::VectorXd> PRM::getPath(WormCell& mw, Eigen::VectorXd start, Eigen::VectorXd goal, PRM::PRMMetrics& prmMetrics)
{
	//start a clock to keep track on how long it took
	clock_t clockStart = clock();	

	//set up the rng and random distribution for the sampleing
	std::mt19937_64 rng(0);
	std::uniform_real_distribution<double> dis(0, 1);

	std::cout << "Starting init sample" << std::endl;
	std::vector<Eigen::Vector5d> configurationsPoints;

	//get the inital sample points
	getSample(mw, rng, dis, initSampleSize, configurationsPoints);
	std::vector<PRM::NodeAttemptPair> nodeFailVct;	
	
	std::cout << "checking connection" << std::endl;
	//test the connectifity of the sample points
	std::vector<PRM::Edge> edges = connectionTesting(mw, nodeFailVct, configurationsPoints,0);

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
	if (goal[0] < 0 || goal[0]>1|| goal[1] < 0 || goal[1]>1
		|| start[0] < 0 || start[0]>1 || start[1] < 0 || start[1]>1)
	{
		std::cout << "unreachable goal or start" << std::endl;
	

		prmMetrics.numberOfNN = k;
		prmMetrics.numberCC = getConectedComponentNumber(edges);
		prmMetrics.numberOfNodes = configurationsPoints.size();
		prmMetrics.numberOfEdges = edges.size();
		double t = (clock() - clockStart) / CLOCKS_PER_SEC;
		prmMetrics.runtime = t;
		return p;
	}

	
	configurationsPoints.push_back(goal);
	configurationsPoints.push_back(start);
	std::vector<PRM::Edge> newEdges = connectionTesting(mw, nodeFailVct, configurationsPoints, configurationsPoints.size()-2);
	for (int i = 0; i < newEdges.size(); i++)
	{
		edges.push_back(newEdges[i]);
	}

	std::cout << "calculating the shortest path" << std::endl;
	std::vector<Eigen::Vector5d> path = getShortestPath(edges, configurationsPoints);	

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
	int numberOfCC = getConectedComponentNumber(edges);
	
	prmMetrics.numberOfNN = k;
	prmMetrics.numberCC = numberOfCC;
	prmMetrics.numberOfNodes = configurationsPoints.size();
	prmMetrics.numberOfEdges = edges.size();
	double t = (clock() - clockStart) / CLOCKS_PER_SEC;
	prmMetrics.runtime = t;
	return p;
}

bool PRM::inCC(PRM::CC cc, int nodeIndex)
{
	//check if the node index is already in the list
	for (int i = 0; i < cc.nodesInCC.size(); i++)
	{
		if (cc.nodesInCC[i] == nodeIndex) {
			return true;
		}
	}
	return false;
}

void PRM::addEdgeToCCs(std::vector<PRM::CC> & cc, PRM::Edge& edge)
{
	//the indecies of the edges endpoints
	int indexA = edge.first;
	int indexB = edge.second;

	//the indecies of the CCs in which A or B are, if they are in one at all
	int ccIndexA = -1;
	int ccIndexB = -1;

	//search for A in the knows CCs
	for (int i = 0; i < cc.size(); i++)
	{
		if (inCC(cc[i], indexA)){
			ccIndexA = i;
			break;
		}
	}

	//search for B in the knows CCs
	for (int i = 0; i < cc.size(); i++)
	{
		if (inCC(cc[i], indexB)) {
			ccIndexB = i;
			break;
		}
	}

	//there are four cases
	//1. edge is in no component => create a new one
	//2. one end of the edge belongs to a CC => add the other end to the CC
	//3. both ends of the edge belong to different CC => combine both CC into one
	//4. both ends belong to the same CC => nothing to do

	if (ccIndexA == -1 && ccIndexB == -1)
	{
		//case 1
		CC newCC;
		newCC.nodesInCC.push_back(indexA);
		newCC.nodesInCC.push_back(indexB);
		cc.push_back(newCC);
	}
	else if (ccIndexA == -1 && ccIndexB != -1)
	{
		//case 2
		cc[ccIndexB].nodesInCC.push_back(indexA);

	}
	else if (ccIndexB == -1 && ccIndexA != -1)
	{
		//case 2, but with the other end beeing connected
		cc[ccIndexA].nodesInCC.push_back(indexB);
	}
	else if (ccIndexB != ccIndexA) {
		//add A to B
		for (int j = 0; j < cc[ccIndexA].nodesInCC.size(); j++)
		{
			cc[ccIndexB].nodesInCC.push_back(cc[ccIndexA].nodesInCC[j]);
		}

		//remove A
		cc.erase(cc.begin() + ccIndexA);
	}
}

int PRM::getConectedComponentNumber(std::vector<PRM::Edge>& edges){
	std::vector<PRM::CC> connectedComp;

	//add all edges to the CCs
	for (int i = 0; i < edges.size(); i++){
		addEdgeToCCs(connectedComp, edges[i]);
	}
	//the number of CCs is simply the size of the vector
	return connectedComp.size();
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
		std::cout << "------- Metric PRM -------" << std::endl;
		std::cout << "Number of nodes: " << metrics.numberOfNodes << std::endl;
		std::cout << "Number of edges: " << metrics.numberOfEdges << std::endl;
		std::cout << "Number of nearest neighbours: " << metrics.numberOfNN << std::endl;
		std::cout << "Number of connected components: " << metrics.numberCC << std::endl;
		std::cout << "Runtime: " << metrics.runtime << "sec" << std::endl;
	}
}

PRM::~PRM()
{
}
