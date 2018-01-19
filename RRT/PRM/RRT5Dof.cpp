#include "stdafx.h"
#include "RRT5Dof.h"
#include <iostream>
#include <fstream>
#include "cell.h"
#include "KDT.h"
#include <iostream>
#include <fstream>
#include "time.h"
#include <limits>
#include <random>
#include "Dijkstra.h"


using namespace std;

RRT5Dof::RRT5Dof(Eigen::VectorXd START, Eigen::VectorXd GOAL) : RRTSimple::RRTSimple(){
	this->START = START;
	this->GOAL = GOAL;
	uniform_real_distribution<double> basedDistro(-0.2, 0.2);
	dis2 = basedDistro;
}

RRT5Dof::~RRT5Dof(){

}

RRT5Dof::Result RRT5Dof::getPath(WormCell &cell, int numberOfSamples,RRT5dofMetrics &metric, float stepSize, double percent) {
	
	this->cell = &cell;
	this->percent = percent;
	this->stepSize = stepSize;
	this->metric = &metric;
	vector<Eigen::VectorXd> tmpResult;
	RRT5Dof::Result result;


	//check for special cases
	//check if the goal and the start can be reached
	if (!cell.CheckPosition(GOAL) || !cell.CheckPosition(START))
	{
		std::cout << "unreachable goal or start" << std::endl;

		result.path = tmpResult;
		result.tree = tree;

		return result;
	}

	//check if start and goal are within the boundry
	if (this->GOAL[0] < 0 || this->GOAL[0]>1 || this->GOAL[1] < 0 || this->GOAL[1]>1
		|| this->START[0] < 0 || this->START[0]>1 || this->START[1] < 0 || this->START[1]>1)
	{
		std::cout << "unreachable goal or start" << std::endl;

		result.path = tmpResult;
		result.tree = tree;

		return result;
	}

	clock_t clockStart = clock();
	
	//get the sample points
	std::cout << "Process 1: Generating samples" << endl;
	vector<Eigen::Vector5d> samples = getSamples(numberOfSamples);
	std::cout << "Process 2: Connecting nodes" << endl;
	
	//build the rrt
	connectNodes(samples);
	this->metric->runtime = (clock() - clockStart) / CLOCKS_PER_SEC;
	std::cout << "Process 3: Generating gnuplot file" << endl;
	
	writeGnuplotFile(tree.nodes, "RRT5dof", tree.edges);

	//check if the goal was reached
	for (int i = 0; i < tree.nodes.size();i++) {

		if (tree.nodes[i] == this->GOAL) {
			this->goalIndex = i;
			break;
		}
	}


	if (this->goalIndex == -1) {
		std::cout << "No connection to goal" <<endl;
	}
	else {
		std::cout << "Process 4: calculating the shortest path" <<endl;
		//goal and start are switched here since the algo goes from goal to start
		vector<Eigen::Vector5d> path = getShortestPath(tree.edges, tree.nodes, this->goalIndex, 0 );

		//path of size one means dijstra found no path
		if (path.size() > 1)
		{
			for (int i = 0; i < path.size(); i++)
			{
				tmpResult.push_back(path[i]);
			}
		}
		else {
			std::cout << "No path found" <<endl;
		}
	}

	result.path = tmpResult;
	result.tree = tree;

	return result;
}


bool RRT5Dof::canConnect(WormCell& mw, Eigen::Vector5d a, Eigen::Vector5d b) {
	//how many collision checks will be done
	int checks = 30;
	//simple but not efficent way to check
	Eigen::Vector5d step = a - b;

	step = step / (checks - 1);

	for (int i = 0; i < (checks - 1); i++) {
		Eigen::Vector5d pos = b + (step*i);

		if (!mw.CheckPosition(pos)) {
			return false;
		}
	}
	return true;
}


vector<RRT5Dof::Edge> RRT5Dof::connectNodes(vector<Eigen::Vector5d>nodes) {

	vector<Edge> edges;
	bool isNodeNew = false;
	int nearestNeighbourIndex;
	tree.nodes.push_back(this->START);
	this->dkdt.addPoint(this->START);
	this->generatedNodes.insert({ convertVector5dToString(this->START),tree.nodes.size() - 1 });

	for (int i = 0; i < nodes.size(); i++)
	{
		
		Eigen::Vector5d currentNode = nodes[i];
		int currentNodeIndex;
	
		//check if the node is new or was added before
		auto search = this->generatedNodes.find(convertVector5dToString(currentNode));
		if (search != this->generatedNodes.end()) {
			currentNodeIndex = search->second;
			isNodeNew = false;

		}
		else {
			currentNodeIndex = - 1;
			isNodeNew = true;
		}

		//get the NN of the current node
		nearestNeighbourIndex = dkdt.getNN(currentNode);
		connectNode(currentNode, nearestNeighbourIndex, currentNodeIndex, isNodeNew,i);
	}

	return edges;
}

void RRT5Dof::connectNode(Eigen::Vector5d node, int nearestNeighbourIndex, int nodeIndex, bool isNodeNew,int sampleIndex) {

		// nearest neighbour from the node
		Eigen::Vector5d nearestNeighbourFromThePoint = tree.nodes[nearestNeighbourIndex];

		// Distance between node and its nearestNeighbour 
		double disNodeToNearestNeighbour = vec5Distance(node, nearestNeighbourFromThePoint);

		//search Find nodes that are connected to the nearestNeighbour
		Eigen::Vector5d pointOnTheLine;
		vector<int> relationsNodesFromNearestNeighbour = getAllRelationNodes(nearestNeighbourIndex, tree.edges);

		double mindistanceBetweenNodeAndLine = numeric_limits<double>::max();
		int mindistanceNodeIndex;
		Eigen::Vector5d mindPointOnTheLine;

		for (int i = 0; i < relationsNodesFromNearestNeighbour.size(); i++) {

			//compute the distance from a Point to a linie and get the point on the line
			double distanceBetweenNodeAndLine = distPointToLine(node, nearestNeighbourFromThePoint, tree.nodes[relationsNodesFromNearestNeighbour[i]], pointOnTheLine);

			if (distanceBetweenNodeAndLine < mindistanceBetweenNodeAndLine) {
				mindistanceBetweenNodeAndLine = distanceBetweenNodeAndLine;
				mindistanceNodeIndex = relationsNodesFromNearestNeighbour[i];
				mindPointOnTheLine = pointOnTheLine;
			}

		}

		//check if a line was found with a shorter distance 
		if (disNodeToNearestNeighbour > mindistanceBetweenNodeAndLine) {

			Eigen::Vector5d stopVector = stopconfiguration(node, mindPointOnTheLine, this->cell);

			if (stopVector == GOAL) {
				goalFound = true;
			}


			// Wenn der stopVector nicht der mindPointOnTheLine ist, füge den mindPointOnTheLine
			if (stopVector != mindPointOnTheLine) {

				// add point on the line
				tree.nodes.push_back(mindPointOnTheLine);
				this->dkdt.addPoint(mindPointOnTheLine);

				// add Edge between new Point and the point on the line
				splittEdges(nearestNeighbourIndex, mindistanceNodeIndex, tree.nodes.size() - 1, tree.edges);
				this->metric->numberOfEdges++;
				this->metric->numberOfNodes++;

				if (stopVector == node) {

					if (isNodeNew == true) {
						tree.nodes.push_back(node);
						this->generatedNodes.insert({ convertVector5dToString(node),tree.nodes.size() - 1 });
						this->dkdt.addPoint(node);
						tree.edges.push_back(Edge(tree.nodes.size() - 1, tree.nodes.size() - 2));
					}
					else {
						tree.edges.push_back(Edge(tree.nodes.size() - 1, nodeIndex));
					}

					//the node we just added was a node that is close to the goal
					//because of that is is likly we can connect to the goal and try to do so
					if (sampleIndex%nearSamlpleRate == 0&&foundPathToGoal && !goalFound) {
						bool connacable = canConnect((*cell), node, GOAL);

						if (connacable) {
							tree.nodes.push_back(GOAL);
							this->generatedNodes.insert({ convertVector5dToString(GOAL),tree.nodes.size() - 1 });
							this->dkdt.addPoint(GOAL);
							tree.edges.push_back(Edge(tree.nodes.size() - 1, tree.nodes.size() - 2));
							foundPathToGoal = false;
						}

					}
					
					this->metric->numberOfEdges++;
				}
				else {
					tree.nodes.push_back(stopVector);
					this->dkdt.addPoint(stopVector);
					// add Edge between new Point and the point on the line
					tree.edges.push_back(Edge(tree.nodes.size() - 2, tree.nodes.size() - 1));
					this->metric->numberOfEdges++;
					this->metric->numberOfNodes++;
				}
			}
		}
		else{
			Eigen::Vector5d stopVector2 = stopconfiguration(node, nearestNeighbourFromThePoint, this->cell);

			if (stopVector2 == GOAL) {
				goalFound = true;
			}

			// Wenn der stopVector nicht der mindPointOnTheLine ist, füge den mindPointOnTheLine
			if (stopVector2 != nearestNeighbourFromThePoint) {

				if (stopVector2 == node) {

					if (isNodeNew == true) {
						tree.nodes.push_back(node);
						this->dkdt.addPoint(node);
						tree.edges.push_back(Edge(tree.nodes.size() - 1, nearestNeighbourIndex));
						this->generatedNodes.insert({ convertVector5dToString(node),tree.nodes.size() - 1 });
					}
					else {
						tree.edges.push_back(Edge(nearestNeighbourIndex, nodeIndex));
					}

					//the node we just added was a node that is close to the goal
					//because of that is is likly we can connect to the goal and try to do so
					if (sampleIndex%nearSamlpleRate == 0 && foundPathToGoal && !goalFound) {
						bool connectable = canConnect((*cell), node, GOAL);

						if (connectable) {
							tree.nodes.push_back(GOAL);
							this->generatedNodes.insert({ convertVector5dToString(GOAL),tree.nodes.size() - 1 });
							this->dkdt.addPoint(GOAL);
							tree.edges.push_back(Edge(tree.nodes.size() - 1, tree.nodes.size() - 2));
							foundPathToGoal = false;
						}
					}

					this->metric->numberOfEdges++;
				}
				else {
					tree.nodes.push_back(stopVector2);
					this->dkdt.addPoint(stopVector2);
					tree.edges.push_back(Edge(nearestNeighbourIndex, tree.nodes.size() - 1));
					this->metric->numberOfEdges++;
					this->metric->numberOfNodes++;
				}
			}
		}
}

vector<Eigen::Vector5d> RRT5Dof::getSamples(int numberofSample) {
	vector<Eigen::Vector5d> samples;

	for (int i = 0; i <= numberofSample; i++) {

		Eigen::Vector5d sample;
		bool foundSample = false;

		// search until a point in free space has been found
		while (!foundSample) {
			// check if the next Sample is a random or the goal
			if (isNextRandomSampleAsGoal(this->percent,i)) {
			
				sample = GOAL;
				
				this->metric->numberOfGoalNodes++;
				foundSample = true;
			
			}
			else {
				
				//generate a random sample near the goal
				if (i % nearSamlpleRate == 0&& i>numberofSample/2) {
					sample = MyWorm::Random(GOAL,rng, dis2);
				}
				else {
					// generating a random sample
					sample = MyWorm::Random(rng, dis);
				}

				
				// check if the random sample lie in free space
				if (this->cell->CheckPosition(sample)) {
					foundSample = true;
				}
			}
		}

		samples.push_back(sample);
		this->metric->numberOfNodes++;
	}

	return samples;
}



Eigen::Vector5d RRT5Dof::stopconfiguration(Eigen::Vector5d node, Eigen::Vector5d nearestNeighbourNode, WormCell *cell)
{
	//how many collision checks will be done
	int checks = 30;
	//simple but not efficent way to check
	Eigen::Vector5d step = node - nearestNeighbourNode;
	Eigen::Vector5d last = nearestNeighbourNode;
	step = step / (checks - 1);

	for (int i = 0; i < (checks - 1); i++) {
		Eigen::Vector5d pos = nearestNeighbourNode + (step*i);

		if (!cell->CheckPosition(pos)) {
			//return the last free configuration
			return last;
		}
		else {
			//store the last free configuration
			last = pos;
		}
	}
	return node;
}


// check if the next Sample is a random or the goal. The result depend on the percent value
bool RRT5Dof::isNextRandomSampleAsGoal(double percent,int number)
{
	if (number % goalSampleRate == 0) {
		return true;
	}
	else {
		return false;
	}
}

// compute the min. distance between a Point and a linie and compute the orthogonal point on the line
double RRT5Dof::distPointToLine(Eigen::Vector5d P, Eigen::Vector5d P0, Eigen::Vector5d P1, Eigen::Vector5d &orthogonalPoint) {

	Eigen::Vector5d v = P1 - P0;
	Eigen::Vector5d w = P - P0;

	double c1 = w.dot(v); // scalar product
	if (c1 <= 0.0) {
		orthogonalPoint = P0;
		return vec5Distance(P, P0);
	}
	double c2 = v.dot(v); // scalar product
	if (c2 <= c1) {
		orthogonalPoint = P1;
		return vec5Distance(P, P1);
	}
	double b = c1 / c2;
	orthogonalPoint = P0 + b * v;

	this->metric->numberOfNodesOnTheLine++;
	return vec5Distance(P, orthogonalPoint);
}

string RRT5Dof::convertVector5dToString(Eigen::Vector5d node) {
	return to_string(node[0])+ to_string(node[1])+ to_string(node[2])+ to_string(node[3])+to_string(node[4]);
}

// print all nodes and metric information on the console
void RRT5Dof::printResult(vector<Eigen::Vector5d> &nodes, RRT5Dof::RRT5dofMetrics &metrics, bool printNodes, bool printMetrics) {
	if (nodes.size() > 1 && printNodes)
	{
		cout << "-------------------- Nodes ------------------------" << endl;
		for (int i = 0; i < nodes.size(); i++)
		{
			cout << "Point " << i << " x:" << nodes[i].x() << " y:" << nodes[i].y() << endl;
		}
		cout << "---------------------------------------------" << endl;
	}

	if (printMetrics) {
		cout << "------- RRT5dof Metrics -------" << endl;
		cout << "Number of nodes: " << metrics.numberOfNodes << endl;
		cout << "Number of edges: " << metrics.numberOfEdges << endl;
		cout << "Number of nodes as goal: " << metrics.numberOfGoalNodes << endl;
		cout << "Number of nodes on the line: " << metrics.numberOfNodesOnTheLine << endl;
		cout << "Number of lines splitts: " << metrics.numberOfLineSplitts << endl;
		cout << "Runtime: " << metrics.runtime << "sec" << endl;
	}

	ofstream myfile;
	string filename = "RT5DofMeta.txt";
	myfile.open(filename);
	myfile << "--------------------Nodes------------------------" << endl;
	Eigen::Vector5d node;
	for (int j = 0; j < nodes.size(); j++){
	
		node = nodes[j];
		if (node == this->GOAL) {
			myfile << "Point " << j << " x:" << node[0] << " y:" << node[1] << " r1:" << node[2] << " r2:" << node[3] << " r3:" << node[4] <<"-- GOAL --"<< endl;
		}
		else {
			myfile << "Point " << j << " x:" << node[0] << " y:" << node[1] << " r1:" << node[2] << " r2:" << node[3] << " r3:" << node[4] << endl;
		}
	}
	myfile << "---------------------------------------------" << endl;
	myfile << "Number of nodes: " << metrics.numberOfNodes << endl;
	myfile << "Number of edges: " << metrics.numberOfEdges << endl;
	myfile << "Number of nodes as goal: " << metrics.numberOfGoalNodes << endl;
	myfile << "Number of nodes on the line: " << metrics.numberOfNodesOnTheLine << endl;
	myfile << "Number of lines splitts: " << metrics.numberOfLineSplitts << endl;
	myfile << "Runtime: " << metrics.runtime << "sec" << endl;
	myfile.close();
}

vector<Eigen::Vector5d> RRT5Dof::getShortestPath(vector<Edge>edges, vector<Eigen::Vector5d>& samplePoint, int startIndex, int endIndex)
{
	std::vector<Eigen::Vector5d> shortestPath;

	std::vector<float> weights;

	//calculate the weigths of the edges
	for (int i = 0; i < edges.size(); i++)
	{
		Edge& e = edges[i];
		weights.push_back(vec5Distance(samplePoint[e.first], samplePoint[e.second]));
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
	dijkstra.DijkstraComputePaths(endIndex, adjacency_list, min_distance, previous);
	std::list<Dijkstra::vertex_t> path = dijkstra.DijkstraGetShortestPathTo(startIndex, previous);

	//convert the path into the format we want
	for (std::list<int>::iterator it = path.begin(); it != path.end(); ++it)
	{
		Eigen::Vector5d p = samplePoint[(*it)];
		shortestPath.push_back(p);
	}

	return shortestPath;

}

vector<int> RRT5Dof::getAllRelationNodes(int currentNodeIndex, vector<Edge> &edges) {
	vector<int> relationNodeIndex;

	for (int i = 0; i < edges.size(); i++) {
		if (edges[i].first == currentNodeIndex) {
			relationNodeIndex.push_back(edges[i].second);
		}
		else if (edges[i].second == currentNodeIndex) {
			relationNodeIndex.push_back(edges[i].first);
		}
	}

	return relationNodeIndex;
}

void RRT5Dof::splittEdges(int nodeIndexOne, int nodeIndexTwo, int nodeIndexBetween, vector<Edge> &edges) {

	for (int i = 0; i < edges.size(); i++) {

		if ((edges[i].first == nodeIndexOne && edges[i].second == nodeIndexTwo)) {
			edges[i].second = nodeIndexBetween;
			tree.edges.push_back(Edge(nodeIndexBetween, nodeIndexTwo));
			return;
		}
		else if ((edges[i].first == nodeIndexTwo && edges[i].second == nodeIndexOne)) {
			edges[i].second = nodeIndexBetween;
			tree.edges.push_back(Edge(nodeIndexBetween, nodeIndexOne));
			return;
		}
	}
}