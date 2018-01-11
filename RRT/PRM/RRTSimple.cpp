#include "stdafx.h"
#include "RRTSimple.h"
#include <iostream>
#include <fstream>
#include "cell.h"
#include "KDT.h"
#include <iostream>
#include <fstream>
#include "time.h"
#include <limits>

using namespace std;

RRTSimple::RRTSimple(){
	uniform_real_distribution<double> dis(0, 1);
	mt19937_64 rng(0);

	this->dis = dis;
	this->rng = rng;
}

RRTSimple::~RRTSimple() {

}

RRTSimple::Tree RRTSimple::createTree(const Eigen::Vector5d start, const int numberOfSample, RRTSimple::SimpleRRTMetrics & metrics){

	this->metrics = &metrics;
	tree.nodes.push_back(start);

	clock_t clockStart = clock();
	cout << "Process 1: Generating samples"<< endl;
	vector<Eigen::Vector5d> samples = getSamples(numberOfSample);
	cout << "Process 2: Connecting nodes" << endl;
	connectNodes(samples);

	metrics.runtime = (clock() - clockStart) / CLOCKS_PER_SEC;
	// number of nodes without the start node
	metrics.numberOfNodes = tree.nodes.size() - 1;
	metrics.numberOfNodes = tree.edges.size();

	cout << "Process 3: Generating gnuplot file" << endl;
	writeGnuplotFile(tree.nodes,"simpleRRT", tree.edges);

	return tree;
}

vector<RRTSimple::Edge> RRTSimple::connectNodes(vector<Eigen::Vector5d>nodes) {

	std::vector<RRTSimple::Edge> edges;
	KDT kdTree;

	for (int i = 0; i < nodes.size(); i++) {

		tree.nodes.push_back(nodes[i]);
		vector<KDT::nodeKnn> nodeNNVct;

		kdTree.getKNN(tree.nodes, nodeNNVct, 0, 2);
		connectNode(nodes[i], nodeNNVct, tree.nodes.size()-1);
	}

	return edges;
}

void RRTSimple::connectNode(Eigen::Vector5d node, vector<KDT::nodeKnn> nodeNNVct, int nodeIndex){

	KDT::nodeKnn KDTnodeNearestNeighbour;

	// find the node in nodeNNVct
	for (int i = nodeNNVct.size()-1; 0 < i; i--) {
		if (nodeNNVct[i].index == nodeIndex) {
			KDTnodeNearestNeighbour = nodeNNVct[i];
			break;
		}
	}

	// check if we found one a nearest neighbour point
	if (KDTnodeNearestNeighbour.nn.size() == 0) {
		cout << "nearest neighbour could not found";
	}else{
		// nearest neighbour from the node
		Eigen::Vector5d nearestNeighbourFromThePoint = tree.nodes[KDTnodeNearestNeighbour.nn[0]];

		// Distance between node and its nearestNeighbour 
		double disNodeToNearestNeighbour = vec5Distance(node, nearestNeighbourFromThePoint);

		//search Find nodes that are connected to the nearestNeighbour
		Eigen::Vector5d pointOnTheLine;
		vector<int> relationsNodesFromNearestNeighbour = getAllRelationNodes(KDTnodeNearestNeighbour.nn[0], tree.edges);

		double mindistanceBetweenNodeAndLine = numeric_limits<double>::max();
		double mindistanceNodeIndex;
		for (int i = 0; i < relationsNodesFromNearestNeighbour.size(); i++) {

			//compute the distance from a Point to a linie and get the point on the line
			double distanceBetweenNodeAndLine = distPointToLine(node, nearestNeighbourFromThePoint, tree.nodes[relationsNodesFromNearestNeighbour[i]], pointOnTheLine);
		
			if (distanceBetweenNodeAndLine < mindistanceBetweenNodeAndLine) {
				mindistanceBetweenNodeAndLine = distanceBetweenNodeAndLine;
				mindistanceNodeIndex = relationsNodesFromNearestNeighbour[i];
			}

		}

		//check if a line was found with a shorter distance 
		if (disNodeToNearestNeighbour > mindistanceBetweenNodeAndLine) {

			// add point on the line
			tree.nodes.push_back(pointOnTheLine);

			// add Edge between new Point and the point on the line
			tree.edges.push_back(Edge(nodeIndex, tree.nodes.size() - 1));
			splittEdges(KDTnodeNearestNeighbour.nn[0], mindistanceNodeIndex,tree.nodes.size() - 1,tree.edges);
			this->metrics->numberOfEdges++;
			this->metrics->numberOfNodes++;

			return;
		}

		// add the edge between the node and its nearest neighbour
		tree.edges.push_back(Edge(nodeIndex, KDTnodeNearestNeighbour.nn[0]));
		this->metrics->numberOfEdges++;
	}
}

// generate a 2d sample with a fixed range and seed
Eigen::Vector5d RRTSimple::getSample() {

	Eigen::Vector5d sample = MyWorm::Random(rng, dis);
	sample[2] = 0;
	sample[3] = 0;
	sample[4] = 0;
	this->metrics->numberOfNodes++;
	return sample;
}

// generate a 2d samples with a fixed range and seed
vector<Eigen::Vector5d> RRTSimple::getSamples(int numberofSample) {

	vector<Eigen::Vector5d> samples;
	
	for (int i = 0; i <= numberofSample; i++) {
		Eigen::Vector5d sample = MyWorm::Random(rng, dis);
		sample[2] = 0;
		sample[3] = 0;
		sample[4] = 0;

		samples.push_back(sample);
		this->metrics->numberOfNodes++;
		//cout << "Point "<<i<<" x:"<<sample[0]<<" y:"<<sample[1] << endl;
	}

	return samples;
}

// write a gnuplot file witch all nodes and its egdes
void RRTSimple::writeGnuplotFile(vector<Eigen::Vector5d> &points, string filename, vector<Edge> &edges)
{
	ofstream myfile;
	myfile.open(filename+".dat");

	myfile << "set title 'Graph'" << endl;
	myfile << "set size ratio 1.0" << endl;
	myfile << "set xrange[0:1]" << endl;
	myfile << "set yrange[0:1]" << endl;

	for (int i = 0; i < tree.edges.size(); i++)
	{
		myfile << tree.nodes[tree.edges[i].first].x() << " " << tree.nodes[tree.edges[i].first].y() << endl;
		myfile << tree.nodes[tree.edges[i].second].x() << " " << tree.nodes[tree.edges[i].second].y() << endl << endl;
	}
	myfile.close();
}

// compute the distance between two vectors
double RRTSimple::vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b)
{
	//simple euklidian distance
	Eigen::Vector5d c = a - b;
	double sum = pow(c[0], 2) + pow(c[1], 2) + pow(c[2], 2) + pow(c[3], 2) + pow(c[4], 2);

	double length = sqrt(sum);

	return length;
}

// get all nodesindexes which have a relation with the current node (nodeindex)
vector<int> RRTSimple::getAllRelationNodes(int currentNodeIndex, vector<Edge> &edges) {
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

void RRTSimple::splittEdges(int nodeIndexOne, int nodeIndexTwo, int nodeIndexBetween, vector<Edge> &edges) {

	for (int i = 0; i < edges.size(); i++) {

		if ((edges[i].first == nodeIndexOne && edges[i].second == nodeIndexTwo)) {
			edges[i].second = nodeIndexBetween;
			tree.edges.push_back(Edge(nodeIndexBetween, nodeIndexTwo));
			this->metrics->numberOfLineSplitts++;
			return;
		}
		else if ((edges[i].first == nodeIndexTwo && edges[i].second == nodeIndexOne)) {
			edges[i].second = nodeIndexBetween;
			tree.edges.push_back(Edge(nodeIndexBetween, nodeIndexOne));
			this->metrics->numberOfLineSplitts++;
			return;
		}
	}
}

// compute the min. distance between a Point and a linie and compute the orthogonal point on the line
double RRTSimple::distPointToLine(Eigen::Vector5d PV, Eigen::Vector5d PV0, Eigen::Vector5d PV1, Eigen::Vector5d &orthogonalPoint) {
	Eigen::Vector2d P,P0,P1;
	
	P[0] = PV[0];
	P[1] = PV[1];

	P0[0] = PV0[0];
	P0[1] = PV0[1];

	P1[0] = PV1[0];
	P1[1] = PV1[1];

	Eigen::Vector2d v = P1 - P0;
	Eigen::Vector2d w = P - P0;

	double c1 = w.dot(v); // scalar product
	if (c1 <= 0.0){
		orthogonalPoint = PV0;
		return vec5Distance(PV, PV0);
	}
	double c2 = v.dot(v); // scalar product
	if (c2 <= c1) {
		orthogonalPoint = PV1;
		return vec5Distance(PV, PV1);
	}
	double b = c1 / c2;
	Eigen::Vector2d Pb = P0 + b * v;

	orthogonalPoint[0] = Pb[0];
	orthogonalPoint[1] = Pb[1];
	orthogonalPoint[2] = 0;
	orthogonalPoint[3] = 0;
	orthogonalPoint[4] = 0;
	this->metrics->numberOfNodesOnTheLine++;
	return vec5Distance(PV, orthogonalPoint);
}

// print all nodes and metric information on the console
void RRTSimple::printResult(vector<Eigen::Vector5d> &nodes, RRTSimple::SimpleRRTMetrics &metrics, bool printNodes, bool printMetrics) {
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
		cout << "------- Metric PRM -------" <<endl;
		cout << "Number of nodes: " << metrics.numberOfNodes << endl;
		cout << "Number of edges: " << metrics.numberOfEdges << endl;
		cout << "Number of nodes on the line: " << metrics.numberOfNodesOnTheLine << endl;
		cout << "Number of lines splitts: " << metrics.numberOfLineSplitts << endl;
		cout << "Runtime: " << metrics.runtime << "sec" << endl;
	}
}

