#include "stdafx.h"
#include "RRTSimple.h"
#include <iostream>
#include <fstream>
#include "cell.h"
#include "KDT.h"
#include <iostream>
#include <fstream>

using namespace std;

RRTSimple::RRTSimple(){
	uniform_real_distribution<double> dis(0, 1);
	mt19937_64 rng(0);

	this->dis = dis;
	this->rng = rng;
}

RRTSimple::~RRTSimple() {

}

void RRTSimple::createTree(const Eigen::Vector5d start, const int numberOfSample){

	tree.nodes.push_back(start);
	//tree.edges.push_back(Edge(0,0));
	vector<Eigen::Vector5d> samples = getSamples(numberOfSample);
	connectNodes(samples, 0);

	writeGnuplotFile(tree.nodes,"simpleRRT", tree.edges);
}

vector<RRTSimple::Edge> RRTSimple::connectNodes(vector<Eigen::Vector5d>nodes, int startIndex) {

	std::vector<RRTSimple::Edge> edges;
	KDT kdTree;

	for (int i = 0; i < nodes.size(); i++) {
		
		if (i == 9) {
			int a = 0;
		}

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

		for (int i = 0; i < relationsNodesFromNearestNeighbour.size(); i++) {

			//compute the distance from a Point to a linie and get the point on the line
			double distanceBetweenNodeAndLine = distPointToLine(node, nearestNeighbourFromThePoint, tree.nodes[relationsNodesFromNearestNeighbour[i]], pointOnTheLine);

			if (disNodeToNearestNeighbour > distanceBetweenNodeAndLine) {

				// add point on the line
				tree.nodes.push_back(pointOnTheLine);

				// add Edge between new Point and the point on the line
				tree.edges.push_back(Edge(nodeIndex, tree.nodes.size()-1));

				return;
			}
		}

		// add the edge between the node and its nearest neighbour
		tree.edges.push_back(Edge(nodeIndex, KDTnodeNearestNeighbour.nn[0]));
	}
}

Eigen::Vector5d RRTSimple::getSample() {

	Eigen::Vector5d sample = MyWorm::Random(rng, dis);
	sample[2] = 0;
	sample[3] = 0;
	sample[4] = 0;

	return sample;
}

vector<Eigen::Vector5d> RRTSimple::getSamples(int numberofSample) {

	vector<Eigen::Vector5d> samples;
	
	for (int i = 0; i <= numberofSample; i++) {
		Eigen::Vector5d sample = MyWorm::Random(rng, dis);
		sample[2] = 0;
		sample[3] = 0;
		sample[4] = 0;

		samples.push_back(sample);
		metrics.numberOfNodes++;
		cout << "Point "<<i<<" x:"<<sample[0]<<" y:"<<sample[1] << endl;
	}

	return samples;
}

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

double RRTSimple::vec5Distance(Eigen::Vector5d a, Eigen::Vector5d b)
{
	//simple euklidian distance
	Eigen::Vector5d c = a - b;
	double sum = pow(c[0], 2) + pow(c[1], 2) + pow(c[2], 2) + pow(c[3], 2) + pow(c[4], 2);

	double length = sqrt(sum);

	return length;
}

vector<int> RRTSimple::getAllRelationNodes(int nodeIndex, vector<Edge> &edges) {
	vector<int> relationNodeIndex;

	for (int i = 0; i < edges.size(); i++) {
		if (edges[i].first == nodeIndex) {
			relationNodeIndex.push_back(edges[i].second);
		}
		else if (edges[i].second == nodeIndex) {
			relationNodeIndex.push_back(edges[i].first);
		}
	}

	return relationNodeIndex;
}

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
	
	return vec5Distance(PV, orthogonalPoint);
}

