#include "stdafx.h"
#include <iostream>
#include "cell.h"
#include "Point.h"
#include "RRTSimple.h"
#include "RRT5Dof.h"
#include "DynamicKDT.h"


using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	DynamicKDT dkdt;
	Eigen::Vector5d p;
	p[0] = 1;
	p[1] = 1;
	p[2] = 0;
	p[3] = 0;
	p[4] = 0;

	Eigen::Vector5d p2;
	p2[0] = 5;
	p2[1] = 5;
	p2[2] = 0;
	p2[3] = 0;
	p2[4] = 0;

	Eigen::Vector5d p3;
	p3[0] = 4;
	p3[1] = 4;
	p3[2] = 0;
	p3[3] = 0;
	p3[4] = 0;

	dkdt.addPoint(p,0);
	dkdt.addPoint(p2, 1);
	

	int index = dkdt.getNN(p3);

	std::cout << index << " is index " << std::endl;

	while (true) {

	}

	return 1;

    WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5);
    vector<Eigen::VectorXd> path; // create a point vector for storing the path
    graph_t g;
    knn_rtree_t rtree;
    const float stepsize = .025f;

#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0
	qStart << 0., 0., 0., 0., 0.;
	qGoal << .6, .9, DEG2RAD(-90.), DEG2RAD(-180.), DEG2RAD(180.);
#elif TEST_CASE == 1
	cout << "Test case 1" << endl;
	qStart << .6, .1, 0., 0., 0.;
    qGoal << .1, .8, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 2
	cout << "Test case 2" << endl;
	qStart << .1, .8, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
#elif TEST_CASE == 3
	cout << "Test case 3" << endl;
	qStart << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .75, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 4
	cout << "Test case 4" << endl;
	qStart << .9, .75, DEG2RAD(-180.f), 0., 0.;
	qGoal << .5, .45, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 5
	cout << "Test case 5" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 6
	cout << "Test case 6" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 7
	cout << "Test case 7 / colliding goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .7, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 8
	cout << "Test case 8 / colliding start" << endl;
	qStart << .7, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 9
	cout << "Test case 9 / unreachable goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 10
	cout << "Test case 10 / unreachable start" << endl;
	qStart << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#endif
#endif
	const int NUMBER_OF_SAMPLES = 10000;
	
	/**
	RRTSimple::SimpleRRTMetrics metric;
	Eigen::VectorXd root(5);
	root << 0.5, 0.5, 0., 0., 0.;

	RRTSimple rrtSimple;
	RRTSimple::Tree tree = rrtSimple.createTree(root,200, metric);
	rrtSimple.printResult(tree.nodes, metric,true,true);
	
	*/
	//qStart << 0.5, 0.5, 0., 0., 0.;

	RRT5Dof::RRT5dofMetrics rrt5dofMetric;
	Eigen::VectorXd movementVector(5);
	RRT5Dof rrt5dof(qStart, qGoal);
	RRT5Dof::Result result = rrt5dof.getPath(cell, NUMBER_OF_SAMPLES,rrt5dofMetric, movementVector, 5.0);
	rrt5dof.printResult(result.tree.nodes, rrt5dofMetric, true, true);
	write_easyrob_program_file(result.path, "RRT5Dof.prg", false);
	//write_easyrob_program_file(path, "RRT5dof.prg", false);
	
	int a = 0;
	
    return EXIT_SUCCESS;
}
