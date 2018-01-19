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
    WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5);
    vector<Eigen::VectorXd> path; // create a point vector for storing the path
    graph_t g;
    knn_rtree_t rtree;
    

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
	const int NUMBER_OF_SAMPLES = 70000;
	const float stepsize = .025f;


	RRT5Dof::RRT5dofMetrics rrt5dofMetric;

	RRT5Dof rrt5dof(qGoal, qStart);
	
	RRT5Dof::Result result = rrt5dof.getPath(cell, NUMBER_OF_SAMPLES,rrt5dofMetric, stepsize, 2.0);

	

	rrt5dof.printResult(result.tree.nodes, rrt5dofMetric, false, true);
	write_easyrob_program_file(result.path, "RRT5Dof.prg", false);
	
    return EXIT_SUCCESS;
}
