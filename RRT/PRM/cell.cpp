#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "cell.h"

std::vector<fcl::Transform3f> MyWorm::ForwardKinematic() const
{
    return ForwardKinematic(q_);
}

std::vector<fcl::Transform3f> MyWorm::ForwardKinematic(const Eigen::Vector5d &q)
{
    std::vector<fcl::Transform3f> transforms(3);
    fcl::Transform3f T, from(fcl::Vec3f(.05f, -.01f, 0.f)), to(fcl::Vec3f(.05f, .01f, 0.f));
    fcl::Quaternion3f R;
    fcl::Vec3f z(0.f, 0.f, 1.f);

    R.fromAxisAngle(z, q(2));
    T = fcl::Transform3f(fcl::Vec3f(q(0), q(1), 0.f)) *  R * to;
    transforms[0] = T;

    R.fromAxisAngle(z, q(3));
    T *= from * R * to;
    transforms[1] = T;

    R.fromAxisAngle(z, q(4));
    T *= from * R * to;
    transforms[2] = T;

    return transforms;
}

Eigen::Vector5d MyWorm::Random(std::mt19937_64 &rng, std::uniform_real_distribution<double>& unif)
{
    Eigen::Vector5d q;

    q(0) = unif(rng);
    q(1) = unif(rng);
    q(2) = M_PI * (2. * unif(rng) - 1.);
    q(3) = M_PI * (2. * unif(rng) - 1.);
    q(4) = M_PI * (2. * unif(rng) - 1.);

    return q;
}

bool MyWorm::IsInsideRange(const Eigen::Vector5d &q)
{
    return q(0) >= 0. && q(0) <= 1.
        && q(1) >= 0. && q(1) <= 1.
        && fabs(q(2)) <= M_PI
        && fabs(q(3)) <= M_PI
        && fabs(q(4)) <= M_PI;
}

void MyWorm::operator()(boxv_t &obj_robot)
{
    obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
    obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
    obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
}

void MyCell::operator()(boxv_t &obj_obstacle, t3fv_t &tf_obstacle)
{
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.45, 0.3, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.75, 0.3, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.3, 0.6, 0)));
    //tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.6, 0.6, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.9, 0.6, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.45, 0.9, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.75, 0.9, 0)));

    for (auto i : tf_obstacle)
        obj_obstacle.push_back(std::make_shared<fcl::Box>(0.2, 0.2, 0.05));
}

void write_nodes_file(graph_t g, std::string filename, bool jump_to)
{
    std::ofstream myfile;

    myfile.open(filename);
    myfile << "ProgramFile" << std::endl;

    std::cout << "Output vertices to " << filename << std::endl;

    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
    {
        vertex_t v = *vp.first;
        if (jump_to)
            myfile << "JUMP_TO_AX ";
        else
            myfile << "PTP_AX ";

        myfile << g[v].q_(0) << "  " << g[v].q_(1) << "  " << RAD2DEG(g[v].q_(2)) << " " << RAD2DEG(g[v].q_(3)) << " " << RAD2DEG(g[v].q_(4)) << std::endl;
    }
    myfile << "EndProgramFile" << std::endl;
    myfile.close();
}

void write_easyrob_program_file(std::vector<Eigen::VectorXd> path, std::string filename, bool jump_to)
{
    std::ofstream myfile;

    myfile.open(filename);
    myfile << "ProgramFile" << std::endl;

    for (int i = path.size() - 1; i >= 0; --i)
    {
        if (jump_to)
            myfile << "JUMP_TO_AX ";
        else
            myfile << "PTP_AX ";

        myfile << path[i](0) << " " << path[i](1) << " " << RAD2DEG(path[i](2)) << " " << RAD2DEG(path[i](3)) << " " << RAD2DEG(path[i](4)) << std::endl;
    }
    myfile << "EndProgramFile" << std::endl;
    myfile.close();
}

void write_gnuplot_file(graph_t g, std::string filename)
{
    int nEdge = 0;
    std::ofstream myfile;

    myfile.open(filename);

    // Iterate through all edges and print them
    std::pair<edge_iter, edge_iter> ep;
    edge_iter ei, ei_end;

    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
        myfile << g[ei->m_source].q_(0) << " " << g[ei->m_source].q_(1) << std::endl;
        myfile << g[ei->m_target].q_(0) << " " << g[ei->m_target].q_(1) << std::endl << std::endl;
        nEdge++;
    }
    myfile.close();

    std::cout << "Number of edges: " << nEdge << std::endl;
}
