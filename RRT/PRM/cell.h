#ifndef CELL_H_INCLUDED
#define CELL_H_INCLUDED

#include <string>
#include <random>
#include <memory>
#include <chrono>

#include <Eigen/Eigen>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/transform.h>
#include <fcl/narrowphase/narrowphase.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>

// includes used for nearest neighbour retrieval
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

namespace Eigen { typedef Matrix<double, 5, 1, 0, 5, 1> Vector5d; }

typedef std::vector<std::shared_ptr<fcl::Box>> boxv_t;
typedef std::vector<fcl::Transform3f> t3fv_t;

class MyWorm
{
public:
    MyWorm() { q_ << 0., 0., 0., 0., 0.; }
    MyWorm(const Eigen::Vector5d &q) : q_(q) { }

    std::vector<fcl::Transform3f> ForwardKinematic() const;

    double &operator[](int i) { return q_(i); }
    double operator[](int i) const { return q_(i); }

    Eigen::Vector5d& q() { return q_; }

    static std::vector<fcl::Transform3f> ForwardKinematic(const Eigen::Vector5d &q);
    static Eigen::Vector5d Random(std::mt19937_64 &rng, std::uniform_real_distribution<double> &unif);
    static bool IsInsideRange(const Eigen::Vector5d &q);

    void operator()(boxv_t &obj_robot);

protected:
    Eigen::Vector5d q_;
};

class MyCell
{
public:
    void operator()(boxv_t &obj_obstacle, t3fv_t &tf_obstacle);
};

// point coordinates of vertex
typedef struct { Eigen::VectorXd q_; } vertex_prop_t;

typedef boost::property<boost::edge_weight_t, float> edge_weight_prop_t;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, vertex_prop_t, edge_weight_prop_t> graph_t;

// Some typedefs for simplicity
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;

typedef boost::property_map<graph_t, boost::vertex_index_t>::type index_map_t;
typedef boost::graph_traits<graph_t>::vertex_iterator vertex_iter;
typedef boost::graph_traits<graph_t>::edge_iterator edge_iter;

typedef std::pair<MyWorm, vertex_t> rtree_value;
//typedef boost::geometry::index::rtree<rtree_value, boost::geometry::index::rstar<16, 4>> knn_rtree_t;
typedef boost::geometry::index::rtree<rtree_value, boost::geometry::index::quadratic<16>> knn_rtree_t;

void write_nodes_file(graph_t, std::string, bool jump_to = true);
void write_easyrob_program_file(std::vector<Eigen::VectorXd>, std::string, bool jump_to = false);
void write_gnuplot_file(graph_t, std::string);

namespace boost { namespace geometry { namespace traits
{
    // Adapt Worm to Boost.Geometry
    template<> struct tag<MyWorm> { typedef point_tag type; };
    template<> struct coordinate_type<MyWorm> { typedef double type; };
    template<> struct coordinate_system<MyWorm> { typedef cs::cartesian type; };
    template<> struct dimension<MyWorm> : boost::mpl::int_<5> {};
    template<> struct access<MyWorm, 0>
    {
        static double get(MyWorm const& p) { return p[0]; }
        static void set(MyWorm& p, double const& value) { p[0] = value; }
    };
    template<> struct access<MyWorm, 1>
    {
        static double get(MyWorm const& p) { return p[1]; }
        static void set(MyWorm& p, double const& value) { p[1] = value; }
    };
    template<> struct access<MyWorm, 2>
    {
        static double get(MyWorm const& p) { return p[2]; }
        static void set(MyWorm& p, double const& value) { p[2] = value; }
    };
    template<> struct access<MyWorm, 3>
    {
        static double get(MyWorm const& p) { return p[3]; }
        static void set(MyWorm& p, double const& value) { p[3] = value; }
    };
    template<> struct access<MyWorm, 4>
    {
        static double get(MyWorm const& p) { return p[4]; }
        static void set(MyWorm& p, double const& value) { p[4] = value; }
    };
}}} // namespace boost::geometry::traits

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
class Cell
{
public:
    Cell();
    virtual ~Cell() { }

    bool JumpTo(const Eigen::VectorXd &q);
    bool CheckPosition(const Eigen::VectorXd &q);
    bool CheckMotion(const Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx = 1e-2);
    bool FirstContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx = 1e-2);
    bool LastContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx = 1e-2);

    Eigen::VectorXd NextRandomCspace();
    Eigen::VectorXd NextRandomCfree();
    void ResetRNG();

    _ROBOT_TYPE& Robot() { return robot_; }

protected:
    Cell(const Cell&);
    const Cell& operator=(const Cell&);

    std::vector<std::shared_ptr<fcl::Box>> obj_obstacle_;
    std::vector<std::shared_ptr<fcl::Box>> obj_robot_;
    std::vector<fcl::Transform3f> tf_obstacle_;
    std::vector<fcl::Transform3f> tf_robot_;
    fcl::GJKSolver_libccd solver_;

    std::mt19937_64 rng_; // Random Number Generator
    std::uniform_real_distribution<double> unif_;
    _ROBOT_TYPE robot_;
};

#include "cell.inl"

typedef Cell<MyWorm, MyCell> WormCell;

#endif
