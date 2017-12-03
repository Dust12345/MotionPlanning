template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Cell<_ROBOT_TYPE, _CELL_TYPE>::Cell()
    : unif_(0., 1.)
{
    _ROBOT_TYPE()(obj_robot_);
    _CELL_TYPE()(obj_obstacle_, tf_obstacle_);
    ResetRNG();
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Eigen::VectorXd Cell<_ROBOT_TYPE, _CELL_TYPE>::NextRandomCspace()
{
    return _ROBOT_TYPE::Random(rng_, unif_);
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
Eigen::VectorXd Cell<_ROBOT_TYPE, _CELL_TYPE>::NextRandomCfree()
{
    Eigen::VectorXd q;

    do
    {
        q = NextRandomCspace();
    } while (!CheckPosition(q));

    return q;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
void Cell<_ROBOT_TYPE, _CELL_TYPE>::ResetRNG()
{
    //uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    uint64_t timeSeed = 0;
    std::seed_seq ss { uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
    rng_.seed(ss);
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::JumpTo(const Eigen::VectorXd &q)
{
    if (!robot_.IsValidRange(q))
        return false;
    robot_.q() = q;
    return true;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::CheckPosition(const Eigen::VectorXd &q)
{
    tf_robot_ = _ROBOT_TYPE::ForwardKinematic(q);

    if (tf_robot_.empty()) return false;

    for (size_t i = 0; i < obj_robot_.size(); ++i)
    {
        for (size_t j = 0; j < obj_obstacle_.size(); ++j)
        {
            if (solver_.shapeIntersect(*obj_robot_[i], tf_robot_[i], *obj_obstacle_[j], tf_obstacle_[j], nullptr))
                return false;
        }
    }

    return true;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::CheckMotion(const Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx)
{
    Eigen::VectorXd segment(to - from), delta, current(from);
    int steps;
    bool cfree;

    delta = segment.normalized() * dx;
    steps = int(segment.norm() / dx);

    do
    {
        cfree = CheckPosition(current);
        current += delta;
    } while (cfree && --steps > 0);

    return cfree;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::FirstContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx)
{
    Eigen::VectorXd segment(to - from), delta;
    int steps;

    delta = segment.normalized() * dx;
    steps = int(segment.norm() / dx);

    Cfree = from;
    Cobstacle = from;

    if (CheckPosition(Cfree))
    {
        do
        {
            Cobstacle += delta;
            if (!CheckPosition(Cobstacle)) return true;
            Cfree = Cobstacle;
        } while (--steps > 0);
    }

    Cobstacle = to;
    return false;
}

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
bool Cell<_ROBOT_TYPE, _CELL_TYPE>::LastContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx)
{
    Eigen::VectorXd segment(to - from), delta;
    int steps;

    delta = segment.normalized() * dx;
    steps = int(segment.norm() / dx);

    Cfree = from;
    Cobstacle = from;

    if (!CheckPosition(Cfree))
    {
        do
        {
            Cobstacle += delta;
            if (CheckPosition(Cobstacle)) return true;
            Cfree = Cobstacle;
        } while (--steps > 0);
    }

    Cobstacle = to;
    return false;
}
