#ifndef WOLFGANG_IK_
#define WOLFGANG_IK_

namespace wolfgang_ik {
class IK{
    public:
        IK();
        bool solve(tf2::Transform goal, std::vector<double> joint_positions);
}
}
#endif
