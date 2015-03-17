// reachability_from_above.cpp
// wsn, March 2015
// compute reachability, w/ z_tool_des = [1;0;0] = x_base
// w/ robot mounted as is, x_base points down
// Fix the value of x, --> const height; scan over y and z
const double x_des = 0.3;


#include <irb120_kinematics.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "irb120_reachability");

    Vectorq6x1 q_in;



    Eigen::Affine3d a_tool;
    a_tool.linear() << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
    a_tool.translation() << x_des,
            0.0,
            0.0;

    Irb120_fwd_solver irb120_fwd_solver;
    Irb120_IK_solver ik_solver;

    std::cout << "==== Test for irb120 kinematics solver ====" << std::endl;
    int ans = 1;
    bool reachable_proposition;
    for (double z_des = -1.0; z_des<1.0; z_des+=0.1) {
        std::cout<<std::endl;
        std::cout<<"z="<<z_des;
       for (double y_des=-1.0; y_des<1.0; y_des+= 0.1) {
            a_tool.translation(1)=y_des;
            a_tool.translation(2)=z_des;
            int nsolns = ik_solver.ik_solve(a_tool);
            std::cout<<nsolns;
        }
    }

    return 0;
}
