// wsn, October, 2014
// joint-space planner, organized as a class, specialized for 8dof joint-space vectors

#include "joint_space_planner_8dof.h"


using namespace std;

JointSpacePlanner8DOF::JointSpacePlanner8DOF(Vectorq8x1 weights) {
    // tell me how many weights there are, and I'll tell you the dimension of the pose states being evaluated
    //weights_= weights;
    point_dimension_ = weights.size();

    cout<<"vector size: "<<point_dimension_<<endl;
    
}

// compute incremental cost to go from pose1 to pose2, weighted, possibly squared
double JointSpacePlanner8DOF::score_move(Vectorq8x1 pose1, Vectorq8x1 pose2)  {
    return 0.0;
}

// for layer "j", choose pose-option "i" and find the index of the lowest cost-to-go to advance to next layer
bool JointSpacePlanner8DOF::find_best_move(int ipose,int jlayer) {
    min_cost_to_go_ = 0.0; // put in best value here
    move_index_min_cost_to_go_ = 0; // put in best index to next move here
    
    return true; // return true, unless there is a problem
}  

//main routine: accept a large array of options, and return the optimal path
bool JointSpacePlanner8DOF::compute_optimal_path(PathOptions path_options,std::vector<Vectorq8x1> optimal_path) {
    ROS_INFO("you wish!  lots of work to do first!");
    return true;
}
