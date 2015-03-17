// wsn, October, 2014
// joint-space planner, organized as a class

// accepts a vector of vectors 

#ifndef JointSpacePlanner
#define	JointSpacePlanner
#include <iostream>
#include <ros/ros.h>

//#include "/usr/include/eigen3/Eigen/Core"
//#include <Eigen/Eigen>
//#include <Eigen/Dense>
#include <Eigen/Core>
//#include <Eigen/LU>




//typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
typedef Eigen::Matrix<double, 8, 1> Vectorq8x1;

union ScoredSoln {
  Vectorq8x1 joint_soln_;
  double cost_to_go;
  int next_step_idx;
};

typedef  std::vector<std::vector<Vectorq8x1>> PathOptions;

// set search ranges and resolutions:

class JointSpacePlanner8DOF {
private:
    // will use convention: trailing underscore ("_") indicates member variable or method
    Vectorq8x1 penalties_;
    std::vector<ScoredSoln> scored_solns_; // this is a vector of ScoredSoln objects
    std::vector<std::vector<ScoredSoln>> vector_all_scored_solns_; // this is a vector of vectors of ScoredSoln objects    
    std::vector<Vectorq8x1> full_path_; // this is a sequence of joint-space poses defining a path

    std::vector<Vectorq8x1> prior_pose_options_;
    Vectorq8x1 prior_pose_;
    Vectorq8x1 weights_;
    std::vector<Vectorq8x1> next_pose_options_;    
    double min_cost_to_go_;
    int move_index_min_cost_to_go_;
    int point_dimension_; // each "point" (e.g., 8dof state-space pose) has this dimension
    
    
public:    
    JointSpacePlanner8DOF(Vectorq8x1 weights); //constructor
    double score_move(Vectorq8x1 pose1, Vectorq8x1 pose2); // compute incremental cost to go from pose1 to pose2, weighted, possibly squared
    bool find_best_move(int ipose,int jlayer);  // for layer "j", choose pose-option "i" and find the index of the lowest cost-to-go to advance to next layer
    bool compute_optimal_path(PathOptions path_options,std::vector<Vectorq8x1> optimal_path);
};



#endif	/* JointSpacePlanner */
