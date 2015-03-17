# example_joint_space_planner

This library defines a class that is a general planner for problems of the following type:  There are Nlayers "layers" of "nodes".  A node is an Eigen::VectorXd.
All nodes have the same dimension.  Any layer can have an arbitrary number of nodes (though there must be at least one node per layer).
The layers define a graph as follows.  Every node in layer i connects to every node i layer i+1.

The function double JointSpacePlanner::score_move(Eigen::VectorXd pose1, Eigen::VectorXd pose2) computes a cost of transition from pose1 to pose2,
where "pose" is a node from a layer.  The score_move function uses weights to compute a weighted sum of squares of the vector from pose1 to pose2.

A planner object is constructed with objects:  path_options_ and penalty_weights.  The object "path_options" is a vector of "layers" (where
each layer contains an arbitrary number of pose nodes).  The number of layers corresponds to the number of steps in the desired path.

The objective is to choose a sequence of nodes--one from each layer--that defines a path from layer 0 to layer nlayers-1 such that this path
has the lowest possible total trip cost from start to goal.  

Use of this library is illustrated by the test routines joint_space_planner_test_main.cpp and joint_space_planner_test_main2.cpp.  These illustrate
use of the planner for arbirary numbers of layers, numbers of nodes per layer, and dimension of each node.  Some details on usage follow.

The path_options_ is passed as a reference variable, to avoid copying.  Instantiating the jointSpacePlanner includes setting up
all necessary dimensioning, which depends on the number of layers, the number of IK solutions in each layer, and the dimension of the
joint-space vectors. These dimensions are inferred by the constructor by examining the path_options dimensions.
 The joint-space planner is general, in that it works for any problem dimension, e.g. whether considering only a few
or many joints.  It is applicable to 6-DOF planning for valve turning, as well as 8-dof planning for wall cutting, and to future applications
(e.g., including more torso joints, etc).  The joint-space planner is indifferent to the meaning of the values in the path_options object.
Generally, these do not even need to be joint-space values (although this is the intended application).

Upon construction, an object of type JointSpacePlanner automatically computes all min cost-to-go options, then traverses this structure to find the min-cost path.
The result is an optimal path with one "pose" (e.g., 8-dof joint values) per "layer" (sample point along the desired motion in task space).
The parent routine can obtain the resulting path using a "get" function, get_soln(optimal_path);  

Optimality of the path through joint space depends on the cost definition
for moving from one pose to another.  This is defined in: double JointSpacePlanner::score_move(Eigen::VectorXd pose1, Eigen::VectorXd pose2).
At present, this is set to be a weighted sum of squares of components in pose1 relative to components in pose2.  That is, each element
of the pose motion is penalized quadratically, then these errors are weighted in importance by the penalty weights.  If desired, the
cost function could be revised to prefer a nominal pose--e.g., a preferred pelvis height--by adding in more terms to the penalty function. 

On oddity of the above design, in which the object constructs its dimensions based on the argument path_options_, is that this object is only useful
once.  After it is instantiated, the answer should be extracted and the object should be deleted.  This can be done implicitly by constructing
the object within a limited scope, implicitly deleting the objecct when it goes out of scope.  
To solve for another plan, a new JointSpacePlanner should be instantiated, using
a new path_options_ object.

Computation time scales linearly with the number of "layers", and with the square of the number of nodes per layer.

## Example usage
See the test functions here, notably, joint_space_planner_test_main2.cpp.  This test harness generates random problem examples of
specified size, then solves for min-cost paths.

## Running tests/demos
start up drcsim
rosrun example_joint_space_planner joint_space_planner_test_main2

Within joint_space_planner_test_main2.cpp, can try varying parameters, such as 
#define VECTOR_DIM 8 // e.g., an 8-dof vector
#define NLAYERS 100  // e.g., the number of points on a path, for which to generate IK solutions
#define MAX_OPTIONS_PER_LAYER 4000 //this can get LARGE; num IK solutions for a given task pose
to get a sense of generality and of computation time incurred
    