// wsn, October, 2014
// test main for joint-space planner, organized as a class
// this version creates larger, more interesting path options for testing
#include "joint_space_planner.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#define VECTOR_DIM 8 // e.g., an 8-dof vector
#define NLAYERS 100  // e.g., the number of points on a path, for which to generate IK solutions
#define MAX_OPTIONS_PER_LAYER 4000 //this can get LARGE; num IK solutions for a given task pose
#define MAX_VEC_COMPONENT 10.0 // generate random vecs with elements between +/-10
#define MIN_VEC_COMPONENT -10.0


Eigen::VectorXd gen_rand_vec() {
 Eigen::VectorXd rand_vec(VECTOR_DIM,1); // holder for a vector of dim VECTOR_DIM
 double rval;
 double vval;
 for (int i=0;i<VECTOR_DIM;i++) {
      rval = ((double) rand())/((double) RAND_MAX);
      vval = (MAX_VEC_COMPONENT-MIN_VEC_COMPONENT)*rval + MIN_VEC_COMPONENT; // rescale to fit desired range
     rand_vec[i]=vval; //need rand generator here
 }
 return rand_vec;   
}

int rand_layer_ntps() {   
    int v1 = rand() % MAX_OPTIONS_PER_LAYER;         // v1 in the range 1 to MAX_OPTIONS_PER_LAYER
    return v1+1;
}

int rand_to_nmax(int nmax) {   
    int v1 = rand() % nmax;         // v1 in the range 0 to nmax-1
    return v1;
}

int main(int argc, char **argv) {
    Eigen::VectorXd weights;
    //Eigen::VectorXd vec2;
    //Eigen::VectorXd vec3; 
    //Eigen::VectorXd arg1,arg2;     

  /* initialize random seed: */
  srand (time(NULL));    
    
    int ndim = VECTOR_DIM;  //just an alias
    int nlayers = NLAYERS; // ditto
    double trip_cost;
    
    int npts_this_layer;
    
    std::vector<std::vector<Eigen::VectorXd> > path_options; 
    std::vector<Eigen::VectorXd>  single_layer_nodes; 
    path_options.resize(NLAYERS);

    std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(NLAYERS);
    
    Eigen::VectorXd test_node(VECTOR_DIM,1);
    test_node= gen_rand_vec();
    Eigen::VectorXd d_node(VECTOR_DIM,1);
    d_node=Eigen::VectorXd::Ones(VECTOR_DIM);
    d_node *= 0.1; //a vector full of vals 0.1   
            
    weights.resize(VECTOR_DIM);
    for (int i=0;i<VECTOR_DIM;i++) {
        weights(i) = 1.0;
    }

    for (int ilayer = 0; ilayer < NLAYERS; ilayer++) {
        // generate random vectors for path options:
        npts_this_layer = rand_layer_ntps();
        single_layer_nodes.resize(npts_this_layer);
        cout << "layer " << ilayer << " will have " << npts_this_layer << " nodes" << endl;
        for (int jnode = 0; jnode < npts_this_layer; jnode++) {
            single_layer_nodes[jnode] = gen_rand_vec();
            //cout<<"node "<<jnode<<": "<<single_layer_nodes[jnode].transpose()<<endl;
        }
        // optionally, insert our test node at a random location in this layer:
        // this should set up the case that there is SOME path with cost=0!
        int n_node = rand_to_nmax(npts_this_layer);
        single_layer_nodes[n_node] = test_node;
        // additional test: insert a node that is slightly perturbed from previous node;
        // planner should discover these inserts as min-cost path
        test_node+=d_node; // next vector close, but not identical...
        path_options[ilayer] = single_layer_nodes;
    }
    cout<<"problem defined; enter 1 to solve: ";
    int ans;
    cin>>ans;
    // EVERYTHING ABOVE MERELY CREATES A RANDOM TEST PROBLEM
    
    
     cout<<"instantiating a JointSpacePlanner:"<<endl;
     { //limit the scope of jsp here:
       JointSpacePlanner jsp (path_options,weights);
       cout<<"recovering the solution..."<<endl;
       jsp.get_soln(optimal_path);
       trip_cost= jsp.get_trip_cost();

     }

     //now, jsp is deleted, but optimal_path lives on:
     cout<<"resulting solution path: "<<endl;
     for (int ilayer=0;ilayer<nlayers;ilayer++) {
         cout<<"ilayer: "<<ilayer<<" node: "<<optimal_path[ilayer].transpose()<<endl;
     }
    cout<<"soln min cost: "<<trip_cost<<endl;
    return 0;
      

}
