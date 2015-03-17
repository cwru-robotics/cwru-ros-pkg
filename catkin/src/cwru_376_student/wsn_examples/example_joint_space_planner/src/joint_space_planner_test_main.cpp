// wsn, October, 2014
// test main for joint-space planner, organized as a class
#include "joint_space_planner.h"
//typedef Eigen::Matrix<double, 8, 1> Vectorq8x1;

using namespace std;

int main(int argc, char **argv) {
    Eigen::VectorXd weights;
    Eigen::VectorXd vec2;
    Eigen::VectorXd vec3; 
    Eigen::VectorXd arg1,arg2;     
    
    int ndim = 8;
    
    int nlayers;
    
    std::vector<std::vector<Eigen::VectorXd> > path_options; 

    std::vector<Eigen::VectorXd> optimal_path;

    
    std::vector<Eigen::VectorXd> layer_options;
    
    weights.resize(ndim);
    vec2.resize(ndim);
    vec3.resize(ndim);
    for (int i=0;i<8;i++) {
        weights(i) = 1.0;
        vec2(i) = 2.0;
        vec3(i) = 3.0*i;
    }
    //dummy test: create a few layers, each with some 8x1 vectors
    // first layer pushed will be index 0, which is the starting layer
    layer_options.push_back(weights);
    path_options.push_back(layer_options);
    
    //2nd layer w/ 2 vectors
    layer_options.clear();
     layer_options.push_back(weights);
     layer_options.push_back(vec2);
     path_options.push_back(layer_options);
     
     //3rd layer has 3 options:
     layer_options.clear();
     layer_options.push_back(weights);
     layer_options.push_back(vec2);
     layer_options.push_back(vec3);     
     path_options.push_back(layer_options);    
     
     nlayers = path_options.size();
     optimal_path.resize(nlayers);
    
    // JointSpacePlanner(std::vector<std::vector<Eigen::VectorXd> > path_options,Eigen::VectorXd weights);
    // this is odd, BUT: instantiate JointSpacePlanner here, use it, get the answer, then throw it away;
    // if want another solution, do this again
    // this is because joint-space planner constructs its dimensions based on the size of the path_options arg
     // this may be LARGE, so constructor tries to avoid making a copy of it
     cout<<"instantiating a JointSpacePlanner:"<<endl;
     { //limit the scope of jsp here:
       JointSpacePlanner jsp (path_options,weights);
       cout<<"recovering the solution..."<<endl;
       jsp.get_soln(optimal_path);
    
     }

     //now, jsp is deleted, but optimal_path lives on:
     cout<<"resulting solution path: "<<endl;
     for (int ilayer=0;ilayer<nlayers;ilayer++) {
         cout<<"ilayer: "<<ilayer<<endl;
         cout<<optimal_path[ilayer].transpose()<<endl;
     }
     
    vec2(7)=3; // test scoring function with arbitrary input values
    arg1 = weights;
    arg2 = vec2;
    cout<<"arg1: "<<arg1.transpose()<<endl;
    cout<<"arg2: "<<arg2.transpose()<<endl;   
    {
      //instantiating another planner object
      JointSpacePlanner jsp (path_options,weights);
      double test_penalty = jsp.score_move(arg1,arg2);  
      cout<<"test_penalty = "<<test_penalty<<endl;
    }
    
    //previous version: construct jsp, then exercise its member methods;
    //jsp.compute_all_min_costs();
    //cout<<"invoking: compute_optimal_path"<<endl;
    //jsp.compute_optimal_path(optimal_path);
    
    

}
