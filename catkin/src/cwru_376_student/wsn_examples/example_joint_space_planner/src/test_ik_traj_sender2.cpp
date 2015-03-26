//test_ik_traj_sender2.cpp:
// compute a sequence of candidate solutions, then plan and execute a path to each
//wsn, March 2015
//used to test/visualize IK solutions.
// e.g.:
//    z_des = 0.3;
//    x_des = 0.4;
//    y_des = 0.45;
// sends robot to 8 different solutions
// watch out...may need to lead joint to goal, else may hit limit w/ wrap-around direction
// 

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <irb120_kinematics.h>
#include <joint_space_planner.h>
#define VECTOR_DIM 6 // chooose t plan w/ 6-dof vectors

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "test_traj_sender"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    double x_des,y_des,z_des;
    std::vector<std::vector<Eigen::VectorXd> > path_options; 
    std::vector<Eigen::VectorXd>  single_layer_nodes;     
    std::vector<Eigen::VectorXd> optimal_path;
    Eigen::VectorXd weights;
    
    z_des = 0.3;
    x_des = 0.4;
    y_des = 0.45;

    b_des<<1,0,0;
    t_des<<0,1,0;
    n_des = t_des.cross(b_des);
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    
   Eigen::Affine3d a_tool_des; // expressed in DH frame
   a_tool_des.linear() = R_des;

    
    //ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate to send new traj points as commands
    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    // build an example trajectory:
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear();  
    
    new_trajectory.points.clear();
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");   
    
    //specify two points: initially, all home angles
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions.push_back(0.0); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point2.positions.push_back(0.0); // stuff in position commands for 6 joints        
    }
    //ros::Duration t_from_start(0); //initialize duration to 0
    double t=0.0;
    //double dt = 3;          
    trajectory_point1.time_from_start =    ros::Duration(0);   
    //trajectory_point2.time_from_start =    ros::Duration(2);  
    new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
    //new_trajectory.points.push_back(trajectory_point2); // add this single trajectory point to the trajectory vector      
    new_trajectory.header.stamp = ros::Time::now();      
    
    ros::Rate sleep_timer(1.0); //1Hz update rate
    Irb120_fwd_solver irb120_fwd_solver;
    Irb120_IK_solver ik_solver;
  
    Eigen::Affine3d A_fwd_DH;
    
    ROS_INFO("going home");
    //for (int i=0;i<5;i++)
   //{
        pub.publish(new_trajectory);
            ros::spinOnce();
            sleep_timer.sleep();
    //}

        new_trajectory.points.clear();
        // start from home pose
        new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
        new_trajectory.header.stamp = ros::Time::now();        
        // now see about multiple solutions:
 
   qvec<<0,0,0,0,0,0;
   int nsolns=0;
   int nlayer = 0;
   for (double y_var = -0.4; y_var<0.4; y_var+=0.01) {
        p[0] = x_des;
        p[1]=  y_var;
        p[2] = z_des;
        a_tool_des.translation()=p;
   
        //is this point reachable?
        nsolns = ik_solver.ik_solve(a_tool_des);
        ROS_INFO("there are %d solutions",nsolns);
        ik_solver.get_solns(q6dof_solns);
        if (nsolns>0) {
            Eigen::VectorXd soln_node;
            single_layer_nodes.clear();
            for (int isoln =0; isoln<nsolns;isoln++) {
                soln_node = q6dof_solns[isoln]; // convert to compatible datatype
                single_layer_nodes.push_back(soln_node);
                
            }
          path_options.push_back(single_layer_nodes);
          nlayer = path_options.size();
          ROS_INFO("filled layer %d",nlayer);
        }

   }
   //return 0;
    optimal_path.resize(nlayer);    
    weights.resize(VECTOR_DIM);
    for (int i=0;i<VECTOR_DIM;i++) { // default--assign all weights equal 
        weights(i) = 1.0;
    }
       //do some planning:
     cout<<"instantiating a JointSpacePlanner:"<<endl;
     { //limit the scope of jsp here:
       JointSpacePlanner jsp (path_options,weights);
       cout<<"recovering the solution..."<<endl;
       jsp.get_soln(optimal_path);
       //double trip_cost= jsp.get_trip_cost();

     }

     //now, jsp is deleted, but optimal_path lives on:
     cout<<"resulting solution path: "<<endl;
     for (int ilayer=0;ilayer<nlayer;ilayer++) {
         cout<<"ilayer: "<<ilayer<<" node: "<<optimal_path[ilayer].transpose()<<endl;
     }        

   

   // try to execute the plan:
     double dt = 0.4;
     //t+= 5.0; // go from home to first point in N sec; for simu, this does not behave same as on actual robot;
   for (int ilayer=0;ilayer<nlayer;ilayer++)
   {
      qvec = optimal_path[ilayer];
      for (int ijnt=0;ijnt<6;ijnt++) {
          trajectory_point2.positions[ijnt] = qvec[ijnt];
      }
      t += dt;    
      trajectory_point2.time_from_start =    ros::Duration(t);  
      new_trajectory.points.push_back(trajectory_point2); // append another point
      // t += dt;    
      //trajectory_point1.time_from_start =    ros::Duration(t);   
      //new_trajectory.points.push_back(trajectory_point1); // go home between pts
      std::cout<<"qsoln = "<<qvec.transpose()<<std::endl;
      A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    std::cout << "A rot: " << std::endl;
    std::cout << A_fwd_DH.linear() << std::endl;
    std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;      
   }
    

     int npts = new_trajectory.points.size(); 
     int njnts = new_trajectory.points[0].positions.size();
    ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts);

            pub.publish(new_trajectory);
            ros::spinOnce();
            sleep_timer.sleep();
    return 0;
} 

