#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pp4/pp4Action.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
using namespace std;

const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 


double timer=0.0;
double final_time;
int g_count = 0;
int turnflag;
int turndir;
bool g_count_failure = false;


// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
//void do_spin(double spin_ang) {
    
//}

//a function to move forward by a specified distance (in meters), then halt
//void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    
//}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 double x=current_pose.position.x;
 double y=current_pose.position.y;
 double z_yaw=convertPlanarQuat2Phi(current_pose.orientation);
 double xd=goal_pose.position.x;
 double yd=goal_pose.position.y;
 double z_yawd=convertPlanarQuat2Phi(goal_pose.orientation);

 dist = sqrt(pow((x-xd),2)+pow((y-yd),2)); //sqrt(x^2+y^2)
 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
    heading = atan2((yd-y),(xd-x));  ///heading angle 
 }

}


class ActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<pp4::pp4Action> as_;
    
    // here are some message types to communicate with our client(s)
    pp4::pp4Goal goal_; // goal message, received from client
    pp4::pp4Result result_; // put results here, to be sent back to the client when done w/ goal
    pp4::pp4Feedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
public:
    ActionServer(); //define the body of the constructor outside of class definition

    ~ActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<pp4::pp4Action>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

ActionServer::ActionServer() :
   as_(nh_, "way_point", boost::bind(&ActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ActionServer::executeCB(const actionlib::SimpleActionServer<pp4::pp4Action>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    
    double yaw_desired, yaw_current, travel_distance, spin_angle, yaw_last_time=0.0;
    geometry_msgs::Pose pose_desired;
    int npts = goal->nav_path.poses.size();
    ROS_INFO("goal input pose nnumber : %d", npts);
    //do work here: this is where your interesting code goes
    for (int i=0;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = goal->nav_path.poses[i].pose; //get next pose from vector of poses
        int turnflag =pose_desired.orientation.z;
        bool turndir=pose_desired.orientation.y;
        ros::Rate loop_timer(1/g_sample_dt);
        //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
        get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);

        

        ROS_INFO("turnflag  =%d",turnflag);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);               

        spin_angle = yaw_desired - yaw_last_time; // spin this much
        spin_angle = min_spin(spin_angle);// change turn dir if angle>180

        if(turnflag==0){
        /////////////////do_spin(spin_angle); // carry out this incremental action



        
     timer=0.0;
     final_time = fabs(spin_angle)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_angle)*g_spin_speed;
    while(timer<final_time) {

        if (as_.isPreemptRequested()){  
          ROS_WARN("goal cancelled!");
          result_.output = 1862;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }

          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 




        
        ROS_INFO("spin angle = %f",spin_angle); //test spin_angle 
        yaw_last_time=yaw_desired; // update yaw_last_time
        g_current_pose.position = pose_desired.position; 
        
        
        //////do_move(travel_distance); // move forward travel_distance

       

       
     timer=0.0;
    final_time = fabs(travel_distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(travel_distance)*g_move_speed;
    while(timer<final_time) {

         if (as_.isPreemptRequested()){  
          ROS_WARN("goal cancelled!");
          result_.output = 1862;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }

          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();



        } 

        else {////////////do_spin(0.1);
            g_current_pose.position.x=0;
 g_current_pose.position.y=0;
            if(turndir==0){

           do_halt();
             timer=0.0;
     final_time = 3.14/g_spin_speed;
    g_twist_cmd.angular.z= 3.14*g_spin_speed;
    while(timer<final_time) {

         if (as_.isPreemptRequested()){  
          ROS_WARN("goal cancelled!");
          result_.output = 1862;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }

          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); }
    else 
        {

           do_halt();
             timer=0.0;
     final_time = 3.14/g_spin_speed;
    g_twist_cmd.angular.z= -3.14*g_spin_speed;
    while(timer<final_time) {

         if (as_.isPreemptRequested()){  
          ROS_WARN("goal cancelled!");
          result_.output = 1862;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }

          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); }

        }

        feedback_.fdbk = i; // populate feedback message with current countdown value
       as_.publishFeedback(feedback_);
        }

    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
   
   
       
       
  
 	
 	   //if here, then goal is still valid; provide some feedback
 	   
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.output = 12345; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}
void do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "p4_action_server_node"); 
    ros::NodeHandle nh_;
    do_inits(nh_);
    ROS_INFO("instantiating the timer_action_server: ");

    ActionServer as_object;
      // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

