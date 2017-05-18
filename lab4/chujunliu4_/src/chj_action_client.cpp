#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <chujunliu4/chujunliu4Action.h> //reference action message in this package
#include <example_ros_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <new_reactive_commander/double_bool.h>
#include <std_msgs/Bool.h> 

using namespace std;
int firsttime=1,firsttimespin=1;
bool g_goal_active = true; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;

// lidar callback
bool g_lidar_alarm=false;
bool turndir=false; // global var for lidar alarm
chujunliu4::chujunliu4Goal goal; 
chujunliu4::chujunliu4Goal goal1;
chujunliu4::chujunliu4Goal goal2;


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const chujunliu4::chujunliu4ResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d",result->output);
    g_result_output= result->output;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const chujunliu4::chujunliu4FeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status = %d",fdbk_msg->fdbk);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}


void alarmCallback(const new_reactive_commander::double_bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.bool1;
  turndir=alarm_msg.bool2; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!"); 
     
  }
} 



int main(int argc, char** argv) {
        ros::init(argc, argv, "timer_client_node"); // name this node 
        ros::NodeHandle n;
        ros::Rate main_timer(100);
        ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback); // subscribe   lidar
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
       // chujunliu4::chujunliu4Goal goal; 
        //chujunliu4::chujunliu4Goal goal1;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;
///////////////////////// way points..//////////////////////////////////
    pose.position.x = 0.0; // say desired x-coord is 0
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    goal.nav_path.poses.push_back(pose_stamped);

    for(int i=0;i<5;i++)
{
     pose_stamped.pose.position.x=1.0; 
     pose_stamped.pose.position.y=0.0; 
     goal.nav_path.poses.push_back(pose_stamped);

     pose_stamped.pose.position.x=1.0; 
     pose_stamped.pose.position.y=1.0;                
     goal.nav_path.poses.push_back(pose_stamped);

     pose_stamped.pose.position.x=0; 
     pose_stamped.pose.position.y=1.0;                
     goal.nav_path.poses.push_back(pose_stamped);

     pose_stamped.pose.position.x=0; 
     pose_stamped.pose.position.y=0;                
     goal.nav_path.poses.push_back(pose_stamped);}

///////////////////////////waypoints///////////////////////////////////
     pose.position.x = 0.0; // say desired x-coord is 0
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 1.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    goal1.nav_path.poses.push_back(pose_stamped);

//////////////////two different goals goal and goal1  ///////////////////////////


        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<chujunliu4::chujunliu4Action> action_client("way_point", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        

while(ros::ok()) {

  
        if(!g_lidar_alarm){

           if(firsttime==1){
           //here are some options:
           //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
           //action_client.sendGoal(goal,&doneCb); // send goal and specify a callback function
           //or, send goal and specify callbacks for "done", "active" and "feedback"
           action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);            
           ROS_INFO("goal");firsttime=0;firsttimespin=1;}
                         }
        else {
               
              if(firsttimespin==1){  ROS_INFO("goal1");

             action_client.sendGoal(goal1, &doneCb, &activeCb, &feedbackCb);
              firsttime=1;
              firsttimespin=0;
              main_timer.sleep();}



            }

         
    

ros::spinOnce();

       } // while loop
    return 0;
}
