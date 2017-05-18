// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h>
#include <new_reactive_commander/double_bool.h> //new defined message
const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=0.7; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1;
int ping_index_left;
int ping_index_right; //-90 and 90 degree index
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool turn_direction=false; // false=turn left;true=turn right

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) 
{
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
      

       
        for(float i=-20.0;i<20;i=i+2.0)  // for loop detect the distance from-90degree to 90 
       {      float j=3.14*i/180.0;
              ping_index_ = (int) ((j -angle_min_)/angle_increment_);
              ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
   
    
   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
                  
    
        if (ping_dist_in_front_<0.5/sin(j+1.57))   
        {
          ROS_WARN("DANGER, WILL ROBINSON!!");
          laser_alarm_=true;break;
        }
   else laser_alarm_=false;
        
   
                                           
   
       
   } //end of for-loop
   ping_index_right=(int) ((-1.57 -angle_min_)/angle_increment_);     //calculate left and right index
   ping_index_left=(int) ((1.57 -angle_min_)/angle_increment_);
 if(laser_scan.ranges[ping_index_left]>laser_scan.ranges[ping_index_right])
  turn_direction=false;  //  left distant>right ;turn left
else  turn_direction=true;

   new_reactive_commander::double_bool lidar_alarm_msg;
   lidar_alarm_msg.bool1 = laser_alarm_;
   lidar_alarm_msg.bool2 = turn_direction;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<new_reactive_commander::double_bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

