#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <deque>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include "std_msgs/String.h"
#include "utils/yamlRead.h"
#include "utils/eigen_typedef.h"
#include "utils/kinetic_math.h"
#define PI (3.1415926)
using namespace std;

mavros_msgs::State current_state;
double takeoff_x,takeoff_y,takeoff_z,takeoff_yaw;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
int    Mission_stage_count = 0;
bool   Initialfromtakeoffpos;
double Trajectory_timestep = 0.02; // in Seconds
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;
double velocity_takeoff,velocity_angular, velocity_mission, altitude_mission;
Vec3 uav_lp_p;
Vec4 uav_lp_q;
Vec8 Current_stage_mission;
geometry_msgs::PoseStamped pose;

// Initial trajectories
deque<Vec8> trajectory1;
Vec2 traj1_information;
bool trajectory1_initflag = false;

// Initial waypoints
deque<Vec8> waypoints;

void constantVtraj( double end_x, double end_y, double end_z, double end_yaw_rad,
                    double velocity, double angular_velocity){
  Quaterniond q(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
  Vec3 start_rpy = Q2rpy(q);
  Vec3 start_xyz = uav_lp_p;
  Vec3 des_xyz = Vec3(end_x,end_y,end_z);
  Vec3 des_rpy = Vec3(0,0,end_yaw_rad);

  double dist = sqrt(pow((des_xyz[0]-start_xyz[0]),2)+pow((des_xyz[1]-start_xyz[1]),2)+pow((des_xyz[2]-start_xyz[2]),2));
  double dist_duration = dist/velocity; // In seconds
  double duration; //total duration in seconds
  Vec3 vxyz = Vec3(((des_xyz[0]-start_xyz[0])/dist)*velocity,((des_xyz[1]-start_xyz[1])/dist)*velocity,((des_xyz[2]-start_xyz[2])/dist)*velocity);
  if (start_rpy[2]>=M_PI)  start_rpy[2]-=2*M_PI;
  if (start_rpy[2]<=-M_PI) start_rpy[2]+=2*M_PI;
  if (des_rpy[2]>=M_PI)    des_rpy[2]-=2*M_PI;
  if (des_rpy[2]<=-M_PI)   des_rpy[2]+=2*M_PI;
  double d_yaw = des_rpy[2] - start_rpy[2];
  if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
  if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
  double yaw_duration = sqrt(pow(d_yaw/angular_velocity,2));
  if(yaw_duration>=dist_duration){duration = yaw_duration;}else{duration = dist_duration;}

  //initialize trajectory1
  trajectory1.clear();

  int wpc = duration/Trajectory_timestep;
  for (int i=0; i<wpc; i++){
    double dt = Trajectory_timestep*i;
    Vec3 xyz;
    Quaterniond q;
    
    // RPY
    if(dt<=yaw_duration){
      q = rpy2Q(Vec3(0,0,start_rpy[2]+dt*angular_velocity));

    }else{
      q = rpy2Q(des_rpy);
    }
    // Position_xyz
    if(dt<=duration){
      xyz = Vec3(start_xyz[0]+dt*vxyz[0],start_xyz[1]+dt*vxyz[1],start_xyz[2]+dt*vxyz[2]);
    }else{
      xyz = des_xyz;
    }

    Vec8 traj1;
    traj1 << dt, xyz[0], xyz[1], xyz[2], q.w(), q.x(), q.y(), q.z();
    trajectory1.push_back(traj1);
  }
}

void Mission_Generator(){
  // Waypoints
  Vec8 wp; // state x y z yaw v av waittime
  wp << 1, 0, 0 , 5, 0, 1, 1, 1 ;   // state = 1; takeoff no heading change.
  waypoints.push_back(wp);
  wp << 2, 5, 5, 5, 0,  1, 1, 1 ;   // state = 2; constant velocity trajectory.
  waypoints.push_back(wp);
  wp << 2,-5, 5, 5, 0,  1, 1, 1 ;
  waypoints.push_back(wp);
  wp << 2,-5,-5, 5, 0,  1, 1, 1 ;
  waypoints.push_back(wp);
  wp << 2, 5,-5, 5, 0,  1, 1, 1 ;
  waypoints.push_back(wp);
  wp << 4, 0, 0, 5, 0,  1, 1, 1 ;  // state = 4; constant velocity RTL but with altitude.
  waypoints.push_back(wp);
  wp << 5, 0, 0, 0, 0,  1, 1, 10;    // state = 5; land.
  waypoints.push_back(wp);
}

void traj_pub(){
  double current_time = ros::Time::now().toSec();
  Vec8 traj1_deque_front = trajectory1.front();

  while (current_time - traj1_information[0] - traj1_deque_front[0] > 0){
    trajectory1.pop_front();
    traj1_deque_front = trajectory1.front();
    // cout << "ddt: " <<  current_time - traj1_information[0] - traj1_deque_front[0] << endl;
  }
   
  pose.header.frame_id = "world";
  pose.pose.position.x = traj1_deque_front[1];
  pose.pose.position.y = traj1_deque_front[2];
  pose.pose.position.z = traj1_deque_front[3];
  pose.pose.orientation.w = traj1_deque_front[4];
  pose.pose.orientation.x = traj1_deque_front[5];
  pose.pose.orientation.y = traj1_deque_front[6];
  pose.pose.orientation.z = traj1_deque_front[7];

  // Trajectory current time > duration than goes on to next stage
  if (traj1_deque_front[0] > traj1_information[1]){ Mission_stage++;}
}

void Finite_state_WP_mission(){
  // Generate trajectory while mission stage change
  
  if (Mission_stage != Current_Mission_stage){
    Vec8 traj1;
    Current_Mission_stage = Mission_stage;  //Update Current_Mission_stage
    Current_stage_mission = waypoints.at(Mission_stage-1);
    Quaterniond q(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
    Vec3 current_rpy = Q2rpy(q);
    Mission_state = Current_stage_mission[0];
    cout<<"!!!!: "<<Mission_state<<endl;

    if (Mission_state == 1){ //state = 1 take off with no heading change
      constantVtraj(uav_lp_p[0], uav_lp_p[1], Current_stage_mission[3], current_rpy[2], Current_stage_mission[6], Current_stage_mission[7]);
      cout << "applied state 1" <<endl;
    }
    if (Mission_state == 2){ //state = 2; constant velocity trajectory with desired heading.
      constantVtraj(Current_stage_mission[1], Current_stage_mission[2], Current_stage_mission[3], Current_stage_mission[4], Current_stage_mission[5], Current_stage_mission[6]);
      cout << "applied state 2" <<endl;
    }
    if (Mission_state == 3){ //state = 3; constant velocity trajectory with no heading change.
      constantVtraj(Current_stage_mission[1], Current_stage_mission[2], Current_stage_mission[3], current_rpy[2], Current_stage_mission[5], Current_stage_mission[6]);
      cout << "applied state 3" <<endl;
    }
    if (Mission_state == 4){ //state = 4; constant velocity RTL but with altitude return to the takeoff heading.
      constantVtraj(takeoff_x, takeoff_y, Current_stage_mission[3], takeoff_yaw, Current_stage_mission[5], Current_stage_mission[6]);
      cout << "applied state 4" <<endl;
    }
    if (Mission_state == 5){ //state = 5; land.
      constantVtraj(takeoff_x, takeoff_y, takeoff_z-1, takeoff_yaw, Current_stage_mission[5], Current_stage_mission[6]);
      cout << "applied state 5" <<endl;
    }
    if (Current_stage_mission[7] != 0){ // Wait after finish stage.
      traj1 = trajectory1.back();
      int wpc = Current_stage_mission[7]/Trajectory_timestep;
      for (int i=0; i<wpc; i++){
        traj1[0] = traj1[0] + Trajectory_timestep;
        trajectory1.push_back(traj1);
      }
    }
    //Default generate 1 second of hover
    int hovertime = 1;
    traj1 = trajectory1.back();
    for (int i=0; i<(hovertime/Trajectory_timestep); i++){
        traj1[0] = traj1[0] + Trajectory_timestep;
        trajectory1.push_back(traj1);
    }
    //Store the Trajectory information 
    //Trajectory staring time, Trajectory duration
    traj1_information = Vec2(ros::Time::now().toSec(),traj1[0]-1);
    cout << "trajectory ready" << endl;
    
    // For Debug section plot the whole trajectory
    // int trajectorysize = trajectory1.size();
    // for (int i = 0; i < trajectorysize; i++){
    //   Vec8 current_traj = trajectory1.at(i);
    //   cout << "dt: " << current_traj[0] << " x: " << current_traj[1] << " y: " << current_traj[2] << " z: " << current_traj[3] << endl;
    // }
  }
  if(trajectory1.size() > 0){traj_pub();}
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
  uav_lp_x = pose->pose.position.x;
  uav_lp_y = pose->pose.position.y;
  uav_lp_z = pose->pose.position.z;
  uav_lp_qx = pose->pose.orientation.x;
  uav_lp_qy = pose->pose.orientation.y;
  uav_lp_qz = pose->pose.orientation.z;
  uav_lp_qw = pose->pose.orientation.w;
  uav_lp_p = Vec3(uav_lp_x,uav_lp_y,uav_lp_z);
  uav_lp_q = Vec4(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh("~");

  string configFilePath;

  takeoff_x = 0.0;
  takeoff_y = 0.0;
  takeoff_z = 0.0;
  takeoff_yaw = 0; //in rad
  velocity_mission = 0.5;
  velocity_takeoff = 0.5;
  altitude_mission = 1;

  nh.getParam("Initialfromtakeoffpos", Initialfromtakeoffpos);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
      ("/mavros/state", 10, state_cb);
  ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
      ("/mavros/local_position/pose", 10, uav_lp_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      ("/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("/mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(25.0);

  //wait for FCU connection 
  cout << "Waiting for FCU connection " << endl;
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
    cout << "Waiting for FCU connection " << endl;
  }

  //send a few setpoints before starting
  pose.header.frame_id = "world";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x=0.0;
  pose.pose.orientation.y=0.0;
  pose.pose.orientation.z=0.0;
  pose.pose.orientation.w=1.0;
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  cout << "change last_request A" << endl;
  ros::Time last_request = ros::Time::now();
  ros::Time init_time = ros::Time::now();

  while(ros::ok()){
    /*Update takeoff Position and Mission**********************************************/
    if(Initialfromtakeoffpos)
    {
      Quaterniond q(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
      Vec3 rpy = Q2rpy(q);
      takeoff_x = uav_lp_x;
      takeoff_y = uav_lp_y;
      takeoff_z = uav_lp_z;
      takeoff_yaw = rpy[2];
      Initialfromtakeoffpos = false;

      cout << "====================================================" <<endl;
      cout << "====================================================" <<endl;
      cout << "Mission Params Initialized" << endl;
      cout << "takeoff_x: " << takeoff_x << endl;
      cout << "takeoff_y: " << takeoff_y << endl;
      cout << "takeoff_z: " << takeoff_z << endl;
      cout << "takeoff_yaw: " << takeoff_yaw << endl;
      Mission_Generator();
      Mission_stage_count = waypoints.size();
      cout << "Mission updated    Mission stage count: " << Mission_stage_count << endl;
      cout << "====================================================" <<endl;
      cout << "====================================================" <<endl;
    }
    /*offboard and arm*****************************************************/
    if( current_state.mode != "OFFBOARD" && 
          (ros::Time::now() - last_request > ros::Duration(1.0)) &&
          (ros::Time::now() - init_time < ros::Duration(10.0))){ //Set Offboard trigger duration here
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      }
      else{
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0)) &&
          (ros::Time::now() - init_time < ros::Duration(10.0))){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            // mission_state = TAKEOFFP1;
            Mission_stage = 1;
            cout << "Mission stage = 1 Mission start!" <<endl;
          }
          last_request = ros::Time::now();
        }
      }
    /*Mission start here*****************************************************/
    Finite_state_WP_mission();
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    
    int coutcounter;
    if(coutcounter > 20){
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "Mission_Stage: " << Mission_stage << "    Mission_State: " << Mission_state <<endl;
      cout << "currentpos_x: " << uav_lp_x << " y: " << uav_lp_y << " z: "<< uav_lp_z << endl;
      cout << "desiredpos_x: " << pose.pose.position.x << " y: " << pose.pose.position.y << " z: "<< pose.pose.position.z << endl;
      cout << "ROS_time: " << ros::Time::now() << endl;
      cout << "Trajectory_init_time: " << traj1_information[0] << endl;
      cout << "Trajectory duration: " << traj1_information[1] <<endl; 
      cout << "Current_trajectory_size: " << trajectory1.size() << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      coutcounter = 0;
    }else{coutcounter++;}
  }
  return 0;
}
