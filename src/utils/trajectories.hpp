#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H
#include "common.h"
#include "am_traj/am_traj.hpp"

deque<Vec8> trajectory_pos;
deque<Vec8> trajectory_vel;
double AM_traj_pos_duration;
std::vector<double> AM_traj_pos_durations;
double AM_traj_vel_duration;
Vec2 traj_pos_information;
Vec2 traj_vel_information;
double Trajectory_timestep = 0.02;

void constantVtraj( Vec7 StartPose, Vec7 EndPose, double velocity, double angular_velocity){
  Quaterniond localq(StartPose[3],StartPose[4],StartPose[5],StartPose[6]);
  Vec3 localrpy = Q2rpy(localq);
  Vec3 start_rpy = Vec3(0,0,localrpy[2]);
  Vec3 start_xyz(StartPose[0],StartPose[1],StartPose[2]);
  Quaterniond endq(EndPose[3],EndPose[4],EndPose[5],EndPose[6]);
  Vec3 des_rpy = Q2rpy(endq);
  double dist = sqrt(pow((EndPose[0]-start_xyz[0]),2)+pow((EndPose[1]-start_xyz[1]),2)+pow((EndPose[2]-start_xyz[2]),2));
  double dist_duration = dist/velocity; // In seconds
  double duration; //total duration in seconds
  Vec3 vxyz = Vec3(((EndPose[0]-start_xyz[0])/dist)*velocity,((EndPose[1]-start_xyz[1])/dist)*velocity,((EndPose[2]-start_xyz[2])/dist)*velocity);
  if (start_rpy[2]>=M_PI)  start_rpy[2]-=2*M_PI;
  if (start_rpy[2]<=-M_PI) start_rpy[2]+=2*M_PI;
  if (des_rpy[2]>=M_PI)    des_rpy[2]-=2*M_PI;
  if (des_rpy[2]<=-M_PI)   des_rpy[2]+=2*M_PI;
  double d_yaw = des_rpy[2] - start_rpy[2];
  if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
  if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
  double yaw_duration = sqrt(pow(d_yaw/angular_velocity,2));
  if(yaw_duration>=dist_duration){duration = yaw_duration;}else{duration = dist_duration;}
  //initialize trajectory
  trajectory_pos.clear();
  double traj_pos_init_time = ros::Time::now().toSec();
  int wpc = duration/Trajectory_timestep;
  for(int i=0; i<wpc; i++){
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
      if(dt<=dist_duration){
          xyz = Vec3(start_xyz[0]+dt*vxyz[0],start_xyz[1]+dt*vxyz[1],start_xyz[2]+dt*vxyz[2]);
      }else{
          xyz << EndPose[0],EndPose[1],EndPose[2];
      }
      Vec8 traj_pos;
      traj_pos << dt+traj_pos_init_time, xyz[0], xyz[1], xyz[2], q.w(), q.x(), q.y(), q.z();
      trajectory_pos.push_back(traj_pos);
  }
}

void AM_traj_pos(vector<Vector3d> WPs,Vec7 UAV_lp, Vec4 UAV_twist, Vec4 End_twist){
    Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
    Vec3 localrpy = Q2rpy(localq);
    Vec3 desrpy(0,0,localrpy[2]);
    Quaterniond desq;
    Trajectory am_traj;
    desq = rpy2Q(desrpy);
    //(weightT,weightAcc,weightJerk,maxVelRate,maxAccRate,iterations,epsilon);
    AmTraj amTrajOpt(256,16,0.4,2.5,5,100,0.02);
    Vector3d twistxyz_begin(UAV_twist[0],UAV_twist[1],UAV_twist[2]);
    Vector3d twistxyz_end(End_twist[0],End_twist[1],End_twist[2]);
    Vector3d zero3(0.0, 0.0, 0.0);
    am_traj = amTrajOpt.genOptimalTrajDTC(WPs, twistxyz_begin, zero3, twistxyz_end, zero3);
    // cout<< "      WPs.size: " << WPs.size() << endl
    //     << "      Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << endl
    //     << "      Lap Time: " << am_traj.getTotalDuration() << " s" << std::endl
    //     << "      Cost: " << amTrajOpt.evaluateObjective(am_traj) << std::endl
    //     << "      Maximum Velocity Rate: " << am_traj.getMaxVelRate() << " m/s" << std::endl
    //     << "      Maximum Acceleration Rate: " << am_traj.getMaxAccRate() << " m/s^2" << std::endl;
    AM_traj_pos_duration = am_traj.getTotalDuration();
    AM_traj_pos_durations = am_traj.getDurations();
    //initialize trajectory
    trajectory_pos.clear();
    double traj_pos_init_time = ros::Time::now().toSec();
    double T = 0.01;
    for  (double dt = 0; dt < am_traj.getTotalDuration(); dt += T){
        Vector3d xyz = am_traj.getPos(dt);
        Vec8 traj_pos;
        traj_pos << dt+traj_pos_init_time, xyz[0], xyz[1], xyz[2], desq.w(), desq.x(), desq.y(), desq.z();
        trajectory_pos.push_back(traj_pos);
    }
    // if(trajectory_pos.size()>0){
    //     for (unsigned int i = 0; i < trajectory_pos.size(); i++){
    //     Vec8 current_traj = trajectory_pos.at(i);
    //     cout << "dt: " << current_traj[0] << " x: " << current_traj[1] << " y: " << current_traj[2] << " z: " << current_traj[3] << endl;
    //     }
    // }
}

double Q2yaw(Vec4 inputQ){
    Quaterniond q(inputQ[0],inputQ[1],inputQ[2],inputQ[3]);
    Vec3 rpy = Q2rpy(q);
    return(rpy[2]);
}

void AM_traj_vel(vector<Vector3d> WPs,Vec7 UAV_lp, Vec4 UAV_twist){
    Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
    Vec3 localrpy = Q2rpy(localq);
    Vec3 desrpy(0,0,localrpy[2]);
    Quaterniond desq;
    Trajectory am_traj;
    desq = rpy2Q(desrpy);
    //(weightT,weightAcc,weightJerk,maxVelRate,maxAccRate,iterations,epsilon);
    AmTraj amTrajOpt(256,16,0.5,1,1,100,0.02);
    Vector3d twistxyz(UAV_twist[0],UAV_twist[1],UAV_twist[2]);
    Vector3d zero3(0.0, 0.0, 0.0);
    am_traj = amTrajOpt.genOptimalTrajDTC(WPs, twistxyz, zero3, zero3, zero3);
    cout<< "      WPs.size: " << WPs.size() << endl
        << "      Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << endl
        << "      Lap Time: " << am_traj.getTotalDuration() << " s" << std::endl
        << "      Cost: " << amTrajOpt.evaluateObjective(am_traj) << std::endl
        << "      Maximum Velocity Rate: " << am_traj.getMaxVelRate() << " m/s" << std::endl
        << "      Maximum Acceleration Rate: " << am_traj.getMaxAccRate() << " m/s^2" << std::endl;
    AM_traj_vel_duration = am_traj.getTotalDuration();
    //initialize trajectory
    trajectory_vel.clear();
    double traj_vel_init_time = ros::Time::now().toSec();
    double T = 0.01;
    for  (double dt = 0; dt < am_traj.getTotalDuration(); dt += T){
        Vector3d vxyz = am_traj.getVel(dt);
        Vec8 traj_vel;
        traj_vel << dt+traj_vel_init_time, vxyz[0], vxyz[1], vxyz[2], desq.w(), desq.x(), desq.y(), desq.z();
        trajectory_vel.push_back(traj_vel);
    }
    Vec8 traj_vel = trajectory_vel.back();
    for (int i=0; i<(5/T); i++){
        traj_vel[0] += T;
        trajectory_pos.push_back(traj_vel);
    }
    traj_vel_information = Vec2(ros::Time::now().toSec(), traj_vel[0]-5);
    // if(trajectory_pos.size()>0){
    //     for (unsigned int i = 0; i < trajectory_pos.size(); i++){
    //     Vec8 current_traj = trajectory_pos.at(i);
    //     cout << "dt: " << current_traj[0] << " x: " << current_traj[1] << " y: " << current_traj[2] << " z: " << current_traj[3] << endl;
    //     }
    // }
}
#endif