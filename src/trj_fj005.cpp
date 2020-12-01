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

#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

#include <sstream>
#include <string>

#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"


mavros_msgs::State current_state;
double takeoff_x,takeoff_y,takeoff_z,takeoff_yaw;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
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
deque<Vec8> trajectory2;
Vec2 traj2_information;
// Initial waypoints
deque<Vec8> waypoints;
// Minimun jerk Traj
deque<Vec4> MJwaypoints;
deque<double> ts;

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
  wp << 5, 0, 0, 0, 0,  1, 1, 10;  // state = 5; land.
  waypoints.push_back(wp);
}

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
  double init_time = ros::Time::now().toSec();

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
    traj1 << dt+init_time, xyz[0], xyz[1], xyz[2], q.w(), q.x(), q.y(), q.z();
    trajectory1.push_back(traj1);
  }
}

// For minimun jerk trajectory
void MJwp_Generator(){
  MJwaypoints.clear();
  Vec4 wp; // x y z yaw
  wp << 0, 0, 1, 0;
  MJwaypoints.push_back(wp);
  wp << 1, 0, 1, 0;
  MJwaypoints.push_back(wp);
  // wp << 1, 1, 1, 0;
  // MJwaypoints.push_back(wp);
  // wp << 0, 1, 1, 0;
  // MJwaypoints.push_back(wp);
  // wp << 0, 0, 1, 0;
  // MJwaypoints.push_back(wp);
}

int prod(MatrixXd ar){  //For computeQ use
  int result = 1;
  int n = ar.size();
  for (int i=0; i<n; i++){
    result = result*ar(i);
  }
  return result;
}

MatrixXd computeQ (int n, int r, double t1, double t2){
  int nr = (n-r)*2+1;
  MatrixXd T = MatrixXd::Zero(nr,1);
  MatrixXd Q = MatrixXd::Zero(n+1,n+1);
  for (int i=1;i<nr+1;i++){
    T(i-1) = pow(t2,i)-pow(t1,i);
  }
  for (int i=r+1; i<n+2; i++){
    for (int j=i; j<n+2; j++){
      int k1 = i-r-1;
      int k2 = j-r-1;
      int k = k1+k2+1;
      MatrixXd ar = MatrixXd::Zero(r,1);
      MatrixXd ar2 = MatrixXd::Zero(r,1);
      for (int ari=0;ari<r;ari++){
        ar(ari) = k1+1+ari;
        ar2(ari) = k2+1+ari;
      }
      Q(i-1,j-1) = prod(ar)*prod(ar2)/k*T(k-1);
      Q(j-1,i-1) = Q(i-1,j-1);
    }
  }
  return Q;
}

MatrixXd blkdiag(MatrixXd Q_all, MatrixXd Q, int count){ // For MinJerkPoly
  int Q_c = Q.cols();
  int Q_r = Q.rows();
  for (int i=0; i<Q_c;i++){
    for (int j=0; j<Q_r;j++){
      Q_all(i+count*Q_c,j+count*Q_r) = Q(i,j);
    }
  }
  return Q_all;
}

MatrixXd calc_tvec(double t, int n_order, int r){ // For MinJerkPoly
  MatrixXd tvec = MatrixXd::Zero(1,n_order+1);
  for(int i=r+1; i<n_order+2; i++){
    MatrixXd ar = MatrixXd::Zero(r,1);
    for (int ari=0;ari<r;ari++){
        ar(ari) = i-r+ari;
    }
    tvec(i-1) = prod(ar)*pow(t,(i-r-1));
  }
  return tvec;
}

MatrixXd matrixnegative(MatrixXd matrix){
  int c = matrix.cols();
  int r = matrix.rows();
  MatrixXd output = MatrixXd::Zero(r,c);
  for (int i=0; i<c; i++){
    for (int j=0; j<r; j++){
      if (matrix(j,i)!=0){
        output(j,i) = matrix(j,i)*(-1);
      }else{
        output(j,i) = 0;
      }
    }
  }
  return output;
}

VectorXd vectornegative(VectorXd vector){
  int s = vector.size();
  VectorXd output(s);
  for (int i=0; i<s; i++){
    if (vector(i)!=0){
      output(i) = vector[i]*-1;
    }else{
      output(i) = vector[i];
    }
  }
  return output;
}

deque<Vec3> MatrixQ2ooqpdeque(MatrixXd matrix){
  int nnzQ = sqrt(matrix.size());
  deque<Vec3> output;
  Vec3 vector3;
  for (int i = 1; i < nnzQ/6 +1; i++){
    int k = i*3+(i-1)*3;
    vector3 = Vec3(k, k, matrix(k,k));
    output.push_back(vector3);
    vector3 = Vec3(k+1, k, matrix(k+1,k));
    output.push_back(vector3);
    vector3 = Vec3(k+2, k, matrix(k+2,k));
    output.push_back(vector3);
    vector3 = Vec3(k+1, k+1, matrix(k+1,k+1));
    output.push_back(vector3);
    vector3 = Vec3(k+2, k+1, matrix(k+2,k+1));
    output.push_back(vector3);
    vector3 = Vec3(k+2, k+2, matrix(k+2,k+2));
    output.push_back(vector3);
  }
  return output;
}

deque<Vec3> MatrixA2ooqpdeque(MatrixXd matrix, int row){
  int matrixsize = matrix.size();
  deque<Vec3> output;
  Vec3 vector3;
  for (int j = 0; j<matrixsize/row; j++){
    for (int i = 0; i<row; i++){
      vector3 = Vec3(i,j,matrix(i,j));
      output.push_back(vector3);
    }
  }
  return output;
}

void MinJerkPoly(deque<Vec4> MJwaypoints,int xyzyaw,deque<double> ts, int n_order,double v0, double a0, double ve, double ae){
  /* Only one axis generate a single axis deque */
  deque<double> waypoints;
  for(int i=0; i<MJwaypoints.size();i++){
    Vec4 MJwaypoint = MJwaypoints.at(i);
    waypoints.push_back(MJwaypoint(xyzyaw));
  }
  double p0 = waypoints.front();
  double pe = waypoints.back();
  int n_poly = waypoints.size()-1;
  int n_coef = n_order+1;
  //Compute Q
  MatrixXd Q_all = MatrixXd::Zero((n_order+1)*n_poly,(n_order+1)*n_poly);
  for (int i=0; i<n_poly;i++){
    Q_all = blkdiag(Q_all,computeQ(n_order,3,ts.at(i),ts.at(i+1)),i);
  }
  VectorXd b_all = VectorXd::Zero((n_order+1)*n_poly);
  MatrixXd Aeq = MatrixXd::Zero(4*n_poly+2,n_coef*n_poly);
  VectorXd beq = VectorXd::Zero(4*n_poly+2);
  // Start/terminal pva constraints  (6 equations)
  for (int i=0;i<n_coef; i++){
    MatrixXd tvec;
    tvec = calc_tvec(ts.front(),n_order,0);
    Aeq(0,i) = tvec(i);
    tvec = calc_tvec(ts.front(),n_order,1);
    Aeq(1,i) = tvec(i);
    tvec = calc_tvec(ts.front(),n_order,2);
    Aeq(2,i) = tvec(i);
  }
  for (int i= n_coef*(n_poly-1); i<n_coef*n_poly; i++){
    int j = i-n_coef*(n_poly-1);
    MatrixXd tvec;
    tvec = calc_tvec(ts.back(),n_order,0);
    Aeq(3,i) = tvec(j);
    tvec = calc_tvec(ts.back(),n_order,1);
    Aeq(4,i) = tvec(j);
    tvec = calc_tvec(ts.back(),n_order,2);
    Aeq(5,i) = tvec(j);
  }
  beq(0) = p0;
  beq(1) = v0;
  beq(2) = a0;
  beq(3) = pe;
  beq(4) = ve;
  beq(5) = ae;
  int neq = 6;
  for(int i=0; i<n_poly-1;i++){
    MatrixXd tvec = calc_tvec(ts.at(i+1),n_order,0);
    beq(neq)=waypoints.at(i+1);
    int k = n_coef*(i+1);
    for (int j=0; j<tvec.size(); j++){
      Aeq(neq,k) = tvec(j);
      k++;
    }
    neq++;
  }
  /* continuous constraints  ((n_poly-1)*3 equations) */
  for(int i=1; i<n_poly; i++){
    MatrixXd tvec_p = calc_tvec(ts.at(i),n_order,0);
    MatrixXd tvec_v = calc_tvec(ts.at(i),n_order,1);
    MatrixXd tvec_a = calc_tvec(ts.at(i),n_order,2);
    neq++;
    int tvec_p_size = tvec_p.size();
    for(int j=n_coef*(i-1)+1; j<n_coef*(i+1)+1; j++){
      int k = j-n_coef*(i-1)-1;
      if (k < tvec_p_size){
        Aeq(neq-1,j-1) = tvec_p(k);
      }else{
        Aeq(neq-1,j-1) = - tvec_p(k-tvec_p_size);
      }
    }
    neq++;
    int tvec_v_size = tvec_v.size();
    for(int j=n_coef*(i-1)+1; j<n_coef*(i+1)+1; j++){
      int k = j-n_coef*(i-1)-1;
      if (k < tvec_v_size){
        Aeq(neq-1,j-1) = tvec_v(k);
      }else{
        Aeq(neq-1,j-1) = -tvec_v(k-tvec_p_size);
      }
    }
    neq++;
    int tvec_a_size = tvec_a.size();
    for(int j=n_coef*(i-1)+1; j<n_coef*(i+1)+1; j++){
      int k = j-n_coef*(i-1)-1;
      if (k < tvec_a_size){
        Aeq(neq-1,j-1) = tvec_a(k);
      }else{
        Aeq(neq-1,j-1) = -tvec_a(k-tvec_p_size);
      } 
    }
  }
  // cout << " Q_all: " << endl;
  // cout << Q_all <<endl;
  // cout << " b_all: " << endl;
  // cout << b_all <<endl;
  // cout << " Aeq: " << endl;
  // cout << Aeq <<endl;
  // cout << " beq: " << endl;
  // cout << beq <<endl;
  /* This section is for OOQP */
  bool ooqp = 1;
  if (ooqp>0){  //use OOQP
    /* Q_all */
    int nnzQ = sqrt(Q_all.size());
    int irowQ[nnzQ], jcolQ[nnzQ];
    double dQ[nnzQ];
    deque<Vec3> Q_all2d = MatrixQ2ooqpdeque(Q_all);
    cout << " Q_all: " << endl;
    for (int i=0; i < Q_all2d.size(); i++){
      Vec3 vectemp = Q_all2d.at(i);
      irowQ[i] = vectemp(0);
      jcolQ[i] = vectemp(1);
      dQ[i] = vectemp(2);
      cout << irowQ[i] << " " << jcolQ[i] << " " << dQ[i] << endl;
    }
    /* b_all */
    /* 本來就什麼都沒有 都是0 */ 
    int nx   = b_all.size();
    double    c[nx],xupp[nx],xlow[nx];
    char      ixupp[nx],ixlow[nx];
    cout << " b_all: " << endl;
    for (int i = 0; i<nx; i++){
      c[i] = b_all[i];
      xupp[i]  = 0;
      ixupp[i] = 0;
      xlow[i]  = 0;
      ixlow[i] = 0;
      cout << b_all[i] << endl;
    }
    /* Aieq bieq */ /* inequality constraints */
    int mz = 0;
    double * clow=0;
    char *  iclow=0;
    double * cupp=0;
    char *  icupp=0;
    int nnzC = 0;
    int *irowC=0;
    int *jcolC=0;
    double *dC=0;
    /* beq */
    int my = beq.size();
    double b[my]; 
    // cout << " beq: " << endl;
    for (int i=0; i< my; i++){
      b[i] = beq(i);
      // cout << b[i] << endl;
    }
    /* Aeq */
    deque<Vec3> Aeq2d = MatrixA2ooqpdeque(Aeq, my);
    int nnzA = Aeq2d.size();
    int irowA[nnzA], jcolA[nnzA];
    double dA[nnzA];
    cout << " A_eq: " << endl;
    for (int i = 0; i<nnzA; i++){
      Vec3 vectemp = Aeq2d.at(i);
      irowA[i] = vectemp(0);
      jcolA[i] = vectemp(1);
      dA[i] =    vectemp(2);
      cout << " i: " << i << " " << irowA[i] << " " << jcolA[i] << " " << dA[i] << endl;
    }
    /* start ooqp */
    QpGenSparseMa27 * qp  = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
    QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
            c,      irowQ,  nnzQ,   jcolQ,  dQ,
            xlow,   ixlow,  xupp,   ixupp,

            irowA,  nnzA,   jcolA,  dA,     b,
            irowC,  nnzC,   jcolC,  dC,
            clow,   iclow,  cupp,   icupp );
    QpGenVars      * vars   = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid  = (QpGenResiduals *) qp->makeResiduals( prob );
    GondzioSolver  * s      = new GondzioSolver( qp, prob );
    
    int ierr = s->solve(prob,vars, resid);
    if( ierr == 0 ) {
    cout.precision(4);
    cout << "Solution: \n";
    vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
    } else {
    cout << "Could not solve the problem.\n";
    }
  }
}

void MinJerkTraj(deque<Vec4> MJwaypoints, double velocity){  //Min Jerk Trajectory main
  cout << "------------------------------------------------------------------------------" << endl;
  cout << "------------------------------------------------------------------------------" << endl;
  cout << " MinJerk Triggered " << endl;
  //              x,y,z,yaw   (xyz in meter yaw in rad)
  double V0[4] = {0,0,0,0};
  double A0[4] = {0,0,0,0};
  double V1[4] = {0,0,0,0};
  double A1[4] = {0,0,0,0};
  double totaldist = 0;
  double totalyawrad = 0;
  int n_order = 5;
  double CorridorSize;
  /* Arrange time according to every wpts using their distance and the total velocity. */
  deque<double> dist; //Distance between each waypoints
  deque<double> yaws; //Yaw changes between each waypoints
  deque<double> ts;   //Time cost between each waypoints
  dist.clear(); yaws.clear(); ts.clear();
  /* Calc total dist and yaw*/
  for (int i = 0; i< MJwaypoints.size()-1; i++){
    Vec4 wp    = MJwaypoints.at(i);
    Vec4 nextwp = MJwaypoints.at(i+1);
    totaldist = totaldist + sqrt(pow(nextwp(0)-wp(0),2)+pow(nextwp(1)-wp(1),2)+pow(nextwp(2)-wp(2),2));
    dist.push_back(sqrt(pow(nextwp(0)-wp(0),2)+pow(nextwp(1)-wp(1),2)+pow(nextwp(2)-wp(2),2)));
  }
  double totaltime = totaldist/velocity;
  double k = totaltime/totaldist;
  ts.clear();
  ts.push_back(0);
  for (int i=0;i<dist.size();i++){
    double tss = ts.back()+dist.at(i)*k;
    ts.push_back(tss);
  }
  MinJerkPoly(MJwaypoints,0,ts,n_order,V0[1],A0[1],V1[1],A1[1]); //Second intut x=0;

  // cout << "------------------------------------------------------------------------------" << endl;
  // cout << "------------------------------------------------------------------------------" << endl;
  // cout << "Minimun Jerk Traj Waypoint counts: " << wpcounts <<endl;
  // cout << "Total dist: " << totaldist << endl;
  // cout << "Total Yaw: " << totalyawrad << endl;
  // // cout << "TS count: " << ts.at(3) << endl;
  // cout << "------------------------------------------------------------------------------" << endl;
  // cout << "------------------------------------------------------------------------------" << endl;
}

// For Trajectory publish
void traj_pub(){
  double current_time = ros::Time::now().toSec();
  Vec8 traj1_deque_front = trajectory1.front();

  while (current_time - traj1_deque_front[0] > 0){
    trajectory1.pop_front();
    traj1_deque_front = trajectory1.front();
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
// FSM
void Finite_state_WP_mission(){
  
  // Generate trajectory while mission stage change
  if (Mission_stage != Current_Mission_stage){
    Vec8 traj1;
    Current_Mission_stage = Mission_stage;  //Update Current_Mission_stage
    Current_stage_mission = waypoints.at(Mission_stage-1);
    Quaterniond q(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
    Vec3 current_rpy = Q2rpy(q);
    Mission_state = Current_stage_mission[0];
    
    if (Mission_state == 1){ //state = 1 take off with no heading change
      constantVtraj(uav_lp_p[0], uav_lp_p[1], Current_stage_mission[3], current_rpy[2], Current_stage_mission[6], Current_stage_mission[7]);
      // cout << "applied state 1" <<endl;
    }
    if (Mission_state == 2){ //state = 2; constant velocity trajectory with desired heading.
      constantVtraj(Current_stage_mission[1], Current_stage_mission[2], Current_stage_mission[3], Current_stage_mission[4], Current_stage_mission[5], Current_stage_mission[6]);
      // cout << "applied state 2" <<endl;
    }
    if (Mission_state == 3){ //state = 3; constant velocity trajectory with no heading change.
      constantVtraj(Current_stage_mission[1], Current_stage_mission[2], Current_stage_mission[3], current_rpy[2], Current_stage_mission[5], Current_stage_mission[6]);
      // cout << "applied state 3" <<endl;
    }
    if (Mission_state == 4){ //state = 4; constant velocity RTL but with altitude return to the takeoff heading.
      constantVtraj(takeoff_x, takeoff_y, Current_stage_mission[3], takeoff_yaw, Current_stage_mission[5], Current_stage_mission[6]);
      // cout << "applied state 4" <<endl;
    }
    if (Mission_state == 5){ //state = 5; land.
      constantVtraj(takeoff_x, takeoff_y, takeoff_z-1, takeoff_yaw, Current_stage_mission[5], Current_stage_mission[6]);
      // cout << "applied state 5" <<endl;
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
    traj1_information = Vec2(ros::Time::now().toSec(),traj1[0]-hovertime);
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
// Main
int main(int argc, char **argv){
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
  ros::Time last_request = ros::Time::now();
  ros::Time init_time = ros::Time::now();
  while(ros::ok()){
    /*Update takeoff Position and Mission**********************************************/
    if(Initialfromtakeoffpos)
    {
      /*Generate waypoits**************************************************************/
      Mission_Generator();
      /*Initialize takeoff*************************************************************/
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
      cout << "Mission updated    Mission stage count: " << waypoints.size() << endl;
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
    MJwp_Generator();
    MinJerkTraj(MJwaypoints, 1);
    // Finite_state_WP_mission();
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    /*Mission End here*******************************************************/
    if (Mission_stage > waypoints.size()){
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "Mission End    Shutting Down system" << endl;
      cout << " " << endl;
      cout << "ThankU for using UAV camel" << endl;
      cout << "First Version written in Sep 2020" << endl;
      cout << "PolyU GH034" << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "------------------------------------------------------------------------------" << endl;
    }
    /*Mission information cout***********************************************/
    int coutcounter;
    if(coutcounter > 3){ //reduce cout rate
      cout << "------------------------------------------------------------------------------" << endl;
      cout << "Mission_Stage: " << Mission_stage << "    Mission_total_stage: " << waypoints.size() << endl;
      cout << "Mission_State: " << Mission_state << endl;
      cout << "currentpos_x: " << uav_lp_x << " y: " << uav_lp_y << " z: "<< uav_lp_z << endl;
      cout << "desiredpos_x: " << pose.pose.position.x << " y: " << pose.pose.position.y << " z: "<< pose.pose.position.z << endl;
      cout << "Trajectory_init_time: " << traj1_information[0] << endl;
      cout << "Trajectory_end_time: " << traj1_information[1] <<endl;
      cout << "ROS_time: " << ros::Time::now() << endl; 
      cout << "Current_trajectory_size: " << trajectory1.size() << endl;
      cout << "------------------------------------------------------------------------------" << endl;
      coutcounter = 0;
    }else{coutcounter++;}
  }
  return 0;
}