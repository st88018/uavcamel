/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"

#include <iostream>
#include <string.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;

using namespace std;

/* Testing stuff start ehere */

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

void MJwp_Generator(){ // Generate a tasting set of WP for MinJerkTraj
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

void MinJerkTraj(deque<Vec4> MJwaypoints, double velocity){  //Min Jerk Trajectory main
  cout << "------------------------------------------------------------------------------" << endl;
  cout << "------------------------------------------------------------------------------" << endl;
  cout << "Start generating MinJerkTraj " << endl;
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
  MinJerkPoly(MJwaypoints,0,ts,n_order,V0[1],A0[1],V1[1],A1[1]); //Second input x=0;

  // cout << "------------------------------------------------------------------------------" << endl;
  // cout << "------------------------------------------------------------------------------" << endl;
  // cout << "Minimun Jerk Traj Waypoint counts: " << wpcounts <<endl;
  // cout << "Total dist: " << totaldist << endl;
  // cout << "Total Yaw: " << totalyawrad << endl;
  // // cout << "TS count: " << ts.at(3) << endl;
  // cout << "------------------------------------------------------------------------------" << endl;
  // cout << "------------------------------------------------------------------------------" << endl;
}

int main( int argc, char * argv[] )
{
  cout << "------Checkpoint 1---------------------------------" << endl;
  MinJerkTraj(MJwaypoints, 1); //The second input is the avg vel for the whole traj
  cout << "------Checkpoint 2---------------------------------" << endl; 
}