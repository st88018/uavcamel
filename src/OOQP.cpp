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

using namespace std;

// const int nx   = 2;
// double    c[]  = { 1.5,  -2 };

// double  xupp[] = { 20,   0 };
// char   ixupp[] = {  1,   0 };

// double  xlow[] = {  0,   0 };
// char   ixlow[] = {  1,   1 };

// const int nnzQ = 3;
// int    irowQ[] = {  0,   1,   1 }; 
// int    jcolQ[] = {  0,   0,   1 };
// double    dQ[] = {  8,   2,  10 };

// int my         = 0;
// double * b     = 0;
// int nnzA       = 0;
// int * irowA    = 0;
// int * jcolA    = 0;
// double * dA    = 0;

// const int mz   = 2;
// double clow[]  = { 2,   0 };
// char  iclow[]  = { 1,   0 };
// double cupp[]  = { 0,   6 };
// char  icupp[]  = { 0,   1 };

// const int nnzC = 4;
// int   irowC[]  = { 0,   0,   1,   1};
// int   jcolC[]  = { 0,   1,   0,   1};
// double   dC[]  = { 2,   1,  -1,   2};

/*------------------------------------------------------------TesT2*/
// //b_all
// const int nx   = 2;
// double    c[]  = { -2,  -6 };
// //l u
// double  xupp[] = { 0 };
// char   ixupp[] = { 0 };
// double  xlow[] = {  0 };
// char   ixlow[] = {  0 };
// // Q_all
// const int nnzQ = 3;
// int    irowQ[] = {  0,   1,   1 }; 
// int    jcolQ[] = {  0,   0,   1 };
// double    dQ[] = {  1,  -1,   2 };
// //Aeqbeq
// int my         = 1;
// double b[]     = {0};
// int nnzA       = 2;
// int irowA[]    = {0,0};
// int jcolA[]    = {0,1};
// double dA[]    = {1,1};
// // int my         = 0;
// // double b[]     = {0};
// // int nnzA       = 0;
// // int *irowA    = 0;
// // int *jcolA    = 0;
// // double *dA    = 0;
// //AieqBieq
// const int mz   = 3;
// double clow[]  = {0,0,0};
// char  iclow[]  = {0,0,0};
// double cupp[]  = {2,2,3};
// char  icupp[]  = {1,1,1};
// const int nnzC = 6;
// int   irowC[]  = {0,0,1,1,2,2};
// int   jcolC[]  = {0,1,0,1,0,1};
// double   dC[]  =  {1,1,-1,2,2,1};
// // const int mz   = 0;
// // double *clow  = 0;
// // char  *iclow  = 0;
// // double *cupp  = 0;
// // char  *icupp  = 0;
// // const int nnzC = 0;
// // int   *irowC  = 0;
// // int   *jcolC  = 0;
// // double   *dC  = 0;

/*------------------------------------------------------------Q_all*/
const int nnzQ = 12;
int    irowQ[] = { 3, 4,  5, 4,   5,  5,  9, 10,   11,   10,   11,    11}; 
int    jcolQ[] = { 3, 3,  3, 4,   4,  5,  9,  9,    9,   10,   10,    11};
double    dQ[] = {36,72,120,192,360,720, 36, 216, 840, 1344, 5400, 22320};
/*------------------------------------------------------------b_all*/
int nx   = 12;
double   c[]  = {0,0,0,0,0,0,0,0,0,0,0,0};  
/*------------------------------------------------------------lbub*/
// double  xupp[] = {0};
// char   ixupp[] = {0};
// double  xlow[] = {0};
// char   ixlow[] = {0};
double  xupp[] = {0,0,0,0,0,0,0,0,0,0,0,0};
char   ixupp[] = {0,0,0,0,0,0,0,0,0,0,0,0};
double  xlow[] = {0,0,0,0,0,0,0,0,0,0,0,0};
char   ixlow[] = {0,0,0,0,0,0,0,0,0,0,0,0};
/*----------------------------------------------------------AeqBeq*/
int my         = 10;
double  b[]    = {0,0,0,1,0,0,1,0,0,0};
int nnzA     = 120;
int irowA[]    = {0,0,0,0,0,0,0,0,0,0,0,0,
                  1,1,1,1,1,1,1,1,1,1,1,1,
                  2,2,2,2,2,2,2,2,2,2,2,2,
                  3,3,3,3,3,3,3,3,3,3,3,3,
                  4,4,4,4,4,4,4,4,4,4,4,4,
                  5,5,5,5,5,5,5,5,5,5,5,5,
                  6,6,6,6,6,6,6,6,6,6,6,6,
                  7,7,7,7,7,7,7,7,7,7,7,7,
                  8,8,8,8,8,8,8,8,8,8,8,8,
                  9,9,9,9,9,9,9,9,9,9,9,9};
int   jcolA[]  = {0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11,
                  0,1,2,3,4,5,6,7,8,9,10,11};
double   dA[]  = {1,0,0,0,0,0,0,0,0,0,0,0,
                  0,1,0,0,0,0,0,0,0,0,0,0,
                  0,0,2,0,0,0,0,0,0,0,0,0,
                  0,0,0,0,0,0,1,2,4,8,16,32,
                  0,0,0,0,0,0,0,1,4,12,32,80,
                  0,0,0,0,0,0,0,0,2,12,48,160,
                  0,0,0,0,0,0,1,1,1,1,1,1,
                  1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,
                  0,1,2,3,4,5,0,-1,-2,-3,-4,-5,
                  0,0,2,6,12,20,0,0,-2,-6,-12,-20};
// /*------------------------------------------------------------AieqBieq*/
// const int mz   = 2;
// double clow[]  = {0,0};
// char   iclow[] = {0,0};
// double cupp[]  = {2,1};
// char  icupp[]  = {1,1};
// const int nnzC = 20;
// int  irowC[]  = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1};
// int  jcolC[]  = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
// double   dC[] = {0,0,0,0,0,1,1.11,1.25,1.39,1.56,
//                  0,0,0,0,0,-1,-1.11,-1.25,-1.39,-1.56};
// int mz   = 1;
// char   iclow[] = {1}; 
// double clow[]  = {1}; //d
// char  icupp[]  = {1}; 
// double cupp[]  = {2}; //f
// int nnzC = 10;
// int   irowC[]  = {0,0,0,0,0,0,0,0,0,0};
// int   jcolC[]  = {0,1,2,3,4,5,6,7,8,9};
// double   dC[]  = {0,0,0,0,0,1,1.11,1.25,1.39,1.56};

const int mz   = 0;
double *clow  = 0;
char  *iclow  = 0;
double *cupp  = 0;
char  *icupp  = 0;
const int nnzC = 0;
int   *irowC  = 0;
int   *jcolC  = 0;
double   *dC  = 0;

int main( int argc, char * argv[] )
{
  int usage_ok = 1, quiet = 0;
  // cout <<" OKOK "<<endl;
  if( argc > 2 ) usage_ok = 0;
  if( argc == 2 ) {
    if( 0 == strcmp( "--quiet", argv[1] ) ) {
      quiet = 1;
    } else {
      usage_ok = 0;
    }
  } 
  if( !usage_ok ) {
    cerr << "Usage: " <<  argv[0] << " [ --quiet ]\n";
    return 1;
  }
    
  QpGenSparseMa27 * qp 
    = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
  
  QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
        c,      irowQ,  nnzQ,   jcolQ,  dQ,
        xlow,   ixlow,  xupp,   ixupp,
        irowA,  nnzA,   jcolA,  dA,     b,
        irowC,  nnzC,   jcolC,  dC,
        clow,   iclow,  cupp,   icupp );

  QpGenVars      * vars 
    = (QpGenVars *) qp->makeVariables( prob );
  QpGenResiduals * resid 
    = (QpGenResiduals *) qp->makeResiduals( prob );
  
  GondzioSolver  * s     = new GondzioSolver( qp, prob );
  
  if( !quiet ) s->monitorSelf();
  int ierr = s->solve(prob,vars, resid);
  
  if( ierr == 0 ) {
    cout.precision(4);
    cout << "Solution: \n";
    vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
  } else {
    cout << "Could not solve the problem.\n";
  }
  return ierr;
}