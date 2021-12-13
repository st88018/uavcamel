#ifndef UAV_MISSION_H
#define UAV_MISSION_H

#include "common.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

deque<Vec8> waypoints;
double velocity_mission = 0.5;
double velocity_angular = 1;

// 1 take off 2 constant velocity traj 3 AM_traj 4 RTL 5 Land 
// 6 PID single pose (step)

deque<Vec8> Mission_generator(string Missionpath){
    fstream Missioncsv;
    Missioncsv.open(Missionpath);
    waypoints.clear();
    string line;
    while (getline( Missioncsv, line,'\n')){
        istringstream templine(line);
        string data;
        int i = 0;
        Vec8 stage;
        while (getline( templine, data,',')){
            // matrix.push_back(atof(data.c_str()));
            stage[i] = atof(data.c_str());
            // cout << "stage[" << i << "]: " << atof(data.c_str()) << endl;
            i++;
        }
        waypoints.push_back(stage);
    }
    if (waypoints.size()==0){
        cout << "Mission File Not Found" << endl;
    }else{
        cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
    }
    return(waypoints);
}
#endif