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

// 1 take off 2 constant velocity traj 4 RTL 5 Land 
// 6 PID single pose (step) 7 PID follow 8 PID land

deque<Vec8> Mission_generator(){
    fstream Missioncsv;
    Missioncsv.open("/home/jeremy/camel_ws/src/uavcamel/src/utils/Missions/AMsquare.csv");
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
    cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
    return(waypoints);
}

// deque<Vec8> Mission_generator(){
//     waypoints.clear();
//     // Waypoints
//     Vec8 stage; // state x y z yaw v av waittime
//     stage << 1, 0, 0, 0.75, 0, velocity_mission, velocity_angular, 5;   // state = 1; takeoff
//     waypoints.push_back(stage);
//     stage << 6, 0, 0, 0.75, 0, 0, 0, 10;
//     waypoints.push_back(stage);
//     stage << 6, -1.5, 1.5, 0.75, 0, 0, 0, 10;
//     waypoints.push_back(stage);
//     stage << 6, -1.5, -1.5, 0.75, 0, 0, 0, 10;
//     waypoints.push_back(stage);
//     stage << 6, 1.5, -1.5, 0.75, 0, 0, 0, 10;
//     waypoints.push_back(stage);
//     stage << 6, 1.5, 1.5, 0.75, 0, 0, 0,  10;
//     waypoints.push_back(stage);
//     stage << 4, 0, 0, 0.75, 0, velocity_mission, velocity_angular, 10;  // state = 4; constant velocity RTL but with altitude.
//     waypoints.push_back(stage);
//     stage << 5, 0, 0, -10, 0, velocity_mission, velocity_angular, 60;  // state = 5; land.
//     waypoints.push_back(stage);
//     cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
//     return(waypoints);
// }

// deque<Vec8> Mission_generator(){ //Normal mission
//     waypoints.clear();
//     // Waypoints
//     Vec8 stage; // state x y z yaw v av waittime
//     stage << 1, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 10;   // state = 1; takeoff
//     waypoints.push_back(stage);
//     stage << 2, 1.5, 1.5, 1, 3.14, velocity_mission, velocity_angular, 0;   // state = 2; constant velocity trajectory.
//     waypoints.push_back(stage);
//     stage << 2,-1.5, 1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 2,-1.5,-1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 2, 1.5,-1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 4, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 10;  // state = 4; constant velocity RTL but with altitude.
//     waypoints.push_back(stage);
//     stage << 5, 0, 0, -10, 3.14, velocity_mission, velocity_angular, 60;  // state = 5; land.
//     waypoints.push_back(stage);
//     cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
//     return(waypoints);
// }
#endif