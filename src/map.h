#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>

using std::string;
using std::vector;

struct Map 
{
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    void load(string map_file);

};


#endif //MAP_H