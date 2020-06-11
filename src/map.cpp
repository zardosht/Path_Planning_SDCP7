#include "map.h"
#include <string>
#include <sstream>
#include <fstream>

using std::string;

void Map::load(string map_file)
{
    std::ifstream in_map(map_file.c_str(), std::ifstream::in);
    string line;
    while (getline(in_map, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);
    }
}