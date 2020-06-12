#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include <math.h>

using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


struct Map 
{
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    void load(string map_file);

    //
    // Helper functions related to waypoints and converting from XY to Frenet
    //   or vice versa
    //
    
    // Calculate closest waypoint to current x, y position
    int closest_waypoint(double x, double y);
    
    // Returns next waypoint of the closest waypoint
    int next_waypoint(double x, double y, double theta);
    
    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> get_frenet(double x, double y, double theta);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> get_xy(double s, double d);

};




#endif //MAP_H