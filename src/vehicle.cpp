#include "vehicle.h"


Vehicle::Vehicle(const int id, double x, double y, double s, double d, double yaw)
{
    this->id = id;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;

}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d)
{
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;

}

Vehicle::~Vehicle() { }

int Vehicle::get_lane()
{ 
    if (d >= 0 && d < 4) 
        return 0;
    else if (d >= 4 && d < 8)
        return 1;
    else if (d >= 8 && d < 12)
        return 2;
}
