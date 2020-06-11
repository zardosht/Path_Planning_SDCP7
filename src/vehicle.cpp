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