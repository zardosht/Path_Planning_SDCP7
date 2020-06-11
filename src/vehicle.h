#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle 
{
    public:
        //consturctor
        Vehicle();

        //destructor
        ~Vehicle();

        //functions
        double get_speed();

        //variables
        double x, y, s, d, yaw, vx, vy, speed;

};


#endif //VEHICLE_H
