#ifndef VEHICLE_H
#define VEHICLE_H


const int EGOCAR_ID = -1;


class Vehicle 
{
    public:
        //consturctor
        Vehicle(const int id, double x, double y, double s, double d, double yaw);
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

        //destructor
        ~Vehicle();

        //functions
        double get_speed();

        //variables
        int id;
        double x, y, s, d, yaw, vx, vy, speed;

};


#endif //VEHICLE_H
