#ifndef _CLASSES_H_
#define _CLASSES_H_

#include <vector>
#include <string>
#include <gazebo_msgs/ModelStates.h>

#include <iostream>
#include <cmath>
#include <numeric>
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"

//******* DEFINIZIONE CLASSE GOAL ************
class Goal{
    public:
        int goalID;
        std::vector<double> coordinate={0,0}; //I VETTORI DI VETTORI SONO DA CONSIDERARE TUTTI SU UN'UNICA COLONNA

    //constructor
    Goal(int ID, double x_coordinate, double y_coordinate);
};


//******* DEFINIZIONE CLASSE OBJECT ************
class Object{
    public:
        std::vector<double> coordinate={0,0};
        double radius;

    //constructor
    Object(double x_coordinate, double y_coordinate, double r);
};

class Agents
{
    // our agents are assumed to be disk-shaped
    public:
        int ID = 0;
        alglib::real_1d_array position = "[0.0,0.0]";
        alglib::real_1d_array velocity = "[0.0,0.0]";
        alglib::real_1d_array goal = "[0.0,0.0]";
        double radius = 0.3;
        double max_speed = 0.4;
        alglib::real_1d_array pref_velocity = "[0.0,0.0]";
        alglib::real_1d_array goal_direction = "[0.0,0.0]";
    //constructor
    Agents();
    
};

class Pedestrian{
    public:
        std::string pedestrianName="";
        std::vector<double> curr_pos={0,0}; //current position of Pedestrian
        double curr_orient=0; //current orientation of Pedestrian
        double radius=0.3;

    //CLASS METHODS
    Pedestrian();
    void setName(std::string name);
    void setCurrentPosition(double x, double y);
    void setCurrentOrientation(double angle);
};

#endif
