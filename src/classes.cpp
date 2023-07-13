#include <orca_planner/classes.h>
#include <iostream>
#include <cmath>    //inclusione libreria matematica per eseguire radici quadrate
#include <cstdlib>
#include <orca_planner/functions.h>
#include <gazebo_msgs/ModelStates.h>


#define DIMENSION 2 //dimensione del problema (due dimensioni, x e y)
#define DES_VEL 0.9312//valore di desired velocity (stesso valore sia per Vx che Vy)
#define LAMBDA 0.9928  //valore di lambda del SFM (articolo della prof)
#define TIME_STEP 0.2


//DEFINIZIONI CLASSI:

Goal::Goal(int ID, double x_coordinate, double y_coordinate){
        goalID=ID;
        coordinate.clear();

        coordinate.push_back(x_coordinate);
        coordinate.push_back(y_coordinate);
}


Object::Object(double x_coordinate, double y_coordinate, double r){
        radius=r;
        coordinate.clear();

        coordinate.push_back(x_coordinate);
        coordinate.push_back(y_coordinate);
}

Agents::Agents(){}

Pedestrian::Pedestrian(){}

void Pedestrian::setName(std::string name){
    pedestrianName=name;
}

void Pedestrian::setCurrentPosition(double x, double y){
    curr_pos={x,y};
}

void Pedestrian::setCurrentOrientation(double angle){
    curr_orient=angle;
}
