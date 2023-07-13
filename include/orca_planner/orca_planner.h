#ifndef ORCA_PLANNER_H_
#define ORCA_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
// libraries from ROS and cpp utilities
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <string>
#include <orca_planner/classes.h>
#include <sensor_msgs/LaserScan.h>
// optimization libraries and other utilities
#include <iostream>
#include <cmath>
#include <numeric>
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"

using namespace std;

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

// robot pose
nav_msgs::Odometry robot_pose_;

// minimum distance obstacle
sensor_msgs::LaserScan obstacle_distances_;
double obs_min_distance;
double angle_obs_min_distance;

// models info from gazebo
//gazebo_msgs::ModelStates people_;
std::vector<geometry_msgs::Pose> positions(10);
std::vector<std::string> stringa_vector(10);

// global model of the pedestrian
Pedestrian global_model;

namespace orca_planner{

    class OrcaPlanner : public nav_core::BaseLocalPlanner{
        public:
            OrcaPlanner();
            OrcaPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            ~OrcaPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool isGoalReached();

            void getOdometry();

            void set_Position_Orientation_Info();

            void setVelocityInfo();
    
            void getPeopleInformation();

            void getObstacleInformation();
        
        private:

            // publishers, subscribers, node handle objects
            ros::Publisher pub;
            ros::Subscriber sub;
            ros::Subscriber sub_odom;
            ros::Subscriber sub_people;
            ros::Subscriber sub_goal;
            ros::Subscriber sub_obs;
            ros::Publisher pub_cmd;
            ros::NodeHandle nh;

            ros::ServiceClient people_client;

            // tf related objects, needed to perform transformations between RF
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener listener;
            tf2_ros::Buffer* tf_;
            
            // costmap related object
            costmap_2d::Costmap2DROS* costmap_ros_;
    
            // velocity command object
            geometry_msgs::Twist cmd_vel_;
            
            // inizialization boolean variable
            bool initialized_;

            //GOAL:
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            geometry_msgs::PoseStamped goal_pose_, local_goal_pose_;
            std::vector<double> goal_coordinates, local_goal_coordinates;
            double goal_orientation, local_goal_orientation;
            double distance_tolerance=0.15;
            double angle_tolerance=0.13;
            bool goal_reached=false;

            //GAZEBO PEDESTRIANS
            std::vector<Pedestrian> pedestrian_list;

            //ROBOT ODOMETRY AND VELOCITY:
            std::vector<double> curr_robot_coordinates;
            double curr_robot_orientation;
            std::vector<double> curr_robot_lin_vel;
            double curr_robot_ang_vel;
            std::vector<double> new_robot_lin_vel;
            double new_robot_ang_vel_z;
            std::vector<double> new_robot_pos;
    
            //ANGULAR VELOCITY CONTROL
            double alfa_angle;
            double beta_angle;
            double theta_angle;
            double k=0.63;
            double h=0.3;
            double gam=0.2;
            double beta;    
            double K_p=0.8; //costante proporzionale per il calcolo della velocità angolare (proporzionale all'errore);
            double max_lin_acc_x=2.5;
            double max_angular_vel_z=1.3; //da ricavare dal file config dell'interbotix
            double desired_vel = 0.5; //valore da ricavare direttamente dal file dell'interbotix
            double delta_t=0.2;
            
    };
};

#endif
