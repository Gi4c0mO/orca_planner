#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <orca_planner/orca_planner.h>
#include <orca_planner/functions.h>
#include <orca_planner/classes.h>

PLUGINLIB_EXPORT_CLASS(orca_planner::OrcaPlanner, nav_core::BaseLocalPlanner)

// IMPLEMENTAZIONE EFFETTIVA PLUGIN

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_pose_ = *msg;
    // ROS_INFO("Odometria ricevuta!");
}

void people_callback(const gazebo_msgs::ModelStates::ConstPtr &people_msg)
{
    stringa_vector.resize(people_msg->name.size());
    positions.resize(people_msg->name.size());

    for (int i = 0; i < people_msg->name.size(); i++)
    {
        stringa_vector[i] = people_msg->name[i];
        positions[i] = people_msg->pose[i];
    }

    // il messaggio people_msg contiene:
    //       name --> vettore di stringhe, ogni elemento è il nome di ogni modello in Gazebo
    //       pose --> vettore di Pose msgs, ogni elemento è un messaggio Pose (legato al corrispondente elemento indicato nella stessa posizione nel vettore "name")
    //       twist --> vettore di Twist msgs, ogni elemento è un messaggio Twist (legato al corrispondente elemento indicato nella stessa posizione nel vettore "name")

    // dobbiamo estrarre il numero di pedoni presenti.
    // I primi due elementi sono:
    //       GroundPlane
    //       TrossenRoboticsBuilding (dipende se definito nel world)

    // L'ultimo elemento è sempre:
    //       Locobot
}

void obstacle_callback(const sensor_msgs::LaserScan::ConstPtr &obs_msg)
{

    obstacle_distances_ = *obs_msg;
    obs_min_distance = 1000;
    for (int i = 0; i < obstacle_distances_.ranges.size(); i++)
    {
        // controllo sulla validità dei valori misurati dal lidar
        if (obstacle_distances_.ranges[i] > obstacle_distances_.range_min && obstacle_distances_.ranges[i] < obstacle_distances_.range_max)
            if (obstacle_distances_.ranges[i] < obs_min_distance)
            {
                // prelevo il valore più piccolo e conoscendo la sua posizione nel vettore, posso calcolare il corrispondente angolo partendo
                // dall'angolo minimo di rilevamento del LiDar
                obs_min_distance = obstacle_distances_.ranges[i];
                angle_obs_min_distance = obstacle_distances_.angle_min + i * obstacle_distances_.angle_increment;
            }

            else
            {
                continue;
            }

        else
        {
            continue;
        }
    }


}

namespace orca_planner
{

    OrcaPlanner::OrcaPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), pedestrian_list(10), listener(tfBuffer) {}

    OrcaPlanner::OrcaPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false), listener(tfBuffer)
    {
        initialize(name, tf, costmap_ros);
    }

    OrcaPlanner::~OrcaPlanner() {}

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void OrcaPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            initialized_ = true;
            goal_reached = false;
        }
    }

    void OrcaPlanner::getOdometry()
    {
        // subscribe to the robot's odometry topic
        //sub_odom = nh.subscribe<nav_msgs::Odometry>("/mobile_base/odom", 1, &odom_callback);
        sub_odom = nh.subscribe<nav_msgs::Odometry>("/mobile_base/odom", 1, &odom_callback);
        /*
            std::cout << "\n";
            std::cout << "**Odometry received: " << std::endl;
            std::cout << "  *Pose (coordinate+orientation) coordinate Frame  : " << robot_pose_.header.frame_id << std::endl; //FRAME = "map"
            std::cout << "      Coordinates (meters) : " << robot_pose_.pose.pose.position.x << " " << robot_pose_.pose.pose.position.y << std::endl;
            std::cout << "      Orientation z-axis (radians) : " << tf2::getYaw(robot_pose_.pose.pose.orientation) << std::endl;
            std::cout << "\n";
            std::cout << "  *Twist (velocity) coordinate Frame  : " << robot_pose_.child_frame_id << std::endl; //FRAME = "locobot/base_footprint"
            std::cout << "      *Linear Velocity (meters/second) : " << robot_pose_.twist.twist.linear.x << " " << robot_pose_.twist.twist.linear.y << std::endl;
            std::cout << "      *Angular Velocity z-axis (radians/second) : " << robot_pose_.twist.twist.angular.z << std::endl;
        */
        return;
    }

    void OrcaPlanner::set_Position_Orientation_Info()
    {
        curr_robot_coordinates = {robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y}; // non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_orientation = tf2::getYaw(robot_pose_.pose.pose.orientation);

        std::cout << "\n";
        std::cout << "**VETTORI COORDINATE GOAL E COORDINATE ROBOT CALCOLATI: " << std::endl;
        std::cout << "  *Goal coordinates vector  : " << goal_coordinates[0] << " " << goal_coordinates[1] << std::endl;
        std::cout << "  *Goal orientation z-axis (radians) [-pi,pi] : " << goal_orientation << std::endl;
        std::cout << "\n";
        std::cout << "  *Robot coordinates vector  : " << curr_robot_coordinates[0] << " " << curr_robot_coordinates[1] << std::endl;
        std::cout << "  *Robot orientation z-axis (radians) [-pi,pi] : " << curr_robot_orientation << std::endl;

        return;
    }

    void OrcaPlanner::setVelocityInfo()
    {
        curr_robot_lin_vel = {robot_pose_.twist.twist.linear.x, robot_pose_.twist.twist.linear.y}; // non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_ang_vel = robot_pose_.twist.twist.angular.z;

        std::cout << "\n";
        std::cout << "**VETTORE VELOCITÀ ROBOT ATTUALE RISPETTO AL BASE_FOOTPRINT (frame del robot): " << std::endl;
        std::cout << "  *Robot velocity vector  : " << curr_robot_lin_vel[0] << " " << curr_robot_lin_vel[1] << std::endl;

        // ci serve trasformare la velocità dal base_footprint al frame map

        ros::Rate rate(10.0);
        geometry_msgs::TransformStamped tf_result;
        try
        {
            tf_result = tfBuffer.lookupTransform("map", "locobot/base_footprint", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            // TODO: handle lookup failure appropriately
        }

        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w);
        tf2::Vector3 p(0, 0, 0);

        tf2::Transform transform(q, p);
        tf2::Vector3 velocity_in_child_frame(curr_robot_lin_vel[0], curr_robot_lin_vel[1], 0);
        tf2::Vector3 velocity_in_target_frame = transform * velocity_in_child_frame;
        curr_robot_lin_vel = {velocity_in_target_frame[0], velocity_in_target_frame[1]};

        std::cout << "VELOCITY NEL BASE LINK FRAME: " << std::endl;
        std::cout << velocity_in_target_frame[0] << " " << velocity_in_target_frame[1] << std::endl;
        std::cout << curr_robot_lin_vel[0] << " " << curr_robot_lin_vel[1] << std::endl;

        return;
    }

    void OrcaPlanner::getPeopleInformation()
    {
        pedestrian_list.clear();

        sub_people = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &people_callback); // per le simulazioni su Gazebo

        // otteniamo "stringa_vector" (contiene tutti i nomi dei modelli) e "positions" (contiene posizione e orientazione dei modelli)
        int counter_actor = 0;
        std::string string_to_find = "actor";

        for (int i = 0; i < stringa_vector.size(); i++)
        {
            if (stringa_vector[i].find(string_to_find) != -1)
            {
                counter_actor += 1;
            }
            else
            {
                continue;
            }
        }

        std::cout << pedestrian_list.size() << std::endl;
        pedestrian_list.resize(counter_actor);

        counter_actor = 0;
        for (int i = 0; i < stringa_vector.size(); i++)
        {
            if (stringa_vector[i].find(string_to_find) != -1)
            {
                Pedestrian modello;
                modello.setName(stringa_vector[i]);
                modello.setCurrentPosition(positions[i].position.x, positions[i].position.y);
                modello.setCurrentOrientation(tf2::getYaw(positions[i].orientation));
                pedestrian_list[counter_actor] = modello;
                counter_actor += 1;
            }
            else
            {
                continue;
            }
        }

        std::cout << "PEDESTRIAN INFO RECEIVED" << std::endl;
        for (int i = 0; i < pedestrian_list.size(); i++)
        {
            std::cout << pedestrian_list[i].pedestrianName << std::endl;
            std::cout << pedestrian_list[i].curr_pos[0] << " " << pedestrian_list[i].curr_pos[1] << std::endl;
            std::cout << pedestrian_list[i].curr_orient << std::endl;
        }

        // RICEZIONE DELLE INFORMAZIONI DI UN DETERMINATO MODELLO SPECIFICATO SU GAZEBO (necessario andare a fornire il nome del modello)
        //  people_client = nh.serviceClient <gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        //  gazebo_msgs::GetModelState model_to_search;
        //  model_to_search.request.model_name= (std::string) "actor1";

        // if(people_client.call(model_to_search))
        // {
        //     ROS_INFO("PR2's magic moving success!!");
        // }
        // else
        // {
        //     ROS_ERROR("Failed to magic move PR2! Error msg:%s",model_to_search.response.status_message.c_str());
        // }
        // std::cout << model_to_search.response.pose.position.x << std::endl;

        return;
    }

    void OrcaPlanner::getObstacleInformation()
    {

        sub_obs = nh.subscribe<sensor_msgs::LaserScan>("/locobot/scan", 1, &obstacle_callback);

        //std::cout << "INFORMAZIONI OSTACOLO: " << std::endl;
        //std::cout << "Min Distance= " << obs_min_distance << std::endl;
        //std::cout << "Angle=  " << angle_obs_min_distance << std::endl;
    }

    bool OrcaPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        // quando settiamo un nuovo goal (planner frequency 0 Hz nel config file .yaml -> global planner chiamato una volta, solo all'inizio),
        // resettiamo il flag per capire se il goal è stato raggiunto
        goal_reached = false;
        // puliamo anche il vettore di coordinate che conteneva le coordinate del goal precedente
        goal_coordinates.clear();

        // RICEZIONE GOAL MESSAGE
        int size_global_plan = global_plan_.size();
        goal_pose_ = global_plan_[size_global_plan - 1];

        goal_coordinates = {goal_pose_.pose.position.x, goal_pose_.pose.position.y};
        goal_orientation = tf2::getYaw(goal_pose_.pose.orientation);

        std::cout << "COORDINATE GOAL RICEVUTE: " << std::endl;
        std::cout << "Pose Frame : " << goal_pose_.header.frame_id << std::endl; // FRAME = "map" (coincide con /locobot/odom)
        std::cout << "  Coordinates (meters) : " << goal_coordinates[0] << " " << goal_coordinates[1] << std::endl;
        std::cout << "  Orientation z-axis (radians) : " << goal_orientation << std::endl;

        return true;
    }

    bool OrcaPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        // other initializations
        bool flag = false;
        // agents initialization
        Agents robot, pedestrian[n_ped];
        alglib::real_1d_array new_vel = "[0.0,0.0]";
        // goal inizialization
        robot.goal[0] = goal_coordinates[0];
        robot.goal[1] = goal_coordinates[1];
        // acquire information about the robot pose and velocity
        getOdometry();
        set_Position_Orientation_Info();
        robot.position[0] = curr_robot_coordinates[0];
        robot.position[1] = curr_robot_coordinates[1];
        setVelocityInfo();
        robot.velocity[0] = curr_robot_lin_vel[0];
        robot.velocity[1] = curr_robot_lin_vel[1];
        // compute the preferred velocity and goal direction
        robot.goal_direction[0] = (robot.goal[0] - robot.position[0]) / (sqrt((robot.goal[0] - robot.position[0]) * (robot.goal[0] - robot.position[0]) + (robot.goal[1] - robot.position[1]) * (robot.goal[1] - robot.position[1])));
        robot.goal_direction[1] = (robot.goal[1] - robot.position[1]) / (sqrt((robot.goal[0] - robot.position[0]) * (robot.goal[0] - robot.position[0]) + (robot.goal[1] - robot.position[1]) * (robot.goal[1] - robot.position[1])));
        robot.pref_velocity[0] = robot.max_speed * robot.goal_direction[0];
        robot.pref_velocity[1] = robot.max_speed * robot.goal_direction[1];
        // acquire information about the pedestrians and of the minimum distance object point
        getPeopleInformation();
        getObstacleInformation();
        ros::Rate rate1(10.0);
        geometry_msgs::TransformStamped tf_result1;
        try
        {
            tf_result1 = tfBuffer.lookupTransform("map", "locobot/laser_frame_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            // TODO: handle lookup failure appropriately
        }
        tf2::Quaternion q1(
            tf_result1.transform.rotation.x,
            tf_result1.transform.rotation.y,
            tf_result1.transform.rotation.z,
            tf_result1.transform.rotation.w);
        tf2::Vector3 p1(
            tf_result1.transform.translation.x,
            tf_result1.transform.translation.y,
            tf_result1.transform.translation.z
        );
        tf2::Transform transform1(q1, p1);
        tf2::Vector3 point_in_child_coordinates(obs_min_distance*cos(angle_obs_min_distance),obs_min_distance*sin(angle_obs_min_distance),0);
        tf2::Vector3 point_in_parent_coordinates = transform1 * point_in_child_coordinates;
        pedestrian[0].position[0] = point_in_parent_coordinates[0];
        pedestrian[0].position[1] = point_in_parent_coordinates[1];
        pedestrian[0].velocity[0] = 0;
        pedestrian[0].velocity[1] = 0;

        std::cout << "COORDINATE OSTACOLO SETTATE: " << std::endl;
        std::cout << "  Coordinates (meters) : " << pedestrian[0].position[0] << " " << pedestrian[0].position[1] << std::endl;

        for (int i = 0; i < n_ped-1; i++)
        {
            pedestrian[i + 1].position[0] = pedestrian_list[i].curr_pos[0];
            pedestrian[i + 1].position[1] = pedestrian_list[i].curr_pos[1];
            pedestrian[i + 1].velocity[0] = 0.8 * cos(pedestrian_list[i].curr_orient);
            pedestrian[i + 1].velocity[1] = 0.8 * sin(pedestrian_list[i].curr_orient);
            pedestrian[i + 1].radius=0.6;
        }

        std::cout << "COORDINATE PEDESTRIAN SETTATE: " << std::endl;
        std::cout << "  Coordinates (meters) : " << pedestrian[1].position[0] << " " << pedestrian[1].position[1] << std::endl;

        // PROCEDURE TO COMPUTE NEW VEL OR STOP THE NAVIGATION
        new_robot_lin_vel = {0, 0};
        // new_robot_ang_vel_z={0,0};
        new_robot_pos = {0, 0};

        // check of the goal reaching
        if (vect_norm2(goal_coordinates, curr_robot_coordinates) <= distance_tolerance)
        {
            std::cout << " ------------- Distanza raggiunta ----------------" << std::endl;
            // coordinate raggiunte. Vettore velocità lineare rimane nullo.
            new_robot_lin_vel = {0, 0};

            if (std::fabs(angles::shortest_angular_distance(curr_robot_orientation, goal_orientation)) <= angle_tolerance)
            {
                // anche l'orientazione del goal è stata raggiunta
                new_robot_ang_vel_z = 0;
                goal_reached = true;
                std::cout << "Orientazione goal raggiunta" << std::endl;
                std::cout << "GOAL RAGGIUNTO" << std::endl;
            }
            else
            {
                // Se le coordinate del goal sono state raggiunte ma l'orientazione finale no, la velocità angolare deve
                // essere calcolata per far ruotare il robot nella posa finale indicata
                std::cout << "Orientazione non raggiunta" << std::endl;
                new_robot_ang_vel_z = K_p * (angles::shortest_angular_distance(curr_robot_orientation, goal_orientation));
            }
        }
        else
        {
            std::cout << "------------- Distanza non raggiunta ----------------" << std::endl;

            new_vel = velocity_computation(robot, pedestrian); // OPTIMIZATION COMPUTATION
            for (int i = 0; i < new_robot_lin_vel.size(); i++)
            {
                new_robot_lin_vel[i] = new_vel[i];

                // in the ORCA the velocity limit is inserted in the optimization problem
                // if(std::fabs(new_robot_lin_vel[i])>desired_vel){
                //    new_robot_lin_vel[i]=sign(new_robot_lin_vel[i])*desired_vel;
                //}

                new_robot_pos[i] = curr_robot_coordinates[i] + delta_t * new_robot_lin_vel[i]; // inserire qui la velocità nuova calcolata dall'ottimizzazione
            }
            
            flag=false;
            if (vect_norm1(new_robot_lin_vel)==0) {
                flag=true;
            }

            beta = std::atan2(new_robot_pos[1] - curr_robot_coordinates[1], new_robot_pos[0] - curr_robot_coordinates[0]);

            std::cout << "-------- INFO PER ANGOLI -------" << std::endl;
            std::cout << "  *beta= " << beta << std::endl;
            std::cout << "  *Robot orientation z-axis (radians) [-pi,pi] : " << curr_robot_orientation << std::endl;
            std::cout << "  *Goal orientation z-axis (radians) [-pi,pi] : " << goal_orientation << std::endl;
            std::cout << "\n";
            std::cout << "-------- NUOVA VELOCITÀ ROBOT CALCOLATA --------- " << std::endl;
            std::cout << "  *New position : " << new_robot_pos[0] << " " << new_robot_pos[1] << std::endl;
            std::cout << "  *New velocity vector  : " << new_robot_lin_vel[0] << " " << new_robot_lin_vel[1] << std::endl;
            std::cout << "  *New angular velocity : " << new_robot_ang_vel_z << std::endl;

            if ((std::fabs(angles::shortest_angular_distance(curr_robot_orientation, beta)) <= _PI/2)&&(!flag))
            {
                // la rotazione per muovere il robot nella direzione della forza è compresa in [-pi/2;pi/2]
                // possiamo eseguire una combo di rotazione e movimento in avanti

                // ROTAZIONE:
                new_robot_ang_vel_z = K_p * (angles::shortest_angular_distance(curr_robot_orientation, beta));

                if (std::fabs(new_robot_ang_vel_z) > max_angular_vel_z)
                {
                    new_robot_ang_vel_z = sign(new_robot_ang_vel_z) * max_angular_vel_z;
                }

                // TRASLAZIONE GIA' CALCOLATA (RISPETTO AL FRAME "map")
            }
            else
            {
                // è preferibile far ruotare il robot verso la direzione della futura posizione prima di farlo muovere linearmente

                new_robot_lin_vel = {0, 0};
                new_robot_ang_vel_z = sign(angles::shortest_angular_distance(curr_robot_orientation, beta)) * max_angular_vel_z;
            }

            // // new_robot_ang_vel_z=K_p*(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0])));
            // new_robot_ang_vel_z=K_p*(angles::shortest_angular_distance(curr_robot_orientation,goal_orientation));
        }

        // NECESSARIO TRASFORMARE LA VELOCITÀ DA "map" AL FRAME DEL LOCOBOT "/locobot/base_link")
        ros::Rate rate(10.0);
        geometry_msgs::TransformStamped tf_result;
        try
        {
            tf_result = tfBuffer.lookupTransform("locobot/base_link", "map", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            // TODO: handle lookup failure appropriately
        }

        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w);
        tf2::Vector3 p(0, 0, 0);

        tf2::Transform transform(q, p);
        tf2::Vector3 velocity_in_child_frame(new_robot_lin_vel[0], new_robot_lin_vel[1], 0);
        tf2::Vector3 velocity_in_target_frame = transform * velocity_in_child_frame;

        std::cout << "VELOCITY NEL BASE LINK FRAME: " << std::endl;
        std::cout << velocity_in_target_frame[0] << " " << velocity_in_target_frame[1] << std::endl;

        // PUBBLICAZIONE MESSAGGIO
        pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = new_robot_ang_vel_z;
        // cmd_vel.linear.x = vect_norm1(new_robot_lin_vel);
        // cmd_vel.linear.y=0.0;
        cmd_vel.linear.x=std::fabs(velocity_in_target_frame[0]);
        // cmd_vel.linear.x=0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;

        pub_cmd.publish(cmd_vel);

        std::cout << "\nmessaggio pubblicato\n"
                  << std::endl;
        std::cout << "\n-------------------------------------------------------------------\n\n\n"
                  << std::endl;

        return true;
    }

    bool OrcaPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        if (goal_reached)
        {
            return true;
        }
        else
        {
            return false;
        }
        return false;
    }
}
