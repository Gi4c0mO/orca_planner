#include <orca_planner/functions.h>
#include <iostream>
#include <cmath>    //inclusione libreria matematica per eseguire radici quadrate
#include <cstdlib>

#define DIMENSION 2 //dimensione del problema (due dimensioni, x e y)

//USEFUL FUNCTIONS:

double dot (alglib::real_1d_array a, alglib::real_1d_array b) {
    return a[0]*b[0]+a[1]*b[1];
} 

alglib::real_2d_array circular_set_approximator (double rho, int n) {
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0, m = 0.0, q = 0.0, theta = 2*alglib::pi()/n;
    alglib::real_2d_array C; C.setlength(n,3);

    for (int i = 0; i < n; i++) {
        x1 = rho*cos(i*theta); x2 = rho*cos((i+1)*theta);
        y1 = rho*sin(i*theta); y2 = rho*sin((i+1)*theta);
        m = (y1-y2)/(x1-x2); q = (x1*y2 - x2*y1)/(x1-x2);
        C[i][0]=1;
        C[i][1]=-m;
        C[i][2]=q;
    }

    return C;
}

alglib::real_1d_array velocity_computation(Agents robot, Agents pedestrian[n_ped])
{
    // output inizialization
    alglib::real_1d_array new_velocity = "[0.0,0.0]";

    // variables initialization for the linear constraint computation
    bool isupper = true;
    alglib::real_1d_array relative_pos = "[0.0,0.0]", relative_vel = "[0.0,0.0]";
    alglib::real_1d_array adjusted_center = "[0.0,0.0]", w = "[0.0,0.0]", u = "[0.0,0.0]", n = "[0.0,0.0]", rotated_x = "[0.0,0.0]", aux = "[0.0,0.0]";
    double radius_sum, norm_sq_relative_pos, leg_len, sine;
    alglib::real_2d_array rot = "[[0.0,0.0],[0.0,0.0]]";

    // variables initialization for the optimization problem computation
    alglib::real_2d_array a = "[[2.0,0.0],[0.0,2.0]]";
    alglib::real_1d_array b = "[0.0,0.0]";
    alglib::real_1d_array x0 = "[0.0,0.0]";
    alglib::real_1d_array s = "[1,1]";
    alglib::real_2d_array c, C;
    // real_2d_array c="[[0.0,0.0,0.0],[0.0,0.0,0.0]]";
    c.setlength(n_edg + n_ped, 3); // the dimensions of this array change on the base of the number of pedestrian sensed in the environment
    C.setlength(n_edg, 3);
    alglib::integer_1d_array ct;
    ct.setlength(n_edg + n_ped);
    alglib::real_1d_array x = "[0.0,0.0]";
    alglib::minqpstate state;
    alglib::minqpreport rep;
    C = circular_set_approximator(robot.max_speed, n_edg); // second order contraint approximation
    for (int ii = 0; ii < n_edg; ii++)
    {
        if (ii < n_edg / 2)
        {
            ct[ii] = -1;
        }
        else
        {
            ct[ii] = 1;
        }
        c[ii][0] = C[ii][0];
        c[ii][1] = C[ii][1];
        c[ii][2] = C[ii][2];
    }

    // preferred velocity computation
    robot.goal_direction[0] = (robot.goal[0] - robot.position[0]) / (sqrt((robot.goal[0] - robot.position[0]) * (robot.goal[0] - robot.position[0]) + (robot.goal[1] - robot.position[1]) * (robot.goal[1] - robot.position[1])));
    robot.goal_direction[1] = (robot.goal[1] - robot.position[1]) / (sqrt((robot.goal[0] - robot.position[0]) * (robot.goal[0] - robot.position[0]) + (robot.goal[1] - robot.position[1]) * (robot.goal[1] - robot.position[1])));
    robot.pref_velocity[0] = robot.max_speed * robot.goal_direction[0];
    robot.pref_velocity[1] = robot.max_speed * robot.goal_direction[1];

    // constraints computation for each pedestrian
    for (int i = 0; i < n_ped; i++)
    { // computation of one linear constraint each pedestrian detected

        relative_pos[0] = -(robot.position[0] - pedestrian[i].position[0]) + 0.0001;
        relative_pos[1] = -(robot.position[1] - pedestrian[i].position[1]) + 0.0001;
        relative_vel[0] = (robot.velocity[0] - pedestrian[i].velocity[0]) + 0.0001;
        relative_vel[1] = (robot.velocity[1] - pedestrian[i].velocity[1]) + 0.0001;
        radius_sum = robot.radius + pedestrian[i].radius;
        norm_sq_relative_pos = relative_pos[0] * relative_pos[0] + relative_pos[1] * relative_pos[1];

        if (norm_sq_relative_pos >= radius_sum * radius_sum)
        {
            adjusted_center[0] = relative_pos[0] / tau * (1 - (radius_sum * radius_sum) / norm_sq_relative_pos);
            adjusted_center[1] = relative_pos[1] / tau * (1 - (radius_sum * radius_sum) / norm_sq_relative_pos);
            aux[0] = relative_vel[0] - adjusted_center[0];
            aux[1] = relative_vel[1] - adjusted_center[1];

            if (dot(aux, adjusted_center) < 0.0)
            {
                w[0] = relative_vel[0] - relative_pos[0] / tau;
                w[1] = relative_vel[1] - relative_pos[1] / tau;
                u[0] = w[0] / (sqrt(w[0] * w[0] + w[1] * w[1])) * radius_sum / tau - w[0];
                u[1] = w[1] / (sqrt(w[0] * w[0] + w[1] * w[1])) * radius_sum / tau - w[1];
                n[0] = w[0] / (sqrt(w[0] * w[0] + w[1] * w[1]));
                n[1] = w[1] / (sqrt(w[0] * w[0] + w[1] * w[1]));
            }
            else
            {
                leg_len = sqrt(norm_sq_relative_pos - radius_sum * radius_sum);
                sine = copysign(radius_sum, (relative_vel[0] * relative_pos[1] - relative_vel[1] * relative_pos[0])); // the sign of sine determines the side of projection
                rot[0][0] = leg_len;
                rot[0][1] = sine;
                rot[1][0] = -sine;
                rot[1][1] = leg_len;
                rotated_x[0] = (leg_len * relative_pos[0] + sine * relative_pos[1]) / norm_sq_relative_pos;
                rotated_x[1] = (-sine * relative_pos[0] + leg_len * relative_pos[1]) / norm_sq_relative_pos;
                n[0] = rotated_x[1];
                n[1] = -rotated_x[0];
                if (sine < 0)
                {
                    n[0] = -n[0];
                    n[1] = -n[1];
                }
                u[0] = rotated_x[0] * dot(relative_vel, rotated_x) - relative_vel[0];
                u[1] = rotated_x[1] * dot(relative_vel, rotated_x) - relative_vel[1];
            }
        }
        else
        {
            w[0] = relative_vel[0] - relative_pos[0] / time_step;
            w[1] = relative_vel[1] - relative_pos[1] / time_step;
            u[0] = w[0] / (sqrt(w[0] * w[0] + w[1] * w[1])) * radius_sum / time_step - w[0];
            u[1] = w[1] / (sqrt(w[0] * w[0] + w[1] * w[1])) * radius_sum / time_step - w[1];
            n[0] = w[0] / (sqrt(w[0] * w[0] + w[1] * w[1]));
            n[1] = w[1] / (sqrt(w[0] * w[0] + w[1] * w[1]));
        }
        c[n_edg + i][0] = n[0];
        c[n_edg + i][1] = n[1];
        c[n_edg + i][2] = n[0] * (robot.velocity[0] + resp * u[0]) + n[1] * (robot.velocity[1] + resp * u[1]);
        ct[n_edg + i] = 1;
    }
    b[0] = -2 * robot.pref_velocity[0];
    b[1] = -2 * robot.pref_velocity[1];

    // solution of the optimization problem
    minqpcreate(2, state);
    minqpsetquadraticterm(state, a, isupper);
    minqpsetlinearterm(state, b);
    minqpsetstartingpoint(state, x0);
    // minqpsetbc(state, bndl, bndu);
    minqpsetlc(state, c, ct);
    minqpsetscale(state, s);
    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    // minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 5);
    minqpoptimize(state);
    minqpresults(state, x, rep);

    // new velocity ouput
    new_velocity[0] = x[0];
    new_velocity[1] = x[1];

    if (isnan(new_velocity[0]))
    {
        new_velocity[0] = 0.0;
    }
    if (isnan(new_velocity[1]))
    {
        new_velocity[1] = 0.0;
    }

    std::cout << "New velocity computed for #" << robot.ID << std::endl;
    std::cout << "Vx=" << new_velocity[0] << " Vy=" << new_velocity[1] << std::endl;

    return new_velocity;
}

//CALCOLO DISTANZA (norma del vettore differenza tra due vettori, vec1 e vec2)
double vect_norm2(std::vector<double> vec1, std::vector<double> vec2){
        double norma;
        std::vector<double> difference={0,0};
        
        for (int i=0; i<DIMENSION; i++){
            difference[i]=vec1[i]-vec2[i];
        }
        
        norma = sqrt(difference[0]*difference[0]+difference[1]*difference[1]);
        return norma;
    }

//CALCOLO NORMA DI UN VETTORE
double vect_norm1(std::vector<double> vec1){
        double norma=0;
        norma = sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1]);
        return norma;
    }

//STABILISCE IL VERSORE DIREZIONE RIVOLTO DA vec1 A vec2
std::vector<double> compute_direction(std::vector<double> vec1, std::vector<double> vec2){
        
        std::vector<double> dir={0,0};
        double norm = vect_norm2(vec1, vec2);

        if (norm<0.01) norm=0.01; //se tende a zero allora la forza diventa troppo grande e può causare problemi

        for(int i=0; i<DIMENSION; i++){
            dir[i]=(vec1[i]-vec2[i])/norm;
        }

        return dir;
    }
    
//CALCOLA IL COSENO DELL'ANGOLO TRA I DUE VETTORI
double compute_cos_gamma(std::vector<double> vec1, std::vector<double> vec2){
    double coseno=0;
    for(int i=0; i<vec1.size(); i++){
        vec2[i]=-vec2[i];
    }

    coseno=(vec1[0]*vec2[0]+vec1[1]*vec2[1])/(vect_norm1(vec1)*vect_norm1(vec2));
    return coseno;
}

//FUNZIONE SEGNO:
int sign(double x){
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}
