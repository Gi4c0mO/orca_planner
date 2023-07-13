#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <vector>
#include "classes.h"

#include <iostream>
#include <cmath>
#include <numeric>
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"

//ORCA PARAMETERS:
const int n_edg = 20;
const double time_step = 0.2;
const double tau = 10.0;
const double resp = 0.6;
const int n_ped = 2; // number of pedestrians plus one (nearest point)

//USEFUL FUNCTIONS:

double dot (alglib::real_1d_array a, alglib::real_1d_array b);

alglib::real_2d_array circular_set_approximator (double rho, int n);

alglib::real_1d_array velocity_computation(Agents robot, Agents pedestrian[n_ped]);

//DEFINIZIONE ALTRE FUNZIONI UTILI:

double vect_norm2(std::vector<double> vec1, std::vector<double> vec2);

double vect_norm1(std::vector<double> vec1);

std::vector<double> compute_direction(std::vector<double> vec1, std::vector<double> vec2);

double compute_cos_gamma(std::vector<double> vec1, std::vector<double> vec2); //calcolo del coseno gamma per poter determinare successivamente l'effetto della forza repulsiva

int sign(double expression);

#endif
