/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"
#include <random>

using namespace std;

/**
 * init Initializes particle filter by initializing particles to Gaussian
 *   distribution around first position and all the weights to 1.
 * @param x Initial x position [m] (simulated estimate from GPS)
 * @param y Initial y position [m]
 * @param theta Initial orientation [rad]
 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
 *   standard deviation of yaw [rad]]
 */
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    // Number of particles to draw

    num_particles = 3;
    default_random_engine gen;
    normal_distribution<double> N_x_init(x, std[0]);
    normal_distribution<double> N_y_init(y, std[1]);
    normal_distribution<double> N_theta_init(theta, std[2]);

    // Initialize all particles to first position
    for (size_t iter = 0; iter < num_particles; iter ++){

        // Initialise all weights to 1
        weights.push_back(1);

        Particle newParticle;
        newParticle.id = iter;
        // Add random Gaussian noise to each particle
        // this is similar to what main does, we just make it here for each particle
        double n_x = N_x_init(gen);
        double n_y = N_y_init(gen);
        double n_theta = N_theta_init(gen);

        newParticle.x = n_x;
        newParticle.y = n_y;
        newParticle.theta = n_theta;
        newParticle.weight = weights[iter];
        particles.push_back(newParticle);
    }

    // Flag, if filter is initialized
    is_initialized = true;


    // BUG
    for (const auto& particle : particles){
        std::cout << particle.id << ", " << particle.x <<  ", " << particle.y <<  ", " << particle.theta <<  ", " << particle.weight << std::endl;
    }
    // BUG
}


/**
 * prediction Predicts the state for the next time step
 *   using the process model. (See motion model)
 * @param delta_t Time between time step t and t+1 in measurements [s]
 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
 *   standard deviation of yaw [rad]]
 * @param velocity Velocity of car from t to t+1 [m/s]
 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;


    for (auto& particle : particles){
        double x_0 = particle.x;
        double y_0 = particle.y;
        double yaw_0 = particle.theta;
        double x_f;
        double y_f;
        double yaw_f;
        // Motion Model - Bicycle Model
        if (fabs(yaw_rate) < 0.001){ // if yaw_rate is zero
            x_f = x_0 + velocity * delta_t * cos(yaw_0);
            y_f = y_0 + velocity * delta_t * sin(yaw_0);
            yaw_f = yaw_0;
        }
        else{ // otherwise
            x_f = x_0 + velocity / yaw_rate * (sin(yaw_0 + yaw_rate * delta_t ) - sin(yaw_0));
            y_f = y_0 + velocity / yaw_rate * (cos(yaw_0) - cos(yaw_0 + yaw_rate * delta_t ));
            yaw_f = yaw_0 + yaw_rate * delta_t;
        }

        // Add random Gaussian noise
        normal_distribution<double> N_x_init(x_0, std_pos[0]);
        normal_distribution<double> N_y_init(y_0, std_pos[1]);
        normal_distribution<double> N_theta_init(yaw_0, std_pos[2]);

        double n_x = N_x_init(gen);
        double n_y = N_y_init(gen);
        double n_theta = N_theta_init(gen);

        particle.x = x_f + n_x;
        particle.y = y_f + n_y;
        particle.theta = yaw_f + n_theta;
    }

}

/**
 * dataAssociation Finds which observations correspond to which landmarks (likely by using
 *   a nearest-neighbors data association).
 * @param predicted Vector of predicted landmark observations
 * @param observations Vector of landmark observations
 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void LocalToGlobal(double& x_land, double&y_land, double x_0, double y_0, double yaw_0){
//        function [global_coords] = LocalToGlobal (~,x_local, y_local, system_x, system_y, system_theta)
//            theta = system_theta;
//            Rot_Mat = [cos(theta) -sin(theta); ...
//                       sin(theta)  cos(theta)];
//            global_coords = (Rot_Mat)*[x_local; y_local]+[system_x; system_y];
//        end

    // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    // Note that you'll need to switch the minus sign in that equation to a plus to account
    //   for the fact that the map's y-axis actually points downwards.)
    x_land = x_land * cos(yaw_0) + y_land * sin(yaw_0) + x_0;
    y_land = y_land * sin(yaw_0) - y_land * cos(yaw_0) + y_0;

}



/**
 * updateWeights Updates the weights for each particle based on the likelihood of the
 *   observed measurements.
 * @param sensor_range Range [m] of sensor
 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
 *   standard deviation of bearing [rad]]
 * @param observations Vector of landmark observations
 * @param map Map class containing map landmarks
 */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
    // TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

    for (size_t iter = 0; iter < particles.size(); iter++){
        auto& particle = particles[iter];
        auto& weight = weights[iter];
        weight = 1;
        // observations are in local coordinate system
        // translate observations to global coordinate system
        for (auto& observation : observations){
//            cout << "local = " << observation.x << "," << observation.y << std::endl;
            LocalToGlobal(observation.x, observation.y, particle.x, particle.y, particle.theta);
//            cout << "global = " << observation.x << "," << observation.y << std::endl;
            // Update the weights of each particle using a multi-variate Gaussian distribution
            //weight = weight * (exp(-1/2*(x_i-mean_i)))

            // using the bivariate formulae as someone suggested on slack channel
            // (x.x - m.x) * (x.x - m.x) / Z(0, 0) + (x.y - m.y) * (x.y - m.y) / Z(1, 1)
             weight = weight * 1/sqrt(2.0*M_PI*std_landmark[0]*std_landmark[1])*
             std::exp(-(std::pow(x_meas-x_mu,2.0)/(pow(std_landmark[0],2.0))+pow(y_meas-y_mu,2.0)/(pow(std_landmark[1],2.0))));
        }
    }
}


/**
 * resample Resamples from the updated set of particles to form
 *   the new set of particles.
 */
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
