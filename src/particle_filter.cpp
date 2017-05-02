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
    for (auto particle : particles){
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


    for (auto particle : particles){
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

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
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
}

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
