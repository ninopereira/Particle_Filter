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

    num_particles = 100;
    default_random_engine gen;
    normal_distribution<double> N_x_init(x, std[0]);
    normal_distribution<double> N_y_init(y, std[1]);
    normal_distribution<double> N_theta_init(theta, std[2]);
    double initial_weight = 1;

    // Initialize all particles to first position
    for (size_t iter = 0; iter < num_particles; iter ++){

        // Initialise all weights to 1
        weights.push_back(initial_weight);

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
        newParticle.weight = initial_weight;
        particles.push_back(newParticle);
    }

    // Flag, if filter is initialized
    is_initialized = true;


    // BUG
//    for (const auto& particle : particles){
//        std::cout << particle.id << ", " << particle.x <<  ", " << particle.y <<  ", " << particle.theta <<  ", " << particle.weight << std::endl;
//    }
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
            x_f = x_0 + (velocity / yaw_rate) * (sin(yaw_0 + yaw_rate * delta_t ) - sin(yaw_0));
            y_f = y_0 + (velocity / yaw_rate) * (cos(yaw_0) - cos(yaw_0 + yaw_rate * delta_t ));
            yaw_f = yaw_0 + yaw_rate * delta_t;
        }

        // Add random Gaussian noise
        normal_distribution<double> N_x_init(x_f, std_pos[0]);
        normal_distribution<double> N_y_init(y_f, std_pos[1]);
        normal_distribution<double> N_theta_init(yaw_f, std_pos[2]);

        double n_x = N_x_init(gen);
        double n_y = N_y_init(gen);
        double n_theta = N_theta_init(gen);

        particle.x = n_x;
        particle.y = n_y;
        particle.theta = n_theta;
    }
}

/**
 * dataAssociation Finds which observations correspond to which landmarks (likely by using
 *   a nearest-neighbors data association).
 * @param predicted Vector of predicted landmark observations
 * @param observations Vector of landmark observations
 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predictedObs, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


    // find wich of the predicted landmarks (map landmarks) corresponds to each observation by computing nearest neighbor
    // assign the id of the landmark on the list of predicted landmarks to the observation id
    // Note: We assume the map is complete and immutable
    // The observation id should match the id of a landmark on the list.
    // We assign the id of the object


    for (auto& observation : observations){
        int nearestObsId = 0;
        double closestDist = 1000000; // very large number
        for (auto const& predObs : predictedObs){
            double distance = sqrt(pow(predObs.x-observation.x,2)+pow(predObs.y-observation.y,2));

            if (distance < closestDist){
                nearestObsId = predObs.id;
                closestDist = distance;
            }
        }
        observation.id = nearestObsId;
    }
}

void LocalToGlobal(double& x_land, double& y_land, double x_0, double y_0, double yaw_0){
//        function [global_coords] = LocalToGlobal (~,x_local, y_local, system_x, system_y, system_theta)
//            theta = system_theta;
//            Rot_Mat = [cos(theta) -sin(theta); ...
//                       sin(theta)  cos(theta)];
//            global_coords = (Rot_Mat)*[x_local; y_local]+[system_x; system_y];
//        end

    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
    //   for the fact that the map's y-axis actually points downwards.)
    //   http://planning.cs.uiuc.edu/node99.html
    x_land = x_land * cos(yaw_0) - y_land * sin(yaw_0) + x_0;
    y_land = x_land * sin(yaw_0) + y_land * cos(yaw_0) + y_0;
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


//  In the updateWeights function for each particle we have to:
//    1) create a vector of predicted landmarks filtered according to the sensor range
//    2) convert local observations to global observations relative to this particle
//    3) associate each observation to a landmark in that filtered vector of predicted landmarks
//    4) compute the weight of each particle given the error


    for (size_t iter = 0; iter < particles.size(); iter++){
        auto& particle = particles[iter];
        auto& weight = weights[iter];
        weight = 1.0;
//        std::cout << "particle.id = " << particle.id << "(" << particle.x << "," << particle.y << ")" << std::endl;

        /// 1) use sensor_range to filter the predicted landmark observations
        std::vector<LandmarkObs> predicted_landmarkObs;

        for (auto const& landmark_s : map_landmarks.landmark_list){
            double distance = sqrt(pow((particle.x-landmark_s.x_f),2) + pow((particle.y-landmark_s.y_f),2));
            if (distance <= sensor_range){
                // need to convert single_landmark_s to landmarkObs
                LandmarkObs observableLandmark;
                observableLandmark.id = landmark_s.id_i;
                observableLandmark.x = landmark_s.x_f;
                observableLandmark.y = landmark_s.y_f;
                predicted_landmarkObs.push_back(observableLandmark);
            }
        }

        /// 2) convert local observations to global observations relative to this particle
        std::vector<LandmarkObs> global_observations;
        for (auto observation : observations){ // creates a copy of the observation
//            cout << "local = " << observation.x << "," << observation.y << endl;
            LocalToGlobal(observation.x, observation.y, particle.x, particle.y, particle.theta);
            global_observations.push_back(observation);
//            cout << "global = " << observation.x << "," << observation.y << endl <<endl;
        }

        /// 3) associate landmarks
        dataAssociation(predicted_landmarkObs, global_observations);

        /// 4) compute the weight of each particle
        // observations are in local coordinate system
        // translate observations to global coordinate system
        for (auto const& global_observation : global_observations){

            // search for this id in the predicted_landmarkObs list:
            for (auto const& landmark : predicted_landmarkObs){
                if (landmark.id == global_observation.id){
                    double x = global_observation.x - landmark.x;
                    double y = global_observation.y - landmark.y;

                    // Update the weights of each particle using a multi-variate Gaussian distribution
                    double std_x = std_landmark[0];
                    double std_y = std_landmark[1];
                    double var_x = std_x * std_x;
                    double var_y = std_y * std_y;
                    weight = weight * (exp (-0.5 * (x*x/var_x + y*y/var_y)) ) / (2 * M_PI * std_x * std_y);
                    // using the bivariate formula instead
//                    weight = weight * 1.0/(2.0 * M_PI * std_x * std_y) * exp(- x * x / (2 * var_x) - y * y / (2 * var_y) );

                    particle.weight = weight;
                    break; // stop looping through the map landmarks, as we've found the one
                }
            }
        }
    }
//    cout << "+++++++++++++++  weights content AFTER ++++++++++++++++++++" << endl;
//    for (auto const& weight : weights){
//        cout << weight << endl;
//    }
}


/**
 * resample Resamples from the updated set of particles to form
 *   the new set of particles.
 */
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    std::vector<Particle> new_particles;
    default_random_engine gen;
    discrete_distribution<> distribution(weights.begin(), weights.end());

    for (int i = 0; i<num_particles; ++i) {
        int number = distribution(gen);
        new_particles.push_back(particles[number]);
    }

    particles = new_particles;
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
