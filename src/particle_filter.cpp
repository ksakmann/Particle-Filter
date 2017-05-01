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


std::default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	//   Sets the number of particles. Initializes all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1 (see main)
	// 	 Adds random Gaussian noise to each particle.
	
	num_particles = 300;
	weights.resize(num_particles, 1.0);

	// This line creates a normal (Gaussian) distribution for x
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {
		
		double sample_x, sample_y, sample_theta;
		
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);

		Particle particle;
		particle.id = i;		
		particle.x = sample_x;
		particle.y = sample_y;
		particle.theta = sample_theta;
		particle.weight = 1.0;

		particles.push_back(particle);

	}

	is_initialized = true;

	return;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	for (auto& p : particles){

		if (fabs(yaw_rate) > 0.001) {
			p.x += velocity/yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
			p.y += velocity/yaw_rate * (cos(p.theta)  - cos(p.theta + yaw_rate * delta_t));
			p.theta  += yaw_rate * delta_t;
		} 
		else {
			p.x += velocity * delta_t * cos(p.theta);
			p.y += velocity * delta_t * sin(p.theta);
		}

		std::normal_distribution<double> dist_x(p.x, std_pos[0]);
		std::normal_distribution<double> dist_y(p.y, std_pos[1]);
		std::normal_distribution<double> dist_theta(p.theta, std_pos[2]);

		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		
	}

	return;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	// implement this method and use it as a helper during the updateWeights phase.

}


std::vector<LandmarkObs> associate_data(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark.
	
	std::vector<LandmarkObs> associated_landmarks;
	LandmarkObs closest;
	
	for (auto obs: observations){
		
		double shortest = 1E10; // some number larger than any possible measurement 

		for (auto pred: predicted){
			double distance = dist(obs.x,obs.y,pred.x,pred.y);
			if (distance < shortest) {
				shortest = distance;
				closest = pred;
			}
		}

		associated_landmarks.push_back(closest);
	}
	
	return associated_landmarks;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
	//  Updates the weights of each particle using a mult-variate Gaussian distribution. 

	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1];

	for(int i=0; i < particles.size(); ++i) {

		// collect all landmarks within sensor range of the current particle in a vector predicted.
	    Particle p = particles[i];

	    // transform observations from the particle coordinate system to the MAP system
		std::vector<LandmarkObs> transformed_observations;
		for (auto observation: observations){

			LandmarkObs transformed_observation;
			transformed_observation.x = p.x + observation.x * cos(p.theta) - observation.y * sin(p.theta);
			transformed_observation.y = p.y + observation.x * sin(p.theta) + observation.y * cos(p.theta);
			transformed_observation.id = observation.id;

			transformed_observations.push_back(transformed_observation);

		}

		// get all landmarks that are within sight of the particle
		std::vector<LandmarkObs> predicted;
		for (auto landmark: map_landmarks.landmark_list){

			double distance = dist(p.x,p.y,landmark.x_f,landmark.y_f);
			if (distance < sensor_range) {
				LandmarkObs one_landmark;
				one_landmark.id = landmark.id_i;
				one_landmark.x = landmark.x_f;
				one_landmark.y = landmark.y_f;
				predicted.push_back(one_landmark);		
			}
		}

		// then associate the nearest landmark to every observation of the particle 
		std::vector<LandmarkObs> associated_landmarks;
		associated_landmarks = associate_data(predicted, transformed_observations);

		double probability = 1;		
		for (int j=0; j < associated_landmarks.size(); ++j){

			double dx = transformed_observations.at(j).x - associated_landmarks.at(j).x;
			double dy = transformed_observations.at(j).y - associated_landmarks.at(j).y;
			probability *= 1.0/(2*M_PI*sigma_x*sigma_y) * exp(-dx*dx / (2*sigma_x*sigma_x))* exp(-dy*dy / (2*sigma_y*sigma_y));			
		}

		p.weight = probability;
		weights[i] = probability;

	}

	return;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::discrete_distribution<int> d(weights.begin(), weights.end());
	std::vector<Particle> weighted_sample(num_particles);

	for(int i = 0; i < num_particles; ++i){
		int j = d(gen);
		weighted_sample.at(i) = particles.at(j);
	}

	particles = weighted_sample;

	return;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();

	return;
}
