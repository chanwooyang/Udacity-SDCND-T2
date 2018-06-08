/*
 * particle_filter.cpp
 *
 */
#include "particle_filter.h"

// random generator engine
default_random_engine gen;

double ParticleFilter::_Bivariate_Gaussian(double mu_x, double mu_y, double sigma_x, double sigma_y, double x, double y){

	double gauss_norm = 1.0/(2.0*M_PI*sigma_x*sigma_y);
	double exponent = (x - mu_x)*(x - mu_x)/(2.0*sigma_x*sigma_x) + (y - mu_y)*(y - mu_y)/(2.0*sigma_y*sigma_y);

	// // Debugging Purpose
	// cout <<gauss_norm*exp(-exponent)<<endl;
	return gauss_norm*exp(-exponent);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. 
	// Initialize all particles to first position (based on estimates of 
	//		x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set the number of particles
	num_particles = 100;

	// Resize weights vector
	weights.resize(num_particles);
	
	// x - Gaussian Distribution
	normal_distribution<double> x_gauss_dist(x,std[0]);
	// y - Gaussian Distribution
	normal_distribution<double> y_gauss_dist(y,std[1]);
	// theta - Gaussian Distribution
	normal_distribution<double> theta_gauss_dist(theta,std[2]);

	for (unsigned int i=0;i<num_particles;i++){
		Particle particle;
		particle.x = x_gauss_dist(gen);
		particle.y = y_gauss_dist(gen);
		particle.theta = theta_gauss_dist(gen);
		particle.weight = 1.0;

		particles.push_back(particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double dt, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.


	for (unsigned int i=0;i<num_particles;i++){

		double theta_o = particles[i].theta;

		if (fabs(yaw_rate) < 0.00001){
			particles[i].x += velocity * dt * cos(theta_o);
			particles[i].y += velocity * dt * sin(theta_o);
		} else {
			particles[i].x += (velocity/yaw_rate)*(sin(theta_o + yaw_rate*dt) - sin(theta_o)); 
			particles[i].y += (velocity/yaw_rate)*(cos(theta_o) - cos(theta_o + yaw_rate*dt));
		}
		particles[i].theta = theta_o + yaw_rate*dt;

		// x - Gaussian Distribution
		normal_distribution<double> x_gauss_dist(particles[i].x,std_pos[0]);
		// y - Gaussian Distribution
		normal_distribution<double> y_gauss_dist(particles[i].y,std_pos[1]);
		// theta - Gaussian Distribution
		normal_distribution<double> theta_gauss_dist(particles[i].theta,std_pos[2]);

		particles[i].x = x_gauss_dist(gen);
		particles[i].y = y_gauss_dist(gen);
		particles[i].theta = theta_gauss_dist(gen);

	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.

	const unsigned int numPredicted = predicted.size();
	const unsigned int numObservations = observations.size();

	for (unsigned int i=0;i<numObservations;i++){
		
		// Initialize
		double minDistance = numeric_limits<double>::max();
		int mapID = -1;
		
		for(unsigned int j=0;j<numPredicted;j++){
		
			double distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);

			if (distance < minDistance){
				minDistance = distance;
				mapID = predicted[j].id;
			}
		}
		// Data Association
		observations[i].id = mapID;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double sigma_landmark_x = std_landmark[0];
	double sigma_landmark_y = std_landmark[1];
	const double TOL = numeric_limits<double>::min();

	for (unsigned int i=0;i<num_particles;i++){
		
		vector<LandmarkObs> transf_obs;
		for (unsigned int j=0;j<observations.size();j++){

			// Observation Point Transformation (from Vehicle coord. to Map coord.)
			double map_x, map_y;
			map_x = cos(particles[i].theta)*observations[j].x - sin(particles[i].theta)*observations[j].y + particles[i].x;
			map_y = sin(particles[i].theta)*observations[j].x + cos(particles[i].theta)*observations[j].y + particles[i].y;
		
			transf_obs.push_back(LandmarkObs{observations[j].id,map_x,map_y});		
		}

		vector<LandmarkObs> predicted_landmarks;
		vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
		for (unsigned int j=0;j<landmarks.size();j++){
			
			// Shortlist landmarks within the sensor range
			double landmark_particle_dist = dist(particles[i].x,particles[i].y,landmarks[j].x_f,landmarks[j].y_f);
			if (landmark_particle_dist < sensor_range){
				predicted_landmarks.push_back(LandmarkObs{landmarks[j].id_i,landmarks[j].x_f,landmarks[j].y_f});
			}
		}

		this -> dataAssociation(predicted_landmarks, transf_obs);

		particles[i].weight = 1.0;
		double gaussian_value = 0.0;
		for (unsigned int j=0;j<transf_obs.size();j++){
			for (unsigned int k=0;k<predicted_landmarks.size();k++){
				if(predicted_landmarks[k].id == transf_obs[j].id){
					gaussian_value = this -> _Bivariate_Gaussian(transf_obs[j].x, transf_obs[j].y, sigma_landmark_x, sigma_landmark_y, predicted_landmarks[k].x, predicted_landmarks[k].y);
					if(gaussian_value == 0.0){
						particles[i].weight *= TOL;
					} else {
						particles[i].weight *= gaussian_value;
					}
				}
			}
		}
		// Update current weights
		for(unsigned int i=0;i<num_particles;i++){
			weights[i] = particles[i].weight;
		}
	}
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Resampled particles vector
	vector<Particle> resampled_particles; 

	// Generate a random initial index from an uniform distribution
	uniform_int_distribution<int> uniform_dist_index(0,num_particles-1);
	int index = uniform_dist_index(gen);

	// Max weight
	double max_weight = *max_element(weights.begin(),weights.end());

	// Resampling Wheel
	uniform_real_distribution<double> uniform_dist_weight(0.0,max_weight);

	double beta = 0.0;

	for (unsigned int i=0;i<num_particles;i++){
		beta += uniform_dist_weight(gen) * 2.0 * max_weight;
		while(beta > weights[index]){
			beta -= weights[index];
			index = (index+1)%num_particles;
		}
		resampled_particles.push_back(particles[index]);
	}

	particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
