/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Implemented by: Wael Farag
 *  on:             Dec 1, 2017
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    default_random_engine gen;

    // set Number of Particles
	num_particles = 50;  // Can work even with 25 particles

    // Define and Initializes GPS reading noises (standard deviations)
	double GPS_x_noise = std[0];
	double GPS_y_noise = std[1];
	double GPS_theta_noise = std[2];

	// Artificial Noise to be added to each initial particle position (Randomization)
	// Set standard deviations for Particle Initial Position: x, y, and theta
    double std_x = 10.0;    // assuming the GPS Positioning Error +- 10 meters
    double std_y = 10.0;    // assuming the GPS Positioning Error +- 10 meters
    double std_theta = 0.05;  // assuming initial orientation error around +- 6 Degrees

	// This line creates a normal (Gaussian) distribution for GPS readings: x, y and theta
	normal_distribution<double> dist_x(x, GPS_x_noise+std_x);
	normal_distribution<double> dist_y(y, GPS_y_noise+std_y);
	normal_distribution<double> dist_theta(theta, GPS_theta_noise+std_theta);

	#define MAX_No_OBSERVATIONS   15

    // Now, initialize each Particle Position & Weight
    for (int i = 0; i < num_particles; ++i)
        {
            particles.push_back(Particle());
            particles[i].id = i;
            particles[i].x = dist_x(gen);
            particles[i].y = dist_y(gen);
            particles[i].theta = dist_theta(gen);
            particles[i].weight = 1.0;
            particles[i].sense_x.assign (MAX_No_OBSERVATIONS,0.0);
            particles[i].sense_y.assign (MAX_No_OBSERVATIONS,0.0);

            // Print your samples to the terminal.
		    //cout << "Sample " << i + 1 << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << endl;
        }

    // Gather all Particle weights in one vector.
    weights.assign(num_particles, 1.0);

    is_initialized = true;
    return;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	#define EPSILON 1e-4  // Very small number

	// Define some variables that will be used in prediction equations
	// like exactly the variables in the text for better readability and maintainability
    double xo, xf, v, theta_dot, theta_o, yo, yf, theta_f;

    // Define the random engine generator
    default_random_engine gen;

    for (int i = 0; i < num_particles; i++)
        {
            // Assign the defined variable to the program variables
            // Using the same variable names in the text for better readability
            xo        = particles[i].x;
            v         = velocity;
            theta_dot = yaw_rate;
            theta_o   = particles[i].theta;
            yo        = particles[i].y;

            // Implement the prediction 3 equations
            if (fabs(theta_dot) > EPSILON)
            {
                // xf = xo + (v/theta_dot)*[sin(theta_o+theta_dot*delta_t)-sin(theta_o)]
                xf = xo + (v/theta_dot)*(sin(theta_o+(theta_dot*delta_t)) - sin(theta_o));

                // yf = yo + (v/theta_dot)*[cos(theta_o) - cos(theta_o+theta_dot*delta_t)]
                yf = yo + (v/theta_dot)*(cos(theta_o) - cos(theta_o+(theta_dot*delta_t)));

            } else
            {
                // In case "theta_dot" almost Zero
                xf = xo + v*delta_t*cos(theta_o);
                yf = yo + v*delta_t*sin(theta_o);
            }

            // theta_f = theta_o + theta_dot * delta_t
            theta_f = theta_o + (theta_dot * delta_t);

            // Artificial Noise to be added to each Particle Position
            // Represents the Model Uncertainty
	        // Set standard deviations for Particle Calculated Position: xf, yf, and theta_f
            double std_xf      = std_pos[0];    // 0.2
            double std_yf      = std_pos[1];    // 0.2
            double std_theta_f = std_pos[2];    // 0.1

	        // This line creates a normal (Gaussian) distribution for Final readings: xf, yf and theta_f
	        normal_distribution<double> dist_xf(xf, std_xf);
	        normal_distribution<double> dist_yf(yf, std_yf);
	        normal_distribution<double> dist_theta_f(theta_f, std_theta_f);

            // Update Particles Positions Predictions
            particles[i].x     = dist_xf(gen);
            particles[i].y     = dist_yf(gen);
            particles[i].theta = dist_theta_f(gen);

            // Print your samples to the terminal.
		    //cout << "Sample " << i + 1 << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << endl;
        }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

    // Not Used: The Association is done within the "UpdateWeights" Function for simplicity
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    ///////////////////////////////////////////////////////////////

    // Define a counter to count the TimeTicks (or Iterations) mainly is used for debugging
    static unsigned int counter=0;

    // Get the sizes of observations and landmarks
    unsigned int observations_size  = observations.size();
    unsigned int map_landmarks_size = map_landmarks.landmark_list.size();

    //cout << "OBS: " << observations_size << " LM : " << map_landmarks_size << endl;


    const double Landmark_Sigma_x = std_landmark[0];
    const double Landmark_Sigma_y = std_landmark[1];

    const double Two_Landmark_Sigma_x_squared = 2.0 * Landmark_Sigma_x * Landmark_Sigma_x;
    const double Two_Landmark_Sigma_y_squared = 2.0 * Landmark_Sigma_y * Landmark_Sigma_y;

    // calculate the normalization term of the Gauss Distribution
    const double normalization_term = (1.0/(2.0 * M_PI * Landmark_Sigma_x * Landmark_Sigma_y));

    double x_obs, y_obs, mu_x, mu_y;

    // First: Clean up the previous associations and the mapped coordinates
    for (unsigned int i=0; i < num_particles; i++)
    {
        particles[i].associations.clear();
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();
    }


    ///***************************** Coordinates Transformations **************************

    for (unsigned int i=0; i < num_particles; i++)
    {
        // Define some variables that will be used in the transformation equations
	    // like exactly the variables in the text for better readability and maintainability
        double Xp    = particles[i].x;
        double Yp    = particles[i].y;
        double theta = particles[i].theta;

        // Initialize each Particle weight to 1
        particles[i].weight = 1.0;

        // Define a Data Structure to hold the information of the associated landmarks
        // (IDs and coordinates) for each Particle
        LandmarkObs associated_landmark;


        for (unsigned int j=0; j < observations_size; j++)
        {
            // Define some variables that will be used in the transformation equations
	        // like exactly the variables in the text for better readability and maintainability
            double Xc     = observations[j].x;
            double Yc     = observations[j].y;
            int    ID_obs = 1000;                   // A Big Number means no landmark associated

            // Xm = Xp + (Xc cos(theta)) - (Yc sin(theta))
            double Xm = Xp + (Xc * cos(theta)) - (Yc * sin(theta));
            // Ym = Yp + (Xc sin(theta)) + (Yc cos(theta))
            double Ym = Yp + (Xc * sin(theta)) + (Yc * cos(theta));

            // Update the sensed landmark coordinated for each particle
            //particles[i].sense_x.push_back(Xm);
            //particles[i].sense_y.push_back(Ym);
            particles[i].sense_x[j] = Xm;
            particles[i].sense_y[j] = Ym;

            ///***************************************** Data Association ****************************

            double SMALLEST_DIST  = sensor_range * sensor_range;     //1e6;   // any big number in initialization

            for (unsigned int k=0; k < map_landmarks_size; k++)
            {
               float X_landmark = map_landmarks.landmark_list[k].x_f;
               float Y_landmark = map_landmarks.landmark_list[k].y_f;
               int  ID_landmark = map_landmarks.landmark_list[k].id_i;

               // Calculate the distance (Squared) from each landmark within Map Coordinates
               double distance_value = (Xm - X_landmark) * (Xm - X_landmark)
                                       + (Ym - Y_landmark) * (Ym - Y_landmark);

               //cout << "LM" << k << "Dist:" << distance_value << endl;

               // Search for an associated landmark for each observation
               if (distance_value < SMALLEST_DIST)
               {
                  SMALLEST_DIST = distance_value;
                  //unsigned int obj_index = j;

                  // Assign the associated landmark ID and coordinates
                  ID_obs = ID_landmark;
                  //associated_landmarks.push_back(LandmarkObs());
                  associated_landmark.id = ID_landmark;
                  associated_landmark.x  = X_landmark;
                  associated_landmark.y  = Y_landmark;

               }
                //cout << "#"<< counter <<"  LM " <<  ID_obs <<":" << " Dist=" << distance_value << endl;

             }  // end: for (unsigned int k=0; k < map_landmarks_size; k++)

             // Associate Each Observation with its Closest Land Mark
             //particles[i].associations[j] = ID_obs;
             particles[i].associations.push_back(ID_obs);


             //if (i == 1) {cout << "LM "<< j << " ID: " << particles[i].associations[j] << "  ";}
             //cout << "LM "<< j << " ID: " << ID_obs << "  ";

             ///*********************************  Weights Update **************************************

             // Assign the defined variable to the program variables
             // Using the same variable names in the text for better readability
                x_obs  = particles[i].sense_x[j];
                y_obs  = particles[i].sense_y[j];
                mu_x   = associated_landmark.x;
                mu_y   = associated_landmark.y;

                //cout << "x_obs:" << x_obs << "  mu_x:" << mu_x << endl;
                //cout << "y_obs:" << y_obs << "  mu_y:" << mu_y << endl;

                // calculate the exponent of the Gauss Distribution
                double exponent = (((x_obs - mu_x)*(x_obs - mu_x))/(Two_Landmark_Sigma_x_squared))
                                + (((y_obs - mu_y)*(y_obs - mu_y))/(Two_Landmark_Sigma_y_squared));

                // calculate the weight of each particle using the normalization and exponent terms
                double weight = normalization_term * exp(-exponent);

                // Aggregate the Final weight for each Particle
                particles[i].weight = particles[i].weight * weight;

        }  // end: for (unsigned int j=0; j < observations_size; j++)

    //cout << endl;

      // Insert each Particle final weight in the weights vector
      // It will be used in the re-sampling process
      weights[i] = particles[i].weight;

      //cout << "weights: " << particles[i].weight << endl;

  } // end: for (unsigned int i=0; i< num_particles; i++)


  ///******************************************************************************************

  // Update Iterations/TimeTicks Counter
  counter++;
  //cout << "Count :" << counter << endl;

}   // end: void ParticleFilter::updateWeights



void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // Set of new updated Particles
	vector<Particle> Updated_particles;

	// Standard Code for setting up the discrete distribution
	random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> d(weights.begin(), weights.end());

    // Loop for re-sampling
    for (unsigned int i=0; i < num_particles; ++i)
    {
        Particle Resampled_Particle = particles[d(gen)];
        Updated_particles.push_back(Resampled_Particle);
    }

    particles = Updated_particles;

    //cout << "size: " << particles.size() << endl;
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
