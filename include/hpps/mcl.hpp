#ifndef MCL_HPP
#define MCL_HPP

#include "api.h"
#include <vector>
#include <random>
#include <tuple>
#include <cmath>
// Remove lemlib dependency
// #include "lemlib/pose.hpp"

// Monte Carlo Localization configuration
const int NUM_PARTICLES = 500;

// Field dimensions for VEX field (in inches) - CENTERED COORDINATE SYSTEM
// (0,0) is the center of the field, (72,72) is northeast corner
const double FIELD_MIN_X = -72.0;  // West wall
const double FIELD_MAX_X = 72.0;   // East wall  
const double FIELD_MIN_Y = -72.0;  // South wall
const double FIELD_MAX_Y = 72.0;   // North wall

// Parameters for MCL algorithm
const double INITIAL_POSE_STD_DEV = 0.5;   // inches
const double INITIAL_THETA_STD_DEV = 0.1;  // radians

const double MOTION_NOISE_X_STD_DEV = 0.1;     // inches (reduced from 0.5)
const double MOTION_NOISE_Y_STD_DEV = 0.1;     // inches (reduced from 0.5)
const double MOTION_NOISE_THETA_STD_DEV = 0.02; // radians (reduced from 0.05)

// Measurement model parameters
const double MEASUREMENT_NOISE_STD_DEV = 4.0;  // inches (increased from 2.0 for less sensitivity)
const double RESAMPLE_THRESHOLD = 0.5;  // Only resample when effective particle count drops below this fraction

// Distance sensor offsets from robot center (inches)
// Positive Y is front, positive X is right
// Modify these values to match your robot's sensor placement
// IMPORTANT: Set these to your actual robot dimensions!
const double FRONT_SENSOR_OFFSET_X = 0.0;  // X offset of front sensor
const double FRONT_SENSOR_OFFSET_Y = 3.0;  // Y offset of front sensor (reduced from 5.0)
const double RIGHT_SENSOR_OFFSET_X = 3.0;  // X offset of right sensor (reduced from 5.0)
const double RIGHT_SENSOR_OFFSET_Y = 0.0;  // Y offset of right sensor
const double BACK_SENSOR_OFFSET_X = 0.0;   // X offset of back sensor
const double BACK_SENSOR_OFFSET_Y = -3.0;  // Y offset of back sensor (reduced from -5.0)
const double LEFT_SENSOR_OFFSET_X = -3.0;  // X offset of left sensor (reduced from -5.0)
const double LEFT_SENSOR_OFFSET_Y = 0.0;   // Y offset of left sensor

// Helper functions
double normalizeAngle(double angle);
double clamp(double value, double min, double max);

// Bearing mode conversion functions
// Convert bearing (0° = North, clockwise) to math (0° = East, counterclockwise) for internal calculations
double bearingToMath(double bearing);
// Convert math (0° = East, counterclockwise) to bearing (0° = North, clockwise) for display
double mathToBearing(double math_angle);

// Our own Pose struct to replace lemlib::Pose
struct Pose {
    double x;       // X position in inches
    double y;       // Y position in inches
    double theta;   // Orientation in radians
    
    // Constructor
    Pose(double x_pos = 0.0, double y_pos = 0.0, double orientation = 0.0)
        : x(x_pos), y(y_pos), theta(orientation) {}
};

// Particle struct
struct Particle {
    double x_position;   // inches
    double y_position;   // inches
    double theta;        // radians
    double weight;       // normalized importance weight
};

class MonteCarloLocalization {
public:
    // Constructor
    MonteCarloLocalization(
        pros::Distance& front_dist,
        pros::Distance& right_dist,
        pros::Distance& back_dist,
        pros::Distance& left_dist,
        pros::v5::Rotation& y_odom,
        pros::v5::IMU& imu
    );

    // Initialize the particles
    void initialize(double x, double y, double theta);

    // Update the MCL state with odometry and sensor data
    Pose update();

    // Get the current estimated pose
    Pose getPose() const;

    // Debug info
    void printParticlesSummary() const;
    double getEffectiveParticleCount() const;

private:
    // Sensors
    pros::Distance& frontDist;
    pros::Distance& rightDist;
    pros::Distance& backDist;
    pros::Distance& leftDist;
    pros::v5::Rotation& yOdom;
    pros::v5::IMU& imu;

    // Particle set
    std::vector<Particle> particles;
    
    // Robot state estimate
    double estimatedX;
    double estimatedY;
    double estimatedTheta;
    
    // Previous odometry readings for computing deltas
    double prevYTicks;
    double prevYaw;
    
    // Effective particle count for adaptive resampling
    double effectiveParticleCount;
    
    // Random number generator
    std::mt19937 randomEngine;

    // Helper methods
    std::vector<double> getDistanceMeasurements();
    std::vector<double> simulateMeasurements(double x, double y, double theta);
    double calculateMeasurementLikelihood(const Particle& p, const std::vector<double>& actualMeasurements);
    double calculateEffectiveParticleCount(const std::vector<Particle>& particles);
    void applyMotionModel(Particle& p, double linearDelta, double deltaTheta,
                         std::normal_distribution<double>& noiseX, 
                         std::normal_distribution<double>& noiseY,
                         std::normal_distribution<double>& noiseTheta);
    void resampleParticles();
    void updateStateEstimate();
};

#endif // MCL_HPP 