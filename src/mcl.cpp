#include "hpps/mcl.hpp"
#include "api.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

// Convert rotation sensor ticks to inches
static inline double ticksToInches(double ticks, double wheel_diameter = 2.75) {
    // For a typical VEX Rotation Sensor (36:1 gear ratio)
    const double TICKS_PER_REV = 36000.0;
    return (ticks / TICKS_PER_REV) * (M_PI * wheel_diameter);
}


double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Convert bearing (0° = North, clockwise) to math (0° = East, counterclockwise)
double bearingToMath(double bearing) {
    return normalizeAngle(M_PI/2 - bearing);
}

// Convert math (0° = East, counterclockwise) to bearing (0° = North, clockwise)
double mathToBearing(double math_angle) {
    return normalizeAngle(M_PI/2 - math_angle);
}

MonteCarloLocalization::MonteCarloLocalization(
    pros::Distance& front_dist,
    pros::Distance& right_dist,
    pros::Distance& back_dist,
    pros::Distance& left_dist,
    pros::v5::Rotation& y_odom,
    pros::v5::IMU& imu
) : frontDist(front_dist),
    rightDist(right_dist),
    backDist(back_dist),
    leftDist(left_dist),
    yOdom(y_odom),
    imu(imu),
    estimatedX(0),
    estimatedY(0),
    estimatedTheta(0),
    prevYTicks(0),
    prevYaw(0),
    effectiveParticleCount(NUM_PARTICLES) {
    
    // Seed random number generator
    std::random_device rd;
    randomEngine.seed(rd());
    
    // Initialize particles vector
    particles.resize(NUM_PARTICLES);
}

void MonteCarloLocalization::initialize(double x, double y, double theta_bearing) {
    // Set initial state estimate
    estimatedX = x;
    estimatedY = y;
    // Convert bearing angle to math angle for internal use
    estimatedTheta = bearingToMath(theta_bearing);
    
    // Reset odometry readings
    prevYTicks = yOdom.get_position();
    prevYaw = imu.get_heading();  // Store IMU reading in degrees (math convention)
    
    // Initialize particles with Gaussian noise around known position
    std::normal_distribution<double> dist_x(x, INITIAL_POSE_STD_DEV);
    std::normal_distribution<double> dist_y(y, INITIAL_POSE_STD_DEV);
    // Use math angle for internal calculations
    std::normal_distribution<double> dist_theta(estimatedTheta, INITIAL_THETA_STD_DEV);
    
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].x_position = clamp(dist_x(randomEngine), FIELD_MIN_X, FIELD_MAX_X);
        particles[i].y_position = clamp(dist_y(randomEngine), FIELD_MIN_Y, FIELD_MAX_Y);
        particles[i].theta = normalizeAngle(dist_theta(randomEngine));
        particles[i].weight = 1.0 / NUM_PARTICLES; // Uniform initial weights
    }
}

std::vector<double> MonteCarloLocalization::getDistanceMeasurements() {
    std::vector<double> measurements(4);
    
    // Get distance readings in mm and convert to inches
    // Filter out invalid readings (too far/too close)
    double frontMm = frontDist.get();
    double rightMm = rightDist.get();
    double backMm = backDist.get();
    double leftMm = leftDist.get();
    
    // Convert mm to inches (25.4mm = 1 inch)
    const double MM_TO_INCHES = 1.0 / 25.4;
    
    // Handle invalid readings (use max range if sensor reads PROS_ERR)
    const double MAX_RANGE_INCHES = 60.0; // VEX distance sensor max reliable range
    
    measurements[0] = (frontMm == PROS_ERR) ? MAX_RANGE_INCHES : frontMm * MM_TO_INCHES;
    measurements[1] = (rightMm == PROS_ERR) ? MAX_RANGE_INCHES : rightMm * MM_TO_INCHES;
    measurements[2] = (backMm == PROS_ERR) ? MAX_RANGE_INCHES : backMm * MM_TO_INCHES;
    measurements[3] = (leftMm == PROS_ERR) ? MAX_RANGE_INCHES : leftMm * MM_TO_INCHES;
    
    return measurements;
}

std::vector<double> MonteCarloLocalization::simulateMeasurements(double x, double y, double theta) {
    std::vector<double> expectedMeasurements(4);
    
    const double MAX_RANGE_INCHES = 60.0;
    
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    double frontSensorX = x + FRONT_SENSOR_OFFSET_X * cos_theta - FRONT_SENSOR_OFFSET_Y * sin_theta;
    double frontSensorY = y + FRONT_SENSOR_OFFSET_X * sin_theta + FRONT_SENSOR_OFFSET_Y * cos_theta;
    

    double rightSensorX = x + RIGHT_SENSOR_OFFSET_X * cos_theta - RIGHT_SENSOR_OFFSET_Y * sin_theta;
    double rightSensorY = y + RIGHT_SENSOR_OFFSET_X * sin_theta + RIGHT_SENSOR_OFFSET_Y * cos_theta;
    
    double backSensorX = x + BACK_SENSOR_OFFSET_X * cos_theta - BACK_SENSOR_OFFSET_Y * sin_theta;
    double backSensorY = y + BACK_SENSOR_OFFSET_X * sin_theta + BACK_SENSOR_OFFSET_Y * cos_theta;
    
    double leftSensorX = x + LEFT_SENSOR_OFFSET_X * cos_theta - LEFT_SENSOR_OFFSET_Y * sin_theta;
    double leftSensorY = y + LEFT_SENSOR_OFFSET_X * sin_theta + LEFT_SENSOR_OFFSET_Y * cos_theta;
    
    // Calculate sensor directions (in math convention, then we'll raycast to walls)
    double frontSensorDir = theta;
    // Right sensor points 90° clockwise from robot's forward direction (theta - π/2)  
    double rightSensorDir = theta - M_PI/2;
    // Back sensor points opposite to robot's forward direction (theta + π)
    double backSensorDir = theta + M_PI;
    // Left sensor points 90° counterclockwise from robot's forward direction (theta + π/2)
    double leftSensorDir = theta + M_PI/2;
    
    auto calculateDistanceToWall = [&](double sensorX, double sensorY, double direction) -> double {
        double cos_dir = cos(direction);
        double sin_dir = sin(direction);
        
        // Check distance to each wall and return the minimum
        double minDist = MAX_RANGE_INCHES;
        
        // Distance to right wall (x = FIELD_MAX_X)
        if (cos_dir > 1e-6) {  
            double dist = (FIELD_MAX_X - sensorX) / cos_dir;
            if (dist > 0) minDist = std::min(minDist, dist);
        }
        
        if (cos_dir < -1e-6) {  
            double dist = (FIELD_MIN_X - sensorX) / cos_dir;
            if (dist > 0) minDist = std::min(minDist, dist);
        }
        
        if (sin_dir > 1e-6) {  
            double dist = (FIELD_MAX_Y - sensorY) / sin_dir;
            if (dist > 0) minDist = std::min(minDist, dist);
        }
        
        if (sin_dir < -1e-6) { 
            double dist = (FIELD_MIN_Y - sensorY) / sin_dir;
            if (dist > 0) minDist = std::min(minDist, dist);
        }
        
        return minDist;
    };
    
    // Calculate distances for each sensor
    expectedMeasurements[0] = calculateDistanceToWall(frontSensorX, frontSensorY, frontSensorDir);
    expectedMeasurements[1] = calculateDistanceToWall(rightSensorX, rightSensorY, rightSensorDir);
    expectedMeasurements[2] = calculateDistanceToWall(backSensorX, backSensorY, backSensorDir);
    expectedMeasurements[3] = calculateDistanceToWall(leftSensorX, leftSensorY, leftSensorDir);
    
    return expectedMeasurements;
}

double MonteCarloLocalization::calculateMeasurementLikelihood(
    const Particle& p, const std::vector<double>& actualMeasurements) {
    
    // Get expected measurements for this particle's position
    std::vector<double> expectedMeasurements = simulateMeasurements(
        p.x_position, p.y_position, p.theta);
    
    // Calculate likelihood for each sensor
    double likelihood = 1.0;
    
    for (size_t i = 0; i < actualMeasurements.size(); i++) {
        // Skip invalid measurements (could be out of range)
        if (actualMeasurements[i] >= 60.0) continue;
        
        // Calculate difference between expected and actual measurement
        double diff = actualMeasurements[i] - expectedMeasurements[i];
        
        // Gaussian model for measurement likelihood
        double variance = MEASUREMENT_NOISE_STD_DEV * MEASUREMENT_NOISE_STD_DEV;
        double p_measurement = exp(-(diff * diff) / (2 * variance));
        
        // Multiply by individual measurement likelihood
        likelihood *= p_measurement;
    }
    
    // Apply penalty if particle is outside field boundaries
    if (p.x_position < FIELD_MIN_X || p.x_position > FIELD_MAX_X ||
        p.y_position < FIELD_MIN_Y || p.y_position > FIELD_MAX_Y) {
        likelihood *= 0.001; // Strong penalty for out-of-bounds
    }
    
    return likelihood;
}

double MonteCarloLocalization::calculateEffectiveParticleCount(const std::vector<Particle>& particles) {
    double sum_squared_weights = 0.0;
    for (const auto& p : particles) {
        sum_squared_weights += p.weight * p.weight;
    }
    return 1.0 / sum_squared_weights;
}

void MonteCarloLocalization::applyMotionModel(
    Particle& p, 
    double linearDelta,
    double deltaTheta,
    std::normal_distribution<double>& noiseX,
    std::normal_distribution<double>& noiseY,
    std::normal_distribution<double>& noiseTheta) {
    
    // With only Y odometry, we use a simpler motion model
    // linearDelta is the forward/backward motion in the robot's local frame
    
    // Convert linear motion to global frame components
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);
    
    // Apply forward motion in global coordinates
    double globalDeltaX = linearDelta * cos_theta;
    double globalDeltaY = linearDelta * sin_theta;
    
    // Apply motion with noise
    p.x_position += globalDeltaX + noiseX(randomEngine);
    p.y_position += globalDeltaY + noiseY(randomEngine);
    p.theta = normalizeAngle(p.theta + deltaTheta + noiseTheta(randomEngine));
    
    // Ensure particles stay within field boundaries
    p.x_position = clamp(p.x_position, FIELD_MIN_X, FIELD_MAX_X);
    p.y_position = clamp(p.y_position, FIELD_MIN_Y, FIELD_MAX_Y);
}

void MonteCarloLocalization::resampleParticles() {
    std::vector<Particle> resampledParticles(NUM_PARTICLES);
    
    // Low variance resampling algorithm
    double step = 1.0 / NUM_PARTICLES;
    double r = step * std::uniform_real_distribution<double>(0.0, 1.0)(randomEngine);
    
    double c = particles[0].weight;
    int i = 0;
    
    for (int m = 0; m < NUM_PARTICLES; m++) {
        double u = r + m * step;
        while (u > c && i < NUM_PARTICLES - 1) {
            i++;
            c += particles[i].weight;
        }
        
        // Copy selected particle and reset weight
        resampledParticles[m] = particles[i];
        resampledParticles[m].weight = 1.0 / NUM_PARTICLES;
    }
    
    particles = resampledParticles;
}

void MonteCarloLocalization::updateStateEstimate() {
    // Weighted average for position estimate
    double weightedSumX = 0.0;
    double weightedSumY = 0.0;
    double sinSum = 0.0;
    double cosSum = 0.0;
    
    // Compute weighted sums
    for (const auto& p : particles) {
        weightedSumX += p.x_position * p.weight;
        weightedSumY += p.y_position * p.weight;
        
        // Use circular statistics for orientation
        sinSum += sin(p.theta) * p.weight;
        cosSum += cos(p.theta) * p.weight;
    }
    
    // Update state estimate
    estimatedX = weightedSumX;
    estimatedY = weightedSumY;
    estimatedTheta = atan2(sinSum, cosSum);  // Circular mean
}

Pose MonteCarloLocalization::update() {
    // Get current odometry readings
    double currentYTicks = yOdom.get_position();
    double currentYawDegrees = imu.get_heading();  // IMU returns math angles in degrees (0° = East)
    
    // Calculate deltas in robot's local frame
    double deltaYTicks = currentYTicks - prevYTicks;
    // Convert degrees to radians and calculate delta (both in math convention)
    double currentYawMath = currentYawDegrees * (M_PI / 180.0);
    double prevYawMath = prevYaw * (M_PI / 180.0);
    double deltaYaw = normalizeAngle(currentYawMath - prevYawMath);
    
    // Convert ticks to distance (inches)
    double linearDelta = ticksToInches(deltaYTicks);
    
    // DEBUG: Print motion deltas
    if (abs(linearDelta) > 0.1 || abs(deltaYaw) > 0.01) {
        pros::lcd::print(0, "Motion: %.2f in, %.1f deg", linearDelta, deltaYaw * 180.0/M_PI);
    }
    
    // Store current readings for next update
    prevYTicks = currentYTicks;
    prevYaw = currentYawDegrees;  // Store in degrees
    
    // Set up noise distributions
    std::normal_distribution<double> noiseX(0, MOTION_NOISE_X_STD_DEV);
    std::normal_distribution<double> noiseY(0, MOTION_NOISE_Y_STD_DEV);
    std::normal_distribution<double> noiseTheta(0, MOTION_NOISE_THETA_STD_DEV);
    
    // Apply motion model to all particles
    for (auto& p : particles) {
        applyMotionModel(p, linearDelta, deltaYaw, noiseX, noiseY, noiseTheta);
    }
    
    // Get current distance sensor measurements
    std::vector<double> measurements = getDistanceMeasurements();
    
    // DEBUG: Print sensor measurements
    pros::lcd::print(1, "Sensors F:%.1f R:%.1f B:%.1f L:%.1f", 
        measurements[0], measurements[1], measurements[2], measurements[3]);
    
    // DEBUG: Print expected measurements at current estimated position
    std::vector<double> expectedAtCurrent = simulateMeasurements(estimatedX, estimatedY, estimatedTheta);
    pros::lcd::print(2, "Expected F:%.1f R:%.1f B:%.1f L:%.1f", 
        expectedAtCurrent[0], expectedAtCurrent[1], expectedAtCurrent[2], expectedAtCurrent[3]);
    
    // Update weights based on sensor measurements
    double totalWeight = 0.0;
    for (auto& p : particles) {
        p.weight = calculateMeasurementLikelihood(p, measurements);
        totalWeight += p.weight;
    }
    
    // Normalize weights
    if (totalWeight > 0) {
        for (auto& p : particles) {
            p.weight /= totalWeight;
        }
    } else {
        // Handle unlikely case where all weights are zero
        for (auto& p : particles) {
            p.weight = 1.0 / NUM_PARTICLES;
        }
    }
    
    // Calculate effective particle count
    effectiveParticleCount = calculateEffectiveParticleCount(particles);
    
    // Resample if effective particle count is too low
    if (effectiveParticleCount < RESAMPLE_THRESHOLD * NUM_PARTICLES) {
        resampleParticles();
    }
    
    // Update state estimate
    updateStateEstimate();
    
    // Return current pose estimate with bearing angle
    return Pose(estimatedX, estimatedY, mathToBearing(estimatedTheta));
}

Pose MonteCarloLocalization::getPose() const {
    // Return pose with bearing angle for user
    return Pose(estimatedX, estimatedY, mathToBearing(estimatedTheta));
}

void MonteCarloLocalization::printParticlesSummary() const {
    // Print first 5 particles for debugging
    pros::lcd::print(0, "MCL Particles (first 5):");
    
    for (int i = 0; i < std::min(5, NUM_PARTICLES); ++i) {
        const auto& p = particles[i];
        // Convert to bearing angle for display
        pros::lcd::print(i+1, "P%d: (%.1f, %.1f, %.1f°) w=%.4f", 
            i, p.x_position, p.y_position, mathToBearing(p.theta) * 180.0/M_PI, p.weight);
    }
    
    pros::lcd::print(6, "Est Pose: (%.1f, %.1f, %.1f°)", //estimate
        estimatedX, estimatedY, mathToBearing(estimatedTheta) * 180.0/M_PI);
    
    pros::lcd::print(7, "Eff Particles: %.1f / %d", //effective particle count
        effectiveParticleCount, NUM_PARTICLES);
}

double MonteCarloLocalization::getEffectiveParticleCount() const {
    return effectiveParticleCount;
} 