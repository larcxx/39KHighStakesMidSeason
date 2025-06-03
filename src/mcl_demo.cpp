#include "main.h"
#include "hpps/mcl.hpp"
// #include "lemlib/pose.hpp"
#include <cmath>


// Initialize MCL instance - use the existing odometry sensors from globals.hpp
MonteCarloLocalization mcl(
    distFront,
    distRight,
    distBack,
    distLeft,
    yOdomPod,  // Use existing global variable
    i1         // Use existing IMU
);

// Task to run MCL continuously
void mclTask() {
    // Initialize MCL with starting position
    // For this example: starting position at (24, 24) inches with 0 degrees (North) orientation
    mcl.initialize(24.0, 24.0, 0.0);  // 0° = North in bearing mode
    
    // Print initial state
    mcl.printParticlesSummary();
    
    int updateCounter = 0;
    
    // Loop to continuously update MCL
    while (true) {
        // Update MCL and get current pose estimate
        Pose currentPose = mcl.update();
        
        // Only update LCD every 10 iterations to reduce flicker
        if (updateCounter % 10 == 0) {
            // Log current pose - angles now in bearing mode (0° = North)
            pros::lcd::print(3, "MCL: (%.1f, %.1f, %.1f°)", 
                currentPose.x, currentPose.y, currentPose.theta * 180.0/M_PI);
            
            // Show effective particle count as a measure of confidence
            pros::lcd::print(4, "Effective particles: %.1f / %d", 
                mcl.getEffectiveParticleCount(), NUM_PARTICLES);
        }
        
        updateCounter++;
        
        // Delay to avoid CPU overuse
        pros::delay(25); // 25ms update rate (40Hz)
    }
}

// Example function to set up and start MCL
void startMCL() {
    // Wait for IMU to calibrate
    i1.reset();  // Use the existing IMU from globals
    //pros::lcd::print(0, "Calibrating IMU...");
    
    while (i1.is_calibrating()) {
        pros::delay(10);
    }
    
    //pros::lcd::print(0, "IMU Calibrated!");
    pros::delay(500);
    
    // Reset rotation sensors
    yOdomPod.reset_position();
    
    // Start MCL task
    pros::Task mclTaskHandle(mclTask);
}

// Example usage in a main program
/*
void opcontrol() {
    // Start MCL when entering driver control
    startMCL();
    
    while (true) {
        // Your regular driver control code here
        
        // You can access the current estimated pose at any time:
        Pose currentPose = mcl.getPose();
        
        // Use the pose for autonomous routines or driver assists
        // ...
        
        pros::delay(20);
    }
}
*/

// Example of how to use MCL for autonomous navigation
void navigateTo(double targetX, double targetY) {
    // Get current pose
    Pose currentPose = mcl.getPose();
    
    // Calculate distance and angle to target
    double dx = targetX - currentPose.x;
    double dy = targetY - currentPose.y;
    double distance = sqrt(dx*dx + dy*dy);
    // Calculate target bearing angle (0° = North, clockwise positive)
    double targetBearing = atan2(dx, dy);  // Note: swapped dx/dy for bearing mode
    
    // Turn to face target
    double angleError = normalizeAngle(targetBearing - currentPose.theta);
    
    //pros::lcd::print(2, "Turning to %.1f degrees", targetBearing * 180.0/M_PI);
    
    // Example turn in place (would need to be implemented with your robot's drive system)
    // turnInPlace(angleError);
    
    // Drive forward to target
   // pros::lcd::print(2, "Driving %.1f inches", distance);
    
    // Example drive forward (would need to be implemented with your robot's drive system)
    // driveForward(distance);
}

// Autonomous routine example using MCL
void runAutonomousMCL() {
    // Start MCL at the beginning of autonomous
    startMCL();
    
    // Wait a moment for initial particles to converge
    pros::delay(500);
    
    // Navigate to various points on the field
    navigateTo(36.0, 36.0);
    pros::delay(500);  // Wait for robot to settle
    
    navigateTo(36.0, 72.0);
    pros::delay(500);
    
    navigateTo(72.0, 72.0);
    pros::delay(500);
    
    // Return to starting position
    navigateTo(24.0, 24.0);
}

// Diagnostic function to check sensor readings without MCL
void testSensorReadings() {
    pros::lcd::clear();
    while (true) {
        // Raw sensor readings
        double frontMm = distFront.get();
        double rightMm = distRight.get();
        double backMm = distBack.get();
        double leftMm = distLeft.get();
        
        // Convert to inches
        const double MM_TO_INCHES = 1.0 / 25.4;
        double frontIn = (frontMm == PROS_ERR) ? -1 : frontMm * MM_TO_INCHES;
        double rightIn = (rightMm == PROS_ERR) ? -1 : rightMm * MM_TO_INCHES;
        double backIn = (backMm == PROS_ERR) ? -1 : backMm * MM_TO_INCHES;
        double leftIn = (leftMm == PROS_ERR) ? -1 : leftMm * MM_TO_INCHES;
        
        pros::lcd::print(0, "Raw Sensors (inches):");
        pros::lcd::print(1, "F:%.1f R:%.1f B:%.1f L:%.1f", frontIn, rightIn, backIn, leftIn);
        pros::lcd::print(2, "IMU Heading: %.1f deg", i1.get_heading());
        pros::lcd::print(3, "Y Odom: %.1f ticks", yOdomPod.get_position());
        
        // Expected distances if robot is at (24,24) facing North
        pros::lcd::print(4, "Expected at (24,24) North:");
        pros::lcd::print(5, "F:48 R:48 B:96 L:96");
        
        pros::delay(100);
    }
} 