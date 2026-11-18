#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

// 1. Define motor groups once with proper port directions and gearsets
// Negative port numbers reverse the motor direction
pros::MotorGroup left_motors({1}, pros::MotorGearset::blue);   // 600 RPM cartridges
pros::MotorGroup right_motors({-18}, pros::MotorGearset::blue); // 600 RPM cartridges
pros::MotorGroup liftMotor({10}, pros::MotorGearset::green); // RPM cartridges

// 2. Drivetrain settings (matching the exact variable names above)
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9.75, // 9.75 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              200, // drivetrain rpm is 200
                              1 // horizontal drift is 2 (for now)
);

// 3. Create sensors
pros::Imu imu(16);
pros::Rotation rotation_sensor(20);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);


// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
}


void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftMotor.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftMotor.move_velocity(-200);
        } else {
            liftMotor.move_velocity(0);
        }
    }
}