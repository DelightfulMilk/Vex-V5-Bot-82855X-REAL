#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue);
pros::MotorGroup clawMotors({-6}, pros::MotorGearset::green);
pros::MotorGroup liftMotors({7, -8}, pros::MotorGearset::red);
pros::MotorGroup toggleandintakeMotors({9}, pros::MotorGearset::blue);

const int THROTTLE_DEADBAND = 3;
const int THROTTLE_MIN_OUT = 10;
const double THROTTLE_CURVE = 1.019;

const int TURN_DEADBAND = 3;
const int TURN_MIN_OUT = 10;
const double TURN_CURVE = 1.019;

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 13,
                              lemlib::Omniwheel::NEW_275,
                              360,
                              2
);

lemlib::ExpoDriveCurve throttle_curve(THROTTLE_DEADBAND, THROTTLE_MIN_OUT, THROTTLE_CURVE);
lemlib::ExpoDriveCurve steer_curve(TURN_DEADBAND, TURN_MIN_OUT, TURN_CURVE);

lemlib::ControllerSettings lateral_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);
