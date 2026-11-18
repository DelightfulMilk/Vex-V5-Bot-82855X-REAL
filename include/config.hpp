#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>


extern pros::Controller controller;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup clawMotors;
extern pros::MotorGroup liftMotors;
extern pros::MotorGroup toggleandintakeMotors;

extern lemlib::Drivetrain drivetrain;

extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::OdomSensors sensors;

extern lemlib::Chassis chassis;

extern const int    THROTTLE_DEADBAND;
extern const int    THROTTLE_MIN_OUT;
extern const double THROTTLE_CURVE;

extern const int    TURN_DEADBAND;
extern const int    TURN_MIN_OUT;
extern const double TURN_CURVE;