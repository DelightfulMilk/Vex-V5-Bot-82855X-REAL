#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue);
pros::MotorGroup clawMotors({-6}, pros::MotorGearset::green);
pros::MotorGroup liftMotors({7, -8}, pros::MotorGearset::red);
pros::MotorGroup toggleMotor({9}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 13,
                              lemlib::Omniwheel::OLD_325,
                              45,
                              2
);

lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

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

const int    THROTTLE_DEADBAND = 3;
const int    THROTTLE_MIN_OUT  = 10;
const double THROTTLE_CURVE    = 1.019;

const int    TURN_DEADBAND = 3;
const int    TURN_MIN_OUT  = 10;
const double TURN_CURVE    = 1.019;

int expoCurve(int input, int deadband, int minOutput, double curve) {
    if (std::abs(input) <= deadband) return 0;

    int sign = (input > 0) ? 1 : -1;
    double x = std::abs(input);

    double curved = std::pow(curve, x - 127) * x;
    double output = minOutput + (127.0 - minOutput) * curved / 127.0;

    return sign * (int)output;
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    clawMotors.tare_position_all();
    toggleMotor.tare_position_all();
    liftMotors.tare_position_all();
    clawMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    toggleMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
}

void opcontrol() {
    bool wasAPressed = false;
    bool isAbsoluteTargetSet = false;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int throttle = expoCurve(leftY, THROTTLE_DEADBAND, THROTTLE_MIN_OUT, THROTTLE_CURVE) * 0.5;
        int turn     = expoCurve(rightX, TURN_DEADBAND, TURN_MIN_OUT, TURN_CURVE) * 0.5;

        int leftPower = throttle + turn;
        int rightPower = (throttle - turn) * 1.8;

        if (rightPower > 127) rightPower = 127;
        if (rightPower < -127) rightPower = -127;

        leftMotors.move(leftPower * 0.75);
        rightMotors.move(rightPower * 0.75);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftMotors.move_voltage(4500);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftMotors.move_voltage(-4500);
        } else {
            liftMotors.brake();
        }

        bool aPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool bPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (bPressed) {
            toggleMotor.move_voltage(-4500);
            isAbsoluteTargetSet = false;
        } else {
            if (aPressed && !wasAPressed) {
                toggleMotor.move_absolute(120, 200);
                isAbsoluteTargetSet = true;
            }
            if (!isAbsoluteTargetSet) {
                toggleMotor.brake();
            }
        }
        wasAPressed = aPressed;

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            clawMotors.move_voltage(9000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            clawMotors.move_voltage(-9000);
        } else {
            clawMotors.brake();
        }

        pros::delay(20);
    }
}