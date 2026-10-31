#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue);
pros::MotorGroup clawMotors ({5}, pros::MotorGearset::green);
pros::MotorGroup liftMotors ({7, -8}, pros::MotorGearset::red);
pros::MotorGroup toggleMotor ({9}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 13,
                              lemlib::Omniwheel::OLD_325,
                              180,
                              2
);

lemlib::ExpoDriveCurve throttle_curve(3,
                                      10,
                                      1.019
);

lemlib::ExpoDriveCurve steer_curve(3,
                                   10,
                                   1.019
);

lemlib::ControllerSettings lateral_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);

lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr
);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    clawMotors.tare_position();
    toggleMotor.tare_position();
    liftMotors.tare_position();
    clawMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    toggleMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    toggleMotor.move_absolute(120, 200);
    leftMotors.move_voltage(12000);
    rightMotors.move_voltage(12000);
    pros::delay(1000);
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
}

void opcontrol() {
    bool wasAPressed = false;
    bool isAbsoluteTargetSet = false;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int throttle = (leftY * 0.5);
        int turn = (rightX * 0.5);

        int leftPower = throttle + turn;
        int rightPower = throttle - turn;

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        double lift_motor_pos = liftMotors.get_position(0);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (lift_motor_pos < 1200) { 
                liftMotors.move_voltage(12000);
            } else {
                liftMotors.move_voltage(0);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (lift_motor_pos > 0) { 
                liftMotors.move_voltage(-12000);
            } else {
                liftMotors.move_voltage(0);
            }
        } else {
            liftMotors.move_voltage(0);
        }

        bool aPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool bPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (bPressed) {
            toggleMotor.move_voltage(12000);
            isAbsoluteTargetSet = false;
        } else {
            if (aPressed && !wasAPressed) {
                toggleMotor.move_absolute(120, 200);
                isAbsoluteTargetSet = true;
            }
            wasAPressed = aPressed;

            if (!isAbsoluteTargetSet) {
                toggleMotor.move_voltage(0);
            }
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            clawMotors.move_voltage(6000); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            clawMotors.move_voltage(-6000); 
        } else {
            clawMotors.move_voltage(0);
        }

        pros::delay(20);
    }
}