#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue);
pros::MotorGroup clawMotors ({5}, pros::MotorGearset::green);
pros::MotorGroup liftMotors ({7, -8}, pros::MotorGearset::red);
pros::MotorGroup toggleMotor ({9}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 13, // 13 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              180, // drivetrain rpm is 180
                              2 // trac wheels = 0, horizontal drift offset
);

// joystick deadzone code
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                      10, // minimum output where drivetrain will move out of 127
                                      1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                   10, // minimum output where drivetrain will move out of 127
                                   1.019 // expo curve gain
);

lemlib::ControllerSettings lateral_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    nullptr // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // throttle input curve
                        &steer_curve // steer input curve
);

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis sensors
    clawMotors.tare_position();
    toggleMotor.tare_position();
    liftMotors.tare_position();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    toggleMotor.move_absolute(120, 200); // move toggle to 120 degrees
    leftMotors.move_voltage(12000); // move left motors forward
    rightMotors.move_voltage(12000); // move right motors forward
}

void opcontrol() {
    // controller
    // L1: Lift Up
    // L2: Lift Down
    // R1: Claw Open
    // R2: Claw Close
    // A: Toggler for a certain amount. When button is pressed, queue a value that is executed only when the other vaue id one

    // loop to continuously update motors
    //sets to 0 degrees claw

    int isClawClosed = 0;
    bool bToggleState = false;

    while (true) {
        int leftY = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // ADJUST DRIVETRAIN SPEED: Change the 0.5 below to adjust overall speed (e.g., 1.0 = full speed, 0.25 = quarter speed)
        int throttle = (leftY * 0.5);
        int turn = (rightX * 0.5);

        int leftPower = throttle + turn;
        int rightPower = throttle - turn;

        // ADJUST LEFT POWER change 0.90 to something else
        leftPower = (int)(leftPower * 0.90);

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        double lift_motor_pos = liftMotors.get_position(0);

        // lift up and down
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            // ADJUST LIFT HEIGHT MAX: Change 350 to allow the lift to raise higher or stop lower
            if (lift_motor_pos < 350) { 
                liftMotors.move_voltage(12000);
            } else {
                liftMotors.move_voltage(0);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // ADJUST LIFT HEIGHT MIN: Change 0 to adjust the lowest point the lift can travel down to
            if (lift_motor_pos > 0) { 
                liftMotors.move_voltage(-12000);
            } else {
                liftMotors.move_voltage(0);
            }
        } else {
            liftMotors.move_voltage(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            liftMotors.move_voltage(0);
            toggleMotor.move_voltage(12000);
        } else {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                bToggleState = !bToggleState;
                if (bToggleState) {
                    // ADJUST TOGGLE POSITION (FORWARD): Change 120 (degrees) and 200 (speed) to adjust the forward target
                    toggleMotor.move_absolute(120, 200);
                } else {
                    // ADJUST TOGGLE POSITION (REVERSE): Change 0 (degrees) and 200 (speed) to adjust the reverse target
                    toggleMotor.move_absolute(0, 200);
                }
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            // ADJUST CLAW CLOSE POSITION: Change 90 (degrees) and 200 (speed) to alter clamping range/speed
            clawMotors.move_absolute(90, 200); 
            isClawClosed = 1;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            // ADJUST CLAW OPEN POSITION: Change 0 (degrees) and 200 (speed) to alter opening range/speed
            clawMotors.move_absolute(0, 200); 
            isClawClosed = 0;
        }

        pros::delay(20);
    }
}
