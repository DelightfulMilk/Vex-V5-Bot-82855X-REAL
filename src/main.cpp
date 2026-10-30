#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({1, 2}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue); 
pros::MotorGroup clawMotors ({5}, pros::MotorGearset::green);
pros::MotorGroup wristMotors ({6}, pros::MotorGearset::green);
pros::MotorGroup liftMotors ({7, -8}, pros::MotorGearset::red);
pros::MotorGroup toggleMotor ({9}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, 
&rightMotors, 
13, // 13 inch track width
lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
360, // drivetrain rpm is 360
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
    nullptr  // inertial sensor
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
    wristMotors.tare_position();
    clawMotors.tare_position();
    
    //claw variables
    int isClawClosed = 0;
    bool wasYPressed = false;
    bool isIntakeOn = false;
    bool isTogglerPressed = false;
    bool wasAPressed = false;
    int toggler = 0; // to
    int togglerhold = 0; 

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        pros::lcd::print(1, "rightX: %d", rightX);
        pros::lcd::print(2, "leftY: %d", leftY);
        chassis.arcade(leftY, rightX);

		// lift up and down
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			liftMotors.move_voltage(-12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftMotors.move_voltage(12000);
            }
		else {
			liftMotors.move_voltage(0);
		}

        if (toggler == 0) {
             if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                 if (!wasAPressed) { 
                 toggler = toggler + 1; // 
                  pros::lcd::print(3, "toggler: %d", toggler);
                 wasAPressed = true;    // lock
                 }
            } else {
             wasAPressed = false;
         }
        }
        else if (toggler >= 1) {
            toggleMotor.move_absolute(120, 200); // move toggle to 120 degrees
            toggler = toggler - 1;
            pros::lcd::print(3, "toggler: %d", toggler);
        }
    
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            togglerhold = 1; 
            pros::lcd::print(4, "togglerhold: %d", togglerhold);
        } else {
            togglerhold = 0;
            pros::lcd::print(4, "togglerhold: %d", togglerhold);
        }

        if (togglerhold == 1) {
            toggleMotor.move_voltage(12000);
        } else if (togglerhold == 0) {
            toggleMotor.move_voltage(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            clawMotors.move_absolute(90, 200); // move claw to 90 degrees
            isClawClosed = 1;
            pros::lcd::print(5, "isClawClosed: %d", isClawClosed);
        } 
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            clawMotors.move_absolute(0, 200); // move claw to 0 degrees
            isClawClosed = 0;
            pros::lcd::print(5, "isClawClosed: %d", isClawClosed);
        }

        pros::delay(20);   
    }
}