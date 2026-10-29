#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({1, 2}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue); 
pros::MotorGroup clawMotors ({5}, pros::MotorGearset::green);
pros::MotorGroup wristMotors ({6}, pros::MotorGearset::green);
pros::MotorGroup liftMotors ({7, -8}, pros::MotorGearset::red);
pros::MotorGroup intakeMotors ({9}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, 
 &rightMotors, 
13.5, // 13.5 inch track width
lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
360, // drivetrain rpm is 360
2 // trac wheels = 0, horizontal drift offset
);


// joystick deadzone code
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
10, // minimum output where drivetrain will move out of 127
1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
10, // minimum output where drivetrain will move out of 127
1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}



void disabled() {}

void competition_initialize() {}

void opcontrol() {
    // controller
    // loop to continuously update motors

    //sets to 0 degrees claw
    wristMotors.tare_position();
    clawMotors.tare_position();
    
    //claw variables
    bool isClawClosed = false;
    bool wasYPressed = false;


    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
		// lift up and down
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			liftMotors.move_voltage(12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftMotors.move_voltage(-12000);
            }
		else {
			liftMotors.move_voltage(0);
		}
            //intake movement
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			intakeMotors.move_voltage(12000);
		} else {
            intakeMotors.move_voltage(0);
        }
         //wrist movements
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            wristMotors.move_absolute(-450, 200); //90 degees down at 200rpm
        } else {
            wristMotors.move_absolute(0, 200); // return to zero degrees at 200rpm
        }
        // claw movements
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (!wasYPressed) { //boolean which checks if its closed or not
                isClawClosed = !isClawClosed; // flip boolean state
                wasYPressed = true;  // y is clicked 
            }
        } else {
            wasYPressed = false; // resets y clicked button 
        }

        // actually closes it
        if (isClawClosed) { //checks if claw is closed and if its not y closes it
            clawMotors.move_absolute(-180, 200); // (-180) to close claw
        } else {
            clawMotors.move_absolute(0, 200);    // 0 to open claw
        }
        }



        pros::delay(10);
           
    }
}