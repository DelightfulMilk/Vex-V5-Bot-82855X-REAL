
#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include "config.hpp"

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
}

void opcontrol() {
    while(true) {
        int leftX = controller.get_analog(ANALOG_LEFT_X);
        int rightY = controller.get_analog(ANALOG_RIGHT_Y);
        chassis.arcade(rightY, leftX);
    }
}