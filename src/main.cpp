#include "main.h"

/* Clawbot Practice
#define LEFT_WHEELS_TOP_PORT 13
#define LEFT_WHEELS_BOTTOM_PORT 14
#define RIGHT_WHEELS_TOP_PORT 17
#define RIGHT_WHEELS_BOTTOM_PORT 18
#define INTAKE_PORT 15
#define INTAKE_BUTTON DIGITAL_A
*/
#define UPPER_FLYWHEEL 1
#define INTAKE_WHEEL 10
#define LOWER_FLYWHEEL 15

// Backward motors are reversed relative to other motors
#define RIGHT_FORWARD_DRIVE 20
#define RIGHT_BACKWARD_DRIVE 19
#define RIGHT_UPPER_DRIVE 18
#define LEFT_FORWARD_DRIVE 11
#define LEFT_BACKWARD_DRIVE 12
#define LEFT_UPPER_DRIVE 13

// Wing ports
const char LEFT_WING_PORT = 'A'; // three wire
const char RIGHT_WING_PORT = 'B';

#define GYRO_PORT 16

double gyro_offset = 0;
pros::Imu gyro(GYRO_PORT);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);
// Drive motors
pros::Motor right_fwd_mtr(RIGHT_FORWARD_DRIVE);
pros::Motor right_upp_mtr(RIGHT_UPPER_DRIVE);
pros::Motor right_bwd_mtr(RIGHT_BACKWARD_DRIVE);
pros::Motor left_fwd_mtr(LEFT_FORWARD_DRIVE);
pros::Motor left_upp_mtr(LEFT_UPPER_DRIVE);
pros::Motor left_bwd_mtr(LEFT_BACKWARD_DRIVE);
// Flywheel and intake motor speed
pros::Motor upper_flywheel_mtr(UPPER_FLYWHEEL);
pros::Motor intake_mtr(INTAKE_WHEEL);
pros::Motor lower_flywheel_mtr(LOWER_FLYWHEEL);
// Piston control for wings
pros::ADIDigitalOut left_wing_piston(LEFT_WING_PORT);
pros::ADIDigitalOut right_wing_piston(RIGHT_WING_PORT);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// Methods to get angle from pros gyroscope
double getRawRotation() {
	double heading = gyro.get_heading();
	return (heading == PROS_ERR_F ? 0 : heading);
}

double getRotation() {
	return getRawRotation() - gyro_offset;
}

void resetRotation() {
	gyro_offset = getRawRotation();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	resetRotation();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	gyro.reset();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// Flywheel motor speed
	int32_t flywheel_speed = 10;
	bool indexer_piston_state = false;

	while (true) {
		// print gyro angle
		pros::lcd::set_text(3, "Angle: " + std::to_string(getRotation()));
		

		// computing joystick inputs for arcade drive
		int forward_pow = master.get_analog(ANALOG_LEFT_Y);
		int turn_pow = master.get_analog(ANALOG_RIGHT_X);
		int left_pow = forward_pow + turn_pow;
		int right_pow = forward_pow - turn_pow;

		// drive motor control (backward motors spin in opposite direction)
		right_fwd_mtr.move(right_pow);
		right_upp_mtr.move(-right_pow);
		right_bwd_mtr.move(right_pow);
		left_fwd_mtr.move(-left_pow);
		left_upp_mtr.move(left_pow);
		left_bwd_mtr.move(-left_pow);
		
		// intake motor control
		if (master.get_digital(DIGITAL_R1)){
			intake_mtr.move_velocity(600);
		} else if (master.get_digital(DIGITAL_R2)) {
			intake_mtr.move_velocity(-600);
		} else {
			intake_mtr.move_velocity(100);
		}
		/*
			have flywheel at move_velocity(rpm);
			if A -> rpm = 600
			if X -> rpm = 100
		*/

		// flywheel motor control
		if (master.get_digital_new_press(DIGITAL_A)) {
			flywheel_speed = -100;
		} else if (master.get_digital_new_press(DIGITAL_X)) {
			flywheel_speed = 10;
		}
		upper_flywheel_mtr.move_velocity(flywheel_speed);
		lower_flywheel_mtr.move_velocity(flywheel_speed);

		// wing control
		if (master.get_digital_new_press(DIGITAL_L2)) {
			left_wing_piston.set_value(true);
			right_wing_piston.set_value(true);
		} else if(master.get_digital_new_press(DIGITAL_L1)) {
			left_wing_piston.set_value(false);
			right_wing_piston.set_value(false);
		}

		// hook is a piston, not on robot currently
		pros::delay(20);
	}
}
