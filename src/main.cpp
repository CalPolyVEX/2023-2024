#include "main.h"

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

// return an angle between 0 and 360 degrees (clockwise increases degrees)
double getRotation() {
	return getRawRotation() - gyro_offset;
}

void resetRotation() {
	gyro_offset = getRawRotation();
}

// positive left and right pow drives robot forward
void moveDriveMotors(int leftPow, int rightPow) {
	right_fwd_mtr.move(rightPow);
	right_upp_mtr.move(-rightPow);
	right_bwd_mtr.move(rightPow);
	left_fwd_mtr.move(-leftPow);
	left_upp_mtr.move(leftPow);
	left_bwd_mtr.move(-leftPow);
}

void drive_straight(double dist, int maxPow = 50) { //implementing a simple p controller for driving straight
	//get internal motor encoders right motor
	double right_pos = right_fwd_mtr.get_position();

	double desired_val = (37.5 * dist) + right_pos;
	int pow;
	double p = 0.6;
	double error = desired_val - right_pos;

	while(abs(error) > 40) {
		pow = (int) (error*p);
		if (pow > maxPow) { // cap power at maxPow
			pow = (pow > 0) ? maxPow:-maxPow;
		}

		moveDriveMotors(pow, pow);
		right_pos = right_fwd_mtr.get_position();
		error = desired_val - right_pos;
		pros::lcd::set_text(0, "Error: " + std::to_string(error));
		pros::lcd::set_text(1, "Position: " + std::to_string(right_pos));
		pros::lcd::set_text(2, "Power: " + std::to_string(pow));
	}
	pros::lcd::set_text(0, "Error: " + std::to_string(error));
	moveDriveMotors(0, 0);
}

void drive_timed(int millis, int pow = 50) {
	uint32_t init_time = pros::c::millis();
	
	while (pros::c::millis() - init_time < millis) {
		moveDriveMotors(pow, pow);
	}

	moveDriveMotors(0, 0);
}

// adjust angle to between -180 and 180 degrees
double adjustAngle(double angle) {
	if (angle <= -180) {
		return angle + 360*(1+floor(-angle/360));
	} else if (angle > 180) {
		return angle - 360*(1+floor(angle/360));
	} else {
		return angle;
	}
}

void turn(double angle, int maxPow = 50) {
	// get gyro angle
	double gyro_angle = getRotation();

	double desired_val = angle - gyro_angle;
	int pow;
	double p = 0.8;
	double error = adjustAngle(desired_val - gyro_angle);

	while(abs(error) > 2) {
		pow = (int) (error*p);
		if (pow > 30){
			pow = (pow > 0) ? maxPow:-maxPow;
		}

		moveDriveMotors(pow, -pow); // turn robot right if pow is positive

		gyro_angle = getRotation();
		if (gyro_angle > 180) {
			gyro_angle -= 360;
		}
		error = adjustAngle(desired_val - gyro_angle);
		pros::lcd::set_text(0, "Error: " + std::to_string(error));
	}

	moveDriveMotors(0, 0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	gyro.reset();
	resetRotation();
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);

	right_fwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_upp_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_bwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_fwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_upp_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_bwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_fwd_mtr.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	left_fwd_mtr.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
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
	// Should not need any initialization specific to competition
	/*
	gyro.reset();

	right_fwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_upp_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_bwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_fwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_upp_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_bwd_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_fwd_mtr.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	left_fwd_mtr.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	*/
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
void autonomous() {
	//drive_straight(-5);
	//drive_straight(20);
	//turn(90);
	//drive_timed(1000);
	intake_mtr.move_velocity(600);
	upper_flywheel_mtr.move_velocity(10);
	lower_flywheel_mtr.move_velocity(10);
	drive_timed(1000, 75);
	pros::delay(1000);
	intake_mtr.move_velocity(100);
	drive_straight(-5);
	turn(105);
	drive_straight(10);
	intake_mtr.move_velocity(-600);
	pros::delay(2000);
	intake_mtr.move_velocity(0);
	upper_flywheel_mtr.move_velocity(0);
	lower_flywheel_mtr.move_velocity(0);
	drive_straight(10);
}

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
	int32_t flywheel_speed_upper = 10;
	int32_t flywheel_speed_lower = 10;
	int32_t upper_speed_test = -120;
	int32_t lower_speed_test = -500;
	uint32_t timeStamp = 0;

	while (true) {
		// print gyro angle and flywheel speeds
		pros::lcd::set_text(0, "Angle: " + std::to_string(getRotation()));
		pros::lcd::set_text(2, "Upper Speed: " + std::to_string(abs(upper_speed_test)));
		pros::lcd::set_text(3, "Lower Speed: " + std::to_string(abs(lower_speed_test)));
		pros::lcd::set_text(4, "Left position: " + std::to_string(left_fwd_mtr.get_position()));
		pros::lcd::set_text(5, "Right position: " + std::to_string(right_fwd_mtr.get_position()));

		// computing joystick inputs for arcade drive
		int forward_pow = master.get_analog(ANALOG_LEFT_Y);
		int turn_pow = master.get_analog(ANALOG_RIGHT_X);
		int left_pow = forward_pow + turn_pow;
		int right_pow = forward_pow - turn_pow;

		// drive motor control (backward motors spin in opposite direction)
		moveDriveMotors(left_pow, right_pow);

		// intake motor control
		if (master.get_digital(DIGITAL_R1)){
			intake_mtr.move_velocity(600);
		} else if (master.get_digital(DIGITAL_R2)) {
			intake_mtr.move_velocity(-600);
		} else {
			intake_mtr.move_velocity(100);
		}

		// flywheel motor control
		if (master.get_digital_new_press(DIGITAL_A)) {
			flywheel_speed_upper = upper_speed_test;
			flywheel_speed_lower = lower_speed_test;
		} else if (master.get_digital_new_press(DIGITAL_X)) {
			timeStamp = pros::c::millis();
			flywheel_speed_upper = 0;
			flywheel_speed_lower = 0;
		}

		// manual flywheel adjustment code
		if (master.get_digital_new_press(DIGITAL_UP) && upper_speed_test > -600) {
			upper_speed_test -= 10;
		} else if (master.get_digital_new_press(DIGITAL_DOWN) && upper_speed_test < 0) {
			upper_speed_test += 10;
		}
		if (master.get_digital_new_press(DIGITAL_RIGHT) && lower_speed_test > -600) {
			lower_speed_test -= 10;
		} else if (master.get_digital_new_press(DIGITAL_LEFT) && lower_speed_test < 0) {
			lower_speed_test += 10;
		}

		// set flywheel rest speed back to 10 after waiting for 3 seconds
		if (timeStamp != 0 && (pros::c::millis() - timeStamp) > 3000) {
			flywheel_speed_upper = 10;
			flywheel_speed_lower = 10;
			timeStamp = 0;
		}
		upper_flywheel_mtr.move_velocity(flywheel_speed_upper);
		lower_flywheel_mtr.move_velocity(flywheel_speed_lower);

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
