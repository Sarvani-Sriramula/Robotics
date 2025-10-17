#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/chassis.hpp"

pros::MotorGroup left_motors({11, 12, 13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({-18, -19, -20}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges
pros::Motor intake(1);
pros::Motor intake2(15);
pros::Motor intake3(16);

lemlib::Drivetrain drivetrain(
	&left_motors,  // pointer to left motors
	&right_motors, // pointer to right motors
	10.0,          // 10 inch track width
	4.0,            // 4 inch wheel diameter
	600.0,        // 600 RPM motors
	2.0          // horizontal drift of 2 (no traction wheels)
);

pros::Rotation rotation_sensor(2);
pros::Imu imu(3);

// Create a new optical shaft encoder on ports 'A' and 'B'
// this sensor is also reversed
pros::adi::Encoder verticalEncoder('A', 'B', true);
// create a new vertical tracking wheel
// it's using a new 2.75 inch wheel
// it's also 5 inches away from the tracking center. This tracking wheel is to the left
// of the tracking center, so we use a negative distance. If it was to the right of the
// tracking center, we would use a positive distance
lemlib::TrackingWheel verticalTrackingWheel(&verticalEncoder, lemlib::Omniwheel::NEW_275, -5);
// create a new optical shaft encoder on ports `C` and `D`
// this sensor is not reversed
pros::adi::Encoder horizontalEncoder('C', 'D', false);
// create a new horizontal tracking wheel
// it's using an old 3.25 inch wheel
// it's also 2 inches away from the tracking center. This tracking wheel is to the back
// of the tracking center, so we use a negative distance. If it was to the front of the
// tracking center, we would use a positive distance
// this wheel also has a 5:3 gear ratio
lemlib::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, lemlib::Omniwheel::OLD_325, -2, 5.0/3.0);

lemlib::OdomSensors sensors(&verticalTrackingWheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontalTrackingWheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// at the top of main.cpp or globals.cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(
    drivetrain,
	lateral_controller,  // pointer to angular PID controller settings
	angular_controller,  // pointer to drivetrain
    sensors     // pointer to odometry sensors (correct variable name)    
); 

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen   
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// void on_center_button() {
// 	static bool pressed = false;
// 	pressed = !pressed;
// 	if (pressed) {
// 		pros::lcd::set_text(2, "I was pressed!");
// 	} else {
// 		pros::lcd::clear_line(2);
// 	}
// }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

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
bool isRedAlliance = true;
bool isLeftSide = true;

void autonomous() {
    if (isRedAlliance && isLeftSide) {
        // ---- RED LEFT ----
        intake.move_velocity(100);
        chassis.moveToPoint(-24, 24, 5); // drive forward 1 tile
        chassis.turnToHeading(337, 1000); // turn to face loader (top center)
        // need code for holding intake until bot reached loader
        chassis.moveToPoint(-72, 45, 5); // drive toward loader
        // need code for holding intake until bot reached goal
        chassis.turnToHeading(-90, 1000); // turn to face goal (bottom left corner)
        chassis.moveToPoint(-24, 45, 5); // back into goal
        intake.move_velocity(0);

    } else if (!isRedAlliance && isLeftSide) {
        // ---- BLUE LEFT ----
        intake.move_velocity(100);
        chassis.moveToPoint(24, -24, 5);
        chassis.turnToHeading(337, 1000);
        // need code for holding intake until bot reached loader
        chassis.moveToPoint(72, -45, 5);
        // need code for holding intake until bot reached goal
        chassis.turnToHeading(-90, 1000);
        chassis.moveToPoint(24, -45, 5);
        intake.move_velocity(0);

    } else if (isRedAlliance && !isLeftSide) {
        // ---- RED RIGHT ---- 
        intake.move_velocity(100);
        chassis.moveToPoint(-24, -24, 5);
        chassis.turnToHeading(337, 1000);
        // need code for holding intake until bot reached loader
        chassis.moveToPoint(-72, -45, 5);
        // need code for holding intake until bot reached goal
        chassis.turnToHeading(-90, 1000);
        chassis.moveToPoint(-24, -45, 5);
        intake.move_velocity(0);

    } else {
        // ---- BLUE RIGHT ----
        intake.move_velocity(100);
        chassis.moveToPoint(24, 24, 5);
        chassis.turnToHeading(337, 1000);
        // need code for holding intake until bot reached loader
        chassis.moveToPoint(72, 45, 5);
        // need code for holding intake until bot reached goal
        chassis.turnToHeading(-90, 1000);
        chassis.moveToPoint(24, 45, 5);
        intake.move_velocity(0);
    }
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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    // int count = 0;
    
    while (true) {
        // Read joystick values
        // int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        //  // deadband

        // left_motors.move_velocity(dir + turn);
        // right_motors.move_velocity(dir - turn);
        intake.move_velocity(100);
        intake2.move_velocity(100);
        intake3.move_velocity(100);
        
        // if (count % 5 == 0) {
        //     pros::lcd::print(0, "Dir:%d Turn:%d X:%.1f Y:%.1f T:%.1f",
        //          dir, turn,
        //          chassis.getPose().x,
        //          chassis.getPose().y,
        //          chassis.getPose().theta);
        // }

        // count++;

        // Small delay for smooth loop
        pros::delay(20);
    }
}