#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/chassis.hpp"

pros::MotorGroup left_motors({-11, -12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({18, 19, 20}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges
pros::Motor intake(-4);
pros::Motor intake2(8);
pros::Motor intake3(-15);

// Define two independent pneumatic solenoids
pros::ADIDigitalOut solenoidD ('D');  // First solenoid on port D //one controls match loader
pros::ADIDigitalOut solenoidH('H');  // Second solenoid on port H // one controls top matchloader thingy

lemlib::Drivetrain drivetrain(
	&left_motors,  // pointer to left motors
	&right_motors, // pointer to right motors
	10.0,          // 10 inch track width
	4.0,            // 4 inch wheel diameter
	600.0,        // 600 RPM motors
	2.0          // horizontal drift of 2 (no traction wheels)
);

pros::Rotation rotation_sensor(9);
pros::Imu imu(3);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
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
    // Reset position to (0,0,0) at start
    chassis.setPose(0, 0, 0);
    
    if (isRedAlliance && isLeftSide) {
        // ---- RED LEFT ----
        intake.move_velocity(100);
        
        // Move to recorded coordinates
        chassis.moveToPoint(24.156, -1.453, 5);  // Move to exact position
        chassis.waitUntilDone();  // Wait until movement is complete
        chassis.turnToHeading(-287.840, 1000);  // Turn to exact recorded angle
        chassis.waitUntilDone();  // Wait until turn is complete

        chassis.moveToPoint(-2.738, 2.887, 5);  // Move to exact position
        chassis.waitUntilDone();  // Wait until movement is complete
        chassis.turnToHeading(-763.364, 1000);  // Turn to exact recorded angle
        chassis.waitUntilDone();  // Wait until turn is complete

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
    
    while (true) {
        // Press A button to run autonomous
        // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     autonomous();  // This will run your autonomous routine
        //     pros::delay(1000);  // Wait 1 second before allowing driver control again
        // }
        
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Apply deadband to ignore small joystick movements
        if (abs(dir) < 10) dir = 0;
        if (abs(turn) < 10) turn = 0;

        // quadratic for better control
        auto curve = [](double x) -> double {
            if (x == 0) return 0;
            return pow(x/10, 2) * (std::abs(x)/x);
        };

        // Scale inputs to velocity (-600 to 600 for blue cartridges)
        double vel_dir = curve(dir) * 6;
        double vel_turn = curve(turn) * 6;

        left_motors.move_velocity(vel_dir + vel_turn);
        right_motors.move_velocity(vel_dir - vel_turn);
        
        // Control intake with R1 (forward) and R2 (reverse) toggle
        static bool intake_forward = false;
        static bool intake_reverse = false;
        static bool last_r1_state = false;
        static bool last_r2_state = false;
        
        bool current_r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool current_r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        
        // Toggle forward on R1
        if (current_r1 && !last_r1_state) {
            intake_forward = !intake_forward;  // Toggle forward
            intake_reverse = false;  // Turn off reverse if it was on
        }
        last_r1_state = current_r1;
        
        // Toggle reverse on R2
        if (current_r2 && !last_r2_state) {
            intake_reverse = !intake_reverse;  // Toggle reverse
            intake_forward = false;  // Turn off forward if it was on
        }
        last_r2_state = current_r2;
        
        // Run intake based on which button was toggled
        if (intake_forward) {
            intake.move(600);     // Forward at full speed
            intake2.move(300);
            intake3.move(300);
        } else if (intake_reverse) {
            intake.move(-600);    // Reverse at full speed
            intake2.move(-300);
            intake3.move(-300);
        } else {
            intake.move(0);       // Stop
            intake2.move(0);
            intake3.move(0);
        }

        // // Control solenoid on port D with L1 button
        // static bool solenoidD_state = false;
        // static bool l1_last_state = false;
        // bool l1_current_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        
        // if (l1_current_state && !l1_last_state) {  // L1 just pressed
        //     solenoidD_state = !solenoidD_state;    // Toggle state
        //     solenoidD.set_value(solenoidD_state);  // Set solenoid D to new state
        // }
        // l1_last_state = l1_current_state;

        // // Control solenoid on port H with L2 button
        // static bool solenoidH_state = false;
        // static bool l2_last_state = false;
        // bool l2_current_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        
        // if (l2_current_state && !l2_last_state) {  // L2 just pressed
        //     solenoidH_state = !solenoidH_state;    // Toggle state
        //     solenoidH.set_value(solenoidH_state);  // Set solenoid H to new state
        // }
        // l2_last_state = l2_current_state;
        



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