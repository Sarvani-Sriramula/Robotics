#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-11, -12, -13},     // Left Chassis Ports (negative port will reverse it!)
    {18, 19, 20},  // Right Chassis Ports (negative port will reverse it!)

    17,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    600);   // Wheel RPM = cartridge * (motor gear / wheel gear)

pros::Motor intake(9);
pros::Motor intake2(-1);
pros::Motor intake3(10);

pros::ADIDigitalOut match ('A');
pros::ADIDigitalOut wings ('B');

pros::Optical color_sensor(3);  // Port 3



// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void telemetry(){
    while (true) {
        pros::delay(500);
        pros::lcd::print(0, "X: %f", chassis.odom_x_get());
        pros::lcd::print(1, "Y: %f", chassis.odom_y_get());
        pros::lcd::print(2, "Theta: %f", chassis.odom_theta_get());
        pros::lcd::print(3, "IMU Heading: %f", chassis.drive_imu_get());
        pros::lcd::print(4, "Hue: %f", color_sensor.get_hue());
        pros::delay(20);
    }
}

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  pros::Task screen_task(telemetry);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
  // . . .
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // Set starting position
  
  if (isRedAlliance && isLeftSide) {
    // intake.move_velocity(127);
    // chassis.pid_odom_set(-18.535_in, 15.627_in, -50_deg, 1500, {.maxSpeed = 100});
    // pros::delay(400);
    // chassis.pid_odom_set(-30.359_in, 25.391_in, -50_deg, 1500, {.maxSpeed = 40});
    // pros::delay(500);
  } else if (isRedAlliance && !isLeftSide) {
      ;
  } else if (!isRedAlliance && isLeftSide) {
      ;
  } else if (!isRedAlliance && !isLeftSide) {
      ;
  }


  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
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
  // This is preference to what you like to drive on
  // chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    // ez_template_extras();

    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Single Stick Arcade Control

    // int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // // Apply deadband to ignore small joystick movements
    // if (abs(dir) < 10) dir = 0;
    // if (abs(turn) < 10) turn = 0;

    // // quadratic for better control
    // auto curve = [](double x) -> double {
    //     if (x == 0) return 0;
    //     return pow(x*0.0787, 2) * (std::abs(x)/x);
    // };

    // // Scale inputs to velocity (-600 to 600 for blue cartridges)
    // double vel_dir = curve(dir) * 6;
    // double vel_turn = curve(turn) * 6;

    // chassis.Drive(vel_dir + vel_turn, vel_dir - vel_turn);

    static bool intake_forward = false;
        static bool intake_reverse = false;
        static bool single_intake_mode = false;
        static bool goal_intake_mode = false;

        static bool last_r1_state = false;
        static bool last_r2_state = false;
        static bool last_l1_state = false;
        static bool last_l2_state = false;
        
        bool current_r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool current_r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool current_l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool current_l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        
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

        // Toggle reverse on L1
        if (current_l1 && !last_l1_state) {
            single_intake_mode = !single_intake_mode;  // Toggle reverse
        }
        last_l1_state = current_l1;

        // Toggle reverse on L2
        if (current_l2 && !last_l2_state) {
            goal_intake_mode = !goal_intake_mode;  // Toggle reverse
        }
        last_l2_state = current_l2;
        
        // Run intake based on which button was toggled
        if (intake_forward) {
            int hue = color_sensor.get_hue();
            if (isRedAlliance) {
              // Red alliance: intake red, eject blue
              if (hue < 30 || hue > 330) {  // Red detected
                  intake.move(127);
                  intake2.move(127);
                  intake3.move(-127);
              } else if (hue > 180 && hue < 270) {  // Blue detected
                  intake.move(-127);
                  intake2.move(-127);
                  intake3.move(-127);
              }
          } else if (!isRedAlliance) {
              // Blue alliance: intake blue, eject red
              if (hue > 180 && hue < 270) {  // Blue detected
                  intake.move(127);
                  intake2.move(127);
                  intake3.move(-127);
              } else if (hue < 30 || hue > 330) {  // Red detected
                  intake.move(-127);
                  intake2.move(-127);
                  intake3.move(-127);
              }
          }
             
        } else if (intake_reverse) {
            intake.move(-127);
            intake2.move(-127);
            intake3.move(-127);
        } else if (single_intake_mode) {
            intake.move(127);     
            intake2.move(0);
            intake3.move(0);
        } else if (goal_intake_mode) {
            intake.move(127);       
            intake2.move(127);
            intake3.move(-127);
        } else {
            intake.move(0);
            intake2.move(0);
            intake3.move(0);
        }

        static bool match_state = false;
        static bool a_last_state = false;
        bool a_current_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        
        if (a_current_state && !a_last_state) {  // L1 just pressed
            match_state = !match_state;    // Toggle state
            match.set_value(match_state);  // Set solenoid D to new state
        }
        a_last_state = a_current_state;

        static bool wings_state = false;
        static bool b_last_state = false;
        bool b_current_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        
        if (b_current_state && !b_last_state) {  // L2 just pressed
            wings_state = !wings_state;    // Toggle state
            wings.set_value(wings_state);  // Set solenoid B to new state
        } 
        b_last_state = b_current_state;

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
