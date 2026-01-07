#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor

extern pros::Motor intake;
extern pros::Motor intake2;
extern pros::Motor intake3;

extern pros::ADIDigitalOut match;
extern pros::ADIDigitalOut wings;
extern pros::ADIDigitalOut stopper; 

extern pros::Optical color_sensor;

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(6.7, 0.0, 15);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.0, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  // // LEFTSIDE AUTOS

  
  // stopper.set_value(false);
  

  // chassis.pid_drive_set(33_in, 90); //move to match load
  // chassis.pid_wait();

  // chassis.pid_turn_set(-90_deg, 90); //turnt to matchload
  // chassis.pid_wait();

  // intake.move_velocity(300); //turn on intake
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);


  // match.set_value(true);  //matchload down
  // pros::delay(300);

  // chassis.pid_drive_set(17_in, 60, true); //matchload 3 balls
  // chassis.pid_wait();
  

  // // intake.move_velocity(-300); //pulse
  // // intake2.move_velocity(-300);
  // // intake3.move_velocity(-300);
  // // pros::delay(50);

  // intake.move_velocity(300); //spin
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);

  // chassis.pid_drive_set(-34_in, 90);//line up for long goal scoring
  // pros::delay(420);
  // stopper.set_value(true);//stopper toggle
  // chassis.pid_wait();

  // // intake.move_velocity(-300); //pulse
  // // intake2.move_velocity(-300);
  // // intake3.move_velocity(-300);
  // // pros::delay(50);

  // // intake.move_velocity(300); //score
  // // intake2.move_velocity(300);
  // // intake3.move_velocity(300);



  // match.set_value(false); //close matchloader

  // chassis.pid_drive_set(14_in, 90); //move back 
  // chassis.pid_wait();

  // stopper.set_value(false);//stopper toggle

  // chassis.pid_turn_set(140_deg, 90);//turn to blocks
  // chassis.pid_wait();

  // chassis.pid_drive_set(25_in,90);//move to intake blocks
  // chassis.pid_wait();

  // intake.move_velocity(300); //outtake into mid goal
  // intake2.move_velocity(300);
  // intake3.move_velocity(-150);

  // chassis.pid_drive_set( 12_in,40);//move to intake blocks
  // match.set_value(true);
  // chassis.pid_wait();
  // pros::delay(100);
  // chassis.pid_drive_set( 3_in,40);//move to intake blocks
  // match.set_value(false);

  // chassis.pid_turn_set(320_deg, 70); //turn to goal
  // chassis.pid_wait();


  // chassis.pid_drive_set(-14_in,70); 
  // chassis.pid_wait();
  // stopper.set_value(true);
  // pros::delay(200);
  // chassis.pid_wait();

//==============================================================================================
//RIGHTSIDE

  // stopper.set_value(false);
  

  // chassis.pid_drive_set(33_in, 90); //move to match load
  // chassis.pid_wait();

  // chassis.pid_turn_set(90_deg, 90); //turnt to matchload
  // chassis.pid_wait();

  // intake.move_velocity(300); //turn on intake
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);

  // match.set_value(true);  //matchload down
  // pros::delay(300);

  // chassis.pid_drive_set(17_in, 60, true); //matchload 3 balls
  // chassis.pid_wait();
  

  // // intake.move_velocity(-300); //pulse
  // // intake2.move_velocity(-300);
  // // intake3.move_velocity(-300);
  // // pros::delay(50);

  // intake.move_velocity(300); //spin
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);

  // chassis.pid_drive_set(-34_in, 70);//ne up for long goal scoring
  // pros::delay(420);
  // stopper.set_value(true);//stopper toggle
  // chassis.pid_wait();

  // // intake.move_velocity(-300); //pulse
  // // intake2.move_velocity(-300);
  // // intake3.move_velocity(-300);
  // // pros::delay(50);

  // intake.move_velocity(300); //score
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);



  // match.set_value(false); //close matchloader

  // chassis.pid_drive_set(14_in, 90, false); //move back 
  // chassis.pid_wait();

  // stopper.set_value(false);//stopper toggle

  // chassis.pid_turn_set(-140_deg, 90);//turn to blocks
  // chassis.pid_wait();

  // chassis.pid_drive_set(25_in,90);//move to intake blocks
  // chassis.pid_wait();

  // intake.move_velocity(300); //outtake into mid goal
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);

  // chassis.pid_drive_set( 27_in,40);//move to intake blocks
  // chassis.pid_wait();


  // intake.move_velocity(-300); //outtake into lower goal
  // intake2.move_velocity(-300);
  // intake3.move_velocity(-300);
  // chassis.pid_wait();
  // pros::delay(300);
  // stopper.set_value(true);
  




//new 

  // chassis.pid_drive_set(-44_in, 127);//back to highgoal
  // chassis.pid_wait();

  // wings.set_value(false);

  // chassis.pid_turn_set(90_deg, 127); //turn to goal
  // chassis.pid_wait();

  // wings.set_value(true);

  // chassis.pid_drive_set(10_in, 100);//push blocks 
  // chassis.pid_wait();


//============================================================================================
// //SOLO AWP

//  stopper.set_value(false);
  

//   chassis.pid_drive_set(32_in, 110,true); //move to match load
//   chassis.pid_wait_quick();

//   chassis.pid_turn_set(-90_deg, 110); //turnt to matchload
//   chassis.pid_wait_quick();

//   intake.move_velocity(300); //turn on intake
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);


//   match.set_value(true);  //matchload down
//   pros::delay(100);

//   chassis.pid_drive_set(17_in, 70); //matchload 3 balls
//   chassis.pid_wait();
  

//   // intake.move_velocity(-300); //pulse
//   // intake2.move_velocity(-300);
//   // intake3.move_velocity(-300);
//   // pros::delay(50);

//   intake.move_velocity(300); //spin
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);

//   chassis.pid_drive_set(-34_in, 110);//line up for long goal scoring
//   stopper.set_value(true);//stopper toggle
//   pros::delay(250);
//   chassis.pid_wait();

//   // intake.move_velocity(-300); //pulse
//   // intake2.move_velocity(-300);
//   // intake3.move_velocity(-300);
//   // pros::delay(50);

//   // intake.move_velocity(300); //score
//   // intake2.move_velocity(300);
//   // intake3.move_velocity(300);

//   match.set_value(false); //close matchloader

//   chassis.pid_drive_set(16_in, 110); //move back 
//   chassis.pid_wait_quick();

//   stopper.set_value(false);//stopper toggle

//   chassis.pid_turn_set(145_deg, 90);//turn to blocks
//   chassis.pid_wait_quick();

//   chassis.pid_drive_set(32_in,90);//move to intake blocks
//   chassis.pid_wait();
//   match.set_value(true);
//   intake.move_velocity(100); //outtake into mid goal
//   intake2.move_velocity(110);
//   intake3.move_velocity(-110);
//   chassis.pid_wait_quick(); 
//   //chassis.pid_drive_set( 13_in,75);//move 
  


//   chassis.pid_turn_set(320_deg, 80); //turn to blocks
//   chassis.pid_wait_quick();

//   chassis.pid_drive_set(-20_in,110); 
//   stopper.set_value(true);
//   chassis.pid_wait_quick();

//   pros::delay(300);

//   chassis.pid_drive_set(7_in, 127); // back out
//   chassis.pid_wait_quick();

//   chassis.pid_turn_set(210, 127); // turn to loader
//   chassis.pid_wait_quick();

//   chassis.pid_drive_set(70_in, 127); // drive to matchload
//   chassis.pid_wait_quick();

//   chassis.pid_turn_set(-90_deg, 127); //turnt to matchload
//   chassis.pid_wait_quick();

//   intake.move_velocity(300); //turn on intake
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);


//   match.set_value(true);  //matchload down
//   pros::delay(150);

//   chassis.pid_drive_set(17_in, 70); //matchload 3 balls
//   chassis.pid_wait_quick();
  

//   // intake.move_velocity(-300); //pulse
//   // intake2.move_velocity(-300);
//   // intake3.move_velocity(-300);
//   // pros::delay(50);

//   intake.move_velocity(300); //spin
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);

//   chassis.pid_drive_set(-34_in, 110);//ne up for long goal scoring
//   pros::delay(300);
//   stopper.set_value(true);//stopper toggle
//   chassis.pid_wait_quick();


//===========================================================================================================
//SOLO AWP V2 RIGHTSIDE

  
  

  chassis.pid_drive_set(34_in, 110); //move to match load
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110); //turnt to matchload
  chassis.pid_wait();  

  pros::Task([=]{
    intake.move_velocity(300); //turn on intake
    intake2.move_velocity(300);
    intake3.move_velocity(300);

    match.set_value(true);
    pros::delay(1000);
    intake.move_velocity(0); //turn on intake
    intake2.move_velocity(0);
    intake3.move_velocity(0);

    pros::delay(1000);
      intake.move_velocity(300); //score
      intake2.move_velocity(300);
      intake3.move_velocity(300);
      stopper.set_value(true);
  });



  chassis.pid_drive_set(15_in, 60); //moves matchload 3 ball
  chassis.pid_wait_until(15);

  // intake.move_velocity(-300); //pulse
  // intake2.move_velocity(-300);
  // intake3.move_velocity(-300);
  // pros::delay(50);

  // intake.move_velocity(300); //spin
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);
  // pros::delay(300);


  chassis.pid_drive_set(-34_in, 127);//ne up for long goal scoring
  chassis.pid_wait_until(-31_in);

  pros::delay(2000); //wait for intake to finish scoring

  match.set_value(false); //close matchloader
  stopper.set_value(false);

  

  // intake.move_velocity(300); //outtake into mid goal
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);

  chassis.pid_turn_set(200_deg, 90); //turnt to matchload
  chassis.pid_wait();

  chassis.pid_drive_set(23, 90);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, 90); //turnt to matchload
  chassis.pid_wait();


  chassis.pid_drive_set(44_in, 60);
  chassis.pid_wait();

  // chassis.pid_drive_set(16_in, 90, false); //move back 
  // chassis.pid_wait();

  // stopper.set_value(false);//stopper toggle

  // chassis.pid_turn_set(-140_deg, 90);//turn to blocks
  // chassis.pid_wait();

  // chassis.pid_drive_set(27_in,90);//move to intake blocks
  // chassis.pid_wait();
  // match.set_value(true);

  // intake.move_velocity(300); //outtake into mid goal
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);


  // chassis.pid_turn_set(180_deg, 90); //turnt to matchload
  // chassis.pid_wait();
  // match.set_value(false);

  // chassis.pid_drive_set(48_in, 90);
  // chassis.pid_wait();
  // match.set_value(true);



//===========================================================================================================
//Auto Skills

//   stopper.set_value(false);
//   intake.move_velocity(300); //turn on intake
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);

  

//   chassis.pid_drive_set(33_in, 90); //move to match load
//   chassis.pid_wait();

//   chassis.pid_turn_set(90_deg, 90); //turnt to matchload
//   chassis.pid_wait();

//   intake.move_velocity(300); //turn on intake
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);

//   match.set_value(true);  //matchload down
//   pros::delay(100);

//   chassis.pid_drive_set(17_in, 40, true); //matchload 
//   chassis.pid_wait();
//   chassis.pid_drive_set(3_in, 20, true); //matchload 
//   chassis.pid_wait();
//   pros::delay(450);

//   chassis.pid_drive_set(-10_in, 90); //move to turning spot
//   chassis.pid_wait();
//   match.set_value(false);

//   chassis.pid_turn_set(120_deg, 90); //turnt to alley
//   chassis.pid_wait();

//   chassis.pid_drive_set(-25_in, 90); //move to alley
//   chassis.pid_wait();

//   chassis.pid_turn_set(90_deg, 90); //turn to parralel
//   chassis.pid_wait();

//   chassis.pid_drive_set(-75, 90); //drive down alley
//   chassis.pid_wait();

//   chassis.pid_swing_set(ez::LEFT_SWING, -83, 90, ez::cw); //-65
//   chassis.pid_wait();
//   // chassis.pid_turn_set(-65_deg, 90); //turn to goal
//   // chassis.pid_wait();

//   intake.move_velocity(0); //turn on intake 
//   intake2.move_velocity(0);
//   intake3.move_velocity(0);

//   chassis.pid_drive_set(-27_in, 90); //move to goal
//   chassis.pid_wait(); 

//   chassis.pid_turn_set(-90_deg, 90); //turn to goal
//   chassis.pid_wait();  

//   chassis.pid_drive_set(-5_in, 127); //align to goal
//   chassis.pid_wait();

//   stopper.set_value(true);
//   intake.move_velocity(-300); //pulse intake
//   intake2.move_velocity(-300);
//   intake3.move_velocity(-300);
//   //chassis.pid_wait();
//   pros::delay(200);

//   match.set_value(true);

//   intake.move_velocity(300); //turn on intake + score on high goal
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);
//   pros::delay(2500);//670

//   chassis.pid_wait(); //move back up later
//   stopper.set_value(false);

//   chassis.pid_drive_set(48_in, 60); //move to matchloader 
//   chassis.pid_wait(); 
//   pros::delay(400);

//   chassis.pid_drive_set(-48_in, 90); //move to goal
//   chassis.pid_wait();
//   stopper.set_value(true);
//   intake.move_velocity(-300); //pulse intake
//   intake2.move_velocity(-300);
//   intake3.move_velocity(-300);
//   //chassis.pid_wait();
//   pros::delay(200);

//   match.set_value(true);

//   intake.move_velocity(300); //turn on intake + score on high goal
//   intake2.move_velocity(300);
//   intake3.move_velocity(300);
//   pros::delay(2500);//670


//   chassis.pid_turn_set(-120_deg, 90); //turn to wall
//   chassis.pid_wait();
//   match.set_value(false);

//   chassis.pid_drive_set(48_in, 90); //drive to wall
//   chassis.pid_wait(); 
// //=========================================================================================================
//   // chassis.pid_swing_set(LEFT_SWING, 40, 90,  ez::counterclockwise);
//   // chassis.pid_wait();

//   chassis.pid_turn_set(180_deg, 90,ez::counterclockwise); //turn to parallel wall
//   chassis.pid_wait();  

//   chassis.pid_drive_set(24_in, 90); //drive to wall
//   chassis.pid_wait(); 

  // chassis.pid_turn_set(-45_deg, 90); //turn to parallel wall
  // chassis.pid_wait();  

  // chassis.pid_drive_set(35_in, 100); //drive to wall
  // chassis.pid_wait(); 

  // stopper.set_value(false);

  // chassis.pid_drive_set(48_in, 90); //move to matchloader 
  // chassis.pid_wait(); 
  // pros::delay(400);

  // chassis.pid_drive_set(-48_in, 90); //move to goal
  // chassis.pid_wait(); 
  // stopper.set_value(true);
  // chassis.pid_wait();
  // pros::delay(400);
  // match.set_value(false);



  

   





  // chassis.pid_turn_set(-20_deg, 127); // wiggle some more


  // chassis.pid_turn_set(20_deg, 127); // wiggle some more


  // chassis.pid_turn_set(0_deg, 127); // reset
  // chassis.pid_wait();
  // pros::delay(200);

  // chassis.pid_drive_set(-20_in, 100); //move out of goal
  // chassis.pid_wait();

  // chassis.pid_turn_set(140_deg, 100); // turn to 4 blocks 
  // pros::delay(100);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-27_in, 100); //move to 4 blocks
  // chassis.pid_wait();

  // chassis.pid_turn_set(225_deg, 100); // turn to low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(10_in, 100); //move to low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(-15_in, 100); //move to low goal
  // chassis.pid_wait();

  // stopper.set_value(true);
  // intake.move_velocity(300); //outake onto high midgoal
  // intake2.move_velocity(300);
  // intake3.move_velocity(-300);

  // chassis.pid_drive_set(50_in, 100); //move infront of matchloader
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, 100); // turn to high goal
  // chassis.pid_wait();

  // stopper.set_value(false);
  // intake.move_velocity(300); //turn on intake
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);

  // match.set_value(true);  //matchload down
  // pros::delay(100);

  // chassis.pid_drive_set(23_in, 100); //drive into matchload
  // chassis.pid_wait();
  // pros::delay(300);

  // chassis.pid_drive_set(-10_in, 100); //drive out of matchload
  // chassis.pid_wait();

  // match.set_value(false);  //matchload up
  // pros::delay(100);

  // chassis.pid_turn_set(124_deg, 100); // turn to side of low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(-19_in, 100); //drive out of matchload
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, 100); // turn to side of low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(-94_in, 100); //drive to other side
  // chassis.pid_wait();

  // chassis.pid_turn_set(90_deg, 100); // turn to side of low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(10_in, 100); //drive into highgoal
  // chassis.pid_wait();

  // chassis.pid_turn_set(0_deg, 100); // make aligner face back
  // chassis.pid_wait();

  // chassis.pid_drive_set(-29_in, 100); //drive into highgoal

  // stopper.set_value(true); //score
  // intake.move_velocity(300); 
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);
  // chassis.pid_wait();

  // pros::delay(100);

  // stopper.set_value(false);
  // match.set_value(true);  //matchload down
  // pros::delay(100);

  // chassis.pid_drive_set(34_in, 100); //drive to matchload
  // chassis.pid_wait();
  // pros::delay(200);

  // chassis.pid_drive_set(-34_in, 100); //drive to highgoal
  // chassis.pid_wait();

  // stopper.set_value(true); //score
  // pros::delay(200);

  // chassis.pid_drive_set(10_in, 100); //drive out of highgoal
  // chassis.pid_wait();

  // chassis.pid_turn_set(90_deg, 100); // turn to other side
  // chassis.pid_wait();

  // chassis.pid_drive_set(95_in, 100); //drive to other highgoal
  // chassis.pid_wait();

  // chassis.pid_turn_set(0_deg, 100); // turn to match loader
  // chassis.pid_wait();

  // chassis.pid_drive_set(24_in, 100); //drive to matchloader
  // chassis.pid_wait();

  // match.set_value(true);  //matchload down
  // pros::delay(50);

  // pros::delay(200); // wait for matchloads

  // chassis.pid_drive_set(-10_in, 100); //drive out of matchload
  // chassis.pid_wait();

  // chassis.pid_turn_set(124_deg, 100); // turn to side of low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(19_in, 100); //drive to outside of high goal 
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, 100); // turn to side of low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(94_in, 100); //drive to outside of high goal 
  // chassis.pid_wait();

  // chassis.pid_turn_set(270_deg, 100); // turn to side of low goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(10_in, 100); //drive into highgoal
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, 100); // turn to side of low goal
  // // chassis.pid_wait();

  // chassis.pid_drive_set(-29_in, 100); //drive into highgoal

  // stopper.set_value(true); //score
  // intake.move_velocity(300); 
  // intake2.move_velocity(300);
  // intake3.move_velocity(300);
  // chassis.pid_wait();

  // pros::delay(100);

  // stopper.set_value(false);
  // match.set_value(true);  //matchload down
  // pros::delay(100);

  // chassis.pid_drive_set(34_in, 100); //drive to matchload
  // chassis.pid_wait();
  // pros::delay(200);

  // chassis.pid_drive_set(-34_in, 100); //drive to highgoal
  // chassis.pid_wait();

  // stopper.set_value(true); //score
  // pros::delay(200);

  // chassis.pid_drive_set(10_in, 100); //drive out of highgoal
  // chassis.pid_wait();

  // chassis.pid_turn_set(270_deg, 100); // turn to other side
  // chassis.pid_wait();

  // chassis.pid_drive_set(48_in, 100); //drive to red parking spot 
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, 100);
  // chassis.pid_wait();

  // chassis.pid_drive_set(20_in, 100); //drive to matchloader
  // chassis.pid_wait();

}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
 
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

void mainAuton() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}