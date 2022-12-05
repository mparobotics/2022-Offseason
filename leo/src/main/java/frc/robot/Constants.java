// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //port of the xbox controller
    public static final int CONTROLLER_PORT = 0;

    //drive motor IDs
    public static final int MOTOR_FL_ID = 9;
    public static final int MOTOR_BL_ID = 3;
    public static final int MOTOR_FR_ID = 8;
    public static final int MOTOR_BR_ID = 7;
    //claw motor ID
    public static final int CLAW_MOTOR_ID = 53; 

    
   


    //placeholders - driving speed of the robot
    public static final double DRIVE_SPEED = 1;
    public static final double TURN_SPEED = 1;

    //placeholders - driving speeds during autonomous
    public static final double AUTO_DRIVE_SPEED = 1;
    public static final double AUTO_TURN_SPEED = 1;
    

}
