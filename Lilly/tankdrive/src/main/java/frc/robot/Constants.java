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
    //motor IDs. Public means acessuble by all files. Final means not going to change. Idk what static means.
    public final static int MOTOR_FL_ID = 9;
    public final static int MOTOR_BL_ID = 3;
    public final static int MOTOR_FR_ID = 8;
    public final static int MOTOR_BR_ID = 7;

    public final static int CONTROLLER_PORT = 0;
}
