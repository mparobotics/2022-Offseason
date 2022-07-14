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
    public final static class DriveConstants{
        public static final int FALCON_FR_ID = 47; //ID of front right falcon
        public static final int FALCON_FL_ID = 48; // ID of front left falcon
        public static final int FALCON_BR_ID = 46; // ID of back right falcon
        public static final int FALCON_BL_ID = 49; // ID of back left falcon
        /**
         * The DriveConstants class is used to define all of the constants we use for driving. 
         * These include the IDs of the motors, conversion rates from tics to feet, etc.
         * To call this, call from the constants file as you normally would, but instead of doing
         * Constants.whateveryouwanttocall say DriveConstants.whateveryouwanttocall
         * IDS are the ports the motors are connected to
         */
    }

    public final static class OIConstants{
        public static final int XBOX_ID = 0;

    }





}
