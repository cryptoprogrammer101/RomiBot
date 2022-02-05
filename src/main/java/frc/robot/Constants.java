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
    // gyro adjustment factor
    public final static double GAIN = 0.03;
    // gyro margin for turning
    public final static double MARGIN = 40;
    // multiplier for precision
    public final static double MULTIPLIER = 0.5;
    // turning sped
    public final static double TURNSPEED = 0.5;
    // set speed tolerance
    public final static double SPEEDTOLERANCE = 5;
    // set velocity tolerance
    public final static double VELOCITYTOLERANCE = 10;
}
